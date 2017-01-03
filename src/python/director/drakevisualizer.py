import math
import os
import json
import numpy as np
from collections import namedtuple
from director import lcmUtils

import director.objectmodel as om
import director.applogic as app
from director import transformUtils
from director.debugVis import DebugData
from director import ioUtils
from director import filterUtils
from director.shallowCopy import shallowCopy
from director import vtkAll as vtk
from director import vtkNumpy as vnp
from director import visualization as vis
from director import packagepath

import bot_core as lcmbot

# Currently, viewer lcm message types are in bot_core_lcmtypes and
# robotlocomotion-lcmtypes, but drake only builds bot_core_lcmtypes.
# When drake starts using robotlocomotion-lcmtypes, then the following
# import can be used instead of getting viewer messages from bot_core_lcmtypes.
#import robotlocomotion as lcmrl
lcmrl = lcmbot

from PythonQt import QtGui

USE_TEXTURE_MESHES = True
USE_SHADOWS = False


class ViewerStatus:
    OK = 0
    MISSING_DATA = 1
    ERROR_UNKNOWN_FORMAT = 3
    ERROR_UNKNOWN_FORMAT_VERSION = 4
    ERROR_HANDLING_REQUEST = 5
    ERROR_UKNOWN_REQUEST_TYPE = 6


class ViewerResponse(namedtuple("ViewerResponse", ["status", "data"])):
    def toJson(self):
        return dict(status=self.status, **self.data)


def transformFromDict(pose_data):
    return transformUtils.transformFromPose(
        pose_data.get("translation", [0, 0, 0]),
        pose_data.get("quaternion", [1, 0, 0, 0]))


class Geometry(object):
    TextureCache = {}
    PackageMap = None

    @staticmethod
    def createBox(params):
        d = DebugData()
        d.addCube(dimensions=params["lengths"], center=(0, 0, 0))
        return [d.getPolyData()]

    @staticmethod
    def createSphere(params):
        d = DebugData()
        d.addSphere(center=(0, 0, 0), radius=params["radius"])
        return [d.getPolyData()]

    @staticmethod
    def createCylinder(params):
        d = DebugData()
        d.addCylinder(center=(0, 0, 0),
                      axis=(0, 0, 1),
                      radius=params["radius"],
                      length=params["length"])
        return [d.getPolyData()]

    @staticmethod
    def createCapsule(params):
        d = DebugData()
        radius = params["radius"]
        length = params["length"]
        d.addCylinder(center=(0, 0, 0),
                      axis=(0, 0, 1),
                      radius=radius,
                      length=length)
        d.addSphere(center=(0, 0, length / 2.0), radius=radius)
        d.addSphere(center=(0, 0, -length / 2.0), radius=radius)
        return [d.getPolyData()]

    @staticmethod
    def createEllipsoid(params):
        d = DebugData()
        radii = params["radii"]
        d.addEllipsoid(center=(0, 0, 0), radii=radii)
        return [d.getPolyData()]

    @staticmethod
    def createMeshFromFile(params):
        polyDataList = Geometry.loadPolyDataMeshes(params["filename"])
        if "scale" in params:
            polyDataList = Geometry.scaleGeometry(polyDataList,
                                                  params["scale"])
        return polyDataList

    @staticmethod
    def createMeshFromData(params):
        verts = np.asarray(params["vertices"])
        faces = np.asarray(params["faces"])
        return [Geometry.createPolyDataFromMeshArrays(verts, faces)]

    @staticmethod
    def createPointcloud(params):
        polyData = vnp.numpyToPolyData(np.asarray(params["points"]),
                                       createVertexCells=True)
        return [polyData]

    @staticmethod
    def createPlanarLidar(params):
        ranges = np.asarray(params["ranges"])
        angle_stop = (params["angle_start"] +
                      len(ranges) * params["angle_step"])
        angles = np.arange(
            params["angle_start"],
            angle_stop,
            params["angle_step"])
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros(x.shape)
        points = np.vstack((x, y, z)).T
        return [vnp.numpyToPolyData(points, createVertexCells=True)]

    @staticmethod
    def createPolyData(params):
        if params["type"] == "box":
            return Geometry.createBox(params)
        elif params["type"] == "sphere":
            return Geometry.createSphere(params)
        elif params["type"] == "cylinder":
            return Geometry.createCylinder(params)
        elif params["type"] == "capsule":
            return Geometry.createCapsule(params)
        elif params["type"] == "ellipsoid":
            return Geometry.createEllipsoid(params)
        elif params["type"] == "mesh_file":
            return Geometry.createMeshFromFile(params)
        elif params["type"] == "mesh_data":
            return Geometry.createMeshFromData(params)
        elif params["type"] == "pointcloud":
            return Geometry.createPointcloud(params)
        elif params["type"] == "planar_lidar":
            return Geometry.createPlanarLidar(params)
        else:
            raise Exception(
                "Unsupported geometry type: {}".format(params["type"]))

    @staticmethod
    def createPolyDataFromMeshArrays(pts, faces):
        pd = vtk.vtkPolyData()
        pd.SetPoints(vtk.vtkPoints())
        pd.GetPoints().SetData(vnp.getVtkFromNumpy(pts.copy()))

        cells = vtk.vtkCellArray()
        for face in faces:
            assert len(face) == 3, "Non-triangular faces are not supported."
            tri = vtk.vtkTriangle()
            tri.GetPointIds().SetId(0, face[0])
            tri.GetPointIds().SetId(1, face[1])
            tri.GetPointIds().SetId(2, face[2])
            cells.InsertNextCell(tri)

        pd.SetPolys(cells)
        return pd

    @staticmethod
    def scaleGeometry(polyDataList, scale):
        if len(scale) == 1:
            scale_x = scale_y = scale_z = scale
        elif len(geom.float_data) == 3:
            scale_x, scale_y, scale_z = scale

        if scale_x != 1.0 or scale_y != 1.0 or scale_z != 1.0:
            t = vtk.vtkTransform()
            t.Scale(scale_x, scale_y, scale_z)
            polyDataList = [filterUtils.transformPolyData(polyData, t) for polyData in polyDataList]

        return polyDataList

    # @staticmethod
    # def transformGeometry(polyDataList, transform):
    #     polyDataList = [filterUtils.transformPolyData(polyData, transform) for polyData in polyDataList]
    #     return polyDataList

    @staticmethod
    def computeNormals(polyDataList):

        def addNormals(polyData):
            hasNormals = polyData.GetPointData().GetNormals() is not None
            return polyData if hasNormals else filterUtils.computeNormals(polyData)

        return [addNormals(polyData) for polyData in polyDataList]


    @staticmethod
    def getTextureFileName(polyData):
        textureArray = vtk.vtkStringArray.SafeDownCast(polyData.GetFieldData().GetAbstractArray('texture_filename'))
        if not textureArray:
            return None
        return textureArray.GetValue(0)

    @staticmethod
    def loadTextureForMesh(polyData, meshFileName):

        textureFileName = Geometry.getTextureFileName(polyData)
        if textureFileName in Geometry.TextureCache or textureFileName is None:
            return

        if not os.path.isabs(textureFileName):
            baseDir = os.path.dirname(meshFileName)
            imageFile = os.path.join(baseDir, textureFileName)
        else:
            imageFile = textureFileName

        if not os.path.isfile(imageFile):
            print 'cannot find texture file:', textureFileName
            return

        image = ioUtils.readImage(imageFile)
        if not image:
            print 'failed to load image file:', imageFile
            return

        texture = vtk.vtkTexture()
        texture.SetInput(image)
        texture.EdgeClampOn()
        texture.RepeatOn()

        Geometry.TextureCache[textureFileName] = texture

    @staticmethod
    def resolvePackageFilename(filename):

        if not packagepath.PackageMap.isPackageUrl(filename):
            return filename

        if Geometry.PackageMap is None:
            import pydrake
            m = packagepath.PackageMap()
            m.populateFromSearchPaths([pydrake.getDrakePath()])
            m.populateFromEnvironment(['DRAKE_PACKAGE_PATH', 'ROS_PACKAGE_PATH'])
            Geometry.PackageMap = m

        return Geometry.PackageMap.resolveFilename(filename) or filename

    @staticmethod
    def loadPolyDataMeshes(filename):

        filename = Geometry.resolvePackageFilename(filename)
        basename, ext = os.path.splitext(filename)

        preferredExtensions = ['.vtm', '.vtp', '.obj']

        for x in preferredExtensions:
            if os.path.isfile(basename + x):
                filename = basename + x
                break

        if not os.path.isfile(filename):
            print 'warning, cannot find file:', filename
            return []

        if filename.endswith('vtm'):
            polyDataList = ioUtils.readMultiBlock(filename)
        else:
            polyDataList = [ioUtils.readPolyData(filename)]

        if USE_TEXTURE_MESHES:
            for polyData in polyDataList:
                Geometry.loadTextureForMesh(polyData, filename)

        return polyDataList

    @staticmethod
    def createPolyDataForGeometry(geom):
        polyDataList = Geometry.createPolyData(geom)
        # polyDataList = Geometry.transformGeometry(
        #     polyDataList,
        #     transformFromDict(geom["pose"]))
        polyDataList = Geometry.computeNormals(polyDataList)
        return polyDataList

    @staticmethod
    def createGeometry(geom):
        polyDataList = Geometry.createPolyDataForGeometry(geom)

        geometry = []
        for polyData in polyDataList:
            g = Geometry(geom, polyData)
            geometry.append(g)
        return geometry

    @staticmethod
    def addColorChannels(polyData, channels):
        if "intensity" in channels:
            colorBy = "intensity"
            intensity = np.asarray(channels["intensity"]) * 255
            vnp.addNumpyToVtk(polyData,
                              intensity.astype(np.uint8),
                              "intensity")
        if "rgb" in channels:
            colorBy = "rgb"  # default to rgb if provided
            colorArray = np.asarray(channels["rgb"]) * 255
            vnp.addNumpyToVtk(polyData, colorArray.astype(np.uint8), "rgb")

    def __init__(self, geomData, polyData):
        if "channels" in geomData:
            Geometry.addColorChannels(polyData, geomData["channels"])
        self.polyDataItem = vis.PolyDataItem("geometry", polyData, view=None)
        self.polyDataItem._updateColorByProperty()

        color = geomData.get("color", [1, 0, 0, 0.5])
        self.polyDataItem.setProperty('Alpha', color[3])
        self.polyDataItem.actor.SetTexture(
            Geometry.TextureCache.get(
                Geometry.getTextureFileName(polyData)))

        if self.polyDataItem.actor.GetTexture():
            self.polyDataItem.setProperty('Color',
                                          QtGui.QColor(255, 255, 255))
        else:
            self.polyDataItem.setProperty(
                'Color',
                QtGui.QColor(*(255 * np.asarray(color[:3]))))

        if USE_SHADOWS:
            self.polyDataItem.shadowOn()


def findPathToAncestor(fromItem, toItem):
    path = [fromItem]
    while fromItem is not toItem:
        parent = fromItem.parent()
        if parent is None:
            raise ValueError(
                "Cannot find a path from {} to {}".format(fromItem, toItem))
        path.append(parent)
        fromItem = parent
    return path


class DrakeVisualizer(object):

    def __init__(self, view):

        self.subscribers = []
        self.view = view
        self.enable()
        self.sendStatusMessage(
            0, [ViewerResponse(ViewerStatus.OK, {"ready": True})])

    def _addSubscribers(self):
        self.subscribers.append(lcmUtils.addSubscriber(
            'DRAKE_VIEWER2_REQUEST',
            lcmrl.viewer2_comms_t,
            self.onViewerRequest))

    def _removeSubscribers(self):
        for sub in self.subscribers:
            lcmUtils.removeSubscriber(sub)
        self.subscribers = []

    def isEnabled(self):
        return bool(self.subscribers)

    def setEnabled(self, enabled):
        if enabled and not self.isEnabled():
            self._addSubscribers()
        elif not enabled and self.isEnabled():
            self._removeSubscribers()

    def enable(self):
        self.setEnabled(True)

    def disable(self):
        self.setEnabled(False)

    @staticmethod
    def consolidateResponses(responses):
        addedGeometries = set()
        setTransforms = set()
        missingTransforms = set()
        missingPaths = set()
        for response in responses:
            if response.status == ViewerStatus.OK:
                if "set_transform" in response.data:
                    setTransforms.add(tuple(response.data["set_transform"]))
                elif "added_geometry" in response.data:
                    addedGeometries.add(tuple(response.data["added_geometry"]))
            elif response.status == ViewerStatus.MISSING_DATA:
                for path in response.data["missing_paths"]:
                    missingPaths.add(tuple(path))
                for path in response.data["missing_transforms"]:
                    missingTransforms.add(tuple(path))
            else:
                raise NotImplementedError()
        missingTransforms.difference_update(setTransforms)
        missingPaths.difference_update(addedGeometries)
        if missingTransforms or missingPaths:
            status = ViewerStatus.MISSING_DATA
        else:
            status = ViewerStatus.OK
        return ViewerResponse(status, {
                            "missing_paths": list(missingPaths),
                            "missing_transforms": list(missingTransforms),
                            "added_geometries": list(addedGeometries),
                            "set_transforms": list(setTransforms)})

    def sendStatusMessage(self, timestamp, responses):
        msg = lcmrl.viewer2_comms_t()
        msg.format = "viewer2_json"
        msg.format_version_major = 1
        msg.format_version_minor = 0
        data = {"timestamp": timestamp}
        data["response"] = self.consolidateResponses(responses).toJson()
        print "responding:", data["response"]
        msg.data = json.dumps(data)
        msg.num_bytes = len(msg.data)
        lcmUtils.publish('DRAKE_VIEWER2_RESPONSE', msg)

    def decodeCommsMsg(self, msg):
        if msg.format == "viewer2_json":
            if msg.format_version_major == 1 and msg.format_version_minor == 0:
                data = json.loads(msg.data)
                return data, ViewerResponse(ViewerStatus.OK, {})
            else:
                return None, ViewerResponse(ViewerStatus.ERROR_UNKNOWN_FORMAT_VERSION,
                                            {"supported_formats": {
                                                 "viewer2_json": [{"major": 1,
                                                                   "minor": 0}]
                                            }})
        else:
            return None, ViewerResponse(ViewerStatus.ERROR_UNKNOWN_FORMAT,
                                        {"supported_formats": {
                                             "viewer2_json": [{"major": 1,
                                                               "minor": 0}]
                                        }})

    def onViewerRequest(self, msg):
        data, response = self.decodeCommsMsg(msg)
        if data is None:
            self.sendStatusMessage(msg.timestamp,
                                   [responses])
        else:
            responses = self.handleViewerRequest(data)
            self.sendStatusMessage(msg.timestamp,
                                   responses)

    def handleViewerRequest(self, data):
        responses = []
        print "commands:", [(c["type"], c["path"]) for c in data["commands"]]
        for command in data["commands"]:
            if command["type"] == "load":
                responses.append(self.handleAddGeometry(command))
            elif command["type"] == "draw":
                responses.append(self.handleSetTransform(command))
            elif command["type"] == "delete":
                responses.append(self.handleDeletePath(command))
            else:
                responses.append(ViewerResponse(
                    ViewerStatus.ERROR_UKNOWN_REQUEST_TYPE,
                    {"supported_requests": ["load", "draw", "delete"]}))
        return responses

    def handleAddGeometry(self, command):
        path = command["path"]
        vtkGeoms = Geometry.createGeometry(command["geometry"])
        return self._addGeometry(path, vtkGeoms)

    def _addGeometry(self, path, geomItems):
        folder = self.getPathFolder(path)
        for item in findPathToAncestor(folder, self.getRootFolder()):
            if not hasattr(item, "transform"):
                item.transform = vtk.vtkTransform()
                item.transform.PostMultiply()
                item.knownTransform = False
        for geom in geomItems:
            existing_item = self.getItemByPath(path + ["geometry"])
            item = geom.polyDataItem
            if existing_item is not None:
                for prop in existing_item.propertyNames():
                    item.setProperty(prop, existing_item.getProperty(prop))
                om.removeFromObjectModel(existing_item)
            else:
                item.setProperty("Point Size", 2)
                for colorBy in ["rgb", "intensity"]:
                    try:
                        item.setProperty("Color By", colorBy)
                    except ValueError:
                        pass
                    else:
                        break

            item.addToView(self.view)
            om.addToObjectModel(item, parentObj=folder)
        return ViewerResponse(ViewerStatus.OK, {"added_geometry": path})

    def getPathForItem(self, item):
        return [x.getProperty("Name") for x in reversed(findPathToAncestor(
            item, self.getRootFolder())[:-1])]

    def updateTransforms(self, item, transformToRoot, unknownTransforms):
        if isinstance(item, om.ContainerItem):
            if not item.knownTransform:
                unknownTransforms.append(self.getPathForItem(item))
            for child in item.children():
                childTransform = vtk.vtkTransform()
                childTransform.DeepCopy(transformToRoot)
                childTransform.PostMultiply()
                childTransform.Concatenate(item.transform)
                self.updateTransforms(child,
                                      childTransform, unknownTransforms)
        else:
            childFrame = item.getChildFrame()
            if childFrame:
                childFrame.copyFrame(transformToRoot)
            else:
                item.actor.SetUserTransform(transformToRoot)
            self.view.render()

    def handleSetTransform(self, command):
        return self._setTransform(command["path"],
                                  transformFromDict(command["transform"]))

    def _setTransform(self, path, transform):
        folder = self.getPathFolder(path)
        folder.transform = transform
        folder.knownTransform = True
        ancestors = findPathToAncestor(folder,
                                       self.getRootFolder())[:-1]
        for t in ancestors:
            if not hasattr(t, "transform"):
                t.transform = vtk.vtkTransform()
                t.knownTransform = False
        transforms = [x.transform for x in reversed(ancestors)]
        unknownTransforms = [self.getPathForItem(x) for x in ancestors
                             if not x.knownTransform]
        transformToRoot = transformUtils.concatenateTransforms(transforms)
        self.updateTransforms(folder, transformToRoot, unknownTransforms)

        if len(folder.children()) == 0:
            missingPaths = [path]
        else:
            missingPaths = []
        if unknownTransforms or missingPaths:
            return ViewerResponse(ViewerStatus.MISSING_DATA,
                                  {"missing_transforms": unknownTransforms,
                                   "missing_paths": missingPaths})
        return ViewerResponse(ViewerStatus.OK,
                              {"set_transform": path})

    def deletePaths(self, data):
        for path in data["paths"]:
            print "deleting path:", path
            item = self.getItemByPath(path)
            print "item:", item
            if item is not None:
                om.removeFromObjectModel(item)
        return ViewerResponse(ViewerStatus.OK, {
                              "deleted_paths": data["paths"]
                              })

    def getRootFolder(self):
        return om.getOrCreateContainer('drake viewer',
                                       parentObj=om.findObjectByName('scene'))

    def getItemByPath(self, path):
        item = self.getRootFolder()
        for element in path:
            item = item.findChild(element)
            if item is None:
                return None
        return item

    def getPathFolder(self, path):
        folder = self.getRootFolder()
        for element in path:
            folder = om.getOrCreateContainer(element, parentObj=folder)
        return folder


##########################################################
from director.screengrabberpanel import ScreenGrabberPanel
from director import lcmgl
from director import objectmodel as om
from director import applogic
from director import appsettings
from director import viewbehaviors
from director import camerabookmarks
from director import cameracontrolpanel
import PythonQt
from PythonQt import QtCore, QtGui


class DrakeVisualizerApp():

    def __init__(self):

        self.applicationInstance().setOrganizationName('RobotLocomotionGroup')
        self.applicationInstance().setApplicationName('drake-visualizer')

        om.init()
        self.view = PythonQt.dd.ddQVTKWidgetView()

        # init grid
        self.gridObj = vis.showGrid(self.view, parent='scene')
        self.gridObj.setProperty('Surface Mode', 'Surface with edges')
        self.gridObj.setProperty('Color', [0,0,0])
        self.gridObj.setProperty('Alpha', 0.1)

        # init view options
        self.viewOptions = vis.ViewOptionsItem(self.view)
        om.addToObjectModel(self.viewOptions, parentObj=om.findObjectByName('scene'))
        self.viewOptions.setProperty('Background color', [0.3, 0.3, 0.35])
        self.viewOptions.setProperty('Background color 2', [0.95,0.95,1])

        # setup camera
        applogic.setCameraTerrainModeEnabled(self.view, True)
        applogic.resetCamera(viewDirection=[-1, 0, -0.3], view=self.view)

        # This setting improves the near plane clipping resolution.
        # Drake often draws a very large ground plane which is detrimental to
        # the near clipping for up close objects.  The trade-off is Z buffer
        # resolution but in practice things look good with this setting.
        self.view.renderer().SetNearClippingPlaneTolerance(0.0005)

        # add view behaviors
        self.viewBehaviors = viewbehaviors.ViewBehaviors(self.view)
        applogic._defaultRenderView = self.view

        self.mainWindow = QtGui.QMainWindow()
        self.mainWindow.setCentralWidget(self.view)
        self.mainWindow.resize(768 * (16/9.0), 768)
        self.mainWindow.setWindowTitle('Drake Visualizer')
        self.mainWindow.setWindowIcon(QtGui.QIcon(':/images/drake_logo.png'))

        self.settings = QtCore.QSettings()

        self.fileMenu = self.mainWindow.menuBar().addMenu('&File')
        self.viewMenu = self.mainWindow.menuBar().addMenu('&View')
        self.viewMenuManager = PythonQt.dd.ddViewMenu(self.viewMenu)

        self.drakeVisualizer = DrakeVisualizer(self.view)
        self.lcmglManager = lcmgl.LCMGLManager(self.view) if lcmgl.LCMGL_AVAILABLE else None

        model = om.getDefaultObjectModel()
        model.getTreeWidget().setWindowTitle('Scene Browser')
        model.getPropertiesPanel().setWindowTitle('Properties Panel')

        self.sceneBrowserDock = self.addWidgetToDock(model.getTreeWidget(), QtCore.Qt.LeftDockWidgetArea, visible=False)
        self.propertiesDock = self.addWidgetToDock(self.wrapScrollArea(model.getPropertiesPanel()), QtCore.Qt.LeftDockWidgetArea, visible=False)

        self.addViewMenuSeparator()

        self.screenGrabberPanel = ScreenGrabberPanel(self.view)
        self.screenGrabberDock = self.addWidgetToDock(self.screenGrabberPanel.widget, QtCore.Qt.RightDockWidgetArea, visible=False)

        self.cameraBookmarksPanel = camerabookmarks.CameraBookmarkWidget(self.view)
        self.cameraBookmarksDock = self.addWidgetToDock(self.cameraBookmarksPanel.widget, QtCore.Qt.RightDockWidgetArea, visible=False)

        self.cameraControlPanel = cameracontrolpanel.CameraControlPanel(self.view)
        self.cameraControlDock = self.addWidgetToDock(self.cameraControlPanel.widget, QtCore.Qt.RightDockWidgetArea, visible=False)

        act = self.fileMenu.addAction('&Quit')
        act.setShortcut(QtGui.QKeySequence('Ctrl+Q'))
        act.connect('triggered()', self.applicationInstance().quit)

        self.fileMenu.addSeparator()

        act = self.fileMenu.addAction('&Open Data...')
        act.setShortcut(QtGui.QKeySequence('Ctrl+O'))
        act.connect('triggered()', self._onOpenDataFile)

        applogic.addShortcut(self.mainWindow, 'F1', self._toggleObjectModel)
        applogic.addShortcut(self.mainWindow, 'F8', applogic.showPythonConsole)
        self.applicationInstance().connect('aboutToQuit()', self._onAboutToQuit)

        for obj in om.getObjects():
            obj.setProperty('Deletable', False)

        self.mainWindow.show()
        self._saveWindowState('MainWindowDefault')
        self._restoreWindowState('MainWindowCustom')

    def _onAboutToQuit(self):
        self._saveWindowState('MainWindowCustom')
        self.settings.sync()

    def _restoreWindowState(self, key):
        appsettings.restoreState(self.settings, self.mainWindow, key)

    def _saveWindowState(self, key):
        appsettings.saveState(self.settings, self.mainWindow, key)

    def _toggleObjectModel(self):
        self.sceneBrowserDock.setVisible(not self.sceneBrowserDock.visible)
        self.propertiesDock.setVisible(not self.propertiesDock.visible)

    def _toggleScreenGrabber(self):
        self.screenGrabberDock.setVisible(not self.screenGrabberDock.visible)

    def _toggleCameraBookmarks(self):
        self.cameraBookmarksDock.setVisible(not self.cameraBookmarksDock.visible)

    def _getOpenDataDirectory(self):
        return self.settings.value('OpenDataDir') or os.path.expanduser('~')

    def _storeOpenDataDirectory(self, filename):

        if os.path.isfile(filename):
            filename = os.path.dirname(filename)
        if os.path.isdir(filename):
            self.settings.setValue('OpenDataDir', filename)


    def _showErrorMessage(self, title, message):
        QtGui.QMessageBox.warning(self.mainWindow, title, message)


    def onOpenVrml(self, filename):
        meshes, color = ioUtils.readVrml(filename)
        folder = om.getOrCreateContainer(os.path.basename(filename), parentObj=om.getOrCreateContainer('files'))
        for i, pair in enumerate(zip(meshes, color)):
            mesh, color = pair
            obj = vis.showPolyData(mesh, 'mesh %d' % i, color=color, parent=folder)
            vis.addChildFrame(obj)

    def _openGeometry(self, filename):

        if filename.lower().endswith('wrl'):
            self.onOpenVrml(filename)
            return

        polyData = ioUtils.readPolyData(filename)

        if not polyData or not polyData.GetNumberOfPoints():
            self._showErrorMessage('Failed to read any data from file: %s' % filename, title='Reader error')
            return

        vis.showPolyData(polyData, os.path.basename(filename), parent='files')

    def _onOpenDataFile(self):
        fileFilters = "Data Files (*.obj *.ply *.stl *.vtk *.vtp *.wrl)";
        filename = QtGui.QFileDialog.getOpenFileName(self.mainWindow, "Open...", self._getOpenDataDirectory(), fileFilters)
        if not filename:
            return

        self._storeOpenDataDirectory(filename)
        self._openGeometry(filename)

    def wrapScrollArea(self, widget):
        w = QtGui.QScrollArea()
        w.setWidget(widget)
        w.setWidgetResizable(True)
        w.setWindowTitle(widget.windowTitle)
        #w.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        return w

    def addWidgetToViewMenu(self, widget):
        self.viewMenuManager.addWidget(widget, widget.windowTitle)

    def addViewMenuSeparator(self):
        self.viewMenuManager.addSeparator()

    def addWidgetToDock(self, widget, dockArea, visible=True):
        dock = QtGui.QDockWidget()
        dock.setWidget(widget)
        dock.setWindowTitle(widget.windowTitle)
        dock.setObjectName(widget.windowTitle + ' Dock')
        dock.setVisible(visible)
        self.mainWindow.addDockWidget(dockArea, dock)
        self.addWidgetToViewMenu(dock)
        return dock

    def quit(self):
        self.applicationInstance().quit()

    def applicationInstance(self):
        return QtCore.QCoreApplication.instance()

    def start(self, globalsDict=None):
        if globalsDict:
            globalsDict['app'] = self
            globalsDict['view'] = self.view
            globalsDict['quit'] = self.quit
            globalsDict['exit'] = self.quit

        return QtCore.QCoreApplication.instance().exec_()


def main(globalsDict=None):

    app = DrakeVisualizerApp()
    app.start(globalsDict)


if __name__ == '__main__':
    main(globals())
