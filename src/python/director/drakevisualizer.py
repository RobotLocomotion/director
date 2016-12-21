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

OK = 0
NO_SUCH_LINK = 1
ERROR_UNKNOWN_FORMAT = 2
ERROR_UNKNOWN_FORMAT_VERSION = 3
ERROR_HANDLING_REQUEST = 4



def transformFromDict(pose_data):
    return transformUtils.transformFromPose(pose_data["translation"],
                                            pose_data["quaternion"])


class Geometry(object):

    TextureCache = {}
    MeshToTexture = {}

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
        pass

    @staticmethod
    def createPolyData(geom):
        if geom["type"] == "box":
            return Geometry.createBox(geom["parameters"])
        elif geom["type"] == "sphere":
            return Geometry.createSphere(geom["parameters"])
        elif geom["type"] == "cylinder":
            return Geometry.createCylinder(geom["parameters"])
        elif geom["type"] == "capsule":
            return Geometry.createCapsule(geom["parameters"])
        elif geom["type"] == "ellipsoid":
            return Geometry.createEllipsoid(geom["parameters"])
        elif geom["type"] == "mesh_file":
            return Geometry.createMeshFromFile(geom["parameters"])
        elif geom["type"] == "mesh_data":
            return Geometry.createMeshFromData(geom["parameters"])
        elif geom["type"] == "pointcloud":
            return Geometry.createPointcloud(geom["parameters"])
        else:
            raise Exception(
                "Unsupported geometry type: {}".format(geom["type"]))

    # @staticmethod
    # def createPolyDataFromMeshMessage(geom):

    #     assert len(geom.float_data) >= 2

    #     numPoints = int(geom.float_data[0])
    #     numTris = int(geom.float_data[1])

    #     headerOffset = 2
    #     ptsOffset = 3*numPoints
    #     trisOffset = 3*numTris

    #     assert len(geom.float_data) == headerOffset + ptsOffset + trisOffset

    #     pts = np.array(geom.float_data[headerOffset:headerOffset+ptsOffset])
    #     pts = pts.reshape((numPoints, 3))
    #     tris = np.array(geom.float_data[headerOffset+ptsOffset:], dtype=int)

    #     return Geometry.createPolyDataFromMeshArrays(pts, tris)

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

    # @staticmethod
    # def createPolyDataFromMeshArrays(pts, faces):
    #     pd = vtk.vtkPolyData()
    #     pd.SetPoints(vtk.vtkPoints())
    #     pd.GetPoints().SetData(vnp.getVtkFromNumpy(pts.copy()))

    #     assert len(faces) % 3 == 0
    #     cells = vtk.vtkCellArray()
    #     for i in xrange(len(faces)/3):
    #         tri = vtk.vtkTriangle()
    #         tri.GetPointIds().SetId(0, faces[i*3 + 0])
    #         tri.GetPointIds().SetId(1, faces[i*3 + 1])
    #         tri.GetPointIds().SetId(2, faces[i*3 + 2])
    #         cells.InsertNextCell(tri)

    #     pd.SetPolys(cells)
    #     return pd

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
    # def scaleGeometry(polyDataList, geom):
    #     if len(geom.float_data) == 1:
    #         scale_x = scale_y = scale_z = geom.float_data[0]
    #     elif len(geom.float_data) == 3:
    #         scale_x = geom.float_data[0]
    #         scale_y = geom.float_data[1]
    #         scale_z = geom.float_data[2]
    #     else:
    #         scale_x = scale_y = scale_z = 1.0

    #     if scale_x != 1.0 or scale_y != 1.0 or scale_z != 1.0:
    #         t = vtk.vtkTransform()
    #         t.Scale(scale_x, scale_y, scale_z)
    #         polyDataList = [filterUtils.transformPolyData(polyData, t) for polyData in polyDataList]

    #     return polyDataList

    # @staticmethod
    # def transformGeometry(polyDataList, geom):
    #     t = transformUtils.transformFromPose(geom.position, geom.quaternion)
    #     polyDataList = [filterUtils.transformPolyData(polyData, t) for polyData in polyDataList]
    #     return polyDataList

    @staticmethod
    def transformGeometry(polyDataList, transform):
        polyDataList = [filterUtils.transformPolyData(polyData, transform) for polyData in polyDataList]
        return polyDataList

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
        polyDataList = Geometry.transformGeometry(
            polyDataList,
            transformFromDict(geom["pose"]))
        polyDataList = Geometry.computeNormals(polyDataList)
        return polyDataList

    # @staticmethod
    # def createPolyDataForGeometry(geom):
    #     if geom.type != lcmrl.viewer_geometry_data_t.MESH:
    #         polyDataList = [Geometry.createPolyDataFromPrimitive(geom)]

    #     else:
    #         if not geom.string_data:
    #             polyDataList = [Geometry.createPolyDataFromMeshMessage(geom)]
    #         else:
    #             polyDataList = Geometry.loadPolyDataMeshes(geom)
    #             polyDataList = Geometry.scaleGeometry(polyDataList, geom)

    #     polyDataList = Geometry.transformGeometry(polyDataList, geom)
    #     polyDataList = Geometry.computeNormals(polyDataList)

    #     return polyDataList

    @staticmethod
    def createGeometry(name, geom, parentTransform):
        polyDataList = Geometry.createPolyDataForGeometry(geom)

        geometry = []
        for polyData in polyDataList:
            g = Geometry(name, geom, polyData, parentTransform)
            geometry.append(g)
        return geometry


    def __init__(self, name, geom, polyData, parentTransform):
        self.polyDataItem = vis.PolyDataItem(name, polyData, view=None)
        self.polyDataItem.setProperty('Alpha', geom["color"][3])
        self.polyDataItem.actor.SetTexture(Geometry.TextureCache.get( Geometry.getTextureFileName(polyData) ))

        if self.polyDataItem.actor.GetTexture():
            self.polyDataItem.setProperty('Color', QtGui.QColor(255, 255, 255))

        else:
            self.polyDataItem.setProperty('Color', QtGui.QColor(255 * geom["color"][0],
                                                                255 * geom["color"][1],
                                                                255 * geom["color"][2]))

        if USE_SHADOWS:
            self.polyDataItem.shadowOn()


class Link(object):

    def __init__(self, path):
        self.transform = vtk.vtkTransform()
        self.path = path
        self.geometry = []

    def addGeometry(self, geom):
        vtkGeometries = Geometry.createGeometry(
            geom["name"],
            geom,
            self.transform)
        self.geometry.extend(vtkGeometries)
        return vtkGeometries

    def setTransform(self, transform):
        self.transform = transform
        for g in self.geometry:
            childFrame = g.polyDataItem.getChildFrame()
            if childFrame:
                childFrame.copyFrame(self.transform)
            else:
                g.polyDataItem.actor.SetUserTransform(self.transform)


class UnknownFormatException(Exception):
    pass


class BadFormatVersionException(Exception):
    pass


ViewerResponse = namedtuple("ViewerResponse", ["status", "data"])


class DrakeVisualizer(object):

    def __init__(self, view):

        self.subscribers = []
        self.view = view
        self.robots = {}
        # self.linkWarnings = set()
        self.enable()
        # self.sendStatusMessage(lcmrl.viewer2_response_t.STATUS_OK, 'loaded')

    def _addSubscribers(self):
        self.subscribers.append(lcmUtils.addSubscriber(
            'DRAKE_VIEWER2_REQUEST',
            lcmrl.viewer2_comms_t,
            self.onViewerRequest))

        # self.subscribers.append(lcmUtils.addSubscriber('DRAKE_VIEWER_LOAD_ROBOT', lcmrl.viewer_load_robot_t, self.onViewerLoadRobot))
        # self.subscribers.append(lcmUtils.addSubscriber('DRAKE_VIEWER_ADD_ROBOT', lcmrl.viewer_load_robot_t, self.onViewerAddRobot))
        # self.subscribers.append(lcmUtils.addSubscriber('DRAKE_VIEWER_DRAW', lcmrl.viewer_draw_t, self.onViewerDraw))
        # self.subscribers.append(lcmUtils.addSubscriber('DRAKE_PLANAR_LIDAR_.*', lcmbot.planar_lidar_t, self.onPlanarLidar, callbackNeedsChannel=True))
        # self.subscribers.append(lcmUtils.addSubscriber('DRAKE_POINTCLOUD_.*', lcmbot.pointcloud_t, self.onPointCloud, callbackNeedsChannel=True))

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

    def sendStatusMessage(self, timestamp, status, data):
        msg = lcmrl.viewer2_comms_t()
        msg.format = "viewer2_json"
        msg.format_version_major = 1
        msg.format_version_minor = 0
        msg.data = json.dumps({
            "timestamp": timestamp,
            "status": status,
            "data": data
            })
        msg.num_bytes = len(msg.data)
        lcmUtils.publish('DRAKE_VIEWER2_RESPONSE', msg)

    def decodeCommsMsg(self, msg):
        if msg.format == "viewer2_json":
            if msg.format_version_major == 1 and msg.format_version_minor == 0:
                data = json.loads(msg.data)
                return data, ViewerResponse(OK, {})
            else:
                return None, ViewerResponse(ERROR_UNKNOWN_FORMAT_VERSION,
                                            {"supported_formats": {
                                                 "viewer2_json": [{"major": 1,
                                                                   "minor": 0}]
                                            }})
        else:
            return None, ViewerResponse(ERROR_UNKNOWN_FORMAT,
                                        {"supported_formats": {
                                             "viewer2_json": [{"major": 1,
                                                               "minor": 0}]
                                        }})

    def onViewerRequest(self, msg):
        data, response = self.decodeCommsMsg(msg)
        if data is not None:
            response = self.handleViewerRequest(data)
        self.sendStatusMessage(msg.timestamp,
                               response.status,
                               response.data)

    def handleViewerRequest(self, data):
        if data["type"] == "load":
            return self.loadLinks(data["data"])
        elif data["type"] == "draw":
            return self.drawLinks(data["data"])

    def loadLinks(self, data):
        try:
            for linkData in data["links"]:
                self.loadLinkData(linkData)
        except Exception as e:
            print e
            return ViewerResponse(ERROR_HANDLING_REQUEST, {"error": str(e)})
        else:
            return ViewerResponse(OK, {})

    def loadLinkData(self, linkData):
        linkFolder = self.getPathFolder(linkData["path"])
        try:
            link = self.robots[tuple(linkData["path"])]
        except KeyError:
            link = Link(linkData["path"])
            self.robots[tuple(linkData["path"])] = link
        for geometry in linkData["geometries"]:
            vtkGeoms = link.addGeometry(geometry)
            for vtkGeom in vtkGeoms:
                vtkGeom.polyDataItem.addToView(self.view)
                om.addToObjectModel(vtkGeom.polyDataItem, parentObj=linkFolder)

    def getRootFolder(self):
        return om.getOrCreateContainer('drake viewer', parentObj=om.findObjectByName('scene'))

    def getPathFolder(self, path):
        folder = self.getRootFolder()
        for element in path:
            folder = om.getOrCreateContainer(element, parentObj=folder)
        return folder

    # def addLinkGeometry(self, geom, linkName, linkFolder):
    #     geom.polyDataItem.addToView(self.view)
    #     om.addToObjectModel(geom.polyDataItem, parentObj=linkFolder)

    #     if linkName == 'world':
    #         #geom.polyDataItem.actor.SetUseBounds(False)
    #         #geom.polyDataItem.actor.GetProperty().LightingOff()
    #         geom.polyDataItem.actor.GetProperty().SetSpecular(0.0)
    #     else:
    #         geom.polyDataItem.actor.GetProperty().SetSpecular(0.9)
    #         geom.polyDataItem.actor.GetProperty().SetSpecularPower(20)

    # def getLink(self, robotNum, linkName):
    #     return self.robots[robotNum][linkName]

    # def removeAllRobots(self):
    #     for child in self.getRootFolder().children():
    #         if child.getProperty('Name') != "pointclouds":
    #             om.removeFromObjectModel(child)
    #     self.robots = {}

    # def removeRobot(self, robotNum):
    #     if robotNum in self.robots:
    #         om.removeFromObjectModel(self.getRobotFolder(robotNum))
    #         del self.robots[robotNum]

    # def sendStatusMessage(self, status, data):
    #     msg = lcmrl.viewer2_response_t()
    #     msg.status = status
    #     msg.json = json.dumps(data)
    #     lcmUtils.publish('DRAKE_VIEWER2_RESPONSE', msg)

    def drawLinks(self, data):
        try:
            missing_paths = []
            for command in data["commands"]:
                path = command["path"]
                transform = transformFromDict(command["pose"])
                try:
                    link = self.robots[tuple(path)]
                except KeyError:
                    missing_paths.append(path)
                else:
                    link.setTransform(transform)

            self.view.render()

            if missing_paths:
                return ViewerResponse(NO_SUCH_LINK,
                                      {"missing_paths": missing_paths})
            else:
                return ViewerResponse(OK, {})
        except Exception as e:
            print e
            return ViewerResponse(ERROR_HANDLING_REQUEST,
                                  {"error": str(e)})

    def onPlanarLidar(self, msg, channel):

        linkName = channel.replace('DRAKE_PLANAR_LIDAR_', '', 1)
        robotNum, linkName = linkName.split('_', 1)
        robotNum = int(robotNum)

        try:
            link = self.getLink(robotNum, linkName)
        except KeyError:
            if linkName not in self.linkWarnings:
                print 'Error locating link name:', linkName
                self.linkWarnings.add(linkName)
        else:
            if len(link.geometry):
                polyData = link.geometry[0].polyDataItem
            else:
                polyData = vtk.vtkPolyData()

                name = linkName + ' geometry data'
                geom = type('', (), {})
                geom.color = [1.0,0.0,0.0,1.0]
                g = Geometry(name, geom, polyData, link.transform)
                link.geometry.append(g)

                linkFolder = self.getLinkFolder(robotNum, linkName)
                self.addLinkGeometry(g, linkName, linkFolder)
                g.polyDataItem.actor.SetUserTransform(link.transform)

            points = vtk.vtkPoints()
            verts = vtk.vtkCellArray()

            t = msg.rad0
            for r in msg.ranges:
                if r >= 0:
                    x = r * math.cos(t)
                    y = r * math.sin(t)

                    pointId = points.InsertNextPoint([x,y,0])
                    verts.InsertNextCell(1)
                    verts.InsertCellPoint(pointId)

                t += msg.radstep

            polyData.SetPoints(points)
            polyData.SetVerts(verts)


    def onPointCloud(self, msg, channel):
        pointcloudName = channel.replace('DRAKE_POINTCLOUD_', '', 1)

        polyData = vnp.numpyToPolyData(np.asarray(msg.points), createVertexCells=True)

        # If the user provided color channels, then use them to colorize
        # the pointcloud.
        channels = {msg.channel_names[i]: msg.channels[i] for i in range(msg.n_channels)}
        if "r" in channels and "g" in channels and "b" in channels:
            colorized = True
            colorArray = np.empty((msg.n_points, 3), dtype=np.uint8)
            for (colorIndex, color) in enumerate(["r", "g", "b"]):
                colorArray[:, colorIndex] = 255 * np.asarray(channels[color])
            vnp.addNumpyToVtk(polyData, colorArray, "rgb")
        else:
            colorized = False

        folder = self.getPointCloudFolder()

        # If there was an existing point cloud by this name, then just
        # set its polyData to the new point cloud.
        # This has the effect of preserving all the user-specified properties
        # like point size, coloration mode, alpha, etc.
        previousPointcloud = folder.findChild(pointcloudName)
        if previousPointcloud is not None:
            previousPointcloud.setPolyData(polyData)
            previousPointcloud._updateColorByProperty()
        else:
            item = vis.PolyDataItem(pointcloudName, polyData, view=None)
            item.addToView(self.view)
            if colorized:
                item._updateColorByProperty()
                item.setProperty("Color By", "rgb")
            om.addToObjectModel(item, parentObj=folder)


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
