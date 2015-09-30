import os
from ddapp import lcmUtils

import ddapp.objectmodel as om
import ddapp.applogic as app
from ddapp import transformUtils
from ddapp.debugVis import DebugData
from ddapp import ioUtils
from ddapp import filterUtils
from ddapp.shallowCopy import shallowCopy
from ddapp import vtkAll as vtk
from ddapp import visualization as vis

try:
    import lcmtypes.drake as lcmdrake
except ImportError as e:
    import warnings
    warnings.warn("The drake python lcmtype bindings now live in lcmtypes.drake. You may want to upgrade your version of drake", FutureWarning)
    import drake as lcmdrake

from PythonQt import QtGui

USE_TEXTURE_MESHES = True
USE_SHADOWS = False

class Geometry(object):

    TextureCache = {}
    MeshToTexture = {}

    @staticmethod
    def createPolyDataFromPrimitive(geom):

        if geom.type == lcmdrake.lcmt_viewer_geometry_data.BOX:
            d = DebugData()
            d.addCube(dimensions=geom.float_data[0:3], center=[0,0,0])
            return d.getPolyData()

        elif geom.type == lcmdrake.lcmt_viewer_geometry_data.SPHERE:
            d = DebugData()
            d.addSphere(center=(0,0,0), radius=geom.float_data[0])
            return d.getPolyData()

        elif geom.type == lcmdrake.lcmt_viewer_geometry_data.CYLINDER:
            d = DebugData()
            d.addCylinder(center=(0,0,0), axis=(0,0,1), radius=geom.float_data[0], length=geom.float_data[1])
            return d.getPolyData()

        elif geom.type == lcmdrake.lcmt_viewer_geometry_data.CAPSULE:
            d = DebugData()
            radius = geom.float_data[0]
            length = geom.float_data[1]
            d.addCylinder(center=(0,0,0), axis=(0,0,1), radius=radius, length=length)
            d.addSphere(center=(0,0,length/2.0), radius=radius)
            d.addSphere(center=(0,0,-length/2.0), radius=radius)
            return d.getPolyData()

        raise Exception('Unsupported geometry type: %s' % geom.type)


    @staticmethod
    def scaleGeometry(polyDataList, geom):
        scale = geom.float_data[0]
        if scale != 1.0:
            t = vtk.vtkTransform()
            t.Scale(scale, scale, scale)
            polyDataList = [filterUtils.transformPolyData(polyData, t) for polyData in polyDataList]

        return polyDataList


    @staticmethod
    def transformGeometry(polyDataList, geom):
        t = transformUtils.transformFromPose(geom.position, geom.quaternion)
        polyDataList = [filterUtils.transformPolyData(polyData, t) for polyData in polyDataList]
        return polyDataList


    @staticmethod
    def computeNormals(polyDataList):
        return [filterUtils.computeNormals(polyData) for polyData in polyDataList]

    @staticmethod
    def getTextureFileName(polyData):
        textureArray = vtk.vtkStringArray.SafeDownCast(polyData.GetFieldData().GetAbstractArray('texture_filename'))
        if not textureArray:
            return None
        return textureArray.GetValue(0)

    @staticmethod
    def loadTextureForMesh(polyData, meshFileName):


        textureFileName = Geometry.getTextureFileName(polyData)
        if textureFileName in Geometry.TextureCache:
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
    def loadMultiBlockMeshes(filename):

        reader = vtk.vtkXMLMultiBlockDataReader()
        reader.SetFileName(filename)
        reader.Update()

        polyDataList = []
        mb = reader.GetOutput()
        for i in xrange(mb.GetNumberOfBlocks()):
            polyData = vtk.vtkPolyData.SafeDownCast(mb.GetBlock(i))
            if polyData and polyData.GetNumberOfPoints():
                polyDataList.append(shallowCopy(polyData))
                Geometry.loadTextureForMesh(polyData, filename)

        return polyDataList


    @staticmethod
    def loadPolyDataMeshes(geom):

        filename = geom.string_data
        basename, ext = os.path.splitext(filename)
        if ext.lower() == '.wrl':
            filename = basename + '.obj'

        alternateFilename = basename + '.vtm'
        if USE_TEXTURE_MESHES and os.path.isfile(alternateFilename):
            polyDataList = Geometry.loadMultiBlockMeshes(alternateFilename)
        else:
            polyDataList = [ioUtils.readPolyData(filename)]

        return polyDataList


    @staticmethod
    def createPolyDataForGeometry(geom):

        polyDataList = []

        if geom.type != lcmdrake.lcmt_viewer_geometry_data.MESH:
            polyDataList = [Geometry.createPolyDataFromPrimitive(geom)]

        else:
            polyDataList = Geometry.loadPolyDataMeshes(geom)
            polyDataList = Geometry.scaleGeometry(polyDataList, geom)

        polyDataList = Geometry.transformGeometry(polyDataList, geom)
        polyDataList = Geometry.computeNormals(polyDataList)

        return polyDataList

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
        self.polyDataItem.setProperty('Alpha', geom.color[3])
        self.polyDataItem.actor.SetUserTransform(parentTransform)
        self.polyDataItem.actor.SetTexture(Geometry.TextureCache.get( Geometry.getTextureFileName(polyData) ))

        if self.polyDataItem.actor.GetTexture():
            self.polyDataItem.setProperty('Color', QtGui.QColor(255, 255, 255))

        else:
            self.polyDataItem.setProperty('Color', QtGui.QColor(geom.color[0]*255, geom.color[1]*255, geom.color[2]*255))

        if USE_SHADOWS:
            self.polyDataItem.shadowOn()


class Link(object):

    def __init__(self, link):
        self.transform = vtk.vtkTransform()

        self.geometry = []
        for g in link.geom:
            self.geometry.extend(Geometry.createGeometry(link.name + ' geometry data', g, self.transform))


    def setTransform(self, pos, quat):
        trans = transformUtils.transformFromPose(pos, quat)
        self.transform.SetMatrix(trans.GetMatrix())
        self.transform.Modified()


class DrakeVisualizer(object):

    def __init__(self, view):

        self.subscribers = []
        self.view = view
        self.robots = {}
        self.sendStatusMessage('loaded')
        self.enable()

    def _addSubscribers(self):
        self.subscribers.append(lcmUtils.addSubscriber('DRAKE_VIEWER_LOAD_ROBOT', lcmdrake.lcmt_viewer_load_robot, self.onViewerLoadRobot))
        self.subscribers.append(lcmUtils.addSubscriber('DRAKE_VIEWER_DRAW', lcmdrake.lcmt_viewer_draw, self.onViewerDraw))

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

    def onViewerLoadRobot(self, msg):
        self.removeAllRobots()
        for link in msg.link:
            l = Link(link)
            self.addLink(l, link.robot_num, link.name)

        self.sendStatusMessage('successfully loaded robot')

    def getRootFolder(self):
        return om.getOrCreateContainer('drake viewer', parentObj=om.findObjectByName('scene'))

    def getRobotFolder(self, robotNum):
        return om.getOrCreateContainer('robot %d' % robotNum, parentObj=self.getRootFolder())

    def getLinkFolder(self, robotNum, linkName):
        return om.getOrCreateContainer(linkName, parentObj=self.getRobotFolder(robotNum))

    def addLink(self, link, robotNum, linkName):
        self.robots.setdefault(robotNum, {})[linkName] = link
        linkFolder = self.getLinkFolder(robotNum, linkName)
        for geom in link.geometry:
            geom.polyDataItem.addToView(self.view)
            om.addToObjectModel(geom.polyDataItem, parentObj=linkFolder)

            if linkName == 'world':
                #geom.polyDataItem.actor.SetUseBounds(False)
                #geom.polyDataItem.actor.GetProperty().LightingOff()
                geom.polyDataItem.actor.GetProperty().SetSpecular(0.0)
            else:
                geom.polyDataItem.actor.GetProperty().SetSpecular(0.9)
                geom.polyDataItem.actor.GetProperty().SetSpecularPower(20)


    def getLink(self, robotNum, linkName):
        return self.robots[robotNum][linkName]

    def removeAllRobots(self):
        for child in self.getRootFolder().children():
            om.removeFromObjectModel(child)
        self.robots = {}

    def sendStatusMessage(self, message):
        msg = lcmdrake.lcmt_viewer_command()
        msg.command_type = lcmdrake.lcmt_viewer_command.STATUS
        msg.command_data = message
        lcmUtils.publish('DRAKE_VIEWER_STATUS', msg)

    def onViewerDraw(self, msg):

        for i in xrange(msg.num_links):

            pos = msg.position[i]
            quat = msg.quaternion[i]
            robotNum = msg.robot_num[i]
            linkName = msg.link_name[i]

            link = self.getLink(robotNum, linkName)
            link.setTransform(pos, quat)

        self.view.render()





##########################################################
from ddapp.screengrabberpanel import ScreenGrabberPanel
from ddapp import lcmgl
from ddapp import objectmodel as om
from ddapp import applogic
from ddapp import viewbehaviors
from ddapp import camerabookmarks
import PythonQt
from PythonQt import QtCore, QtGui


class DrakeVisualizerApp():

    def __init__(self):

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

        # add view behaviors
        viewBehaviors = viewbehaviors.ViewBehaviors(self.view)
        applogic._defaultRenderView = self.view

        self.mainWindow = QtGui.QMainWindow()
        self.mainWindow.setCentralWidget(self.view)
        self.mainWindow.resize(768 * (16/9.0), 768)
        self.mainWindow.setWindowTitle('Drake Visualizer')
        self.mainWindow.setWindowIcon(QtGui.QIcon(':/images/drake_logo.png'))
        self.mainWindow.show()

        self.drakeVisualizer = DrakeVisualizer(self.view)
        self.lcmglManager = lcmgl.LCMGLManager(self.view) if lcmgl.LCMGL_AVAILABLE else None

        self.screenGrabberPanel = ScreenGrabberPanel(self.view)
        self.screenGrabberDock = self.addWidgetToDock(self.screenGrabberPanel.widget, QtCore.Qt.RightDockWidgetArea)
        self.screenGrabberDock.setVisible(False)

        self.cameraBookmarksPanel = camerabookmarks.CameraBookmarkWidget(self.view)
        self.cameraBookmarksDock = self.addWidgetToDock(self.cameraBookmarksPanel.widget, QtCore.Qt.RightDockWidgetArea)
        self.cameraBookmarksDock.setVisible(False)

        model = om.getDefaultObjectModel()
        model.getTreeWidget().setWindowTitle('Scene Browser')
        model.getPropertiesPanel().setWindowTitle('Properties Panel')
        model.setActiveObject(self.viewOptions)

        self.sceneBrowserDock = self.addWidgetToDock(model.getTreeWidget(), QtCore.Qt.LeftDockWidgetArea)
        self.propertiesDock = self.addWidgetToDock(self.wrapScrollArea(model.getPropertiesPanel()), QtCore.Qt.LeftDockWidgetArea)
        self.sceneBrowserDock.setVisible(False)
        self.propertiesDock.setVisible(False)

        applogic.addShortcut(self.mainWindow, 'Ctrl+Q', self.applicationInstance().quit)
        applogic.addShortcut(self.mainWindow, 'F1', self._toggleObjectModel)
        applogic.addShortcut(self.mainWindow, 'F2', self._toggleScreenGrabber)
        applogic.addShortcut(self.mainWindow, 'F3', self._toggleCameraBookmarks)
        applogic.addShortcut(self.mainWindow, 'F8', applogic.showPythonConsole)

    def _toggleObjectModel(self):
        self.sceneBrowserDock.setVisible(not self.sceneBrowserDock.visible)
        self.propertiesDock.setVisible(not self.propertiesDock.visible)

    def _toggleScreenGrabber(self):
        self.screenGrabberDock.setVisible(not self.screenGrabberDock.visible)

    def _toggleCameraBookmarks(self):
        self.cameraBookmarksDock.setVisible(not self.cameraBookmarksDock.visible)

    def wrapScrollArea(self, widget):
        self.w = QtGui.QScrollArea()
        self.w.setWidget(widget)
        self.w.setWidgetResizable(True)
        #self.w.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        return self.w

    def addWidgetToDock(self, widget, dockArea):
        dock = QtGui.QDockWidget()
        dock.setWidget(widget)
        dock.setWindowTitle(widget.windowTitle)
        self.mainWindow.addDockWidget(dockArea, dock)
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
