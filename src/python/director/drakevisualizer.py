import os
from director import lcmUtils

import director.objectmodel as om
import director.applogic as app
from director import transformUtils
from director.debugVis import DebugData
from director import ioUtils
from director import filterUtils
from director.timercallback import TimerCallback
from director import vtkAll as vtk
from director import visualization as vis

try:
    import drake as lcmdrake
except ImportError as e:
    import warnings
    warnings.warn("The drake python lcmtype bindings have been moved back to the `drake` package. You may want to upgrade your version of drake", FutureWarning)
    import lcmtypes.drake as lcmdrake

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
        if len(geom.float_data) == 1:
            scale_x = scale_y = scale_z = geom.float_data[0]
        elif len(geom.float_data) == 3:
            scale_x = geom.float_data[0]
            scale_y = geom.float_data[1]
            scale_z = geom.float_data[2]
        else:
            scale_x = scale_y = scale_z = 1.0

        if scale_x != 1.0 or scale_y != 1.0 or scale_z != 1.0:
            t = vtk.vtkTransform()
            t.Scale(scale_x, scale_y, scale_z)
            polyDataList = [filterUtils.transformPolyData(polyData, t) for polyData in polyDataList]

        return polyDataList


    @staticmethod
    def transformGeometry(polyDataList, geom):
        t = transformUtils.transformFromPose(geom.position, geom.quaternion)
        polyDataList = [filterUtils.transformPolyData(polyData, t) for polyData in polyDataList]
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
    def loadPolyDataMeshes(geom):

        filename = geom.string_data
        basename, ext = os.path.splitext(filename)

        preferredExtensions = ['.vtm', '.vtp', '.obj']

        for x in preferredExtensions:
            if os.path.isfile(basename + x):
                filename = basename + x
                break

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


class LidarSource(TimerCallback):

    def __init__(self, view):
        super(LidarSource, self).__init__(targetFps=30)

        self.lidars = {}
        self.view = view

        lcm = lcmUtils.getGlobalLCMThread()
        self.lidarAggregator = PythonQt.dd.ddLidarPointCloudLCM(lcm)

        self.lidarAggregator.channelAdded.connect(self.addChannel)

        self.channels = set(self.lidarAggregator.channels())
        self.lidarAggregator.init(lcm)

        self.callback = self._updateSource

    def addChannel(self, channel):
        self.channels.add(channel)

    def _updateSource(self):
        for channel in self.channels:
            pd = vtk.vtkPolyData()
            self.lidarAggregator.extractPolyData(channel, pd)

            if not channel in self.lidars:
                name = '%s lidar source' % channel
                pdObj = vis.PolyDataItem(name, pd, self.view)
                pdObj.setProperty('Visible', True)
                pdObj.initialized = True
                self.lidars[channel] = pdObj
            else:
                self.lidars[channel].setPolyData(pd)

        self.view.render()


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

        self.lidarSource = LidarSource(view)

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
            self.lidarSource.start()
        elif not enabled and self.isEnabled():
            self._removeSubscribers()
            self.lidarSource.stop()

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
from director.screengrabberpanel import ScreenGrabberPanel
from director import lcmgl
from director import objectmodel as om
from director import applogic
from director import viewbehaviors
from director import camerabookmarks
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

        # This setting improves the near plane clipping resolution.
        # Drake often draws a very large ground plane which is detrimental to
        # the near clipping for up close objects.  The trade-off is Z buffer
        # resolution but in practice things look good with this setting.
        self.view.renderer().SetNearClippingPlaneTolerance(0.0005)

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
