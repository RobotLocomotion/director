import os
from ddapp import lcmUtils
from ddapp import jointcontrol
from ddapp import roboturdf

import ddapp.objectmodel as om
import ddapp.applogic as app
from ddapp import transformUtils
from ddapp.debugVis import DebugData
from ddapp import ioUtils
from ddapp import filterUtils
from ddapp import vtkAll as vtk
from ddapp import visualization as vis

import drake as lcmdrake

from PythonQt import QtGui


class Geometry(object):


    @staticmethod
    def createPolyData(geom):

        if geom.type == lcmdrake.lcmt_viewer_geometry_data.BOX:
            d = DebugData()
            d.addCube(dimensions=geom.float_data[0:3], center=[0,0,0])
            polyData = d.getPolyData()

        elif geom.type == lcmdrake.lcmt_viewer_geometry_data.SPHERE:
            d = DebugData()
            d.addSphere(center=(0,0,0), radius=geom.float_data[0])
            polyData = d.getPolyData()

        elif geom.type == lcmdrake.lcmt_viewer_geometry_data.CYLINDER:
            d = DebugData()
            d.addCylinder(center=(0,0,0), axis=(0,0,1), radius=geom.float_data[0], length=geom.float_data[1])
            polyData = d.getPolyData()

        elif geom.type == lcmdrake.lcmt_viewer_geometry_data.CAPSULE:
            d = DebugData()
            radius = geom.float_data[0]
            length = geom.float_data[1]
            d.addCylinder(center=(0,0,0), axis=(0,0,1), radius=radius, length=length)
            d.addSphere(center=(0,0,length/2.0), radius=radius)
            d.addSphere(center=(0,0,-length/2.0), radius=radius)
            polyData = d.getPolyData()

        elif geom.type == lcmdrake.lcmt_viewer_geometry_data.MESH:
            filename = geom.string_data
            basename, ext = os.path.splitext(filename)
            if ext.lower() == '.wrl':
                filename = basename + '.obj'

            scale = geom.float_data[0]
            polyData = ioUtils.readPolyData(filename)

            if scale != 1.0:
                t = vtk.vtkTransform()
                t.Scale(scale, scale, scale)
                polyData = filterUtils.transformPolyData(polyData, t)

        t = transformUtils.transformFromPose(geom.position, geom.quaternion)
        return filterUtils.transformPolyData(filterUtils.computeNormals(polyData), t)


    def __init__(self, geom, parentTransform):
        polyData = self.createPolyData(geom)
        self.polyDataItem = vis.PolyDataItem('geometry_data', polyData, view=None)
        self.polyDataItem.setProperty('Color', QtGui.QColor(geom.color[0]*255, geom.color[1]*255, geom.color[2]*255))
        self.polyDataItem.setProperty('Alpha', geom.color[3])
        self.polyDataItem.actor.SetUserTransform(parentTransform)


class Link(object):

    def __init__(self, link):
        self.transform = vtk.vtkTransform()
        self.geometry = [Geometry(g, self.transform) for g in link.geom]

    def setTransform(self, pos, quat):
        trans = transformUtils.transformFromPose(pos, quat)
        self.transform.SetMatrix(trans.GetMatrix())
        self.transform.Modified()


class DrakeVisualizer(object):

    def __init__(self, view):
        lcmUtils.addSubscriber('DRAKE_VIEWER_LOAD_ROBOT', lcmdrake.lcmt_viewer_load_robot, self.onViewerLoadRobot)
        lcmUtils.addSubscriber('DRAKE_VIEWER_DRAW', lcmdrake.lcmt_viewer_draw, self.onViewerDraw)

        self.view = view
        self.robots = {}

    def onViewerLoadRobot(self, msg):
        self.removeAllRobots()
        for link in msg.link:
            l = Link(link)
            self.addLink(l, link.robot_num, link.name)

        self.sendStatusMessage('successfully loaded robot')

    def getRootFolder(self):
        return om.getOrCreateContainer('drake viewer')

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
                geom.polyDataItem.actor.SetUseBounds(False)
                geom.polyDataItem.actor.GetProperty().SetSpecular(0.0)
                geom.polyDataItem.actor.GetProperty().LightingOff()
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


class DrakeVisualizerOld(object):

    def __init__(self, view):
        lcmUtils.addSubscriber('DRAKE_VIEWER_COMMAND', lcmdrake.lcmt_viewer_command, self.onViewerCommand)
        lcmUtils.addSubscriber('DRAKE_VIEWER_STATE', lcmdrake.lcmt_robot_state, self.onViewerState)

        self.view = view
        self.models = []
        self.jointControllers = []
        self.filenames = []

    def onViewerCommand(self, msg):
        if msg.command_type == lcmdrake.lcmt_viewer_command.LOAD_URDF:
            msg.command_type = msg.STATUS
            lcmUtils.publish('DRAKE_VIEWER_STATUS', msg)
            urdfFile = msg.command_data
            for model in self.models:
                if model.model.filename() == urdfFile:
                    return
            self.loadURDF(urdfFile)

    def addRobotModelItem(self, model):
        obj = roboturdf.RobotModelItem(model)
        om.addToObjectModel(obj, om.getOrCreateContainer('Drake Viewer Models'))
        obj.setProperty('Color', QtGui.QColor(255, 180, 0))
        obj.addToView(self.view)
        return obj

    def loadURDF(self, filename):
        model = roboturdf.loadRobotModelFromFile(filename)
        jointController = jointcontrol.JointController([model])
        jointController.setZeroPose()
        obj = self.addRobotModelItem(model)
        self.models.append(obj)
        self.jointControllers.append(jointController)

    def onViewerState(self, msg):

          if not self.models:
              return

          if self.models[0] not in om.getObjects():
              self.models[0] = self.addRobotModelItem(self.models[0].model)

          assert msg.num_robots == 1
          pose = msg.joint_position
          self.jointControllers[0].setPose('drake_viewer_pose', pose)
