import os
import PythonQt
from PythonQt import QtCore, QtGui
import ddapp.applogic as app
import ddapp.objectmodel as om
import ddapp.visualization as vis
import ddapp.vtkAll as vtk
from ddapp import jointcontrol
from ddapp import getDRCBaseDir
from ddapp import lcmUtils
from ddapp import filterUtils

import drc as lcmdrc
import math
import numpy as np



defaultUrdfHands = 'LR_RR'


def getRobotGrayColor():
    return QtGui.QColor(190, 190, 190)


def getRobotOrangeColor():
    return QtGui.QColor(255, 180, 0)


def loadRobotModel(name, view=None, parent='planning', urdfHands=None, color=None, visible=True):

    urdfHands = urdfHands or defaultUrdfHands
    urdfFile = os.path.join(getRobotModelDir(), 'model_%s.urdf' % urdfHands)
    folder = om.getOrCreateContainer(parent)

    model = loadRobotModelFromFile(urdfFile)
    obj = om.addRobotModel(model, folder)
    obj.setProperty('Visible', visible)
    obj.setProperty('Name', name)
    obj.setProperty('Color', color or getRobotGrayColor())
    if view is not None:
        obj.addToView(view)

    jointController = jointcontrol.JointController([obj])
    jointController.setNominalPose()

    return obj, jointController


def getRobotModelDir():
    return os.path.join(getDRCBaseDir(), 'software/models/mit_gazebo_models/mit_robot')


def loadRobotModelFromFile(filename):
    model = PythonQt.dd.ddDrakeModel()
    if not model.loadFromFile(filename):
        return None
    return model


def loadRobotModelFromString(xmlString):
    model = PythonQt.dd.ddDrakeModel()
    if not model.loadFromXML(xmlString):
        return None
    return model


def getExistingRobotModels():
    return [obj for obj in om.objects.values() if isinstance(obj, om.RobotModelItem)]


_modelPublisherString = None

def getModelPublisherString():
    return _modelPublisherString


def updateModelPublisherString(msg):
    global _modelPublisherString
    _modelPublisherString = msg.urdf_xml_string
    return _modelPublisherString


def onModelPublisherString(msg):

    lastStr = getModelPublisherString()
    if updateModelPublisherString(msg) == lastStr:
        return

    print 'reloading models with new model publisher string'

    if lastStr is not None:
        app.showInfoMessage('A model publisher string was received that differs from the previous string. '
                            'Models will be reloaded with the new string.',
                            title='Model publisher string changed')

    objs = getExistingRobotModels()
    for obj, jointController in _modelsToReload:
        print 'reloading model:', obj.getProperty('Name')
        newModel = loadRobotModelFromString(getModelPublisherString())
        obj.setModel(newModel)
        jointController.push()


def startModelPublisherListener(modelsToReload):
    global _modelsToReload
    _modelsToReload = modelsToReload
    lcmUtils.addSubscriber('ROBOT_MODEL', lcmdrc.robot_urdf_t, onModelPublisherString)



def setupPackagePaths():

    searchPaths = [
        'ros_workspace/mit_drcsim_scripts',
        'ros_workspace/sandia-hand/ros/sandia_hand_description',
        'software/models/mit_gazebo_models/mit_robot',
        'software/models/mit_gazebo_models/irobot_hand',
        'software/models/mit_gazebo_models/multisense_sl',
        'software/models/mit_gazebo_models/handle_description',
        'software/models/mit_gazebo_models/hook_description',
        'software/models/mit_gazebo_models/hook_description',
        'software/models/mit_gazebo_models/robotiq_hand_description',
                  ]

    for path in searchPaths:
        PythonQt.dd.ddDrakeModel.addPackageSearchPath(os.path.join(getDRCBaseDir(), path))


setupPackagePaths()






class HandLoader(object):

    def __init__(self, handType, robotModel, view):


        def toFrame(xyzrpy):
            rpy = [math.degrees(rad) for rad in xyzrpy[3:]]
            return transformUtils.frameFromPositionAndRPY(xyzrpy[:3], rpy)


        self.side, self.handType = handType.split('_')
        assert self.side in ('left', 'right')

        if self.handType == 'irobot':

            self.robotUrdf = 'model_LI_RI.urdf'
            self.handLinkName = '%s_hand' % self.side[0]
            self.handUrdf = 'irobot_hand_%s.urdf' % self.side
            self.handJointName = '%s_irobot_hand_joint' % self.side

            if self.side == 'left':
                baseToPalm = [0.0, -0.20516, -0.015, 0.0, 0.0, 0.0]
            else:
                baseToPalm = [0.0, -0.20516, 0.015, 0.0, 0.0, math.radians(180)]


            handRootLink = '%s_base_link' % self.side
            robotMountLink = '%s_hand' % self.side[0]
            palmLink = '%s_hand_face' % self.side[0]
            robotMountToPalm = toFrame(baseToPalm)

            self.loadHandModel(view)

            baseToHandRoot = self.getLinkToLinkTransform(self.handModel, 'plane::xy::base', handRootLink)
            robotMountToHandRoot = self.getLinkToLinkTransform(robotModel, robotMountLink, handRootLink)

            t = vtk.vtkTransform()
            t.PostMultiply()
            t.Concatenate(baseToHandRoot)
            t.Concatenate(robotMountToHandRoot.GetLinearInverse())
            t.Concatenate(robotMountToPalm)
            self.modelToPalm = t

            self.handLinkToPalm = robotMountToPalm
            self.palmToHandLink = self.handLinkToPalm.GetLinearInverse()



        elif self.handType == 'robotiq':

            self.robotUrdf = 'model_LR_RR.urdf'
            self.handLinkName = '%s_hand' % self.side[0]
            self.handUrdf = 'robotiq_hand_%s.urdf' % self.side
            self.handJointName = '%s_robotiq_hand_joint' % self.side
            self.baseToPalm = [0.0, -0.070, 0.0, 0.0, -math.radians(90), 0.0]

            handRootLink = '%s_palm' % self.side
            robotMountLink = '%s_hand_force_torque' % self.side[0]
            palmLink = '%s_hand_face' % self.side[0]
            robotMountToPalm = self.getLinkToLinkTransform(robotModel, robotMountLink, palmLink)

            self.loadHandModel(view)

            baseToHandRoot = self.getLinkToLinkTransform(self.handModel, 'plane::xy::base', handRootLink)
            robotMountToHandRoot = self.getLinkToLinkTransform(robotModel, robotMountLink, handRootLink)
            robotMountToHandLink = self.getLinkToLinkTransform(robotModel, robotMountLink, self.handLinkName)

            t = vtk.vtkTransform()
            t.PostMultiply()
            t.Concatenate(baseToHandRoot)
            t.Concatenate(robotMountToHandRoot.GetLinearInverse())
            t.Concatenate(robotMountToPalm)
            self.modelToPalm = t

            self.handLinkToPalm = self.getLinkToLinkTransform(robotModel, self.handLinkName, palmLink)
            self.palmToHandLink = self.handLinkToPalm.GetLinearInverse()


        elif self.handType == 'hook':

            self.robotUrdf = 'model_LP_RP.urdf'
            self.handLinkName = '%s_hand' % self.side[0]
            self.handUrdf = 'inert_hand_%s.urdf' % self.side
            self.handJointName = '%s_hook_hand_joint' % self.side

            if self.side == 'left':
                self.baseToPalm = [-0.040, -0.185, 0.0, -math.radians(90), math.radians(90), 0.0]
            else:
                self.baseToPalm = [-0.040, -0.185, 0.0, -math.radians(90), -math.radians(90), 0.0]

        else:
            raise Exception('Unexpected hand type: %s' % self.handType)


    def getHandUrdf(self):
        urdfBase = os.path.join(getDRCBaseDir(), 'software/models/mit_gazebo_models')
        return os.path.join(urdfBase, 'mit_robot_hands', self.handUrdf)

    def getRobotUrdf(self):
        urdfBase = os.path.join(getDRCBaseDir(), 'software/models/mit_gazebo_models')
        return os.path.join(urdfBase, 'mit_robot', self.robotUrdf)

    @staticmethod
    def getLinkToLinkTransform(model, linkA, linkB):
        linkAToWorld = model.getLinkFrame(linkA)
        linkBToWorld = model.getLinkFrame(linkB)
        assert linkAToWorld
        assert linkBToWorld
        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(linkAToWorld)
        t.Concatenate(linkBToWorld.GetLinearInverse())
        return t


    def moveToRobot(self, robotModel):

        handLinkToWorld = robotModel.getLinkFrame(self.handLinkName)

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(self.modelToPalm)
        t.Concatenate(self.palmToHandLink)
        t.Concatenate(handLinkToWorld)

        self.moveHandModelToFrame(self.handModel, t)

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(self.palmToHandLink)
        t.Concatenate(handLinkToWorld)
        vis.updateFrame(t, '%s palm' % self.side)


    def moveToGraspFrame(self, frame):

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(self.modelToPalm)
        t.Concatenate(frame)

        self.moveHandModelToFrame(self.handModel, t)


    def loadHandModel(self, view):

        color = [1.0, 1.0, 0.0]
        if self.side == 'right':
            color = [0.33, 1.0, 0.0]

        filename = self.getHandUrdf()
        handModel = loadRobotModelFromFile(filename)
        handModel = om.addRobotModel(handModel, om.getOrCreateContainer('hands'))
        handModel.setProperty('Name', os.path.basename(filename).replace('.urdf', '').replace('_', ' '))
        handModel.setProperty('Visible', False)
        color = np.array(color)*255
        handModel.setProperty('Color', QtGui.QColor(color[0], color[1], color[2]))
        handModel.setProperty('Alpha', 1.0)

        handModel.addToView(view)
        self.handModel = handModel

    def newPolyData(self):
        self.handModel.model.setJointPositions(np.zeros(self.handModel.model.numberOfJoints()))
        polyData = vtk.vtkPolyData()
        self.handModel.model.getModelMesh(polyData)
        polyData = filterUtils.transformPolyData(polyData, self.modelToPalm)

        name = '%s %s' % (self.handType, self.side)
        color = [1.0, 1.0, 0.0]
        if self.side == 'right':
            color = [0.33, 1.0, 0.0]
        obj = vis.showPolyData(polyData, name, color=color, visible=False, parent=om.getOrCreateContainer('hands'))
        frame = vtk.vtkTransform()
        frame.PostMultiply()
        obj.actor.SetUserTransform(frame)
        frameObj = vis.showFrame(frame, '%s frame' % name, scale=0.2, visible=False, parent=obj)
        return obj

    @staticmethod
    def moveHandModelToFrame(model, frame):
        pos, quat = transformUtils.poseFromTransform(frame)
        rpy = botpy.quat_to_roll_pitch_yaw(quat)
        pose = np.hstack((pos, rpy))
        model.model.setJointPositions(pose, ['base_x', 'base_y', 'base_z', 'base_roll', 'base_pitch', 'base_yaw'])
