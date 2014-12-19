import os
import PythonQt
from PythonQt import QtCore, QtGui
from ddapp import callbacks
import ddapp.applogic as app
import ddapp.objectmodel as om
import ddapp.visualization as vis
import ddapp.vtkAll as vtk
from ddapp import botpy
from ddapp import jointcontrol
from ddapp import getDRCBaseDir
from ddapp import lcmUtils
from ddapp import filterUtils
from ddapp import transformUtils
from ddapp import drcargs

import drc as lcmdrc
import math
import numpy as np
import json

with open(drcargs.args().urdf_config) as urdf_config_file:
    jsonConfig = json.load(urdf_config_file)
    urdf_config_path = os.path.dirname(os.path.abspath(urdf_config_file.name))
    fixedPointFile = os.path.join(urdf_config_path, jsonConfig['fixedPointFile'])
    urdf_config = jsonConfig['urdfConfig']
    for key, urdf in list(urdf_config.items()):
        urdf_config[key] = os.path.join(urdf_config_path, urdf)



defaultUrdfHands = 'LR_RR'


def getRobotGrayColor():
    return QtGui.QColor(177, 180, 190)


def getRobotOrangeColor():
    return QtGui.QColor(255, 190, 0)


class RobotModelItem(om.ObjectModelItem):

    MODEL_CHANGED_SIGNAL = 'MODEL_CHANGED_SIGNAL'

    def __init__(self, model):

        modelName = os.path.basename(model.filename())
        om.ObjectModelItem.__init__(self, modelName, om.Icons.Robot)

        self.model = model
        model.connect('modelChanged()', self.onModelChanged)
        model.connect('displayChanged()', self.onDisplayChanged)
        self.callbacks = callbacks.CallbackRegistry([self.MODEL_CHANGED_SIGNAL])
        self.useUrdfColors = False

        self.addProperty('Filename', model.filename())
        self.addProperty('Visible', model.visible())
        self.addProperty('Alpha', model.alpha(),
                         attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=1.0, singleStep=0.1, hidden=False))
        self.addProperty('Textures', False)
        self.addProperty('Color', model.color())
        self.views = []

    def _onPropertyChanged(self, propertySet, propertyName):
        om.ObjectModelItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Alpha':
            self.model.setAlpha(self.getProperty(propertyName))
        elif propertyName == 'Visible':
            self.model.setVisible(self.getProperty(propertyName))
        elif propertyName == 'Textures':
            self.model.setTexturesEnabled(self.getProperty(propertyName))
            self._updateModelColor()
        elif propertyName == 'Color':
            self._updateModelColor()

        self._renderAllViews()

    def hasDataSet(self, dataSet):
        return len(self.model.getLinkNameForMesh(dataSet)) != 0

    def connectModelChanged(self, func):
        return self.callbacks.connect(self.MODEL_CHANGED_SIGNAL, func)

    def disconnectModelChanged(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def onModelChanged(self):
        self.callbacks.process(self.MODEL_CHANGED_SIGNAL, self)
        if self.getProperty('Visible'):
            self._renderAllViews()

    def onDisplayChanged(self):
        if self.getProperty('Visible'):
            self._renderAllViews()

    def _renderAllViews(self):
        for view in self.views:
            view.render()

    def getLinkFrame(self, linkName):
        t = vtk.vtkTransform()
        t.PostMultiply()
        if self.model.getLinkToWorld(linkName, t):
            return t
        else:
            return None

    def setModel(self, model):
        assert model is not None
        if model == self.model:
            return

        views = list(self.views)
        self.removeFromAllViews()
        self.model = model
        self.model.setAlpha(self.getProperty('Alpha'))
        self.model.setVisible(self.getProperty('Visible'))
        self.model.setTexturesEnabled(self.getProperty('Textures'))
        self._updateModelColor()

        self.setProperty('Filename', model.filename())
        model.connect('modelChanged()', self.onModelChanged)

        for view in views:
            self.addToView(view)
        self.onModelChanged()

    def _updateModelColor(self):
        if self.getProperty('Textures'):
            self._setupTextureColors()
        elif not self.useUrdfColors:
            self.model.setColor(self.getProperty('Color'))

    def _setupTextureColors(self):

        # custom colors for non-textured robotiq hand
        for name in self.model.getLinkNames():
            strs = name.split('_')
            if len(strs) >= 2 and strs[0] in ['left', 'right'] and strs[1] in ('finger', 'palm') or name.endswith('hand_force_torque'):
                self.model.setLinkColor(name, QtGui.QColor(90, 90, 90) if strs[1] == 'finger' else QtGui.QColor(20,20,20))
            else:
                self.model.setLinkColor(name, QtGui.QColor(255,255,255))

    def addToView(self, view):
        if view in self.views:
            return
        self.views.append(view)
        self.model.addToRenderer(view.renderer())
        view.render()

    def onRemoveFromObjectModel(self):
        self.removeFromAllViews()

    def removeFromAllViews(self):
        for view in list(self.views):
            self.removeFromView(view)
        assert len(self.views) == 0

    def removeFromView(self, view):
        assert view in self.views
        self.views.remove(view)
        self.model.removeFromRenderer(view.renderer())
        view.render()


def loadRobotModel(name, view=None, parent='planning', urdfFile=None, color=None, visible=True):

    if not urdfFile:
        #urdfFile = os.path.join(getRobotModelDir(), 'model_%s.urdf' % defaultUrdfHands)
        urdfFile = urdf_config['default']

    if isinstance(parent, str):
        parent = om.getOrCreateContainer(parent)

    model = loadRobotModelFromFile(urdfFile)
    obj = RobotModelItem(model)
    om.addToObjectModel(obj, parent)

    obj.setProperty('Visible', visible)
    obj.setProperty('Name', name)
    obj.setProperty('Color', color or getRobotGrayColor())
    if view is not None:
        obj.addToView(view)

    jointController = jointcontrol.JointController([obj], fixedPointFile)
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
    return [obj for obj in om.getObjects() if isinstance(obj, RobotModelItem)]


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
        'software/models/atlas_v4',
        'software/models/common_components/robotiq_hand_description/',
        'software/models/common_components/multisense_sl/',
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




class HandFactory(object):

    def __init__(self, robotModel, defaultLeftHandType=None, defaultRightHandType=None):

        handTypesMap = {
          'LR' : 'left_robotiq',
          'LP' : 'left_pointer',
          'LI' : 'left_irobot',
          'RR' : 'right_robotiq',
          'RP' : 'right_pointer',
          'RI' : 'right_irobot',
          }

        if not defaultLeftHandType:
            defaultLeftHandType = handTypesMap[defaultUrdfHands.split('_')[0]]
        if not defaultRightHandType:
            defaultRightHandType = handTypesMap[defaultUrdfHands.split('_')[1]]

        self.robotModel = robotModel
        self.loaders = {}
        self.defaultHandTypes = {
            'left' : defaultLeftHandType,
            'right' : defaultRightHandType
            }


    def getLoader(self, side):

        assert side in self.defaultHandTypes.keys()
        handType = self.defaultHandTypes[side]
        loader = self.loaders.get(handType)
        if loader is None:
            loader = HandLoader(handType, self.robotModel)
            self.loaders[handType] = loader
        return loader

    def newPolyData(self, side, view, name=None, parent=None):
        loader = self.getLoader(side)
        name = name or self.defaultHandTypes[side].replace('_', ' ')
        return loader.newPolyData(name, view, parent=parent)

    def placeHandModelWithTransform(self, transform, view, side, name=None, parent=None):
        handObj = self.newPolyData(side, view, name=name, parent=parent)
        handObj.setProperty('Visible', True)
        handFrame = handObj.children()[0]
        handFrame.copyFrame(transform)
        return handObj, handFrame


class HandLoader(object):

    def __init__(self, handType, robotModel):


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

            self.loadHandModel()

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

            handRootLink = '%s_palm' % self.side
            robotMountLink = '%s_hand_force_torque' % self.side[0]
            palmLink = '%s_hand_face' % self.side[0]


        elif self.handType == 'pointer':

            self.robotUrdf = 'model_LP_RP.urdf'
            self.handLinkName = '%s_hand' % self.side[0]
            self.handUrdf = 'pointer_hand_%s.urdf' % self.side
            self.handJointName = '%s_hook_hand_joint' % self.side

            handRootLink = '%s_base_link' % self.side
            robotMountLink = '%s_hand_force_torque' % self.side[0]
            palmLink = '%s_pointer_tip' % self.side

        else:
            raise Exception('Unexpected hand type: %s' % self.handType)

        self.loadHandModel()

        baseToHandRoot = self.getLinkToLinkTransform(self.handModel, 'plane::xy::base', handRootLink)
        robotMountToHandRoot = self.getLinkToLinkTransform(robotModel, robotMountLink, handRootLink)
        robotMountToHandLink = self.getLinkToLinkTransform(robotModel, robotMountLink, self.handLinkName)
        robotMountToPalm = self.getLinkToLinkTransform(robotModel, robotMountLink, palmLink)

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(baseToHandRoot)
        t.Concatenate(robotMountToHandRoot.GetLinearInverse())
        t.Concatenate(robotMountToPalm)
        self.modelToPalm = t

        self.handLinkToPalm = self.getLinkToLinkTransform(robotModel, self.handLinkName, palmLink)
        self.palmToHandLink = self.handLinkToPalm.GetLinearInverse()

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


    def loadHandModel(self):

        filename = self.getHandUrdf()
        handModel = loadRobotModelFromFile(filename)
        handModel = RobotModelItem(handModel)
        self.handModel = handModel

        '''
        color = [1.0, 1.0, 0.0]
        if self.side == 'right':
            color = [0.33, 1.0, 0.0]

        handModel = RobotModelItem(handModel)
        om.addToObjectModel(handModel, om.getOrCreateContainer('hands'))
        handModel.setProperty('Name', os.path.basename(filename).replace('.urdf', '').replace('_', ' '))
        handModel.setProperty('Visible', False)
        color = np.array(color)*255
        handModel.setProperty('Color', QtGui.QColor(color[0], color[1], color[2]))
        handModel.setProperty('Alpha', 1.0)
        #handModel.addToView(view)
        '''


    def newPolyData(self, name, view, parent=None):
        self.handModel.model.setJointPositions(np.zeros(self.handModel.model.numberOfJoints()))
        polyData = vtk.vtkPolyData()
        self.handModel.model.getModelMesh(polyData)
        polyData = filterUtils.transformPolyData(polyData, self.modelToPalm)

        if isinstance(parent, str):
            parent = om.getOrCreateContainer(parent)

        color = [1.0, 1.0, 0.0]
        if self.side == 'right':
            color = [0.33, 1.0, 0.0]
        obj = vis.showPolyData(polyData, name, view=view, color=color, visible=False, parent=parent)
        obj.side = self.side
        frame = vtk.vtkTransform()
        frame.PostMultiply()
        obj.actor.SetUserTransform(frame)
        frameObj = vis.showFrame(frame, '%s frame' % name, view=view, scale=0.2, visible=False, parent=obj)
        return obj


    def getPalmToWorldTransform(self, robotModel):

        handLinkToWorld = robotModel.getLinkFrame(self.handLinkName)

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(self.palmToHandLink)
        t.Concatenate(handLinkToWorld)

        return t


    def moveToRobot(self, robotModel):

        handLinkToWorld = robotModel.getLinkFrame(self.handLinkName)

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(self.modelToPalm)
        t.Concatenate(self.palmToHandLink)
        t.Concatenate(handLinkToWorld)

        self.moveHandModelToFrame(self.handModel, t)
        vis.updateFrame(self.getPalmToWorldTransform(), '%s palm' % self.side)


    def moveToGraspFrame(self, frame):

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(self.modelToPalm)
        t.Concatenate(frame)

        self.moveHandModelToFrame(self.handModel, t)


    @staticmethod
    def moveHandModelToFrame(model, frame):
        pos, quat = transformUtils.poseFromTransform(frame)
        rpy = botpy.quat_to_roll_pitch_yaw(quat)
        pose = np.hstack((pos, rpy))
        model.model.setJointPositions(pose, ['base_x', 'base_y', 'base_z', 'base_roll', 'base_pitch', 'base_yaw'])


def setRobotiqJointsToOpenHand(robotModel):
    for side in ['left', 'right']:
        setRobotiqJoints(robotModel, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

def setRobotiqJointsToClosedHand(robotModel):
    for side in ['left', 'right']:
        setRobotiqJoints(robotModel, [1.0, 1.0, 1.0], [0.0, 0.0, 0.0])

def setRobotiqJoints(robotModel, side, fingers=[0.0, 0.0, 0.0], palm=[0.0, 0.0, 0.0]):
    robotModel.model.setJointPositions(np.tile(fingers, 3), ['%s_finger_%s_joint_%d' % (side, n, i+1) for n in ['1', '2', 'middle'] for i in range(3)])
    robotModel.model.setJointPositions(palm, ['%s_palm_finger_%s_joint' % (side, n) for n in ['1', '2', 'middle']])
