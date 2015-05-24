from ddapp.consoleapp import ConsoleApp
from ddapp import robotsystem
from ddapp import robotstate
from ddapp import planplayback
from ddapp import lcmUtils
from ddapp import drcargs
from ddapp import roboturdf
from ddapp import simpletimer
from ddapp.timercallback import TimerCallback
from ddapp.fieldcontainer import FieldContainer
from PythonQt import QtCore, QtGui, QtUiTools
import drc as lcmdrc
import numpy as np
import math
import argparse

from ddapp.utime import getUtime
from drake import lcmt_qp_controller_input, lcmt_whole_body_data

import scipy.interpolate



def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


def newAtlasCommandMessageAtZero():

    msg = lcmdrc.atlas_command_t()
    msg.joint_names = [str(v) for v in robotstate.getRobotStateJointNames()]
    msg.num_joints = len(msg.joint_names)
    zeros = np.zeros(msg.num_joints)
    msg.k_q_p = getDefaultUserModePositionGains()
    msg.k_q_i = zeros.tolist()
    msg.k_qd_p = zeros.tolist()
    msg.k_f_p = zeros.tolist()
    msg.ff_qd = zeros.tolist()
    msg.ff_qd_d = zeros.tolist()
    msg.ff_f_d = zeros.tolist()
    msg.ff_const = zeros.tolist()
    msg.effort = zeros.tolist()
    msg.velocity = zeros.tolist()
    msg.position = zeros.tolist()
    msg.desired_controller_period_ms = 3
    msg.k_effort = '0'*msg.num_joints
    return msg

def getDefaultUserModePositionGains():
    return [10.0, 70.0, 70.0, 1000.0, 60.0, 78.0, 50.0, 140.0, 2000.0, 2000.0, 60.0, 78.0, 50.0, 140.0, 2000.0, 2000.0, 4.0, 4.0, 4.0, 4.0, 20.0, 20.0, 20.0, 4.0, 4.0, 4.0, 4.0, 20.0, 20.0, 20.0]


def drakePoseToAtlasCommand(drakePose):
    jointIndexMap = robotstate.getRobotStateToDrakePoseJointMap()
    robotState = np.zeros(len(jointIndexMap))
    for jointIdx, drakeIdx in jointIndexMap.iteritems():
        robotState[jointIdx] = drakePose[drakeIdx]
    position = robotState.tolist()
    msg = newAtlasCommandMessageAtZero()
    msg.position = position
    return msg

def atlasCommandToDrakePose(msg):
    jointIndexMap = robotstate.getRobotStateToDrakePoseJointMap()
    drakePose = np.zeros(len(robotstate.getDrakePoseJointNames()))
    for jointIdx, drakeIdx in jointIndexMap.iteritems():
        drakePose[drakeIdx] = msg.position[jointIdx]
    return drakePose.tolist()

def drakePoseToQPInput(pose, atlasVersion=5):
    if atlasVersion == 4:
        numPositions = 34
    else:
        numPositions = 36

    msg = lcmt_qp_controller_input()
    msg.timestamp = getUtime()
    msg.num_support_data = 0
    msg.num_tracked_bodies = 0
    msg.num_external_wrenches = 0
    msg.num_joint_pd_overrides = 0
    msg.param_set_name = 'position_control'

    whole_body_data = lcmt_whole_body_data()
    whole_body_data.timestamp = getUtime()
    whole_body_data.num_positions = numPositions
    whole_body_data.q_des = pose
    whole_body_data.num_constrained_dofs = numPositions - 6
    whole_body_data.constrained_dofs = range(7, numPositions+1)
    msg.whole_body_data = whole_body_data

    return msg


def getJointGroups():
    return [
        { "name" : "Back",
          "joints" : [
            "back_bkx",
            "back_bky",
            "back_bkz"
            ],
          "labels" : ["x", "y", "z"]
        },

        { "name" : "Left Arm",
          "joints" : [
            "l_arm_shz",
            "l_arm_shx",
            "l_arm_ely",
            "l_arm_elx",
            "l_arm_uwy",
            "l_arm_mwx",
            "l_arm_lwy",
            ],
          "labels" : ["shz", "shx", "ely", "elx", "uwy", "mwx", "lwy"]
        },

        { "name" : "Right Arm",
          "joints" : [
            "r_arm_shz",
            "r_arm_shx",
            "r_arm_ely",
            "r_arm_elx",
            "r_arm_uwy",
            "r_arm_mwx",
            "r_arm_lwy",
            ],
          "labels" : ["shz", "shx", "ely", "elx", "uwy", "mwx", "lwy"]
        },

        { "name" : "Left Leg",
          "joints" : [
            "l_leg_hpx",
            "l_leg_hpy",
            "l_leg_hpz",
            "l_leg_kny",
            "l_leg_akx",
            "l_leg_aky"
            ],
          "labels" : ["hpx", "hpy", "hpz", "knee", "akx", "aky"]
        },

        { "name" : "Right Leg",
          "joints" : [
            "r_leg_hpx",
            "r_leg_hpy",
            "r_leg_hpz",
            "r_leg_kny",
            "r_leg_akx",
            "r_leg_aky"
            ],
          "labels" : ["hpx", "hpy", "hpz", "knee", "akx", "aky"]
        }
      ]



class AtlasCommandStream(object):

    def __init__(self):
        self.timer = TimerCallback(targetFps=10)
        #self.timer.disableScheduledTimer()
        self.app = ConsoleApp()
        self.robotModel, self.jointController = roboturdf.loadRobotModel('robot model')
        self.fpsCounter = simpletimer.FPSCounter()
        self.drakePoseJointNames = robotstate.getDrakePoseJointNames()
        self.fpsCounter.printToConsole = True
        self.timer.callback = self._tick
        self._initialized = False
        self.publishChannel = 'JOINT_POSITION_GOAL'
        # self.lastCommandMessage = newAtlasCommandMessageAtZero()
        self._numPositions = len(robotstate.getDrakePoseJointNames())
        self._previousElapsedTime = 100
        self._baseFlag = 0;
        self.jointLimitsMin = np.array([self.robotModel.model.getJointLimits(jointName)[0] for jointName in robotstate.getDrakePoseJointNames()])
        self.jointLimitsMax = np.array([self.robotModel.model.getJointLimits(jointName)[1] for jointName in robotstate.getDrakePoseJointNames()])
        self.useControllerFlag = False
        self.drivingGainsFlag = False
        self.applyDefaults()

    def initialize(self, currentRobotPose):
        assert not self._initialized
        self._currentCommandedPose = np.array(currentRobotPose)
        self._previousCommandedPose = np.array(currentRobotPose)
        self._goalPose = np.array(currentRobotPose)
        self._initialized = True

    def useController(self):
        self.useControllerFlag = True
        self.publishChannel = 'QP_CONTROLLER_INPUT'

    def setKp(self, Kp, jointName=None):
        if jointName is None:
            self._Kp = Kp*np.ones(self._numPositions)
        else:
            idx = robotstate.getDrakePoseJointNames().index(jointName)
            self._maxSpeed[idx] = Kp

        self.updateKd()

    def setMaxSpeed(self, speed, jointName=None):
        if jointName is None:
            self._maxSpeed = np.deg2rad(speed)*np.ones(self._numPositions)
        else:
            idx = robotstate.getDrakePoseJointNames().index(jointName)
            self._maxSpeed[idx] = np.deg2rad(speed)

    def updateKd(self):
        self._dampingRatio = 1;
        self._Kd = 2*self._dampingRatio*np.sqrt(self._Kp)

    def applyDefaults(self):
        self.setKp(10)
        self.setMaxSpeed(10)

        if self.drivingGainsFlag:
            self.setMaxSpeed(70,'l_arm_lwy')
            self.setKp(50,'l_arm_lwy')
            self.setMaxSpeed(100,'l_leg_aky')
            self.setKp(100,'l_leg_aky')
    
    def applyDrivingDefaults(self):
        self.drivingGainsFlag = True
        self.applyDefaults()

    def applyPlanDefaults(self):
        self.setKp(10)
        self.setMaxSpeed(60)        
        
    def startStreaming(self):
        assert self._initialized
        if not self.timer.isActive():
            self.timer.start()

    def stopStreaming(self):
        self.timer.stop()

    def setGoalPose(self, pose):         
        self._goalPose = self.clipPoseToJointLimits(pose)

    def setIndividualJointGoalPose(self, pose, jointName):
        jointIdx = self.drakePoseJointNames.index(jointName)
        self._goalPose[jointIdx] = pose

    def clipPoseToJointLimits(self,pose):
        pose = np.array(pose)
        pose = np.clip(pose, self.jointLimitsMin, self.jointLimitsMax)
        return pose

    def waitForRobotState(self):
        msg = lcmUtils.captureMessage('EST_ROBOT_STATE', lcmdrc.robot_state_t)
        pose = robotstate.convertStateMessageToDrakePose(msg)
        pose[:6] = np.zeros(6)
        self.initialize(pose)

    def _updateAndSendCommandMessage(self):
        self._currentCommandedPose = self.clipPoseToJointLimits(self._currentCommandedPose)
        if self._baseFlag:
            msg = robotstate.drakePoseToRobotState(self._currentCommandedPose)
        else:
            msg = drakePoseToAtlasCommand(self._currentCommandedPose)

        if self.useControllerFlag:
            msg = drakePoseToQPInput(self._currentCommandedPose)

        lcmUtils.publish(self.publishChannel, msg)

    def _tick(self):

        self.fpsCounter.tick()

        elapsed = self.timer.elapsed # time since last tick
        #nominalElapsed = (1.0 / self.timer.targetFps)
        #if elapsed > 2*nominalElapsed:
        #    elapsed = nominalElapsed

        # move current pose toward goal pose
        previousPose = self._previousCommandedPose.copy()
        currentPose = self._currentCommandedPose.copy()
        goalPose = self.clipPoseToJointLimits(self._goalPose.copy())
        nextPose = self._computeNextPose(previousPose,currentPose, goalPose, elapsed,
            self._previousElapsedTime, self._maxSpeed)
        self._currentCommandedPose = nextPose

        # have the base station directly send the command through
        if self._baseFlag:
            self._currentCommandedPose = goalPose

        # publish
        self._updateAndSendCommandMessage()

        # bookkeeping
        self._previousElapsedTime = elapsed
        self._previousCommandedPose = currentPose

    def _computeNextPose(self, previousPose, currentPose, goalPose, elapsed, elapsedPrevious, maxSpeed):
        v = 1.0/elapsedPrevious * (currentPose - previousPose)
        u = -self._Kp*(currentPose - goalPose) - self._Kd*v # u = -K*x
        v_next = v + elapsed*u
        v_next = np.clip(v_next,-maxSpeed,maxSpeed) # velocity clamp
        nextPose = currentPose + v_next*elapsed
        nextPose = self.clipPoseToJointLimits(nextPose)
        return nextPose

commandStream = AtlasCommandStream()


class DebugAtlasCommandListener(object):

    def __init__(self):

        self.app = ConsoleApp()
        self.view = self.app.createView()
        self.robotModel, self.jointController = roboturdf.loadRobotModel('robot model', self.view)
        self.jointController.setZeroPose()
        self.view.show()
        self.sub = lcmUtils.addSubscriber('ATLAS_COMMAND', lcmdrc.atlas_command_t, self.onAtlasCommand)
        self.sub.setSpeedLimit(60)

    def onAtlasCommand(self, msg):
        pose = atlasCommandToDrakePose(msg)
        self.jointController.setPose('ATLAS_COMMAND', pose)


class CommittedRobotPlanListener(object):

    def __init__(self):
        self.sub = lcmUtils.addSubscriber('COMMITTED_ROBOT_PLAN', lcmdrc.robot_plan_t, self.onRobotPlan)
        lcmUtils.addSubscriber('COMMITTED_PLAN_PAUSE', lcmdrc.plan_control_t, self.onPause)
        self.animationTimer = None


    def onPause(self, msg):
        commandStream.stopStreaming()
        if self.animationTimer:
            self.animationTimer.stop()

    def onRobotPlan(self, msg):


        playback = planplayback.PlanPlayback()
        playback.interpolationMethod = 'pchip'
        poseTimes, poses = playback.getPlanPoses(msg)
        f = playback.getPoseInterpolator(poseTimes, poses)

        print 'received robot plan, %.2f seconds' % (poseTimes[-1] - poseTimes[0])

        commandStream.applyPlanDefaults()
        commandStream.startStreaming()

        timer = simpletimer.SimpleTimer()

        def setPose(pose):
            commandStream.setGoalPose(pose)

        def updateAnimation():

            tNow = timer.elapsed()

            if tNow > poseTimes[-1]:
                pose = poses[-1]
                setPose(pose)
                commandStream.applyDefaults()
                print 'plan ended.'
                return False

            pose = f(tNow)
            setPose(pose)

        self.animationTimer = TimerCallback()
        self.animationTimer.targetFps = 1000
        self.animationTimer.callback = updateAnimation
        self.animationTimer.start()



class PositionGoalListener(object):

    def __init__(self):
        self.sub = lcmUtils.addSubscriber('JOINT_POSITION_GOAL', lcmdrc.robot_state_t, self.onJointPositionGoal)
        self.sub = lcmUtils.addSubscriber('SINGLE_JOINT_POSITION_GOAL', lcmdrc.joint_position_goal_t, self.onSingleJointPositionGoal)
        lcmUtils.addSubscriber('COMMITTED_PLAN_PAUSE', lcmdrc.plan_control_t, self.onPause)
        self.debug = False

        if self.debug:

            self.app = ConsoleApp()
            self.view = self.app.createView()
            self.robotModel, self.jointController = roboturdf.loadRobotModel('robot model', self.view)
            self.jointController.setPose('ATLAS_COMMAND', commandStream._currentCommandedPose)
            self.view.show()
            self.timer = TimerCallback(targetFps=30)
            self.timer.callback = self.onDebug


    def onPause(self, msg):
        commandStream.stopStreaming()

    def onJointPositionGoal(self, msg):
        #lcmUtils.publish('ATLAS_COMMAND', msg)

        commandStream.startStreaming()
        pose = robotstate.convertStateMessageToDrakePose(msg)
        self.setGoalPose(pose)

    def setGoalPose(self, pose):
        commandStream.setGoalPose(pose)

    def onDebug(self):
        self.jointController.setPose('ATLAS_COMMAND', commandStream._currentCommandedPose)

    def onSingleJointPositionGoal(self, msg):
        jointPositionGoal = msg.joint_position
        jointName = msg.joint_name
        allowedJointNames = ['l_leg_aky','l_arm_lwy']

        if not (jointName in allowedJointNames):
            print 'Position goals are not allowed for this joint'
            print 'ignoring this position goal'
            print 'use the sliders instead'
            return
            
        commandStream.setIndividualJointGoalPose(jointPositionGoal, jointName)


class JointCommandPanel(object):

    def __init__(self, robotSystem):

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddJointCommandPanel.ui')
        assert uifile.open(uifile.ReadOnly)

        self.robotSystem = robotSystem

        self.widget = loader.load(uifile)
        self.ui = WidgetDict(self.widget.children())
        self.ui.streamingButton.connect('clicked()', self.onStreamingClicked)
        self.ui.resetButton.connect('clicked()', self.onResetClicked)
        self.ui.hidePreviewButton.connect('clicked()', self.onHidePreviewClicked)
        self.ui.previewSlider.connect('valueChanged(int)', self.onPreviewSliderChanged)
        self.ui.speedSpinBox.connect('valueChanged(double)', self.onSpeedChanged)
        self.ui.steeringButton.connect('clicked()', self.onSteeringButtonClicked)
        self.ui.throttleButton.connect('clicked()', self.onThrottleButtonClicked)
        self.hidePreviewModels()

        self.throttleControlEnabled = False
        self.steeringControlEnabled = False

    def onPreviewSliderChanged(self, value):

        self.ui.hidePreviewButton.setEnabled(True)

        value = value / float(self.ui.previewSlider.maximum)

        startPose = self.robotSystem.robotStateJointController.q.copy()
        endPose = self.robotSystem.teleopJointController.q.copy()
        interpPose = startPose + (endPose-startPose) * value
        self.robotSystem.playbackJointController.setPose('playback_pose', interpPose)
        self.showPreviewModels()

    def onHidePreviewClicked(self):
        self.ui.previewSlider.setValue(0)
        self.ui.hidePreviewButton.setEnabled(False)
        self.hidePreviewModels()

    def onResetClicked(self):
        self.hidePreviewModels()

    def onSpeedChanged(self, value):
        commandStream._maxSpeed = np.deg2rad(value)

    def onStreamingClicked(self):
        streamingEnabled = bool(self.ui.streamingButton.checked)

        if streamingEnabled:
            self.onHidePreviewClicked()
            self.ui.previewSlider.setEnabled(False)
            self.ui.streamingButton.setText('Stop Streaming')

            if not commandStream._initialized:
                commandStream.initialize(self.robotSystem.robotStateJointController.q)
                commandStream.setGoalPose(self.robotSystem.teleopJointController.q)
            commandStream.startStreaming()
        else:
            self.ui.previewSlider.setEnabled(True)
            self.ui.streamingButton.setText('Start Streaming')

            self.sendPlanPause()
            commandStream.stopStreaming()

    def onSteeringButtonClicked(self):
        steeringControlEnabled = bool(self.ui.steeringButton.checked)

        if steeringControlEnabled:
            self.ui.steeringButton.setText('Stop Steering')
            self.steeringControlEnabled = True
        else:
            self.ui.steeringButton.setText('Start Steering')
            self.steeringControlEnabled = False

    def onThrottleButtonClicked(self):
        throttleControlEnabled = bool(self.ui.throttleButton.checked)

        if throttleControlEnabled:
            self.ui.throttleButton.setText('Stop Throttle')
            self.throttleControlEnabled = True
        else:
            self.ui.throttleButton.setText('Start Throttle')
            self.throttleControlEnabled = False


    def sendPlanPause(self):
        msg = lcmdrc.plan_control_t()
        msg.utime = getUtime()
        msg.control = lcmdrc.plan_control_t.PAUSE
        lcmUtils.publish('COMMITTED_PLAN_PAUSE', msg)

    def showPreviewModels(self):
        self.robotSystem.playbackRobotModel.setProperty('Visible', True)
        self.robotSystem.teleopRobotModel.setProperty('Visible', False)
        self.robotSystem.robotStateModel.setProperty('Visible', True)
        self.robotSystem.robotStateModel.setProperty('Alpha', 0.1)

    def hidePreviewModels(self):
        self.robotSystem.playbackRobotModel.setProperty('Visible', False)
        self.robotSystem.teleopRobotModel.setProperty('Visible', True)
        self.robotSystem.robotStateModel.setProperty('Visible', True)
        self.robotSystem.robotStateModel.setProperty('Alpha', 0.1)



class JointTeleopPanel(object):

    def __init__(self, robotSystem, jointGroups):

        self.widget = QtGui.QTabWidget()

        self.robotStateModel = robotSystem.robotStateModel
        self.teleopRobotModel = robotSystem.teleopRobotModel
        self.teleopJointController = robotSystem.teleopJointController
        self.robotStateJointController = robotSystem.robotStateJointController

        self.jointLimitsMin = np.array([self.teleopRobotModel.model.getJointLimits(jointName)[0] for jointName in robotstate.getDrakePoseJointNames()])
        self.jointLimitsMax = np.array([self.teleopRobotModel.model.getJointLimits(jointName)[1] for jointName in robotstate.getDrakePoseJointNames()])

        self.mirrorArms = False
        self.mirrorLegs = False

        self.buildTabWidget(jointGroups)
        self.resetPoseToRobotState()

    def buildTabWidget(self, jointGroups):

        self.slidersMap = {}
        self.labelMap = {}

        for group in jointGroups:
            groupName = group['name']
            joints = group['joints']
            labels = group['labels']
            if len(labels) != len(joints):
                print 'error, joints/labels mismatch for joint group:', name
                continue

            jointGroupWidget = QtGui.QWidget()
            gridLayout = QtGui.QGridLayout(jointGroupWidget)
            gridLayout.setColumnStretch(0, 1)

            for jointName, labelText in zip(joints, labels):
                label = QtGui.QLabel(labelText)
                numericLabel = QtGui.QLabel('0.0')
                slider = QtGui.QSlider(QtCore.Qt.Vertical)
                column = gridLayout.columnCount()
                gridLayout.addWidget(label, 0, column)
                gridLayout.addWidget(slider, 1, column)
                gridLayout.addWidget(numericLabel, 2, column)
                self.slidersMap[jointName] = slider
                self.labelMap[slider] = numericLabel

            gridLayout.setColumnStretch(gridLayout.columnCount(), 1)
            self.widget.addTab(jointGroupWidget, groupName)

        self.widget.usesScrollButtons = False
        self.signalMapper = QtCore.QSignalMapper()

        self.sliderMax = 1000.0
        for jointName, slider in self.slidersMap.iteritems():
            slider.connect('valueChanged(int)', self.signalMapper, 'map()')
            self.signalMapper.setMapping(slider, jointName)
            slider.setMaximum(self.sliderMax)

        self.signalMapper.connect('mapped(const QString&)', self.sliderChanged)

    def resetButtonClicked(self):
        self.resetPoseToRobotState()

    def showPose(self, pose):
        self.teleopJointController.setPose('teleop_pose', pose)
        self.teleopRobotModel.setProperty('Visible', True)
        self.robotStateModel.setProperty('Visible', True)
        self.robotStateModel.setProperty('Alpha', 0.1)
        commandStream.setGoalPose(self.teleopJointController.q)


    def resetPoseToRobotState(self):
        self.endPose = self.robotStateJointController.q.copy()
        self.updateSliders()
        self.showPose(self.endPose)

    def toJointIndex(self, jointName):
        return robotstate.getDrakePoseJointNames().index(jointName)

    def toJointName(self, jointIndex):
        return robotstate.getDrakePoseJointNames()[jointIndex]

    def toJointValue(self, jointIndex, sliderValue):
        assert 0.0 <= sliderValue <= 1.0
        jointRange = self.jointLimitsMin[jointIndex], self.jointLimitsMax[jointIndex]
        return jointRange[0] + (jointRange[1] - jointRange[0])*sliderValue

    def toSliderValue(self, jointIndex, jointValue):
        jointRange = self.jointLimitsMin[jointIndex], self.jointLimitsMax[jointIndex]
        return (jointValue - jointRange[0]) / (jointRange[1] - jointRange[0])

    def getSlider(self, joint):
        jointName = self.toJointName(joint) if isinstance(joint, int) else joint
        return self.slidersMap[jointName]

    def getJointValue(self, jointIndex):
        return self.endPose[jointIndex]

    def sliderChanged(self, jointName):


        slider = self.slidersMap[jointName]
        jointIndex = self.toJointIndex(jointName)
        jointValue = self.toJointValue(jointIndex, slider.value / float(self.sliderMax))

        if ('arm' in jointName and self.mirrorArms) or ('leg' in jointName and self.mirrorLegs):
            mirrorJoint = jointName.replace('l_', 'r_') if jointName.startswith('l_') else jointName.replace('r_', 'l_')
            mirrorIndex = self.toJointIndex(mirrorJoint)
            mirrorValue = jointValue
            if mirrorJoint in drcargs.getDirectorConfig()['mirrorJointSignFlips']:
                mirrorValue = -mirrorValue
            self.endPose[mirrorIndex] = mirrorValue

        self.endPose[jointIndex] = jointValue
        self.updateLabel(jointName, jointValue)
        self.showPose(self.endPose)
        self.updateSliders()

    def updateLabel(self, jointName, jointValue):

        slider = self.slidersMap[jointName]
        label = self.labelMap[slider]

        fill = 6 if jointValue >= 0 else 5

        if jointName in ['base_x', 'base_y', 'base_z']:
            label.text = str('%.3f' % jointValue).center(fill, ' ')
        else:
            label.text = str('%.1f' % math.degrees(jointValue)).center(fill, ' ')


    def updateSliders(self):

        for jointName, slider in self.slidersMap.iteritems():
            jointIndex = self.toJointIndex(jointName)
            jointValue = self.getJointValue(jointIndex)

            slider.blockSignals(True)
            slider.setValue(self.toSliderValue(jointIndex, jointValue)*self.sliderMax)
            slider.blockSignals(False)
            self.updateLabel(jointName, jointValue)



class AtlasCommandPanel(object):

    def __init__(self):

        self.app = ConsoleApp()
        self.view = self.app.createView()
        self.robotSystem = robotsystem.create(self.view)

        jointGroups = getJointGroups()
        self.jointTeleopPanel = JointTeleopPanel(self.robotSystem, jointGroups)
        self.jointCommandPanel = JointCommandPanel(self.robotSystem)

        self.jointCommandPanel.ui.speedSpinBox.setEnabled(False)

        self.jointCommandPanel.ui.mirrorArmsCheck.setChecked(self.jointTeleopPanel.mirrorArms)
        self.jointCommandPanel.ui.mirrorLegsCheck.setChecked(self.jointTeleopPanel.mirrorLegs)
        self.jointCommandPanel.ui.resetButton.connect('clicked()', self.resetJointTeleopSliders)
        self.jointCommandPanel.ui.mirrorArmsCheck.connect('clicked()', self.mirrorJointsChanged)
        self.jointCommandPanel.ui.mirrorLegsCheck.connect('clicked()', self.mirrorJointsChanged)

        self.widget = QtGui.QWidget()

        gl = QtGui.QGridLayout(self.widget)
        gl.addWidget(self.app.showObjectModel(), 0, 0, 4, 1) # row, col, rowspan, colspan
        gl.addWidget(self.view, 0, 1, 1, 1)
        gl.addWidget(self.jointCommandPanel.widget, 1, 1, 1, 1)
        gl.addWidget(self.jointTeleopPanel.widget, 0, 2, -1, 1)
        gl.setRowStretch(0,1)
        gl.setColumnStretch(1,1)

        #self.sub = lcmUtils.addSubscriber('COMMITTED_ROBOT_PLAN', lcmdrc.robot_plan_t, self.onRobotPlan)
        lcmUtils.addSubscriber('STEERING_COMMAND_POSITION_GOAL', lcmdrc.joint_position_goal_t, self.onSingleJointPositionGoal)
        lcmUtils.addSubscriber('THROTTLE_COMMAND_POSITION_GOAL', lcmdrc.joint_position_goal_t, self.onSingleJointPositionGoal)


    def onSingleJointPositionGoal(self, msg):
        jointPositionGoal = msg.joint_position
        jointName = msg.joint_name
        allowedJointNames = ['l_leg_aky','l_arm_lwy']

        if not (jointName in allowedJointNames):
            print 'Position goals are not allowed for this joint'
            print 'ignoring this position goal'
            print 'use the sliders instead'
            return

        if (jointName == 'l_arm_lwy') and (not self.jointCommandPanel.steeringControlEnabled):
            print 'Steering control not enabled'
            print 'ignoring steering command'
            return

        if (jointName == 'l_leg_aky') and (not self.jointCommandPanel.throttleControlEnabled):
            print 'Throttle control not enabled'
            print 'ignoring throttle command'
            return
            
        jointIdx = self.jointTeleopPanel.toJointIndex(joint_name)
        self.jointTeleopPanel.endPose[jointIdx] = jointPositionGoal
        self.jointTeleopPanel.updateSliders()
        self.jointTeleopPanel.sliderChanged(jointName)

            
    def onRobotPlan(self, msg):
        playback = planplayback.PlanPlayback()
        playback.interpolationMethod = 'pchip'
        poseTimes, poses = playback.getPlanPoses(msg)
        f = playback.getPoseInterpolator(poseTimes, poses)

        jointController = self.robotSystem.teleopJointController

        timer = simpletimer.SimpleTimer()

        def setPose(pose):
            jointController.setPose('plan_playback', pose)
            self.jointTeleopPanel.endPose = pose
            self.jointTeleopPanel.updateSliders()
            commandStream.setGoalPose(pose)

        def updateAnimation():

            tNow = timer.elapsed()

            if tNow > poseTimes[-1]:
                pose = poses[-1]
                setPose(pose)
                return False

            pose = f(tNow)
            setPose(pose)


        self.animationTimer = TimerCallback()
        self.animationTimer.targetFps = 60
        self.animationTimer.callback = updateAnimation
        self.animationTimer.start()


    def mirrorJointsChanged(self):
        self.jointTeleopPanel.mirrorLegs = self.jointCommandPanel.ui.mirrorLegsCheck.checked
        self.jointTeleopPanel.mirrorArms = self.jointCommandPanel.ui.mirrorArmsCheck.checked

    def resetJointTeleopSliders(self):
        self.jointTeleopPanel.resetPoseToRobotState()
  


def parseArgs():

    parser = argparse.ArgumentParser()

    p = parser.add_mutually_exclusive_group(required=True)
    p.add_argument('--base', dest='mode', action='store_const', const='base')
    p.add_argument('--robot', dest='mode', action='store_const', const='robot')
    p.add_argument('--debug', dest='mode', action='store_const', const='debug')
    p.add_argument('--robotDrivingGains', dest='mode', action='store_const', const='robotDrivingGains')
    p.add_argument('--robotWithoutController', dest='mode', action='store_const', const='robotWithoutController')
    p.add_argument('--robotDrivingGainsWithoutController', dest='mode', action='store_const', const='robotDrivingGainsWithoutController')
    args, unknown = parser.parse_known_args()
    return args


def main():

    args = parseArgs()

    if args.mode == 'base':
        baseMain()
    elif args.mode == 'robot':
        robotMain()
    elif args.mode == 'robotDrivingGains':
        robotMain(useDrivingGains=True)
    elif args.mode == 'robotWithoutController':
        robotMain(useController=False)
    elif args.mode == 'robotDrivingGainsWithoutController':
        robotMain(useDrivingGains=True, useController=False)
    else:
        debugMain()

def baseMain():
    p = AtlasCommandPanel()
    commandStream._baseFlag = 1
    p.widget.show()
    p.widget.resize(1400, 1400*9/16.0)
    p.app.setupGlobals(globals())
    p.app.start()


def debugMain():

    listener = DebugAtlasCommandListener()
    ConsoleApp.start()


def robotMain(useDrivingGains=False, useController=True):

    print 'waiting for robot state...'
    commandStream.waitForRobotState()
    print 'starting.'
    commandStream.timer.targetFps = 1000

    if useController==True:
        commandStream.useController()
    else:
        commandStream.publishChannel = 'ATLAS_COMMAND'

    if useDrivingGains:
        commandStream.applyDrivingDefaults()

    commandStream.startStreaming()
    positionListener = PositionGoalListener()
    planListener = CommittedRobotPlanListener()
    ConsoleApp.start()


if __name__ == '__main__':
    main()

