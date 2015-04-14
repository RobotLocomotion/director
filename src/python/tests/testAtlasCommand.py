from ddapp.consoleapp import ConsoleApp
from ddapp import robotsystem
from ddapp import robotstate
from ddapp import planplayback
from ddapp import simpletimer
from ddapp import lcmUtils
from ddapp import drcargs
from ddapp import roboturdf
from ddapp.timercallback import TimerCallback
from ddapp.fieldcontainer import FieldContainer
from PythonQt import QtCore, QtGui, QtUiTools
import drc as lcmdrc
import numpy as np
import math
import argparse

from ddapp.utime import getUtime


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


def drakePoseToAtlasCommandPosition(drakePose):
    jointIndexMap = robotstate.getRobotStateToDrakePoseJointMap()
    robotState = np.zeros(len(jointIndexMap))
    for jointIdx, drakeIdx in jointIndexMap.iteritems():
        robotState[jointIdx] = drakePose[drakeIdx]
    return robotState.tolist()

def atlasCommandToDrakePose(msg):
    jointIndexMap = robotstate.getRobotStateToDrakePoseJointMap()
    drakePose = np.zeros(len(robotstate.getDrakePoseJointNames()))
    for jointIdx, drakeIdx in jointIndexMap.iteritems():
        drakePose[drakeIdx] = msg.position[jointIdx]
    return drakePose.tolist()



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
        self.timer = TimerCallback(targetFps=500)
        #self.timer.disableScheduledTimer()
        self.fpsCounter = simpletimer.FPSCounter()
        self.fpsCounter.printToConsole = True
        self.timer.callback = self._tick
        self._maxSpeed = np.deg2rad(2)
        self._initialized = False
        self.lastCommandMessage = newAtlasCommandMessageAtZero()

    def initialize(self, currentRobotPose):
        assert not self._initialized
        self._currentCommandedPose = np.array(currentRobotPose)
        self._goalPose = np.array(currentRobotPose)
        self._initialized = True

    def startStreaming(self):
        assert self._initialized
        self.timer.start()

    def stopStreaming(self):
        self.timer.stop()

    def setGoalPose(self, pose):
        self._goalPose = np.array(pose)

    def _updateAndSendCommandMessage(self):
        self.lastCommandMessage.position = drakePoseToAtlasCommandPosition(self._currentCommandedPose)
        self.lastCommandMessage.utime = getUtime()
        lcmUtils.publish('JOINT_POSITION_GOAL', self.lastCommandMessage)

    def _tick(self):

        self.fpsCounter.tick()

        elapsed = self.timer.elapsed
        nominalElapsed = (1.0 / self.timer.targetFps)

        if elapsed > 2*nominalElapsed:
            elapsed = nominalElapsed

        # move current pose toward goal pose
        startPose = self._currentCommandedPose.copy()
        goalPose = self._goalPose.copy()

        maxDelta = self._maxSpeed * elapsed

        for i in xrange(len(startPose)):
            delta = np.clip(goalPose[i] - startPose[i], -maxDelta, maxDelta)
            startPose[i] += delta
            if np.abs(delta) > 1e-6:
                print robotstate.getDrakePoseJointNames()[i], '-->', np.rad2deg(startPose[i])

        self._currentCommandedPose = startPose

        # publish
        self._updateAndSendCommandMessage()


commandStream = AtlasCommandStream()


class PositionGoalListener(object):

    def __init__(self):
        self.sub = lcmUtils.addSubscriber('JOINT_POSITION_GOAL', lcmdrc.atlas_command_t, self.onJointPositionGoal)

        self.debug = True

        if self.debug:
            self.app = ConsoleApp()
            self.view = self.app.createView()
            self.robotModel, self.jointController = roboturdf.loadRobotModel('robot model', self.view)
            self.jointController.setZeroPose()
            self.view.show()

    def onJointPositionGoal(self, msg):
        lcmUtils.publish('ATLAS_COMMAND', msg)

        if self.debug:
            pose = atlasCommandToDrakePose(msg)
            self.jointController.setPose('ATLAS_COMMAND', pose)


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
        self.ui.editPositionGainsButton.connect('clicked()', self.onEditPositionGains)
        self.hidePreviewModels()
        self.gainsEditor = GainAdjustmentPanel()

    def onEditPositionGains(self):
        self.gainsEditor.widget.show()

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
            commandStream.stopStreaming()

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


class GainAdjustmentPanel(object):

    def __init__(self):

        jointGroups = getJointGroups()
        self.widget = QtGui.QTabWidget()
        self.widget.setWindowTitle('Edit Position Gains')
        self.jointLimitsMax = np.array(getDefaultUserModePositionGains())
        self.jointLimitsMin = np.zeros(len(self.jointLimitsMax))
        self.buildTabWidget(jointGroups)


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

        self.resetPoseToRobotState()
        self.signalMapper.connect('mapped(const QString&)', self.sliderChanged)



    def showPose(self, pose):
        commandStream.lastCommandMessage.k_q_p = list(pose)

    def resetPoseToRobotState(self):
        self.endPose = np.array(getDefaultUserModePositionGains())
        self.updateSliders()
        self.showPose(self.endPose)

    def toJointIndex(self, jointName):
        return robotstate.getRobotStateJointNames().index(jointName)

    def toJointName(self, jointIndex):
        return robotstate.getRobotStateJointNames()[jointIndex]

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
        self.endPose[jointIndex] = jointValue
        self.updateLabel(jointName, jointValue)
        self.showPose(self.endPose)

    def updateLabel(self, jointName, jointValue):

        slider = self.slidersMap[jointName]
        label = self.labelMap[slider]

        fill = 6 if jointValue >= 0 else 5

        label.text = str('%.1f' % jointValue).center(fill, ' ')


    def updateSliders(self):

        for jointName, slider in self.slidersMap.iteritems():
            jointIndex = self.toJointIndex(jointName)
            jointValue = self.getJointValue(jointIndex)

            slider.blockSignals(True)
            slider.setValue(self.toSliderValue(jointIndex, jointValue)*self.sliderMax)
            slider.blockSignals(False)
            self.updateLabel(jointName, jointValue)



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
        self.mirrorLegs = True

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

        self.jointCommandPanel.ui.resetButton.connect('clicked()', self.resetJointTeleopSliders)

        self.widget = QtGui.QWidget()

        gl = QtGui.QGridLayout(self.widget)
        gl.addWidget(self.app.showObjectModel(), 0, 0, 4, 1) # row, col, rowspan, colspan
        gl.addWidget(self.view, 0, 1, 1, 1)
        gl.addWidget(self.jointCommandPanel.widget, 1, 1, 1, 1)
        gl.addWidget(self.jointTeleopPanel.widget, 0, 2, -1, 1)
        gl.setRowStretch(0,1)
        gl.setColumnStretch(1,1)


    def resetJointTeleopSliders(self):
        self.jointTeleopPanel.resetPoseToRobotState()



def parseArgs():

    parser = argparse.ArgumentParser()

    p = parser.add_mutually_exclusive_group(required=True)
    p.add_argument('--base', dest='mode', action='store_const', const='base')
    p.add_argument('--robot', dest='mode', action='store_const', const='robot')

    args, unknown = parser.parse_known_args()
    return args


def main():

    args = parseArgs()

    if args.mode == 'base':
        baseMain()
    else:
        robotMain()

def baseMain():
    p = AtlasCommandPanel()
    p.widget.show()
    p.widget.resize(1400, 1400*9/16.0)
    p.app.setupGlobals(globals())
    p.app.start()


def robotMain():

    listener = PositionGoalListener()
    ConsoleApp.start()


if __name__ == '__main__':
    main()

