from PythonQt import QtCore, QtGui, QtUiTools
import ddapp.applogic as app
import ddapp.objectmodel as om
from ddapp.timercallback import TimerCallback
from ddapp import robotstate
from ddapp import visualization as vis
from ddapp import transformUtils
import math
import numpy as np



def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


def clearLayout(w):
    children = w.findChildren(QtGui.QWidget)
    for child in children:
        child.delete()



class EndEffectorTeleopPanel(object):

    def __init__(self, panel):
        self.panel = panel
        self.ui = panel.ui
        self.ui.eeTeleopButton.connect('clicked()', self.teleopButtonClicked)
        self.ui.planButton.connect('clicked()', self.planClicked)
        self.ui.fixBaseCheck.connect('clicked()', self.fixBaseChanged)
        self.ui.fixBackCheck.connect('clicked()', self.fixBackChanged)
        self.ui.fixOrientCheck.connect('clicked()', self.fixOrientChanged)
        self.ui.eeTeleopSideCombo.connect('currentIndexChanged(const QString&)', self.sideChanged)
        self.frameSync = None

    def getSide(self):
        return str(self.ui.eeTeleopSideCombo.currentText)

    def getBaseFixed(self):
        return self.ui.fixBaseCheck.checked

    def getBackFixed(self):
        return self.ui.fixBackCheck.checked

    def getOrientFixed(self):
        return self.ui.fixOrientCheck.checked

    def fixBaseChanged(self):
        self.updateConstraints()

    def fixBackChanged(self):
        self.updateConstraints()

    def fixOrientChanged(self):
        self.updateConstraints()

    def sideChanged(self):
        self.updateConstraints()

    def removeGoals(self):
        om.removeFromObjectModel(om.findObjectByName('reach goal left'))
        om.removeFromObjectModel(om.findObjectByName('reach goal right'))
        self.frameSync = None

    def getGoalFrame(self, side):
        goal = om.findObjectByName('reach goal %s' % side)
        return transformUtils.copyFrame(goal.transform) if goal is not None else None

    def updateGoalFrame(self, transform, side):
        goalFrame = om.findObjectByName('reach goal %s' % side)
        if not goalFrame:
            return

        if transform is not None:
            goalFrame.copyFrame(transform)
        else:
            goalFrame.transform.Modified()

        return goalFrame


    def updateHandModels(self):

        def syncHandFrame(handModel, goalFrame):

            handObj = handModel.newPolyData('reach goal left hand', self.panel.teleopRobotModel.views[0], parent=goalFrame)
            handFrame = om.getObjectChildren(handObj)[0]
            handFrame.copyFrame(goalFrame.transform)

            frameSync = vis.FrameSync()
            frameSync.addFrame(goalFrame)
            frameSync.addFrame(handFrame)
            return frameSync


        leftGoal = om.findObjectByName('reach goal left')
        rightGoal = om.findObjectByName('reach goal right')

        if leftGoal:
            self.leftFrameSync = syncHandFrame(self.panel.lhandModel, leftGoal)
        if rightGoal:
            self.rightFrameSync = syncHandFrame(self.panel.rhandModel, rightGoal)


    def updateConstraints(self):

        if not self.ui.eeTeleopButton.checked:
            return

        leftGoal = self.getGoalFrame('left')
        rightGoal = self.getGoalFrame('right')
        self.removeGoals()

        side = self.getSide()
        lockBack = self.getBackFixed()
        lockBase = self.getBaseFixed()
        lockOrient = self.getOrientFixed()

        ikPlanner = self.panel.ikPlanner

        poseName = ikPlanner.newReachStartPosture()
        constraints = ikPlanner.newReachConstraints(poseName, lockBack=lockBack, lockBase=lockBase, lockLeftArm=side=='right', lockRightArm=side=='left')

        sides = ['left', 'right'] if side == 'both' else [side]

        for side in sides:
            ikPlanner.reachingSide = side
            ikPlanner.newReachGoal(constraints=constraints, lockOrient=lockOrient, runIk=False, showPoseFunction=self.panel.showPose)

        self.updateHandModels()
        self.updateGoalFrame(leftGoal, 'left')
        self.updateGoalFrame(rightGoal, 'right')


    def planClicked(self):
        if not self.ui.eeTeleopButton.checked:
            return

        self.generatePlan()

    def generatePlan(self):
        self.panel.ikPlanner.planReach()
        self.panel.showPlan()

    def teleopButtonClicked(self):
        if self.ui.eeTeleopButton.checked:
            self.activate()
        else:
            self.deactivate()

    def activate(self):
        self.panel.endEffectorTeleopActivated()
        self.updateConstraints()


    def deactivate(self):
        self.removeGoals()
        self.panel.endEffectorTeleopDeactivated()


inf = np.inf

class JointTeleopPanel(object):


    jointLimitsMin = [-inf, -inf, -inf, -inf, -inf, -inf, -0.663225, -0.262, -0.698132, -1.5708, -1.5708, 0.0, 0.0, 0.0, -0.174533, -0.523599, -1.72072, 0.0, -1.0, -0.8, -1.1781, -1.5708, -1.5708, 0.0, -2.35619, 0.0, -1.22173, -0.523599, -1.72072, 0.0, -1.0, -0.8, -1.1781, -0.602139]
    jointLimitsMax = [inf, inf, inf, inf, inf, inf, 0.663225, 0.524, 0.698132, 0.785398, 1.5708, 3.14159, 2.35619, 3.14159, 1.22173, 0.523599, 0.524821, 2.38569, 0.7, 0.8, 1.1781, 0.785398, 1.5708, 3.14159, 0.0, 3.14159, 0.174533, 0.523599, 0.524821, 2.38569, 0.7, 0.8, 1.1781, 1.14494]

    jointNames = ['base_x', 'base_y', 'base_z', 'base_roll', 'base_pitch', 'base_yaw', 'back_bkz', 'back_bky', 'back_bkx', 'l_arm_usy', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_uwy', 'l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy', 'l_leg_kny', 'l_leg_aky', 'l_leg_akx', 'l_arm_mwx', 'r_arm_usy', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_uwy', 'r_leg_hpz', 'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny', 'r_leg_aky', 'r_leg_akx', 'r_arm_mwx', 'neck_ay']

    def __init__(self, panel):
        self.panel = panel
        self.ui = panel.ui
        self.ui.jointTeleopButton.connect('clicked()', self.teleopButtonClicked)
        self.ui.resetJointsButton.connect('clicked()', self.resetButtonClicked)
        self.ui.planButton.connect('clicked()', self.planClicked)

        self.timerCallback = TimerCallback()
        self.timerCallback.callback = self.onTimerCallback

        self.slidersMap = {
            'back_bkx' : self.ui.backRollSlider,
            'back_bky' : self.ui.backPitchSlider,
            'back_bkz' : self.ui.backYawSlider,

            'l_leg_kny' : self.ui.leftKneeSlider,
            'r_leg_kny' : self.ui.rightKneeSlider,

            'l_arm_usy' : self.ui.leftShoulderXSlider,
            'l_arm_shx' : self.ui.leftShoulderYSlider,
            'l_arm_ely' : self.ui.leftShoulderZSlider,
            'l_arm_elx' : self.ui.leftElbowSlider,
            'l_arm_uwy' : self.ui.leftWristXSlider,
            'l_arm_mwx' : self.ui.leftWristYSlider,

            'r_arm_usy' : self.ui.rightShoulderXSlider,
            'r_arm_shx' : self.ui.rightShoulderYSlider,
            'r_arm_ely' : self.ui.rightShoulderZSlider,
            'r_arm_elx' : self.ui.rightElbowSlider,
            'r_arm_uwy' : self.ui.rightWristXSlider,
            'r_arm_mwx' : self.ui.rightWristYSlider,
            }

        self.labelMap = {
            self.ui.backRollSlider : self.ui.backRollLabel,
            self.ui.backPitchSlider : self.ui.backPitchLabel,
            self.ui.backYawSlider : self.ui.backYawLabel,

            self.ui.leftKneeSlider : self.ui.leftKneeLabel,
            self.ui.rightKneeSlider : self.ui.rightKneeLabel,

            self.ui.leftShoulderXSlider : self.ui.leftShoulderXLabel,
            self.ui.leftShoulderYSlider : self.ui.leftShoulderYLabel,
            self.ui.leftShoulderZSlider : self.ui.leftShoulderZLabel,
            self.ui.leftElbowSlider : self.ui.leftElbowLabel,
            self.ui.leftWristXSlider : self.ui.leftWristXLabel,
            self.ui.leftWristYSlider : self.ui.leftWristYLabel,

            self.ui.rightShoulderXSlider : self.ui.rightShoulderXLabel,
            self.ui.rightShoulderYSlider : self.ui.rightShoulderYLabel,
            self.ui.rightShoulderZSlider : self.ui.rightShoulderZLabel,
            self.ui.rightElbowSlider : self.ui.rightElbowLabel,
            self.ui.rightWristXSlider : self.ui.rightWristXLabel,
            self.ui.rightWristYSlider : self.ui.rightWristYLabel,
            }

        self.signalMapper = QtCore.QSignalMapper()

        for jointName, slider in self.slidersMap.iteritems():
            slider.connect('valueChanged(int)', self.signalMapper, 'map()')
            self.signalMapper.setMapping(slider, jointName)

        self.signalMapper.connect('mapped(const QString&)', self.sliderChanged)

        self.updateWidgetState()


    def planClicked(self):
        if not self.ui.jointTeleopButton.checked:
            return

        self.generatePlan()


    def generatePlan(self):
        self.panel.ikPlanner.computePostureGoal(self.basePose, self.pose)
        self.panel.showPlan()


    def teleopButtonClicked(self):
        self.timerCallback.stop()

        if self.ui.jointTeleopButton.checked:
            self.panel.jointTeleopActivated()
            self.grabCurrentPose()
            self.updateSliders()
        else:
            self.panel.jointTeleopDeactivated()

        self.updateWidgetState()

    def updateWidgetState(self):

        enabled = self.ui.jointTeleopButton.checked

        for slider in self.slidersMap.values():
            slider.setEnabled(enabled)

        self.ui.resetJointsButton.setEnabled(enabled)

        if not enabled:
            self.timerCallback.start()


    def resetButtonClicked(self):
        self.grabCurrentPose()
        self.updateSliders()
        self.panel.showPose(self.pose)

    def onTimerCallback(self):
        if not self.ui.tabWidget.visible:
            return
        self.grabCurrentPose()
        self.updateSliders()

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
        #if jointValue < jointRange[0] or jointValue > jointRange[1]:
        #    print 'warning: joint %s value %f is out of expected range [%f, %f]' % (self.toJointName(jointIndex), jointValue, jointRange[0], jointRange[1])
        return (jointValue - jointRange[0]) / (jointRange[1] - jointRange[0])

    def getSlider(self, joint):
        jointName = self.toJointName(joint) if isinstance(joint, int) else joint
        return self.slidersMap[jointName]

    def grabCurrentPose(self):
        self.basePose = np.array(self.panel.robotStateJointController.q)
        self.pose = np.array(self.basePose)

    def getJointValue(self, jointIndex):
        return self.pose[jointIndex]

    def sliderChanged(self, jointName):

        slider = self.slidersMap[jointName]
        jointIndex = self.toJointIndex(jointName)
        jointValue = self.toJointValue(jointIndex, slider.value / 99.0)

        self.pose[jointIndex] = jointValue

        if 'kny' in jointName:
            if jointName == 'l_leg_kny':
                self.pose[self.toJointIndex('r_leg_kny')] = jointValue

            elif jointName == 'r_leg_kny':
                self.pose[self.toJointIndex('l_leg_kny')] = jointValue

            ik = self.panel.ikPlanner

            startPoseName = 'posture_goal_start'
            ik.jointController.addPose(startPoseName, self.basePose)
            ik.ikServer.sendPoseToServer(ik.jointController.getPose(startPoseName), startPoseName)

            endPoseName = 'posture_goal_end'
            ik.jointController.addPose(endPoseName, self.pose)
            ik.ikServer.sendPoseToServer(ik.jointController.getPose(endPoseName), endPoseName)

            jointNames = self.slidersMap.keys()
            p = ik.createPostureConstraint(endPoseName, jointNames)

            constraints = [p]
            constraints.extend(ik.createFixedFootConstraints(startPoseName))
            constraints.append(ik.createMovingBasePostureConstraint(startPoseName))

            self.pose, info = ik.ikServer.runIk(constraints, seedPostureName=startPoseName)
            self.updateSliders()

        self.panel.showPose(self.pose)

    def updateLabels(self):

        for jointName, slider in self.slidersMap.iteritems():
            jointIndex = self.toJointIndex(jointName)
            jointValue = self.getJointValue(jointIndex)
            label = self.labelMap[slider]
            label.text = str('%.1f' % math.degrees(jointValue)).center(5, ' ')

    def updateSliders(self):

        for jointName, slider in self.slidersMap.iteritems():
            jointIndex = self.toJointIndex(jointName)
            jointValue = self.getJointValue(jointIndex)

            slider.blockSignals(True)
            slider.setValue(self.toSliderValue(jointIndex, jointValue)*99)
            slider.blockSignals(False)

        self.updateLabels()


class TeleopPanel(object):

    def __init__(self, robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController, ikPlanner, manipPlanner, lhandModel, rhandModel, showPlanFunction):

        self.robotStateModel = robotStateModel
        self.robotStateJointController = robotStateJointController
        self.teleopRobotModel = teleopRobotModel
        self.teleopJointController = teleopJointController
        self.lhandModel = lhandModel
        self.rhandModel = rhandModel
        self.ikPlanner = ikPlanner
        self.manipPlanner = manipPlanner
        self.showPlanFunction = showPlanFunction

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddTeleopPanel.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)
        uifile.close()

        self.ui = WidgetDict(self.widget.children())

        self.endEffectorTeleop = EndEffectorTeleopPanel(self)
        self.jointTeleop = JointTeleopPanel(self)

    def disableJointTeleop(self):
        self.ui.jointTeleopFrame.setEnabled(False)

    def disableEndEffectorTeleop(self):
        self.ui.endEffectorTeleopFrame.setEnabled(False)

    def jointTeleopActivated(self):
        self.disableEndEffectorTeleop()

    def endEffectorTeleopActivated(self):
        self.disableJointTeleop()

    def endEffectorTeleopDeactivated(self):
        self.hideTeleopModel()
        self.enablePanels()

    def jointTeleopDeactivated(self):
        self.hideTeleopModel()
        self.enablePanels()

    def enablePanels(self):
        self.ui.endEffectorTeleopFrame.setEnabled(True)
        self.ui.jointTeleopFrame.setEnabled(True)

    def hideTeleopModel(self):
        self.teleopRobotModel.setProperty('Visible', False)
        self.robotStateModel.setProperty('Visible', True)
        self.robotStateModel.setProperty('Alpha', 1.0)

    def showTeleopModel(self):
        self.teleopRobotModel.setProperty('Visible', True)
        self.robotStateModel.setProperty('Visible', True)
        self.robotStateModel.setProperty('Alpha', 0.1)

    def showPose(self, pose):
        self.teleopJointController.setPose('teleop_pose', pose)
        self.showTeleopModel()

    def showPlan(self):
        plan = self.ikPlanner.lastManipPlan
        self.hideTeleopModel()
        self.showPlanFunction(plan)


def init(robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController, debrisPlanner, manipPlanner, lhandModel, rhandModel, showPlanFunction):
    global panel
    panel = TeleopPanel(robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController, debrisPlanner, manipPlanner, lhandModel, rhandModel, showPlanFunction)
    dock = app.addWidgetToDock(panel.widget)
    dock.hide()
