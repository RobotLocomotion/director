import json
from PythonQt import QtCore, QtGui, QtUiTools
import ddapp.applogic as app
import ddapp.objectmodel as om
from ddapp.timercallback import TimerCallback
from ddapp import robotstate
from ddapp import visualization as vis
from ddapp import transformUtils
from ddapp import ikplanner
from ddapp import footstepsdriver
from ddapp import vtkAll as vtk
from ddapp import drcargs
import ddapp.applogic as app

import math
import numpy as np


with open(drcargs.args().urdf_config) as urdf_config_file:
    jointMap = json.load(urdf_config_file)['teleopJointMap']

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


class ConstraintItem(om.ObjectModelItem):

    def __init__(self, constraint):

        linkStr = '(%s)' % constraint.linkName if hasattr(constraint, 'linkName') else ''
        name = '%s %s' % (type(constraint).__name__, linkStr)

        om.ObjectModelItem.__init__(self, name)
        self.constraint = constraint

        for propertyName, propertyValue in constraint:

            if isinstance(propertyValue, np.ndarray):
                propertyValue = propertyValue.tolist()

            if isinstance(propertyValue, vtk.vtkTransform):
                propertyValue = list(propertyValue.GetPosition()) + list(propertyValue.GetOrientation())

            self.addProperty(propertyName, propertyValue, attributes=om.PropertyAttributes(decimals=3, minimum=-100, maximum=100))

    def _onPropertyChanged(self, propertySet, propertyName):
        om.ObjectModelItem._onPropertyChanged(self, propertySet, propertyName)
        self.constraint.__setattr__(propertyName, propertySet.getProperty(propertyName))


class EndEffectorTeleopPanel(object):

    def __init__(self, panel):
        self.panel = panel
        self.ui = panel.ui
        self.ui.eeTeleopButton.connect('clicked()', self.teleopButtonClicked)
        self.ui.planButton.connect('clicked()', self.planClicked)
        self.ui.updateIkButton.connect('clicked()', self.onUpdateIkClicked)
        self.ui.baseCombo.connect('currentIndexChanged(const QString&)', self.baseComboChanged)
        self.ui.backCombo.connect('currentIndexChanged(const QString&)', self.backComboChanged)
        self.ui.lhandCombo.connect('currentIndexChanged(const QString&)', self.lhandComboChanged)
        self.ui.rhandCombo.connect('currentIndexChanged(const QString&)', self.rhandComboChanged)
        self.ui.lfootCombo.connect('currentIndexChanged(const QString&)', self.lfootComboChanged)
        self.ui.rfootCombo.connect('currentIndexChanged(const QString&)', self.rfootComboChanged)

        self.constraintSet = None

        #self.ui.interactiveCheckbox.visible = False
        #self.ui.updateIkButton.visible = False


    def setComboText(self, combo, text):
        index = combo.findText(text)
        assert index >= 0
        combo.setCurrentIndex(index)

    def getComboText(self, combo):
        return str(combo.currentText)

    def getBaseConstraint(self):
        return self.getComboText(self.ui.baseCombo)

    def setBaseConstraint(self, value):
        return self.setComboText(self.ui.baseCombo, value)

    def getBackConstraint(self):
        return self.getComboText(self.ui.backCombo)

    def setBackConstraint(self, value):
        return self.setComboText(self.ui.backCombo, value)

    def getLHandConstraint(self):
        return self.getComboText(self.ui.lhandCombo)

    def setLHandConstraint(self, value):
        return self.setComboText(self.ui.lhandCombo, value)

    def getRHandConstraint(self):
        return self.getComboText(self.ui.rhandCombo)

    def setRHandConstraint(self, value):
        return self.setComboText(self.ui.rhandCombo, value)

    def getLFootConstraint(self):
        return self.getComboText(self.ui.lfootCombo)

    def setLFootConstraint(self, value):
        return self.setComboText(self.ui.lfootCombo, value)

    def getRFootConstraint(self):
        return self.getComboText(self.ui.rfootCombo)

    def setRFootConstraint(self, value):
        return self.setComboText(self.ui.rfootCombo, value)

    def baseComboChanged(self):
        self.updateConstraints()

    def backComboChanged(self):
        self.updateConstraints()

    def lhandComboChanged(self):
        self.updateConstraints()

    def rhandComboChanged(self):
        self.updateConstraints()

    def lfootComboChanged(self):
        self.updateConstraints()

    def rfootComboChanged(self):
        self.updateConstraints()

    def onGoalFrameModified(self, frame):
        if self.constraintSet and self.ui.interactiveCheckbox.checked:
            self.updateIk()

    def onUpdateIkClicked(self):
        self.updateIk()

    def updateIk(self):
        endPose, info = self.constraintSet.runIk()
        self.panel.showPose(self.constraintSet.endPose)
        app.displaySnoptInfo(info)


    def planClicked(self):
        if not self.ui.eeTeleopButton.checked:
            return
        self.generatePlan()

    def generatePlan(self):

        self.updateConstraints()
        if not self.ui.interactiveCheckbox.checked:
            self.updateIk()

        # todo- need an option here
        goalMode = ikplanner.getIkOptions().getProperty('Goal planning mode')
        if goalMode == 1 or ikplanner.getIkOptions().getProperty('Use collision'):
            plan = self.constraintSet.runIkTraj()
        else:
            plan = self.constraintSet.planEndPoseGoal()

        self.panel.showPlan(plan)

    def teleopButtonClicked(self):
        if self.ui.eeTeleopButton.checked:
            self.activate()
        else:
            self.deactivate()

    def activate(self):
        self.ui.eeTeleopButton.blockSignals(True)
        self.ui.eeTeleopButton.checked = True
        self.ui.eeTeleopButton.blockSignals(False)
        self.panel.endEffectorTeleopActivated()

        self.createGoalFrames()
        self.updateConstraints()

    def deactivate(self):
        self.ui.eeTeleopButton.blockSignals(True)
        self.ui.eeTeleopButton.checked = False
        self.ui.eeTeleopButton.blockSignals(False)
        self.removePlanFolder()
        self.panel.endEffectorTeleopDeactivated()


    def getGoalFrame(self, linkName):
        return om.findObjectByName('%s constraint frame' % linkName)


    def updateGoalFrame(self, linkName, transform):
        goalFrame = self.getGoalFrame(linkName)
        if not goalFrame:
            return

        goalFrame.copyFrame(transform)
        return goalFrame



    def updateConstraints(self):

        if not self.ui.eeTeleopButton.checked:
            return


        ikPlanner = self.panel.ikPlanner

        startPoseName = 'reach_start'
        startPose = np.array(self.panel.robotStateJointController.q)
        ikPlanner.addPose(startPose, startPoseName)


        constraints = []
        constraints.append(ikPlanner.createQuasiStaticConstraint())

        if self.getLFootConstraint() == 'fixed':
            constraints.extend(ikPlanner.createFixedLinkConstraints(startPoseName, 'l_foot', tspan=[0.0, 1.0]))
        elif self.getLFootConstraint() == 'sliding':
            constraints.extend(ikPlanner.createSlidingFootConstraints(startPoseName)[:2])

        if self.getRFootConstraint() == 'fixed':
            constraints.extend(ikPlanner.createFixedLinkConstraints(startPoseName, 'r_foot', tspan=[0.0, 1.0]))
        elif self.getRFootConstraint() == 'sliding':
            constraints.extend(ikPlanner.createSlidingFootConstraints(startPoseName)[2:])


        if self.getBackConstraint() == 'fixed':
            constraints.append(ikPlanner.createLockedBackPostureConstraint(startPoseName))
            ikPlanner.setBackLocked(True)
        elif self.getBackConstraint() == 'limited':
            constraints.append(ikPlanner.createMovingBackLimitedPostureConstraint())
            ikPlanner.setBackLocked(False)
        elif self.getBackConstraint() == 'free':
            constraints.append(ikPlanner.createMovingBackPostureConstraint())
            ikPlanner.setBackLocked(False)


        if self.getBaseConstraint() == 'fixed':
            constraints.append(ikPlanner.createLockedBasePostureConstraint(startPoseName, lockLegs=False))
            ikPlanner.setBaseLocked(True)
        if self.getBaseConstraint() == 'constrained':
            constraints.extend(ikPlanner.createFixedLinkConstraints(startPoseName, 'pelvis', tspan=[1.0, 1.0]))
            ikPlanner.setBaseLocked(False)
        elif self.getBaseConstraint() == 'xyz only':
            constraints.append(ikPlanner.createXYZMovingBasePostureConstraint(startPoseName))
            constraints.append(ikPlanner.createKneePostureConstraint([0.6, 2.5]))
            ikPlanner.setBaseLocked(False)
        elif self.getBaseConstraint() == 'z only':
            constraints.append(ikPlanner.createZMovingBasePostureConstraint(startPoseName))
            constraints.append(ikPlanner.createKneePostureConstraint([0.6, 2.5]))
            ikPlanner.setBaseLocked(False)
        elif self.getBaseConstraint() == 'limited':
            constraints.append(ikPlanner.createMovingBaseSafeLimitsConstraint())
            constraints.append(ikPlanner.createKneePostureConstraint([0.6, 2.5]))
            ikPlanner.setBaseLocked(False)
        elif self.getBaseConstraint() == 'free':
            constraints.append(ikPlanner.createKneePostureConstraint([0.6, 2.5]))
            ikPlanner.setBaseLocked(False)


        side = 'left'
        linkName = ikPlanner.getHandLink(side)
        graspToPalm = vtk.vtkTransform()
        graspToHand = ikPlanner.newGraspToHandFrame(side, graspToPalm)
        #graspToWorld = ikPlanner.newGraspToWorldFrame(startPose, side, graspToHand)
        graspToWorld = self.getGoalFrame(linkName)

        p, q = ikPlanner.createPositionOrientationGraspConstraints(side, graspToWorld, graspToHand)
        g = ikPlanner.createGazeGraspConstraint(side, graspToWorld, graspToHand)

        p.tspan = [1.0, 1.0]
        q.tspan = [1.0, 1.0]
        g.tspan = [1.0, 1.0]



        if self.getLHandConstraint() == 'arm fixed':
            constraints.append(ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
            ikPlanner.setArmLocked(side,True)

        elif self.getLHandConstraint() == 'ee fixed':
            constraints.extend([p, q])
            ikPlanner.setArmLocked(side,False)

        elif self.getLHandConstraint() == 'position':
            constraints.extend([p])
            ikPlanner.setArmLocked(side,False)

        elif self.getLHandConstraint() == 'gaze':
            constraints.extend([p, g])
            ikPlanner.setArmLocked(side,False)

        elif self.getLHandConstraint() == 'orbit':
            graspToHand = ikPlanner.newPalmOffsetGraspToHandFrame(side, distance=0.07)
            constraints.extend(ikPlanner.createGraspOrbitConstraints(side, graspToWorld, graspToHand))
            constraints[-3].tspan = [1.0, 1.0]

            if ikPlanner.ikServer.useCollision:
                constraints[-2].tspan = [0.5, 1.0]
                constraints[-1].tspan = [0.5, 1.0]
            else:
                constraints[-2].tspan = [1.0, 1.0]
                constraints[-1].tspan = [1.0, 1.0]

            ikPlanner.setArmLocked(side,False)

        elif self.getLHandConstraint() == 'free':
            ikPlanner.setArmLocked(side,False)


        if self.getLHandConstraint() != 'free' and hasattr(self,'reachTargetObject'):
            constraints.append(ikPlanner.createExcludeReachTargetCollisionGroupConstraint(self.reachTargetObject.getProperty('Name')))


        side = 'right'
        linkName = ikPlanner.getHandLink(side)
        graspToPalm = vtk.vtkTransform()
        graspToHand = ikPlanner.newGraspToHandFrame(side, graspToPalm)
        #graspToWorld = ikPlanner.newGraspToWorldFrame(startPose, side, graspToHand)
        graspToWorld = self.getGoalFrame(linkName)

        p, q = ikPlanner.createPositionOrientationGraspConstraints(side, graspToWorld, graspToHand)
        g = ikPlanner.createGazeGraspConstraint(side, graspToWorld, graspToHand)

        p.tspan = [1.0, 1.0]
        q.tspan = [1.0, 1.0]
        g.tspan = [1.0, 1.0]



        if self.getRHandConstraint() == 'arm fixed':
            constraints.append(ikPlanner.createLockedRightArmPostureConstraint(startPoseName))
            ikPlanner.setArmLocked(side,True)

        elif self.getRHandConstraint() == 'ee fixed':
            constraints.extend([p, q])
            ikPlanner.setArmLocked(side,False)

        elif self.getRHandConstraint() == 'position':
            constraints.extend([p])
            ikPlanner.setArmLocked(side,False)

        elif self.getRHandConstraint() == 'gaze':
            constraints.extend([p, g])
            ikPlanner.setArmLocked(side,False)

        elif self.getRHandConstraint() == 'orbit':
            graspToHand = ikPlanner.newPalmOffsetGraspToHandFrame(side, distance=0.07)
            constraints.extend(ikPlanner.createGraspOrbitConstraints(side, graspToWorld, graspToHand))
            constraints[-3].tspan = [1.0, 1.0]

            if ikPlanner.ikServer.useCollision:
                constraints[-2].tspan = [0.5, 1.0]
                constraints[-1].tspan = [0.5, 1.0]
            else:
                constraints[-2].tspan = [1.0, 1.0]
                constraints[-1].tspan = [1.0, 1.0]

            ikPlanner.setArmLocked(side,False)

        elif self.getLHandConstraint() == 'free':
            ikPlanner.setArmLocked(side,False)


        if self.getRHandConstraint() != 'free' and hasattr(self,'reachTargetObject'):
            constraints.append(ikPlanner.createExcludeReachTargetCollisionGroupConstraint(self.reachTargetObject.getProperty('Name')))


        self.constraintSet = ikplanner.ConstraintSet(ikPlanner, constraints, 'reach_end', startPoseName)


        for constraint in constraints:
            if hasattr(constraint, 'linkName') and constraint.linkName in ('l_hand', 'r_hand'):
                continue

            if isinstance(constraint, ikplanner.ik.PositionConstraint):
                frameObj = self.getGoalFrame(constraint.linkName)
                if frameObj:
                    constraint.referenceFrame = frameObj.transform

            elif isinstance(constraint, ikplanner.ik.QuatConstraint):
                frameObj = self.getGoalFrame(constraint.linkName)
                if frameObj:
                    constraint.quaternion = frameObj.transform

            elif isinstance(constraint, ikplanner.ik.WorldGazeDirConstraint):
                frameObj = self.getGoalFrame(constraint.linkName)
                if frameObj:
                    constraint.targetFrame = frameObj.transform


        self.onGoalFrameModified(None)


        om.removeFromObjectModel(self.getConstraintFolder())
        folder = self.getConstraintFolder()

        for i, pc in enumerate(constraints):
            constraintItem = ConstraintItem(pc)
            om.addToObjectModel(constraintItem, parentObj=folder)




    def addHandMesh(self, handModel, goalFrame):

        handObj = handModel.newPolyData('reach goal left hand', self.panel.teleopRobotModel.views[0], parent=goalFrame)
        handFrame = handObj.children()[0]
        handFrame.copyFrame(goalFrame.transform)

        frameSync = vis.FrameSync()
        frameSync.addFrame(goalFrame)
        frameSync.addFrame(handFrame)
        goalFrame.sync = frameSync

        #handModels = {'left':self.panel.lhandModel, 'right':self.panel.rhandModel}


    def removePlanFolder(self):
        om.removeFromObjectModel(om.findObjectByName('teleop plan'))

    def getConstraintFrameFolder(self):
        return om.getOrCreateContainer('constraint frames', parentObj=om.getOrCreateContainer('teleop plan', parentObj=om.findObjectByName('planning')))

    def getConstraintFolder(self):
        return om.getOrCreateContainer('ik constraints', parentObj=om.getOrCreateContainer('teleop plan', parentObj=om.findObjectByName('planning')))

    def createGoalFrames(self):

        ikPlanner = self.panel.ikPlanner
        startPose = np.array(self.panel.robotStateJointController.q)

        self.removePlanFolder()
        folder = self.getConstraintFrameFolder()

        for side in ['left', 'right']:

            linkName = ikPlanner.getHandLink(side)
            frameName = '%s constraint frame' % linkName

            graspToPalm = vtk.vtkTransform()
            graspToHand = ikPlanner.newGraspToHandFrame(side, graspToPalm)
            graspToWorld = ikPlanner.newGraspToWorldFrame(startPose, side, graspToHand)

            om.removeFromObjectModel(om.findObjectByName(frameName))
            frame = vis.showFrame(graspToWorld, frameName, parent=folder, scale=0.2)
            #frame.setProperty('Edit', True)
            frame.connectFrameModified(self.onGoalFrameModified)
            #addHandMesh(handModels[side], frame)


        #for linkName in ['l_foot', 'r_foot', 'pelvis', 'utorso', 'head']:
        for linkName in ['l_foot', 'r_foot', 'pelvis']:
            frameName = linkName + ' constraint frame'
            om.removeFromObjectModel(om.findObjectByName(frameName))
            frame = vis.showFrame(ikPlanner.getLinkFrameAtPose(linkName, startPose), frameName, parent=folder, scale=0.2)
            frame.connectFrameModified(self.onGoalFrameModified)


    def newReachTeleop(self, frame, side, reachTargetObject=None):
        '''
        reachTarget is the object we are reaching to.  For some types of plans
        this object may be treated in a special way, for example, when doing
        planning with collision avoidance.
        '''
        self.deactivate()
        self.panel.jointTeleop.deactivate()

        self.setBaseConstraint('xyz only')
        self.setBackConstraint('limited')
        self.setLFootConstraint('fixed')
        self.setRFootConstraint('fixed')

        self.setLHandConstraint('arm fixed')
        self.setRHandConstraint('arm fixed')

        if side == 'left':
          if self.panel.ikPlanner.ikServer.useCollision:
            self.setLHandConstraint('orbit')
          else:
            self.setLHandConstraint('ee fixed')
        elif side == 'right':
          if self.panel.ikPlanner.ikServer.useCollision:
            self.setRHandConstraint('orbit')
          else:
            self.setRHandConstraint('ee fixed')

        self.reachTargetObject = reachTargetObject
        self.activate()
        return self.updateGoalFrame(self.panel.ikPlanner.getHandLink(side), frame)



class JointLimitChecker(object):

    def __init__(self, robotModel, sensorJointController):

        self.robotModel = robotModel
        self.sensorJointController = sensorJointController
        self.jointLimitsMin = np.array([self.robotModel.model.getJointLimits(jointName)[0] for jointName in robotstate.getDrakePoseJointNames()])
        self.jointLimitsMax = np.array([self.robotModel.model.getJointLimits(jointName)[1] for jointName in robotstate.getDrakePoseJointNames()])
        self.joints = robotstate.matchJoints('^(?!base_)') # all but base joints
        self.inflationAmount = np.radians(0.1)
        self.timer = TimerCallback(targetFps=1)
        self.timer.callback = self.update
        self.action = None

    def update(self):
        self.checkJointLimits()

    def start(self):
        self.action.checked = True
        self.timer.start()

    def stop(self):
        self.action.checked = False
        self.timer.stop()

    def setupMenuAction(self):
        self.action = app.addMenuAction('Tools', 'Joint Limit Checker')
        self.action.setCheckable(True)
        self.action.checked = self.timer.isActive()
        self.action.connect('triggered()', self.onActionChanged)

    def onActionChanged(self):
        if self.action.checked:
            self.start()
        else:
            self.stop()

    def notifyUser(self, limitData):

        message = '\n'.join(['%s by %.2f degrees' % (name, np.degrees(epsilon)) for name, epsilon in limitData])
        message = 'The following joints have been detected to exceed joint limts specified by the model:\n\n' + message + '\n\n'
        message += 'Would to like to update the joint limits used by the planning robot model?  If you select no '\
                   'then the joint limit checker will be disabled (use the Tools menu to re-enable).'

        choice = QtGui.QMessageBox.warning(app.getMainWindow(), 'Joint Limit Exceeded', message,
                  QtGui.QMessageBox.Yes | QtGui.QMessageBox.No,
                  QtGui.QMessageBox.Yes)

        if choice == QtGui.QMessageBox.No:
            self.stop()
        else:

            # inflate the epsilon
            limitData = [(jointName, epsilon+np.sign(epsilon)*self.inflationAmount) for jointName, epsilon in limitData]

            # update limits on server
            panel.ikPlanner.ikServer.updateJointLimits(limitData)

            # update limits on checker
            for jointName, epsilon in limitData:
                limitsArray = self.jointLimitsMin if epsilon < 0 else self.jointLimitsMax
                limitsArray[self.toJointIndex(jointName)] += epsilon

    def checkJointLimits(self):

        limitData = []

        for jointName in self.joints:
            jointIndex = self.toJointIndex(jointName)
            jointPosition = self.sensorJointController.q[jointIndex]
            jointMin, jointMax = self.jointLimitsMin[jointIndex], self.jointLimitsMax[jointIndex]
            if not (jointMin <= jointPosition <= jointMax):
                epsilon = jointPosition - np.clip(jointPosition, jointMin, jointMax)
                #print 'detected joint outside limit:', jointName, ' by %.3f degrees' % np.degrees(epsilon)
                limitData.append((jointName, epsilon))

        if limitData:
            self.notifyUser(limitData)

    def toJointIndex(self, jointName):
        return robotstate.getDrakePoseJointNames().index(jointName)



class JointTeleopPanel(object):

    def __init__(self, panel):
        self.panel = panel
        self.ui = panel.ui
        self.ui.jointTeleopButton.connect('clicked()', self.teleopButtonClicked)
        self.ui.resetJointsButton.connect('clicked()', self.resetButtonClicked)
        self.ui.planButton.connect('clicked()', self.planClicked)

        self.timerCallback = TimerCallback()
        self.timerCallback.callback = self.onTimerCallback

        self.jointLimitsMin = np.array([self.panel.teleopRobotModel.model.getJointLimits(jointName)[0] for jointName in robotstate.getDrakePoseJointNames()])
        self.jointLimitsMax = np.array([self.panel.teleopRobotModel.model.getJointLimits(jointName)[1] for jointName in robotstate.getDrakePoseJointNames()])

        self.jointLimitsMin[0:6] = [-0.25, -0.25, 0.61, -math.radians(20),  -math.radians(20),  -math.radians(20)]
        self.jointLimitsMax[0:6] = [0.25, 0.25, 0.92, math.radians(20),  math.radians(20),  math.radians(20)]

        self.slidersMap = {
            jointMap['backRoll'] : self.ui.backRollSlider,
            jointMap['backPitch'] : self.ui.backPitchSlider,
            jointMap['backYaw'] : self.ui.backYawSlider,

            jointMap['baseX'] : self.ui.baseXSlider,
            jointMap['baseY'] : self.ui.baseYSlider,
            jointMap['baseZ'] : self.ui.baseZSlider,

            jointMap['baseRoll'] : self.ui.baseRollSlider,
            jointMap['basePitch'] : self.ui.basePitchSlider,
            jointMap['baseYaw'] : self.ui.baseYawSlider,

            jointMap['leftShoulderX'] : self.ui.leftShoulderXSlider,
            jointMap['leftShoulderY'] : self.ui.leftShoulderYSlider,
            jointMap['leftShoulderZ'] : self.ui.leftShoulderZSlider,
            jointMap['leftElbow'] : self.ui.leftElbowSlider,
            jointMap['leftWristX'] : self.ui.leftWristXSlider,
            jointMap['leftWristY'] : self.ui.leftWristYSlider,

            jointMap['rightShoulderX'] : self.ui.rightShoulderXSlider,
            jointMap['rightShoulderY'] : self.ui.rightShoulderYSlider,
            jointMap['rightShoulderZ'] : self.ui.rightShoulderZSlider,
            jointMap['rightElbow'] : self.ui.rightElbowSlider,
            jointMap['rightWristX'] : self.ui.rightWristXSlider,
            jointMap['rightWristY'] : self.ui.rightWristYSlider,
        }

        self.labelMap = {
            self.ui.backRollSlider : self.ui.backRollLabel,
            self.ui.backPitchSlider : self.ui.backPitchLabel,
            self.ui.backYawSlider : self.ui.backYawLabel,

            self.ui.baseXSlider : self.ui.baseXLabel,
            self.ui.baseYSlider : self.ui.baseYLabel,
            self.ui.baseZSlider : self.ui.baseZLabel,

            self.ui.baseRollSlider : self.ui.baseRollLabel,
            self.ui.basePitchSlider : self.ui.basePitchLabel,
            self.ui.baseYawSlider : self.ui.baseYawLabel,

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

        self.sliderMax = 1000.0
        for jointName, slider in self.slidersMap.iteritems():
            slider.connect('valueChanged(int)', self.signalMapper, 'map()')
            self.signalMapper.setMapping(slider, jointName)
            slider.setMaximum(self.sliderMax)

        self.signalMapper.connect('mapped(const QString&)', self.sliderChanged)

        self.startPose = None
        self.endPose = None
        self.userJoints = {}

        self.updateWidgetState()


    def planClicked(self):
        if not self.ui.jointTeleopButton.checked:
            return

        self.computeEndPose()
        self.generatePlan()


    def generatePlan(self):
        plan = self.panel.ikPlanner.computePostureGoal(self.startPose, self.endPose)
        self.panel.showPlan(plan)


    def teleopButtonClicked(self):
        if self.ui.jointTeleopButton.checked:
            self.activate()
        else:
            self.deactivate()


    def activate(self):
        self.timerCallback.stop()
        self.panel.jointTeleopActivated()
        self.resetPose()
        self.updateWidgetState()


    def deactivate(self):
        self.ui.jointTeleopButton.blockSignals(True)
        self.ui.jointTeleopButton.checked = False
        self.ui.jointTeleopButton.blockSignals(False)
        self.timerCallback.stop()
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
        self.resetPose()
        self.panel.showPose(self.endPose)

    def resetPose(self):
        self.userJoints = {}
        self.computeEndPose()
        self.updateSliders()

    def onTimerCallback(self):
        if not self.ui.tabWidget.visible:
            return
        self.resetPose()

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

    def computeBaseJointOffsets(self):

        baseReferenceFrame = footstepsdriver.FootstepsDriver.getFeetMidPoint(self.panel.ikPlanner.getRobotModelAtPose(self.startPose))
        baseReferenceWorldPos = np.array(baseReferenceFrame.GetPosition())
        baseReferenceWorldYaw = math.radians(baseReferenceFrame.GetOrientation()[2])

        self.baseJointOffsets = {
          'base_x'   : baseReferenceWorldPos[0],
          'base_y'   : baseReferenceWorldPos[1],
          'base_z'   : baseReferenceWorldPos[2],
          'base_yaw' : baseReferenceWorldYaw,
          }


    def computeEndPose(self):

        self.startPose = np.array(self.panel.robotStateJointController.q)
        self.endPose = self.startPose.copy()

        hasBase = False
        for jointIndex, jointValue in self.userJoints.iteritems():
            jointName = self.toJointName(jointIndex)
            self.endPose[jointIndex] = jointValue
            if 'base' in jointName:
                hasBase = True

        if hasBase:

            ikPlanner = self.panel.ikPlanner

            startPoseName = 'posture_goal_start'
            ikPlanner.addPose(self.startPose, startPoseName)

            endPoseName = 'posture_goal_end'
            ikPlanner.addPose(self.endPose, endPoseName)

            jointNames = self.slidersMap.keys()

            # uncomment to constraint only joints adjusted by user
            #jointNames = [self.toJointName(jointIndex) for jointIndex in sorted(self.userJoints.keys())]

            p = ikPlanner.createPostureConstraint(endPoseName, jointNames)

            constraints = [p]
            constraints.extend(ikPlanner.createFixedFootConstraints(startPoseName))
            constraints.append(ikPlanner.createQuasiStaticConstraint())

            self.endPose, info = ikPlanner.ikServer.runIk(constraints, nominalPostureName=startPoseName, seedPostureName='q_end')
            app.displaySnoptInfo(info)


    def getJointValue(self, jointIndex):
        return self.endPose[jointIndex]


    def sliderChanged(self, jointName):

        slider = self.slidersMap[jointName]
        jointIndex = self.toJointIndex(jointName)
        jointValue = self.toJointValue(jointIndex, slider.value / float(self.sliderMax))

        self.userJoints[jointIndex] = jointValue
        if 'base' in jointName:
            self.computeBaseJointOffsets()
            self.userJoints[jointIndex] += self.baseJointOffsets.get(jointName, 0.0)

        self.computeEndPose()
        self.panel.showPose(self.endPose)
        self.updateLabel(jointName, jointValue)

    def updateLabel(self, jointName, jointValue):

        slider = self.slidersMap[jointName]
        label = self.labelMap[slider]

        if jointName in ['base_x', 'base_y', 'base_z']:
            label.text = str('%.3f' % jointValue).center(5, ' ')
        else:
            label.text = str('%.1f' % math.degrees(jointValue)).center(5, ' ')


    def updateSliders(self):

        self.computeBaseJointOffsets()

        for jointName, slider in self.slidersMap.iteritems():
            jointIndex = self.toJointIndex(jointName)
            jointValue = self.getJointValue(jointIndex) - self.baseJointOffsets.get(jointName, 0.0)

            slider.blockSignals(True)
            slider.setValue(self.toSliderValue(jointIndex, jointValue)*self.sliderMax)
            slider.blockSignals(False)
            self.updateLabel(jointName, jointValue)


class TeleopPanel(object):

    def __init__(self, robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController, ikPlanner, manipPlanner, lhandModel, rhandModel, showPlanFunction, hidePlanFunction):

        self.robotStateModel = robotStateModel
        self.robotStateJointController = robotStateJointController
        self.teleopRobotModel = teleopRobotModel
        self.teleopJointController = teleopJointController
        self.lhandModel = lhandModel
        self.rhandModel = rhandModel
        self.ikPlanner = ikPlanner
        self.manipPlanner = manipPlanner
        self.showPlanFunction = showPlanFunction
        self.hidePlanFunction = hidePlanFunction

        manipPlanner.connectPlanCommitted(self.onPlanCommitted)

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddTeleopPanel.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)
        uifile.close()

        self.ui = WidgetDict(self.widget.children())
        self.ui.postureDatabaseButton.connect('clicked()', self.onPostureDatabaseClicked)

        self.endEffectorTeleop = EndEffectorTeleopPanel(self)
        self.jointTeleop = JointTeleopPanel(self)

    def onPostureDatabaseClicked(self):
        ikplanner.RobotPoseGUIWrapper.show()

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

    def onPlanCommitted(self, plan):
        self.hideTeleopModel()

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
        self.hidePlanFunction()
        self.showTeleopModel()

    def showPlan(self, plan):
        self.hideTeleopModel()
        self.showPlanFunction(plan)


def _getAction():
    return app.getToolBarActions()['ActionTeleopPanel']


def init(robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController, debrisPlanner, manipPlanner, lhandModel, rhandModel, showPlanFunction, hidePlanFunction):

    global panel
    global dock

    panel = TeleopPanel(robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController, debrisPlanner, manipPlanner, lhandModel, rhandModel, showPlanFunction, hidePlanFunction)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel
