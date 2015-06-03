import PythonQt
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
from ddapp import affordanceurdf
import ddapp.applogic as app

import functools
import math
import numpy as np
import types


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
        self.ui.leftFootPlanningSupportCheckbox.connect('toggled(bool)', self.leftFootPlanningSupportCheckboxChanged)
        self.ui.rightFootPlanningSupportCheckbox.connect('toggled(bool)', self.rightFootPlanningSupportCheckboxChanged)
        self.ui.leftHandPlanningSupportCheckbox.connect('toggled(bool)', self.leftHandPlanningSupportCheckboxChanged)
        self.ui.rightHandPlanningSupportCheckbox.connect('toggled(bool)', self.rightHandPlanningSupportCheckboxChanged)
        self.ui.pelvisPlanningSupportCheckbox.connect('toggled(bool)', self.pelvisPlanningSupportCheckboxChanged)
        self.ui.leftFootExecutionSupportCheckbox.connect('toggled(bool)', self.leftFootExecutionSupportCheckboxChanged)
        self.ui.rightFootExecutionSupportCheckbox.connect('toggled(bool)', self.rightFootExecutionSupportCheckboxChanged)
        self.ui.leftHandExecutionSupportCheckbox.connect('toggled(bool)', self.leftHandExecutionSupportCheckboxChanged)
        self.ui.rightHandExecutionSupportCheckbox.connect('toggled(bool)', self.rightHandExecutionSupportCheckboxChanged)
        self.ui.pelvisExecutionSupportCheckbox.connect('toggled(bool)', self.pelvisExecutionSupportCheckboxChanged)
        self.ui.executionSupportCheckbox.connect('toggled(bool)', self.executionSupportCheckboxChanged)

        self.palmOffsetDistance = 0.0
        self.palmGazeAxis = [0.0, 1.0, 0.0]
        self.constraintSet = None

        #self.ui.interactiveCheckbox.visible = False
        #self.ui.updateIkButton.visible = False

        if 'kneeJointLimits' in drcargs.getDirectorConfig():
            self.kneeJointLimits = drcargs.getDirectorConfig()['kneeJointLimits']

    def setComboText(self, combo, text):
        index = combo.findText(text)
        assert index >= 0
        combo.setCurrentIndex(index)

    def getComboText(self, combo):
        return str(combo.currentText)

    def setCheckboxState(self, checkbox, state):
        assert type(state) is types.BooleanType
        checkbox.checked = state

    def getCheckboxState(self, checkbox):
        return checkbox.checked

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

    def getLFootPlanningSupportEnabled(self):
        return self.getCheckboxState(self.ui.leftFootPlanningSupportCheckbox)

    def setLFootPlanningSupportEnabled(self, value):
        self.setCheckboxState(self.ui.leftFootPlanningSupportCheckbox, value)

    def getRFootPlanningSupportEnabled(self):
        return self.getCheckboxState(self.ui.rightFootPlanningSupportCheckbox)

    def setRFootPlanningSupportEnabled(self, value):
        self.setCheckboxState(self.ui.rightFootPlanningSupportCheckbox, value)

    def getLHandPlanningSupportEnabled(self):
        return self.getCheckboxState(self.ui.leftHandPlanningSupportCheckbox)

    def setLHandPlanningSupportEnabled(self, value):
        self.setCheckboxState(self.ui.leftHandPlanningSupportCheckbox, value)

    def getRHandPlanningSupportEnabled(self):
        return self.getCheckboxState(self.ui.rightHandPlanningSupportCheckbox)

    def setRHandPlanningSupportEnabled(self, value):
        self.setCheckboxState(self.ui.rightHandPlanningSupportCheckbox, value)

    def getPelvisPlanningSupportEnabled(self):
        return self.getCheckboxState(self.ui.pelvisPlanningSupportCheckbox)

    def setPelvisPlanningSupportEnabled(self, value):
        self.setCheckboxState(self.ui.pelvisPlanningSupportCheckbox, value)

    def getLFootExecutionSupportEnabled(self):
        return self.getCheckboxState(self.ui.leftFootExecutionSupportCheckbox)

    def setLFootExecutionSupportEnabled(self, value):
        self.setCheckboxState(self.ui.leftFootExecutionSupportCheckbox, value)

    def getRFootExecutionSupportEnabled(self):
        return self.getCheckboxState(self.ui.rightFootExecutionSupportCheckbox)

    def setRFootExecutionSupportEnabled(self, value):
        self.setCheckboxState(self.ui.rightFootExecutionSupportCheckbox, value)

    def getLHandExecutionSupportEnabled(self):
        return self.getCheckboxState(self.ui.leftHandExecutionSupportCheckbox)

    def setLHandExecutionSupportEnabled(self, value):
        self.setCheckboxState(self.ui.leftHandExecutionSupportCheckbox, value)

    def getRHandExecutionSupportEnabled(self):
        return self.getCheckboxState(self.ui.rightHandExecutionSupportCheckbox)

    def setRHandExecutionSupportEnabled(self, value):
        self.setCheckboxState(self.ui.rightHandExecutionSupportCheckbox, value)

    def getPelvisExecutionSupportEnabled(self):
        return self.getCheckboxState(self.ui.pelvisExecutionSupportCheckbox)

    def setPelvisExecutionSupportEnabled(self, value):
        self.setCheckboxState(self.ui.pelvisExecutionSupportCheckbox, value)

    def getExecutionSupportEnabled(self):
        return self.getCheckboxState(self.ui.executionSupportCheckbox)

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

    def leftFootExecutionSupportCheckboxChanged(self):
        if not self.getLFootExecutionSupportEnabled():
            self.setLFootPlanningSupportEnabled(False)
        self.panel.manipPlanner.leftFootSupportEnabled = self.getLFootExecutionSupportEnabled()
        self.updateQuasistaticFlag()

    def rightFootExecutionSupportCheckboxChanged(self):
        if not self.getRFootExecutionSupportEnabled():
            self.setRFootPlanningSupportEnabled(False)
        self.panel.manipPlanner.rightFootSupportEnabled = self.getRFootExecutionSupportEnabled()
        self.updateQuasistaticFlag()

    def leftHandExecutionSupportCheckboxChanged(self):
        if not self.getLHandExecutionSupportEnabled():
            self.setLHandPlanningSupportEnabled(False)
        self.panel.manipPlanner.leftHandSupportEnabled = self.getLHandExecutionSupportEnabled()
        self.updateQuasistaticFlag()

    def rightHandExecutionSupportCheckboxChanged(self):
        if not self.getRHandExecutionSupportEnabled():
            self.setRHandPlanningSupportEnabled(False)
        self.panel.manipPlanner.rightHandSupportEnabled = self.getRHandExecutionSupportEnabled()
        self.updateQuasistaticFlag()

    def pelvisExecutionSupportCheckboxChanged(self):
        if not self.getPelvisExecutionSupportEnabled():
            self.setPelvisPlanningSupportEnabled(False)
        self.panel.manipPlanner.pelvisSupportEnabled = self.getPelvisExecutionSupportEnabled()
        self.updateQuasistaticFlag()

    def executionSupportCheckboxChanged(self):
        self.updateQuasistaticFlag()
        self.panel.manipPlanner.setPublishPlansWithSupports(self.getExecutionSupportEnabled())

    def leftFootPlanningSupportCheckboxChanged(self):
        if self.getLFootPlanningSupportEnabled():
            self.setLFootExecutionSupportEnabled(True)
        self.updatePlanningSupports()
        self.updateConstraints()

    def rightFootPlanningSupportCheckboxChanged(self):
        if self.getRFootPlanningSupportEnabled():
            self.setRFootExecutionSupportEnabled(True)
        self.updatePlanningSupports()
        self.updateConstraints()


    def leftHandPlanningSupportCheckboxChanged(self):
        if self.getLHandPlanningSupportEnabled():
            self.setLHandExecutionSupportEnabled(True)
        self.updatePlanningSupports()
        self.updateConstraints()


    def rightHandPlanningSupportCheckboxChanged(self):
        if self.getRHandPlanningSupportEnabled():
            self.setRHandExecutionSupportEnabled(True)
        self.updatePlanningSupports()
        self.updateConstraints()


    def pelvisPlanningSupportCheckboxChanged(self):
        if self.getPelvisPlanningSupportEnabled():
            self.setPelvisExecutionSupportEnabled(True)
        self.updatePlanningSupports()
        self.updateConstraints()


    def updateQuasistaticFlag(self):
        lfootEnabled = self.getLFootExecutionSupportEnabled()
        rfootEnabled = self.getRFootExecutionSupportEnabled()
        lhandEnabled = self.getLHandExecutionSupportEnabled()
        rhandEnabled = self.getRHandExecutionSupportEnabled()
        pelvisEnabled = self.getPelvisExecutionSupportEnabled()

        if (lhandEnabled or rhandEnabled or pelvisEnabled) or (lfootEnabled and rfootEnabled):
            self.panel.manipPlanner.plansWithSupportsAreQuasistatic = True
        else:
            self.panel.manipPlanner.plansWithSupportsAreQuasistatic = False


    def onGoalFrameModified(self, frame):
        if self.constraintSet and self.ui.interactiveCheckbox.checked:
            self.updateIk()

    def onUpdateIkClicked(self):
        self.updateIk()

    def updateIk(self):
        endPose, info = self.constraintSet.runIk()
        self.panel.showPose(self.constraintSet.endPose)
        app.displaySnoptInfo(info)


    def updateCollisionEnvironment(self):
        affs = self.panel.affordanceManager.getCollisionAffordances()
        if not affs:
            self.panel.ikPlanner.ikServer.clearEnvironment()
        else:
            urdfStr = affordanceurdf.urdfStringFromAffordances(affs)
            self.panel.ikPlanner.ikServer.setEnvironment(urdfStr)

    def planClicked(self):
        if not self.ui.eeTeleopButton.checked:
            return
        self.updateCollisionEnvironment()
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

    @staticmethod
    def getGoalFrame(linkName):
        return om.findObjectByName('%s constraint frame' % linkName)

    def updateGoalFrame(self, linkName, transform):
        goalFrame = self.getGoalFrame(linkName)
        if not goalFrame:
            return

        goalFrame.copyFrame(transform)
        return goalFrame


    def updatePlanningSupports(self):
        self.panel.ikPlanner.leftFootSupportEnabled = self.getLFootPlanningSupportEnabled()
        self.panel.ikPlanner.rightFootSupportEnabled = self.getRFootPlanningSupportEnabled()
        self.panel.ikPlanner.leftHandSupportEnabled = self.getLHandPlanningSupportEnabled()
        self.panel.ikPlanner.rightHandSupportEnabled = self.getRHandPlanningSupportEnabled()
        self.panel.ikPlanner.pelvisSupportEnabled = self.getPelvisPlanningSupportEnabled()

    def updateConstraints(self):

        if not self.ui.eeTeleopButton.checked:
            return


        self.updatePlanningSupports()
        ikPlanner = self.panel.ikPlanner

        startPoseName = 'reach_start'
        startPose = np.array(self.panel.robotStateJointController.q)
        ikPlanner.addPose(startPose, startPoseName)

        if (ikPlanner.fixedBaseArm==False):

            constraints = []
            constraints.append(ikPlanner.createQuasiStaticConstraint())
            constraints.append(ikPlanner.createLockedNeckPostureConstraint(startPoseName))

            if self.getLFootConstraint() == 'fixed':
                constraints.append(ikPlanner.createFixedLinkConstraints(startPoseName, ikPlanner.leftFootLink, tspan=[0.0, 1.0], lowerBound=-0.0001*np.ones(3), upperBound=0.0001*np.ones(3), angleToleranceInDegrees=0.1))
            elif self.getLFootConstraint() == 'constrained':
                constraints.extend(ikPlanner.createSixDofLinkConstraints(startPoseName, ikPlanner.leftFootLink, tspan=[1.0, 1.0]))
            elif self.getLFootConstraint() == 'sliding':
                constraints.extend(ikPlanner.createSlidingFootConstraints(startPoseName)[:2])

            if self.getRFootConstraint() == 'fixed':
                constraints.append(ikPlanner.createFixedLinkConstraints(startPoseName, ikPlanner.rightFootLink, tspan=[0.0, 1.0], lowerBound=-0.0001*np.ones(3), upperBound=0.0001*np.ones(3), angleToleranceInDegrees=0.1))
            elif self.getRFootConstraint() == 'constrained':
                constraints.extend(ikPlanner.createSixDofLinkConstraints(startPoseName, ikPlanner.rightFootLink, tspan=[1.0, 1.0]))
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
                constraints.extend(ikPlanner.createSixDofLinkConstraints(startPoseName, ikPlanner.pelvisLink, tspan=[1.0, 1.0]))
                ikPlanner.setBaseLocked(False)
            elif self.getBaseConstraint() == 'xyz only':
                constraints.append(ikPlanner.createXYZMovingBasePostureConstraint(startPoseName))
                constraints.append(ikPlanner.createKneePostureConstraint(self.kneeJointLimits))
                ikPlanner.setBaseLocked(False)
            elif self.getBaseConstraint() == 'z only':
                constraints.append(ikPlanner.createZMovingBasePostureConstraint(startPoseName))
                constraints.append(ikPlanner.createKneePostureConstraint(self.kneeJointLimits))
                ikPlanner.setBaseLocked(False)
            elif self.getBaseConstraint() == 'limited':
                constraints.append(ikPlanner.createMovingBaseSafeLimitsConstraint())
                constraints.append(ikPlanner.createKneePostureConstraint(self.kneeJointLimits))
                ikPlanner.setBaseLocked(False)
            elif self.getBaseConstraint() == 'free':
                constraints.append(ikPlanner.createKneePostureConstraint(self.kneeJointLimits))
                ikPlanner.setBaseLocked(False)

        # Remove all except the fixed base constraint if you only have an arm:
        else:
            constraints = []
            constraints.append(ikPlanner.createLockedBasePostureConstraint(startPoseName, lockLegs=False))


        if ikPlanner.robotNoFeet == True:
            constraints = []
            constraints.append(ikPlanner.createLockedBasePostureConstraint(startPoseName))
            if self.getBackConstraint() == 'fixed':
                constraints.append(ikPlanner.createLockedBackPostureConstraint(startPoseName))
                ikPlanner.setBackLocked(True)
            elif self.getBackConstraint() == 'limited':
                constraints.append(ikPlanner.createMovingBackLimitedPostureConstraint())
                ikPlanner.setBackLocked(False)
            elif self.getBackConstraint() == 'free':
                constraints.append(ikPlanner.createMovingBackPostureConstraint())
                ikPlanner.setBackLocked(False)


        for handModel in ikPlanner.handModels:
            side = handModel.side
            if (side == "left"):
                thisHandConstraint = self.getLHandConstraint()
            elif (side == "right"):
                thisHandConstraint = self.getRHandConstraint()

            linkName = ikPlanner.getHandLink(side)
            graspToHand = ikPlanner.newPalmOffsetGraspToHandFrame(side, self.palmOffsetDistance)
            graspToWorld = self.getGoalFrame(linkName)

            p, q = ikPlanner.createPositionOrientationGraspConstraints(side, graspToWorld, graspToHand)
            g = ikPlanner.createGazeGraspConstraint(side, graspToWorld, graspToHand, targetAxis=list(self.palmGazeAxis), bodyAxis=list(self.palmGazeAxis))

            p.tspan = [1.0, 1.0]
            q.tspan = [1.0, 1.0]
            g.tspan = [1.0, 1.0]

            if thisHandConstraint == 'arm fixed':
                if (side == "left"):
                    constraints.append(ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
                elif (side == "right"):
                    constraints.append(ikPlanner.createLockedRightArmPostureConstraint(startPoseName))
                ikPlanner.setArmLocked(side,True)

            elif thisHandConstraint == 'ee fixed':
                constraints.extend([p, q])
                ikPlanner.setArmLocked(side,False)

            elif thisHandConstraint == 'position':
                constraints.extend([p])
                ikPlanner.setArmLocked(side,False)

            elif thisHandConstraint == 'gaze':
                constraints.extend([p, g])
                ikPlanner.setArmLocked(side,False)

            elif thisHandConstraint == 'orbit':
                graspToHand = ikPlanner.newPalmOffsetGraspToHandFrame(side, distance=0.07)
                constraints.extend(ikPlanner.createGraspOrbitConstraints(side, graspToWorld, graspToHand))
                constraints[-3].tspan = [1.0, 1.0]

                if ikPlanner.defaultIkParameters.useCollision:
                    constraints[-2].tspan = [0.5, 1.0]
                    constraints[-1].tspan = [0.5, 1.0]
                else:
                    constraints[-2].tspan = [1.0, 1.0]
                    constraints[-1].tspan = [1.0, 1.0]

                ikPlanner.setArmLocked(side,False)

            elif thisHandConstraint == 'free':
                ikPlanner.setArmLocked(side,False)


        if hasattr(self,'reachSide'):
            if self.reachSide == 'left':
                endEffectorName = ikPlanner.handModels[0].handLinkName # 'l_hand'
            else:
                endEffectorName = ikPlanner.handModels[1].handLinkName # 'r_hand'

            constraints.append(ikPlanner.createActiveEndEffectorConstraint(endEffectorName,ikPlanner.getPalmPoint(self.reachSide)))


        self.constraintSet = ikplanner.ConstraintSet(ikPlanner, constraints, 'reach_end', startPoseName)


        handLinks = []
        for handModel in ikPlanner.handModels: handLinks.append(handModel.handLinkName)

        for constraint in constraints:
            if hasattr(constraint, 'linkName') and constraint.linkName in handLinks:
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

    @staticmethod
    def removePlanFolder():
        om.removeFromObjectModel(om.findObjectByName('teleop plan'))

    @staticmethod
    def getConstraintFrameFolder():
        return om.getOrCreateContainer('constraint frames', parentObj=om.getOrCreateContainer('teleop plan', parentObj=om.findObjectByName('planning')))

    @staticmethod
    def getConstraintFolder():
        return om.getOrCreateContainer('ik constraints', parentObj=om.getOrCreateContainer('teleop plan', parentObj=om.findObjectByName('planning')))

    def createGoalFrames(self):

        ikPlanner = self.panel.ikPlanner
        startPose = np.array(self.panel.robotStateJointController.q)

        self.removePlanFolder()
        folder = self.getConstraintFrameFolder()

        for handModel in ikPlanner.handModels:
            side = handModel.side

            linkName = ikPlanner.getHandLink(side)
            frameName = '%s constraint frame' % linkName

            graspToHand = ikPlanner.newPalmOffsetGraspToHandFrame(side, self.palmOffsetDistance)
            graspToWorld = ikPlanner.newGraspToWorldFrame(startPose, side, graspToHand)

            om.removeFromObjectModel(om.findObjectByName(frameName))
            frame = vis.showFrame(graspToWorld, frameName, parent=folder, scale=0.2)
            #frame.setProperty('Edit', True)
            frame.connectFrameModified(self.onGoalFrameModified)
            #addHandMesh(handModels[side], frame)

        if not ikPlanner.fixedBaseArm and not ikPlanner.robotNoFeet:
            for linkName in [ikPlanner.leftFootLink, ikPlanner.rightFootLink, ikPlanner.pelvisLink]:
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
          if self.panel.ikPlanner.defaultIkParameters.useCollision:
            self.setLHandConstraint('ee fixed')
          else:
            self.setLHandConstraint('ee fixed')
        elif side == 'right':
          if self.panel.ikPlanner.defaultIkParameters.useCollision:
            self.setRHandConstraint('ee fixed')
          else:
            self.setRHandConstraint('ee fixed')

        self.reachTargetObject = reachTargetObject
        self.reachSide = side
        self.activate()
        return self.updateGoalFrame(self.panel.ikPlanner.getHandLink(side), frame)


class PosturePlanShortcuts(object):

    def __init__(self, jointController, ikPlanner, widget=None):
        self.jointController = jointController
        self.ikPlanner = ikPlanner

        widget = widget or app.getMainWindow()
        app.addShortcut(widget, 'Ctrl+Shift+S', self.planStand)
        app.addShortcut(widget, 'Ctrl+Shift+N', self.planNominal)
        app.addShortcut(widget, 'Ctrl+Shift+L', functools.partial(self.planPreGrasp, 'left'))
        app.addShortcut(widget, 'Ctrl+Shift+R', functools.partial(self.planPreGrasp, 'right'))

    def planStand(self):
        self.ikPlanner.computeStandPlan(self.jointController.q)

    def planNominal(self):
        self.ikPlanner.computeNominalPlan(self.jointController.q)

    def planPreGrasp(self, side):
        startPose = self.jointController.q
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'arm up pregrasp', side=side)
        self.ikPlanner.computePostureGoal(startPose, endPose)


class JointLimitChecker(object):

    def __init__(self, robotModel, sensorJointController):

        self.robotModel = robotModel
        self.sensorJointController = sensorJointController
        self.jointLimitsMin = np.array([self.robotModel.model.getJointLimits(jointName)[0] for jointName in robotstate.getDrakePoseJointNames()])
        self.jointLimitsMax = np.array([self.robotModel.model.getJointLimits(jointName)[1] for jointName in robotstate.getDrakePoseJointNames()])
        self.joints = robotstate.matchJoints('^(?!base_)') # all but base joints
        self.inflationAmount = np.radians(0.3)
        self.timer = TimerCallback(targetFps=1)
        self.timer.callback = self.update
        self.warningButton = None
        self.action = None

    def update(self):
        limitData = self.checkJointLimits()
        if limitData:
            self.notifyUserStatusBar(limitData)
        else:
            self.clearStatusBarWarning()

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

    def clearStatusBarWarning(self):
        if self.warningButton:
            self.warningButton.deleteLater()
            self.warningButton = None

    def notifyUserStatusBar(self, limitData):

        if self.warningButton:
            return

        def showDialog():
            limitData = self.checkJointLimits()
            if limitData:
                self.notifyUserDialog(limitData)
            self.clearStatusBarWarning()

        self.warningButton = QtGui.QPushButton('Joint Limit Warning')
        self.warningButton.setStyleSheet("background-color:red")
        self.warningButton.connect('clicked()', showDialog)
        app.getMainWindow().statusBar().insertPermanentWidget(0, self.warningButton)

    def notifyUserDialog(self, limitData):

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

        return limitData

    def toJointIndex(self, jointName):
        return robotstate.getDrakePoseJointNames().index(jointName)




class GeneralEndEffectorTeleopPanel(object):

    def __init__(self, ikPlanner, teleopPanel, robotStateModel, robotStateJointController):
        self.ikPlanner = ikPlanner
        self.teleopPanel = teleopPanel
        self.robotStateModel = robotStateModel
        self.robotStateJointController = robotStateJointController

        self.widget = QtGui.QWidget()
        l = QtGui.QVBoxLayout(self.widget)

        h = QtGui.QHBoxLayout()
        l.addLayout(h)
        h.addWidget(QtGui.QLabel('End effector:'))
        self.endEffectorCombo = QtGui.QComboBox()
        h.addWidget(self.endEffectorCombo)

        def addButton(name, func):
            b = QtGui.QPushButton(name)
            b.connect('clicked()', func)
            l.addWidget(b)

        addButton('start ik', self.startIk)
        addButton('end ik', self.endIk)
        addButton('plan', self.planIk)

        config = drcargs.getDirectorConfig()['endEffectorConfig']
        self.endEffectorLinkNames = config['endEffectorLinkNames']
        self.graspOffsetFrame = transformUtils.frameFromPositionAndRPY(config['graspOffsetFrame'][0], np.degrees(config['graspOffsetFrame'][1]))
        self.fixedJoints = config['fixedJoints']

        for linkName in self.endEffectorLinkNames:
            self.endEffectorCombo.addItem(linkName)


    def planIk(self):

        startPoseName = 'reach_start'
        endPoseName = 'reach_end'
        startPose = np.array(self.robotStateJointController.q)
        self.ikPlanner.addPose(startPose, startPoseName)

        plan = self.constraintSet.runIkTraj()
        self.teleopPanel.showPlan(plan)

    def endIk(self):
        self.teleopPanel.hideTeleopModel()
        EndEffectorTeleopPanel.removePlanFolder()

    def startIk(self, reachGoal=None):

        EndEffectorTeleopPanel.removePlanFolder()

        ikPlanner = self.ikPlanner

        startPoseName = 'reach_start'
        endPoseName = 'reach_end'
        startPose = np.array(self.robotStateJointController.q)
        ikPlanner.addPose(startPose, startPoseName)

        endEffectorLinkName = str(self.endEffectorCombo.currentText)

        if reachGoal is None:
            endEffectorLinkFrame = self.robotStateModel.getLinkFrame(endEffectorLinkName)
            assert endEffectorLinkFrame is not None
            graspToWorld = vtk.vtkTransform()
            graspToWorld.PostMultiply()
            graspToWorld.Concatenate(self.graspOffsetFrame)
            graspToWorld.Concatenate(endEffectorLinkFrame)
            reachGoal = graspToWorld

        om.removeFromObjectModel('reach goal')
        goalFrame = vis.showFrame(reachGoal, 'reach goal', scale=0.1, parent=EndEffectorTeleopPanel.getConstraintFrameFolder())
        goalFrame.setProperty('Edit', True)

        constraints = []
        for pattern in self.fixedJoints:
            constraints.append(ikPlanner.createPostureConstraint(startPoseName, robotstate.matchJoints(pattern)))

        constraints.extend(ikPlanner.createPositionOrientationConstraint(endEffectorLinkName, goalFrame, self.graspOffsetFrame, positionTolerance=0.0, angleToleranceInDegrees=0.0))
        constraints[-1].tspan = [1.0, 1.0]
        constraints[-2].tspan = [1.0, 1.0]

        self.constraintSet = ikplanner.ConstraintSet(ikPlanner, constraints, endPoseName, startPoseName)

        def onGoalFrameModified(frame):
            endPose, info = self.constraintSet.runIk()
            self.teleopPanel.showPose(self.constraintSet.endPose)
            app.displaySnoptInfo(info)

        goalFrame.connectFrameModified(onGoalFrameModified)
        onGoalFrameModified()

        folder = EndEffectorTeleopPanel.getConstraintFolder()
        for i, constraint in enumerate(constraints):
            constraintItem = ConstraintItem(constraint)
            om.addToObjectModel(constraintItem, parentObj=folder)


class JointTeleopPanel(object):

    def __init__(self, panel, jointGroups=None):
        self.panel = panel
        self.ui = panel.ui
        self.ui.jointTeleopButton.connect('clicked()', self.teleopButtonClicked)
        self.ui.resetJointsButton.connect('clicked()', self.resetButtonClicked)
        self.ui.planButton.connect('clicked()', self.planClicked)

        self.timerCallback = TimerCallback()
        self.timerCallback.callback = self.onTimerCallback

        self.jointLimitsMin = np.array([self.panel.teleopRobotModel.model.getJointLimits(jointName)[0] for jointName in robotstate.getDrakePoseJointNames()])
        self.jointLimitsMax = np.array([self.panel.teleopRobotModel.model.getJointLimits(jointName)[1] for jointName in robotstate.getDrakePoseJointNames()])

        # this need to be generalized
        if 'baseZJointLimits' in drcargs.getDirectorConfig():
            baseZLimits = drcargs.getDirectorConfig()['baseZJointLimits']
        else: # TODO generalise so the base sliders are deactivated
            baseZLimits = [-0.1, 0.1]

        self.jointLimitsMin[0:6] = [-0.25, -0.25, baseZLimits[0], -math.radians(20), -math.radians(20), -math.radians(20)]
        self.jointLimitsMax[0:6] = [ 0.25 , 0.25, baseZLimits[1],  math.radians(20),  math.radians(20),  math.radians(20)]


        if jointGroups is None:
            # Add only these joint groups:
            telopJointGroupNames = ['Back', 'Base', 'Left Arm', 'Right Arm', 'Neck']
            allJointGroups = drcargs.getDirectorConfig()['teleopJointGroups']
            jointGroups = []
            for jointGroup in allJointGroups:
                if jointGroup['name'] in telopJointGroupNames:
                    jointGroups.append( jointGroup )

        self.jointGroups = jointGroups

        self.buildTabWidget(jointGroups)

        self.startPose = None
        self.endPose = None
        self.userJoints = {}

        self.updateWidgetState()


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
            self.ui.tabWidget.addTab(jointGroupWidget, groupName)

        self.signalMapper = QtCore.QSignalMapper()

        self.sliderMax = 1000.0
        for jointName, slider in self.slidersMap.iteritems():
            slider.connect('valueChanged(int)', self.signalMapper, 'map()')
            self.signalMapper.setMapping(slider, jointName)
            slider.setMaximum(self.sliderMax)

        self.signalMapper.connect('mapped(const QString&)', self.sliderChanged)


    def planClicked(self):
        if not self.ui.jointTeleopButton.checked:
            return

        self.computeEndPose()
        self.generatePlan()


    def generatePlan(self):

        hasBase = False
        for jointIndex, jointValue in self.userJoints.iteritems():
            if self.toJointName(jointIndex).startswith('base_'):
                hasBase = True

        plan = self.panel.ikPlanner.computePostureGoal(self.startPose, self.endPose, feetOnGround=hasBase)
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
            if jointName.startswith('base_'):
                hasBase = True

        if hasBase:

            ikPlanner = self.panel.ikPlanner

            startPoseName = 'posture_goal_start'
            ikPlanner.addPose(self.startPose, startPoseName)

            endPoseName = 'posture_goal_end'
            ikPlanner.addPose(self.endPose, endPoseName)

            jointNamesAll = self.slidersMap.keys()

            # remove leg joints
            jointNames = []
            for name in jointNamesAll:
                if not 'leg' in name:
                    jointNames.append(name)

            # uncomment to constraint only joints adjusted by user
            #jointNames = [self.toJointName(jointIndex) for jointIndex in sorted(self.userJoints.keys())]

            p = ikPlanner.createPostureConstraint(endPoseName, jointNames)

            constraints = [p]
            constraints.extend(ikPlanner.createFixedFootConstraints(startPoseName))
            constraints.append(ikPlanner.createQuasiStaticConstraint())

            self.endPose, info = ikPlanner.ikServer.runIk(constraints, ikPlanner.defaultIkParameters, nominalPostureName=startPoseName, seedPostureName='q_end')
            app.displaySnoptInfo(info)


    def getJointValue(self, jointIndex):
        return self.endPose[jointIndex]


    def sliderChanged(self, jointName):

        slider = self.slidersMap[jointName]
        jointIndex = self.toJointIndex(jointName)
        jointValue = self.toJointValue(jointIndex, slider.value / float(self.sliderMax))
        self.userJoints[jointIndex] = jointValue

        if jointName.startswith('base_'):
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

        baseJointOffsets = None


        for jointName, slider in self.slidersMap.iteritems():
            jointIndex = self.toJointIndex(jointName)
            jointValue = self.getJointValue(jointIndex)

            if (self.panel.ikPlanner.fixedBaseArm==False):
                if jointName.startswith('base_'):
                    if baseJointOffsets is None:
                        baseJointOffsets = self.computeBaseJointOffsets()
                    jointValue -= self.baseJointOffsets.get(jointName, 0.0)

            slider.blockSignals(True)
            slider.setValue(self.toSliderValue(jointIndex, jointValue)*self.sliderMax)
            slider.blockSignals(False)
            self.updateLabel(jointName, jointValue)


class TeleopPanel(object):

    def __init__(self, robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController, ikPlanner, manipPlanner, affordanceManager, showPlanFunction, hidePlanFunction):

        self.robotStateModel = robotStateModel
        self.robotStateJointController = robotStateJointController
        self.teleopRobotModel = teleopRobotModel
        self.teleopJointController = teleopJointController
        self.ikPlanner = ikPlanner
        self.manipPlanner = manipPlanner
        self.affordanceManager = affordanceManager
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

        if 'endEffectorConfig' in drcargs.getDirectorConfig():
            self.ui.endEffectorTeleopFrame.setVisible(False)
            self.generalEndEffectorTeleopPanel = GeneralEndEffectorTeleopPanel(ikPlanner, self, robotStateModel, robotStateJointController)
            self.widget.layout().addWidget(self.generalEndEffectorTeleopPanel.widget, 0, 0, 1, 2)
        PythonQt.dd.ddGroupBoxHider(self.ui.paramsContainer)

    def onPostureDatabaseClicked(self):
        ikplanner.RobotPoseGUIWrapper.initCaptureMethods(self.robotStateJointController, self.teleopJointController)
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


def extendJointLimitsForTesting(teleopPanel, jointLimitChecker):

    # add +/- 3 degrees to joint teleop sliders
    jointTeleop = teleopPanel.jointTeleop
    extra = np.zeros(len(jointTeleop.jointLimitsMin))
    extra += np.deg2rad(3.0)
    jointTeleop.jointLimitsMin -= extra
    jointTeleop.jointLimitsMax += extra

    # add +/- 4 degrees to planner joint limits
    limitDataMin = [(name, -np.deg2rad(4.0)) for name in jointLimitChecker.joints]
    limitDataMax = [(name, np.deg2rad(4.0)) for name in jointLimitChecker.joints]
    teleopPanel.ikPlanner.ikServer.updateJointLimits(limitDataMin)
    teleopPanel.ikPlanner.ikServer.updateJointLimits(limitDataMax)


def _getAction():
    return app.getToolBarActions()['ActionTeleopPanel']


def init(robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController, debrisPlanner, manipPlanner, affordanceManager, showPlanFunction, hidePlanFunction):

    global panel
    global dock

    panel = TeleopPanel(robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController, debrisPlanner, manipPlanner, affordanceManager, showPlanFunction, hidePlanFunction)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel
