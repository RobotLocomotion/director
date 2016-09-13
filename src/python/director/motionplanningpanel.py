import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
import director.applogic as app
import director.objectmodel as om
from director.timercallback import TimerCallback
from director import robotstate
from director import visualization as vis
from director import transformUtils
from director import ikplanner
from director import footstepsdriver
from director import vtkAll as vtk
from director import drcargs
from director import affordanceurdf
from director.roboturdf import HandFactory
from director import lcmUtils

import functools
import math
import numpy as np
from numpy import linalg as npla
import types
import lcm
from bot_core.pose_t import pose_t

from director import propertyset
from director.debugVis import DebugData
from director.pointpicker import PlacerWidget
from director import filterUtils
from director import vtkNumpy as vnp
from director import ikconstraints

def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)

class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)
        
class MotionPlanningPanel(object):
    def __init__(self, planningUtils, robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController, ikPlanner, manipPlanner, affordanceManager, showPlanFunction, hidePlanFunction, footDriver):
        self.planningUtils = planningUtils
        self.robotStateModel = robotStateModel
        self.robotStateJointController = robotStateJointController
        self.teleopRobotModel = teleopRobotModel
        self.teleopJointController = teleopJointController
        self.ikPlanner = ikPlanner
        self.manipPlanner = manipPlanner
        self.affordanceManager = affordanceManager
        self.showPlanFunction = showPlanFunction
        self.hidePlanFunction = hidePlanFunction
        self.footDriver = footDriver
        
        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddMotionPlanningPanel.ui')
        assert uifile.open(uifile.ReadOnly)
        self.widget = loader.load(uifile)
        self.ui = WidgetDict(self.widget.children())
        
        # Check motion planning mode
        self.ui.mpModeButton.connect('clicked()', self.onMotionPlanningMode)

        # End-pose planning
        self.ui.handCombo.connect('currentIndexChanged(const QString&)', self.onHandChanged)
        self.ui.baseComboBox.connect('currentIndexChanged(const QString&)', self.onBaseConstraintChanged)
        self.ui.backComboBox.connect('currentIndexChanged(const QString&)', self.onBackConstraintChanged)
        self.ui.feetComboBox.connect('currentIndexChanged(const QString&)', self.onFeetConstraintChanged)
        self.ui.otherHandComboBox.connect('currentIndexChanged(const QString&)', self.onOtherHandConstraintChanged)

        self.ui.fpButton.connect('clicked()', self.onSearchFinalPose)
                
        if 'kneeJointLimits' in drcargs.getDirectorConfig():
            self.kneeJointLimits = drcargs.getDirectorConfig()['kneeJointLimits']
        self.constraintSet = None
        self.palmOffsetDistance = 0.0        
        
        # Foot step planning
        self.placer = None
        self.ui.walkingPlanButton.connect('clicked()', self.onWalkingPlan)
        self.ui.teleportRobotButton.connect('clicked()', self.onTeleportRobotToStanceFrame)
        # Motion Planning
        self.ui.motionPlanButton.connect('clicked()', self.onMotionPlan)
        
        self.deactivate()
    def onMotionPlanningMode(self):
        if self.ui.mpModeButton.checked:
            self.activate()
        else:
            self.deactivate()
        
    def activate(self):
        self.ui.mpModeButton.blockSignals(True)
        self.ui.mpModeButton.checked = True
        self.ui.mpModeButton.blockSignals(False)
        self.ui.EndPosePlanningPanel.setEnabled(True)
        if self.getComboText(self.ui.feetComboBox) == 'Sliding': 
            self.ui.walkingPlanningPanel.setEnabled(True)
            self.ui.fixFeetPlanningPanel.setEnabled(False)
        elif self.getComboText(self.ui.feetComboBox) == 'Fixed': 
            self.ui.walkingPlanningPanel.setEnabled(False)
            self.ui.fixFeetPlanningPanel.setEnabled(True)
        self.createHandGoalFrame()
        self.updateIKConstraints()
        
    def deactivate(self):
        self.ui.mpModeButton.blockSignals(True)
        self.ui.mpModeButton.checked = False
        self.ui.mpModeButton.blockSignals(False)
        self.ui.EndPosePlanningPanel.setEnabled(False)
        self.ui.walkingPlanningPanel.setEnabled(False)
        self.ui.fixFeetPlanningPanel.setEnabled(False)
        self.removePlanFolder()
        self.hideTeleopModel()
        
    # Check final-pose and reaching planning separately
    # so that one can switch between drake and exotica without restarting director 
    def checkFinalPosePlanningMode(self):
        algorithm = self.getComboText(self.ui.fpAlgComboBox)
        if 'Drake' in algorithm:
            self.ikPlanner.planningMode = 'drake'
        elif 'Exotica' in algorithm:
            self.ikPlanner.planningMode = 'exotica'
        
    def checkReachingPlanningMode(self):
        algorithm = self.getComboText(self.ui.mpAlgComboBox)
        if 'Drake' in algorithm:
            self.ikPlanner.planningMode = 'drake'
        elif 'Exotica' in algorithm:
            self.ikPlanner.planningMode = 'exotica'
            
    def onHandChanged(self, combo):
        if self.ui.mpModeButton.checked:
            self.createHandGoalFrame()
            self.updateIKConstraints()
            self.hideTeleopModel()
            if self.getReachHand() == 'Both':
                self.ui.otherHandComboBox.setEnabled(False)
            else:
                self.ui.otherHandComboBox.setEnabled(True)
        
    def onBaseConstraintChanged(self):
        if self.getComboText(self.ui.baseComboBox) == 'Fixed':
            self.ui.feetComboBox.setCurrentIndex(1)
            self.ui.feetComboBox.show()
            self.removeWalkingPlanningInfo()
        self.updateIKConstraints()
        if self.ui.fpInteractiveCheck.checked:
            self.updateIk()
    
    def onBackConstraintChanged(self):
        self.updateIKConstraints()
        if self.ui.fpInteractiveCheck.checked:
            self.updateIk()
        
    def onFeetConstraintChanged(self):
        if self.getComboText(self.ui.feetComboBox) == 'Fixed': 
            self.ui.walkingPlanningPanel.setEnabled(False)
            self.ui.fixFeetPlanningPanel.setEnabled(True)
            self.removeWalkingPlanningInfo()
        elif self.getComboText(self.ui.feetComboBox) == 'Sliding': 
            self.ui.walkingPlanningPanel.setEnabled(True)
            self.ui.fixFeetPlanningPanel.setEnabled(False)
            self.ui.baseComboBox.setCurrentIndex(0)
            self.ui.baseComboBox.show()
        self.updateIKConstraints()
        if self.ui.fpInteractiveCheck.checked:
            self.updateIk()
    
    def onOtherHandConstraintChanged(self):
        self.updateIKConstraints()
        if self.ui.fpInteractiveCheck.checked:
            self.updateIk()
        
    def getComboText(self, combo):
        return str(combo.currentText)

    def getReachHand(self):
        return self.getComboText(self.ui.handCombo)

    @staticmethod
    def getGoalFrame(linkName):
        return om.findObjectByName('%s constraint frame' % linkName)
    
    def updateIKConstraints(self):
        startPoseName = 'reach_start'
        startPose = self.planningUtils.getPlanningStartPose()
        self.ikPlanner.addPose(startPose, startPoseName)

        constraints = []
        constraints.append(self.ikPlanner.createQuasiStaticConstraint())
        constraints.append(self.ikPlanner.createLockedNeckPostureConstraint(startPoseName))
        
        # Get base constraint
        if self.getComboText(self.ui.baseComboBox) == 'Fixed':
            constraints.append(self.ikPlanner.createLockedBasePostureConstraint(startPoseName, lockLegs=False))
            self.ikPlanner.setBaseLocked(True)
        elif self.getComboText(self.ui.baseComboBox) == 'XYZ only':
            constraints.append(self.ikPlanner.createXYZMovingBasePostureConstraint(startPoseName))
            constraints.append(self.ikPlanner.createKneePostureConstraint(self.kneeJointLimits))
        elif self.getComboText(self.ui.baseComboBox) == 'Limited':
            constraints.append(self.ikPlanner.createMovingBaseSafeLimitsConstraint())
            constraints.append(self.ikPlanner.createKneePostureConstraint(self.kneeJointLimits))
            self.ikPlanner.setBaseLocked(False)
            
        # Get back constraint 
        if self.getComboText(self.ui.backComboBox) == 'Fixed':
            constraints.append(self.ikPlanner.createLockedBackPostureConstraint(startPoseName))
            self.ikPlanner.setBackLocked(True)
        elif self.getComboText(self.ui.backComboBox) == 'Limited':
            constraints.append(self.ikPlanner.createMovingBackLimitedPostureConstraint())
            self.ikPlanner.setBackLocked(False)
            
        # Get feet constraint
        if self.getComboText(self.ui.feetComboBox) == 'Fixed':                
            constraints.append(self.ikPlanner.createFixedLinkConstraints(startPoseName, self.ikPlanner.leftFootLink, tspan=[0.0, 1.0], lowerBound=-0.0001*np.ones(3), upperBound=0.0001*np.ones(3), angleToleranceInDegrees=0.1))
            constraints.append(self.ikPlanner.createFixedLinkConstraints(startPoseName, self.ikPlanner.rightFootLink, tspan=[0.0, 1.0], lowerBound=-0.0001*np.ones(3), upperBound=0.0001*np.ones(3), angleToleranceInDegrees=0.1))
        elif self.getComboText(self.ui.feetComboBox) == 'Sliding':
            constraints.extend(self.ikPlanner.createSlidingFootConstraints(startPoseName)[:2])
            constraints.extend(self.ikPlanner.createSlidingFootConstraints(startPoseName)[2:])
            
            # Ensure the end-pose's relative distance between two feet is the same as start pose
            [pos_left, quat_left] = transformUtils.poseFromTransform(self.robotStateModel.getLinkFrame(self.ikPlanner.leftFootLink))
            [pos_right, quat_right] = transformUtils.poseFromTransform(self.robotStateModel.getLinkFrame(self.ikPlanner.rightFootLink))
            dist = npla.norm(pos_left - pos_right)
            constraints.append(ikconstraints.PointToPointDistanceConstraint(bodyNameA=self.ikPlanner.leftFootLink, bodyNameB=self.ikPlanner.rightFootLink, lowerBound=np.array([dist - 0.0001]), upperBound=np.array([dist + 0.0001])))

        sides = []
        if self.getReachHand() == 'Left':
            sides.append('left')
        elif self.getReachHand() == 'Right':
            sides.append('right')
        elif self.getReachHand() == 'Both':
            sides.append('left')
            sides.append('right')
        
        if self.getComboText(self.ui.otherHandComboBox) == 'Fixed':
            if not 'left' in sides:
                self.ikPlanner.setArmLocked('left', True)
                constraints.append(self.ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
            if not 'right' in sides:
                self.ikPlanner.setArmLocked('right', True)
                constraints.append(self.ikPlanner.createLockedRightArmPostureConstraint(startPoseName))
        
        for side in sides:        
            linkName = self.ikPlanner.getHandLink(side)
            graspToHand = self.ikPlanner.newPalmOffsetGraspToHandFrame(side, self.palmOffsetDistance)
            graspToWorld = self.getGoalFrame(linkName)
            p, q = self.ikPlanner.createPositionOrientationGraspConstraints(side, graspToWorld, graspToHand)
            p.tspan = [1.0, 1.0]
            q.tspan = [1.0, 1.0]
            constraints.extend([p, q])
            constraints.append(self.ikPlanner.createActiveEndEffectorConstraint(linkName, self.ikPlanner.getPalmPoint(side)))
            self.constraintSet = ikplanner.ConstraintSet(self.ikPlanner, constraints, 'reach_end', startPoseName)
        
    @staticmethod
    def removePlanFolder():
        om.removeFromObjectModel(om.findObjectByName('teleop plan'))
        om.removeFromObjectModel(om.findObjectByName('footstep plan'))
        om.removeFromObjectModel(om.findObjectByName('iDRMStancePoses'))

    @staticmethod
    def removeWalkingPlanningInfo():
        om.removeFromObjectModel(om.findObjectByName('footstep plan'))
        om.removeFromObjectModel(om.findObjectByName('iDRMStancePoses'))
        
    @staticmethod
    def getConstraintFrameFolder():
        return om.getOrCreateContainer('constraint frames', parentObj=om.getOrCreateContainer('teleop plan', parentObj=om.findObjectByName('planning')))
    
    def removeHandFrames(self):
        sides = ['left', 'right']
        for side in sides:
            linkName = self.ikPlanner.getHandLink(side)
            frameName = '%s constraint frame' % linkName
            frame = om.findObjectByName(frameName)
            if frame:
                om.removeFromObjectModel(frame)
        
    def createHandGoalFrame(self):
        sides = []
        if self.getReachHand() == 'Left':
            sides.append('left')
        elif self.getReachHand() == 'Right':
            sides.append('right')
        elif self.getReachHand() == 'Both':
            sides.append('left')
            sides.append('right')
        self.removeHandFrames()

        folder = self.getConstraintFrameFolder()
        startPose = self.planningUtils.getPlanningStartPose()
        for side in sides:
            linkName = self.ikPlanner.getHandLink(side)
            frameName = '%s constraint frame' % linkName
            graspToHand = self.ikPlanner.newPalmOffsetGraspToHandFrame(side, self.palmOffsetDistance)
            graspToWorld = self.ikPlanner.newGraspToWorldFrame(startPose, side, graspToHand)
            frame = vis.showFrame(graspToWorld, frameName, parent=folder, scale=0.2)
            frame.connectFrameModified(self.onGoalFrameModified)
        
    def updateIk(self):
        if not self.constraintSet:
            self.updateIKConstraints()
        self.checkFinalPosePlanningMode()
        endPose, info = self.constraintSet.runIk()
        endPoseName = 'reach_end'
        self.ikPlanner.addPose(endPose, endPoseName)
        self.showPose(self.constraintSet.endPose)
        app.displaySnoptInfo(info)
        
        if self.ui.walkingPlanningPanel.enabled and self.ui.walkInteractiveCheck.checked:
            self.onWalkingPlan()
            
    def onSearchFinalPose(self):
        self.updateIk()
        
    def onGoalFrameModified(self, frame):
        if self.ui.fpInteractiveCheck.checked:
            self.updateIk()
            
    def showPose(self, pose):
        self.teleopJointController.setPose('MP_EndPose', pose)
        self.hidePlanFunction()
        self.showMPModel()
        
    def showMPModel(self):
        self.teleopRobotModel.setProperty('Visible', True)
        self.robotStateModel.setProperty('Visible', True)
        self.robotStateModel.setProperty('Alpha', 0.1)
        
        
    def getCurrentWalkingGoal(self):
        t = self.footDriver.getFeetMidPoint(self.teleopRobotModel)
        return t
    
    def onWalkingPlan(self):
        walkingGoal = self.getCurrentWalkingGoal();
        if self.placer:
            self.placer.stop()
        self.onWalkingGoalModified(walkingGoal)
        
    def onTeleportRobotToStanceFrame(self):
        self.robotStateJointController.q[:6] = self.teleopJointController.q[:6]
        self.robotStateJointController.push()
        startPoseName = 'reach_start'
        startPose = self.planningUtils.getPlanningStartPose()
        self.ikPlanner.addPose(startPose, startPoseName)
        self.ui.feetComboBox.setCurrentIndex(1)   
        
    def onWalkingGoalModified(self, frame):
        request = self.footDriver.constructFootstepPlanRequest(self.robotStateJointController.q, frame)
        self.footDriver.sendFootstepPlanRequest(request)
            
    def onMotionPlan(self):
        self.ui.feetComboBox.setCurrentIndex(1)
        startPoseName = 'reach_start'
        startPose = self.planningUtils.getPlanningStartPose()
        self.checkReachingPlanningMode()
        self.ikPlanner.addPose(startPose, startPoseName)
        plan = self.constraintSet.runIkTraj()
        self.showPlan(plan)
        
    def hideTeleopModel(self):
        self.teleopRobotModel.setProperty('Visible', False)
        self.robotStateModel.setProperty('Visible', True)
        self.robotStateModel.setProperty('Alpha', 1.0)
        
    def showPlan(self, plan):
        self.hideTeleopModel()
        self.showPlanFunction(plan)
def _getAction():
    return app.getToolBarActions()['ActionMotionPlanningPanel']

def init(planningUtils, robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController, debrisPlanner, manipPlanner, affordanceManager, showPlanFunction, hidePlanFunction, footDriver):

    global panel
    global dock

    panel = MotionPlanningPanel(planningUtils, robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController, debrisPlanner, manipPlanner, affordanceManager, showPlanFunction, hidePlanFunction, footDriver)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel
