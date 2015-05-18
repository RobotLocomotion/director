#import ddapp
from ddapp import cameraview
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp import objectmodel as om
from ddapp import ik
from ddapp.ikplanner import ConstraintSet
from ddapp import polarisplatformplanner
from ddapp import robotstate
from ddapp import segmentation
from ddapp import sitstandplanner
from ddapp.timercallback import TimerCallback
from ddapp import visualization as vis
from ddapp import planplayback
from ddapp import lcmUtils

import os
import functools
import numpy as np
import scipy.io
import vtkAll as vtk
from ddapp.tasks.taskuserpanel import TaskUserPanel
import ddapp.tasks.robottasks as rt
from ddapp import filterUtils
from ddapp import ioUtils


class PolarisModel(object):

    def __init__(self):
        pose = transformUtils.poseFromTransform(vtk.vtkTransform())
        desc = dict(classname='MeshAffordanceItem', Name='polaris',
                    Filename='software/models/polaris/polaris_cropped.vtp', pose=pose)
        self.affordance = segmentation.affordanceManager.newAffordanceFromDescription(desc)
        self.aprilTagFrame = vis.updateFrame(vtk.vtkTransform(), 'grab bar april tag',
                                             visible=True, scale=0.2, parent=self.affordance)

        t = transformUtils.transformFromPose(np.array([-0.99548239, 0.04156693, 0.35259928]),
                                             np.array([ 0.18827199, 0.84761397, 0.41552535,
                                                       0.27100351]))
        self.leftFootEgressStartFrame  = vis.updateFrame(t, 'left foot start', scale=0.2,visible=True, parent=self.affordance)

        t = transformUtils.transformFromPose(np.array([-0.93707546,  0.07409333,  0.32871604]),
                                             np.array([ 0.22455191,  0.71396247,  0.60983921,
                                                       0.26063418]))
        self.leftFootEgressInsideFrame  = vis.updateFrame(t, 'left foot inside', scale=0.2,visible=True, parent=self.affordance)

        t = transformUtils.transformFromPose(np.array([-0.89783714,  0.23503719,  0.29039189]),
                                             np.array([ 0.2331762 ,  0.69031269,  0.6311807,
                                                       0.2659101]))
        self.leftFootEgressMidFrame  = vis.updateFrame(t, 'left foot mid', scale=0.2,visible=True, parent=self.affordance)

        t = transformUtils.transformFromPose(np.array([-0.88436275,  0.50939115,  0.31281047]),
                                             np.array([ 0.22600245,  0.69177731,  0.63305905,
                                                       0.26382435]))
        self.leftFootEgressOutsideFrame  = vis.updateFrame(t, 'left foot outside', scale=0.2,visible=True, parent=self.affordance)
        self.frameSync = vis.FrameSync()
        self.frameSync.addFrame(self.aprilTagFrame)
        self.frameSync.addFrame(self.affordance.getChildFrame(), ignoreIncoming=True)
        self.frameSync.addFrame(self.leftFootEgressStartFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.leftFootEgressInsideFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.leftFootEgressMidFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.leftFootEgressOutsideFrame, ignoreIncoming=True)

        self.timerCallback = TimerCallback()
        self.timerCallback.targetFps = 5
        self.timerCallback.callback = self.updateAprilTagFrame
        self.timerCallback.start()

    def updateAprilTagFrame(self):
        t = vtk.vtkTransform()
        cameraview.imageManager.queue.getTransform('april_tag_car_beam', 'local', t)
        self.aprilTagFrame.copyFrame(t)


class EgressPlanner(object):

    def __init__(self, robotSystem):

        self.pelvisLiftX = 0.0
        self.pelvisLiftZ = 0.05
        self.legLiftAngle = 8

        self.robotSystem = robotSystem
        self.polaris = None

    def spawnPolaris(self):
        self.polaris = PolarisModel()

    def createLeftFootPoseConstraint(self, targetFrame, tspan=[-np.inf,np.inf]):
        positionConstraint, orientationConstraint = self.robotSystem.ikPlanner.createPositionOrientationConstraint('l_foot', targetFrame, vtk.vtkTransform())
        positionConstraint.tspan = tspan
        orientationConstraint.tspan = tspan
        return positionConstraint, orientationConstraint

    def createAllButLeftLegPostureConstraint(self, poseName):
        joints = robotstate.matchJoints('^(?!l_leg)')
        return self.robotSystem.ikPlanner.createPostureConstraint(poseName, joints)


    def getPlanningStartPose(self):
        return self.robotSystem.robotStateJointController.getPose('EST_ROBOT_STATE')

    def planFootLiftInCar(self):
        startPose = self.getPlanningStartPose()
        startPoseName = 'q_lift_start'
        jointId = robotstate.getDrakePoseJointNames().index('l_leg_kny')
        knyDesired = startPose[jointId] + np.radians(self.legLiftAngle)
        jointId = robotstate.getDrakePoseJointNames().index('l_leg_aky')
        akyDesired = startPose[jointId] - np.radians(self.legLiftAngle)
        postureJoints = {'l_leg_kny': knyDesired, 'l_leg_aky': akyDesired}
        endPose = self.robotSystem.ikPlanner.mergePostures(startPose, postureJoints)
        plan = self.robotSystem.ikPlanner.computePostureGoal(startPose, endPose, feetOnGround=False)
        return plan

    def planFootStartPlacement(self):
        startPose = self.getPlanningStartPose()
        startPoseName = 'q_egress_start'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_egress_end'
        #footFrame = self.robotSystem.ikPlanner.getLinkFrameAtPose('l_foot', startPose)
        #t = transformUtils.frameFromPositionAndRPY([0, 0, self.liftHeight], [0, 0, 0])
        #liftFrame = transformUtils.concatenateTransforms([t, footFrame])
        constraints = []
        constraints.append(self.createAllButLeftLegPostureConstraint(startPoseName))
        constraints.extend(self.createLeftFootPoseConstraint(self.polaris.leftFootEgressStartFrame, tspan=[1,1]))
        constraintSet = ConstraintSet(self.robotSystem.ikPlanner, constraints, endPoseName, startPoseName)

        constraintSet.runIk()
        return constraintSet.planEndPoseGoal(feetOnGround=False)

    def planGetWeightOverFeet(self):
        ikParameterDict = {'usePointwise': False,
                           'leftFootSupportEnabled': True,
                           'rightFootSupportEnabled': True,
                           'pelvisSupportEnabled': False}
        originalIkParameterDict = self.robotSystem.ikPlanner.setIkParameters(ikParameterDict)
        startPose = self.getPlanningStartPose()
        startPoseName = 'q_egress_start'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_egress_end'
        constraints = []
        constraints.append(self.robotSystem.ikPlanner.createQuasiStaticConstraint())
        constraints.append(self.robotSystem.ikPlanner.createLockedBasePostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedRightArmPostureConstraint(startPoseName))
        constraintSet = ConstraintSet(self.robotSystem.ikPlanner, constraints, endPoseName, startPoseName)

        constraintSet.runIk()
        plan =  constraintSet.planEndPoseGoal(feetOnGround=False)
        self.robotSystem.ikPlanner.setIkParameters(originalIkParameterDict)
        return plan

    def planStandUp(self):
        ikParameterDict = {'usePointwise': True,
                           'leftFootSupportEnabled': True,
                           'rightFootSupportEnabled': True,
                           'pelvisSupportEnabled': False}
        originalIkParameterDict = self.robotSystem.ikPlanner.setIkParameters(ikParameterDict)
        startPose = self.getPlanningStartPose()
        startPoseName = 'q_egress_start'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_egress_end'
        pelvisFrame = self.robotSystem.ikPlanner.getLinkFrameAtPose('pelvis', startPose)
        t = transformUtils.frameFromPositionAndRPY([self.pelvisLiftX, 0, self.pelvisLiftZ], [0, 0, 0])
        liftFrame = transformUtils.concatenateTransforms([t, pelvisFrame])

        constraints = []
        p = ik.PositionConstraint(linkName='pelvis', referenceFrame=liftFrame,
                                  lowerBound=np.array([0.0, -np.inf, 0.0]),
                                  upperBound=np.array([np.inf, np.inf, 0.0]))
        constraints.append(p)
        constraints.append(self.robotSystem.ikPlanner.createQuasiStaticConstraint())
        constraints.append(self.robotSystem.ikPlanner.createXYZMovingBasePostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedRightArmPostureConstraint(startPoseName))
        constraints.extend(self.robotSystem.ikPlanner.createFixedFootConstraints(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createKneePostureConstraint([0.7, 2.5]))
        constraintSet = ConstraintSet(self.robotSystem.ikPlanner, constraints, endPoseName, startPoseName)

        constraintSet.runIk()
        plan = constraintSet.planEndPoseGoal(feetOnGround=False)
        self.robotSystem.ikPlanner.setIkParameters(originalIkParameterDict)
        return plan

    def planShiftWeightOut(self):
        ikParameterDict = {'usePointwise': True,
                           'leftFootSupportEnabled': False,
                           'rightFootSupportEnabled': True,
                           'pelvisSupportEnabled': False}
        originalIkParameterDict = self.robotSystem.ikPlanner.setIkParameters(ikParameterDict)

        startPose = self.getPlanningStartPose()
        startPoseName = 'q_egress_start'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_egress_end'
        constraints = []

        utorsoFrame = self.robotSystem.ikPlanner.getLinkFrameAtPose('utorso', startPose)
        g = ik.WorldGazeDirConstraint()
        g.linkName = 'utorso'
        g.targetFrame = vtk.vtkTransform()
        axes = transformUtils.getAxesFromTransform(self.polaris.leftFootEgressInsideFrame.transform)
        g.targetAxis = axes[0]
        g.bodyAxis = [1,0,0]
        g.coneThreshold = 0.0
        g.tspan = [1,1]
        constraints.append(g)
        constraints.append(self.robotSystem.ikPlanner.createQuasiStaticConstraint())
        constraints.append(self.robotSystem.ikPlanner.createXYZMovingBasePostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedRightArmPostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createFixedLinkConstraints(startPoseName, 'l_foot'))
        constraints.append(self.robotSystem.ikPlanner.createFixedLinkConstraints(startPoseName, 'r_foot'))
        constraintSet = ConstraintSet(self.robotSystem.ikPlanner, constraints, endPoseName, startPoseName)

        constraintSet.runIk()
        plan = constraintSet.planEndPoseGoal(feetOnGround=False)

        self.robotSystem.ikPlanner.setIkParameters(originalIkParameterDict)
        return plan


    def planFootOut(self):
        ikParameterDict = {'usePointwise': True,
                           'leftFootSupportEnabled': False,
                           'rightFootSupportEnabled': True,
                           'pelvisSupportEnabled': False}
        originalIkParameterDict = self.robotSystem.ikPlanner.setIkParameters(ikParameterDict)

        startPose = self.getPlanningStartPose()
        startPoseName = 'q_egress_start'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_egress_end'
        constraints = []

        utorsoFrame = self.robotSystem.ikPlanner.getLinkFrameAtPose('utorso', startPose)
        g = ik.WorldGazeDirConstraint()
        g.linkName = 'utorso'
        g.targetFrame = vtk.vtkTransform()
        axes = transformUtils.getAxesFromTransform(self.polaris.leftFootEgressInsideFrame.transform)
        g.targetAxis = axes[0]
        g.bodyAxis = [1,0,0]
        g.coneThreshold = 0.0
        g.tspan = [1,1]
        constraints.append(g)
        constraints.append(self.robotSystem.ikPlanner.createQuasiStaticConstraint())
        #constraints.append(self.robotSystem.ikPlanner.createMovingBaseSafeLimitsConstraint())
        constraints.append(self.robotSystem.ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedRightArmPostureConstraint(startPoseName))
        #constraints.append(self.robotSystem.ikPlanner.createLockedBackPostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createFixedLinkConstraints(startPoseName, 'r_foot'))
        constraints.extend(self.createLeftFootPoseConstraint(self.polaris.leftFootEgressOutsideFrame, tspan=[1,1]))
        constraintSet = ConstraintSet(self.robotSystem.ikPlanner, constraints, endPoseName, startPoseName)

        constraintSet.runIk()

        footFrame = self.robotSystem.ikPlanner.getLinkFrameAtPose('l_foot', startPose)
        t = transformUtils.frameFromPositionAndRPY([0, 0, self.polaris.leftFootEgressInsideFrame.transform.GetPosition()[2]-footFrame.GetPosition()[2]], [0, 0, 0])
        liftFrame = transformUtils.concatenateTransforms([t, footFrame])
        vis.updateFrame(liftFrame, 'lift frame')

        c = ik.WorldFixedOrientConstraint()
        c.linkName = 'l_foot'
        c.tspan = [0.0, 0.1, 0.2]
        constraints.append(c)
        constraints.extend(self.createLeftFootPoseConstraint(liftFrame, tspan=[0.2,0.2]))
        #constraints.extend(self.createLeftFootPoseConstraint(self.polaris.leftFootEgressInsideFrame, tspan=[0.5,0.5]))
        constraintSet.constraints.extend(self.createLeftFootPoseConstraint(self.polaris.leftFootEgressMidFrame, tspan=[0.8,0.8]))
        #plan = constraintSet.planEndPoseGoal(feetOnGround=False)
        plan = constraintSet.runIkTraj()

        self.robotSystem.ikPlanner.setIkParameters(originalIkParameterDict)
        return plan

    def planLeftFootDown(self):
        ikPlanner = self.robotSystem.ikPlanner
        startPose = self.getPlanningStartPose()
        startPoseName = 'q_footdown_start'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_footdown_end'
        lFoot2World = transformUtils.copyFrame(self.polaris.leftFootEgressOutsideFrame.transform)
        rFoot2World = self.robotSystem.ikPlanner.getLinkFrameAtPose('r_foot', startPose)
        lFoot2World.PostMultiply()
        lFoot2World.Translate(np.array(rFoot2World.GetPosition()) - lFoot2World.GetPosition())
        lFoot2World.PreMultiply()
        lFoot2World.Translate([0.05, 0.26, 0.0])
        
        rFootRPY = transformUtils.rollPitchYawFromTransform(rFoot2World)
        lFootRPY = transformUtils.rollPitchYawFromTransform(lFoot2World);
        lFootxyz,_ = transformUtils.poseFromTransform(lFoot2World)

        lFootRPY[0] = rFootRPY[0]
        lFootRPY[1] = rFootRPY[1]
        lFoot2World = transformUtils.frameFromPositionAndRPY(lFootxyz, np.rad2deg(lFootRPY))
        
        ikParameterDict = {'leftFootSupportEnabled': False, 'rightfootSupportEnabled':True, 'pelvisSupportEnabled': False,
        'quasiStaticShrinkFactor': 0.2}
        ikParametersOriginal = ikPlanner.setIkParameters(ikParameterDict)
        quasiStaticConstraint = ikPlanner.createQuasiStaticConstraint()
        rfootFixedConstraint = ikPlanner.createFixedFootConstraints(startPoseName)
        identityFrame = vtk.vtkTransform()
        lfootPositionOrientationConstraint = ikPlanner.createPositionOrientationConstraint('l_foot', lFoot2World, identityFrame)
        backLocked = ikPlanner.createLockedBackPostureConstraint(startPoseName)
        armsLocked = ikPlanner.createLockedArmsPostureConstraints(startPoseName)

        constraints = [quasiStaticConstraint, backLocked]
        constraints.extend(lfootPositionOrientationConstraint)
        constraints.extend(rfootFixedConstraint)
        constraints.extend(armsLocked)
        # return constraints
        cs = ConstraintSet(ikPlanner, constraints, endPoseName, startPoseName)
        cs.seedPoseName = 'q_start'
        cs.nominalPoseName = 'q_start'
        endPose = cs.runIk()
        keyFramePlan = cs.planEndPoseGoal()
        poseTimes, poses = planplayback.PlanPlayback.getPlanPoses(keyFramePlan)
        ts = [poseTimes[0], poseTimes[-1]]
        supportsList = [['r_foot'], ['r_foot','l_foot']]
        self.publishPlanWithSupports(keyFramePlan, supportsList, ts, False)

        ikPlanner.setIkParameters(ikParametersOriginal)


    def planCenterWeight(self):
        ikPlanner = self.robotSystem.ikPlanner
        startPose = self.getPlanningStartPose()
        startPoseName = 'q_lean_right'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_egress_end'
        ikParameterDict = {'leftFootSupportEnabled': True, 'rightfootSupportEnabled':True, 'pelvisSupportEnabled': False,
        'quasiStaticShrinkFactor': 0.2}

        ikParametersOriginal = ikPlanner.setIkParameters(ikParameterDict)
        quasiStaticConstraint = ikPlanner.createQuasiStaticConstraint()

        footFixedConstraints = ikPlanner.createFixedFootConstraints(startPoseName)
        backConstraint = ikPlanner.createMovingBackLimitedPostureConstraint()
        armsLocked = ikPlanner.createLockedArmsPostureConstraints(startPoseName)

        constraints = [backConstraint, quasiStaticConstraint]
        constraints.extend(footFixedConstraints)
        constraints.extend(armsLocked)

        cs = ConstraintSet(ikPlanner, constraints, endPoseName, startPoseName)
        cs.seedPoseName = 'q_start'
        cs.nominalPoseName = 'q_nom'
        endPose = cs.runIk()
        keyFramePlan = cs.planEndPoseGoal()
        poseTimes, poses = planplayback.PlanPlayback.getPlanPoses(keyFramePlan)
        ts = [poseTimes[0]]
        supportsList = [['r_foot','l_foot']]
        self.publishPlanWithSupports(keyFramePlan, supportsList, ts, True)

        ikPlanner.setIkParameters(ikParametersOriginal)


    def publishPlanWithSupports(self, keyFramePlan, supportsList, ts, isQuasistatic):
        manipPlanner = self.robotSystem.manipPlanner
        msg_robot_plan_t = manipPlanner.convertKeyframePlan(keyFramePlan)
        supports = manipPlanner.getSupportLCMFromListOfSupports(supportsList,ts)
        msg_robot_plan_with_supports_t = manipPlanner.convertPlanToPlanWithSupports(msg_robot_plan_t, supports, ts, isQuasistatic)
        lcmUtils.publish('CANDIDATE_ROBOT_PLAN_WITH_SUPPORTS', msg_robot_plan_with_supports_t)


class EgressPanel(TaskUserPanel):

    def __init__(self, robotSystem):

        TaskUserPanel.__init__(self, windowTitle='Egress')

        self.robotSystem = robotSystem
        self.sitStandPlanner = sitstandplanner.SitStandPlanner(robotSystem.ikServer, robotSystem)
        self.platformPlanner = polarisplatformplanner.PolarisPlatformPlanner(robotSystem.ikServer, robotSystem)
        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()


    def addButtons(self):
        #sit/stand buttons
        self.addManualButton('Start', self.onStart)
        self.addManualButton('Sit', functools.partial(self.onPlan, 'sit'))
        self.addManualButton('Stand', functools.partial(self.onPlan, 'stand'))
        self.addManualButton('Squat', functools.partial(self.onPlan, 'squat'))
        self.addManualButton('Stand From Squat', functools.partial(self.onPlan, 'stand_from_squat'))
        self.addManualButton('Sit From Current', functools.partial(self.onPlan, 'sit_from_current'))
        self.addManualButton('Hold With Pelvis Contact', functools.partial(self.onPlan, 'hold_with_pelvis_contact'))
        self.addManualButton('Hold Without Pelvis Contact', functools.partial(self.onPlan, 'hold_without_pelvis_contact'))

        # polaris step down buttons
        self.addManualButton('Fit Platform Affordance', self.platformPlanner.fitRunningBoardAtFeet)
        self.addManualButton('Spawn Ground Affordance', self.platformPlanner.spawnGroundAffordance)
        self.addManualButton('Raycast Terrain', self.platformPlanner.requestRaycastTerrain)
        self.addManualButton('Update Affordance', self.platformPlanner.updateAffordance)
        self.addManualButton('Arms Up',self.onArmsUp)
        self.addManualButton('Plan Turn', self.onPlanTurn)
        self.addManualButton('Plan Step Down', self.onPlanStepDown)
        self.addManualButton('Plan Weight Shift', self.onPlanWeightShift)
        self.addManualButton('Plan Step Off', self.onPlanStepOff)

    def addDefaultProperties(self):
        self.params.addProperty('Chair Height', 0.57, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Sit Back Distance', 0.2, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Speed', 1, attributes=om.PropertyAttributes(singleStep=0.1, decimals=3))
        self.params.addProperty('Back Gaze Bound', 0.4, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Min Distance', 0.1, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Pelvis Gaze Bound', 0.1, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Pelvis Gaze Angle', 0, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Back y Angle', -0.2, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))

        self.params.addProperty('Step Off Direction', 0, attributes=om.PropertyAttributes(enumNames=['Forwards','Sideways']))

    def _syncProperties(self):
        self.sitStandPlanner.planOptions['chair_height'] = self.params.getProperty('Chair Height')
        self.sitStandPlanner.planOptions['sit_back_distance'] = self.params.getProperty('Sit Back Distance')
        self.sitStandPlanner.planOptions['speed'] = self.params.getProperty('Speed')
        self.sitStandPlanner.planOptions['back_gaze_bound'] = self.params.getProperty('Back Gaze Bound')
        self.sitStandPlanner.planOptions['min_distance'] = self.params.getProperty('Min Distance')
        self.sitStandPlanner.planOptions['pelvis_gaze_bound']= self.params.getProperty('Pelvis Gaze Bound')
        self.sitStandPlanner.planOptions['pelvis_gaze_angle'] = self.params.getProperty('Pelvis Gaze Angle')
        self.sitStandPlanner.planOptions['bky_angle'] = self.params.getProperty('Back y Angle')
        self.stepOffDirection = self.params.getPropertyEnumValue('Step Off Direction').lower()
        self.sitStandPlanner.applyParams()

    def onStart(self):
        self._syncProperties()
        print 'Egress Planner Ready'

    def onUpdateAffordance(self):
        if not self.platformPlanner.initializedFlag:
            self.platformPlanner.initialize()

        self.platformPlanner.updateAffordance()

    def onPlan(self,planType):
        self._syncProperties()
        self.sitStandPlanner.plan(planType)

    def onPlanTurn(self):
        self._syncProperties()
        self.platformPlanner.planTurn()

    def onArmsUp(self):
        self.platformPlanner.planArmsUp(self.stepOffDirection)

    def onPropertyChanged(self, propertySet, propertyName):
        self._syncProperties()

    def onPlanStepDown(self):
        self._syncProperties()
        if self.stepOffDirection == 'forwards':
            self.platformPlanner.planStepDownForwards()
        else:
            self.platformPlanner.planStepDown()

    def onPlanWeightShift(self):
        self._syncProperties()
        if self.stepOffDirection == 'forwards':
            self.platformPlanner.planWeightShiftForwards()
        else:
            self.platformPlanner.planWeightShift()

    def onPlanStepOff(self):
        self._syncProperties()
        if self.stepOffDirection == 'forwards':
            self.platformPlanner.planStepOffForwards()
        else:
            self.platformPlanner.planStepOff()

    def addTasks(self):



        # some helpers
        self.folder = None
        def addTask(task, parent=None):
            parent = parent or self.folder
            self.taskTree.onAddTask(task, copy=False, parent=parent)
        def addFunc(func, name, parent=None):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)
        def addFolder(name, parent=None):
            self.folder = self.taskTree.addGroup(name, parent=parent)
            return self.folder

        def addManipTask(name, planFunc, userPrompt=False):

            folder = addFolder(name)
            addFunc(planFunc, name='plan')
            if not userPrompt:
                addTask(rt.CheckPlanInfo(name='check manip plan info'))
            else:
                addTask(rt.UserPromptTask(name='approve manip plan', message='Please approve manipulation plan.'))
            addFunc(pp.commitManipPlan, name='execute manip plan')
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'))

        pp = self.platformPlanner

        folder = addFolder('Prep')
        addFunc(self.onStart, 'start')
        addManipTask('arms up', self.onArmsUp, userPrompt=True)
        addFunc(pp.fitRunningBoardAtFeet, 'fit running board')
        addFunc(pp.spawnGroundAffordance, 'spawn ground affordance')
        addFunc(pp.requestRaycastTerrain, 'raycast terrain')
        addTask(rt.UserPromptTask(name="set walking params", message="Please set walking params to 'Polaris Platform'"))

        folder = addFolder('Step Down')
        addFunc(pp.spawnGroundAffordance, 'spawn ground affordance')
        addFunc(pp.requestRaycastTerrain, 'raycast terrain')
        addFunc(self.onPlanStepDown, 'plan step down')
        addTask(rt.UserPromptTask(name="approve/adjust footsteps", message="Please approve footsteps, modify if necessary"))
        addTask(rt.UserPromptTask(name="execute step down", message="Please execute walking plan"))
        addTask(rt.WaitForWalkExecution(name='wait for step down'))

        folder = addFolder('Step Off')
        addFunc(pp.spawnGroundAffordance, 'spawn ground affordance')
        addFunc(pp.requestRaycastTerrain, 'raycast terrain')
        addFunc(self.onPlanStepOff, 'plan step off')
        addTask(rt.UserPromptTask(name="approve/adjust footsteps", message="Please approve footsteps, modify if necessary"))
        addTask(rt.UserPromptTask(name="execute step off", message="Please execute walking plan"))

