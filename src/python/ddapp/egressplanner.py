#import ddapp
from ddapp import cameraview
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp import objectmodel as om
from ddapp import ik
from ddapp.ikparameters import IkParameters
from ddapp.ikplanner import ConstraintSet
from ddapp import polarisplatformplanner
from ddapp import robotstate
from ddapp import segmentation
from ddapp import sitstandplanner
from ddapp.timercallback import TimerCallback
from ddapp import visualization as vis
from ddapp import planplayback
from ddapp import lcmUtils
from ddapp import affordancepanel
from ddapp.uuidutil import newUUID

import os
import functools
import numpy as np
import scipy.io
import vtkAll as vtk
import bot_core as lcmbotcore
from ddapp.tasks.taskuserpanel import TaskUserPanel
import ddapp.tasks.robottasks as rt
from ddapp import filterUtils
from ddapp import ioUtils


class PolarisModel(object):

    def __init__(self):
        self.aprilTagSubsciber = lcmUtils.addSubscriber('APRIL_TAG_TO_CAMERA_LEFT', lcmbotcore.rigid_transform_t, self.onAprilTag)
        pose = transformUtils.poseFromTransform(vtk.vtkTransform())
        desc = dict(classname='MeshAffordanceItem', Name='polaris',
                    Filename='software/models/polaris/polaris_cropped.vtp', pose=pose)
        self.pointcloudAffordance = segmentation.affordanceManager.newAffordanceFromDescription(desc)
        self.originFrame = self.pointcloudAffordance.getChildFrame()
        self.originToAprilTransform = vtk.vtkTransform() # this should be updated when we remount the April tag

        t = transformUtils.transformFromPose(np.array([-0.96427236,  0.06193934,  0.3543806 ]),
            np.array([ 0.18153211,  0.83526159,  0.42728504,  0.29463821]))
        self.leftFootEgressStartFrame  = vis.updateFrame(t, 'left foot start', scale=0.2,visible=True, parent=self.pointcloudAffordance)

        t = transformUtils.transformFromPose(np.array([-0.93707546,  0.07409333,  0.32871604]),
                                             np.array([ 0.22455191,  0.71396247,  0.60983921,
                                                       0.26063418]))
        self.leftFootEgressInsideFrame  = vis.updateFrame(t, 'left foot inside', scale=0.2,visible=True, parent=self.pointcloudAffordance)

        t = transformUtils.transformFromPose(np.array([-0.89783714,  0.23503719,  0.29039189]),
                                             np.array([ 0.2331762 ,  0.69031269,  0.6311807,
                                                       0.2659101]))
        self.leftFootEgressMidFrame  = vis.updateFrame(t, 'left foot mid', scale=0.2,visible=True, parent=self.pointcloudAffordance)

        t = transformUtils.transformFromPose(np.array([-0.88436275,  0.50939115,  0.31281047]),
                                             np.array([ 0.22600245,  0.69177731,  0.63305905,
                                                       0.26382435]))
        self.leftFootEgressOutsideFrame  = vis.updateFrame(t, 'left foot outside', scale=0.2,visible=True, parent=self.pointcloudAffordance)


        pose = [np.array([-0.43284877, -0.82362299, -0.24939116]), np.array([ 0.00764553,  0.64088459, -0.01054815,  0.7675267 ])]

        desc = dict(classname='CapsuleRingAffordanceItem', Name='Steering Wheel', uuid=newUUID(), pose=pose,
                    Color=[1, 0, 0], Radius=float(0.18), Segments=20)
        self.steeringWheelAffordance = segmentation.affordanceManager.newAffordanceFromDescription(desc)

        t = transformUtils.transformFromPose(np.array([-0.95565326,  0.00645136,  0.30410111]),
            np.array([ 0.2638261 ,  0.88689326,  0.36270714,  0.11072339]))

        self.leftFootPedalSwingFrame = vis.updateFrame(t,'left foot pedal swing', scale=0.2, visible=True, parent=self.pointcloudAffordance)

        t = transformUtils.transformFromPose(np.array([-0.90084703, -0.05962708,  0.31975967]),
            np.array([ 0.03968859,  0.99239755,  0.03727381,  0.11037472]))

        self.leftFootDrivingFrame = vis.updateFrame(t,'left foot driving', scale=0.2, visible=True, parent=self.pointcloudAffordance)

        t = transformUtils.transformFromPose(np.array([ 0.0584505 ,  0.43832744,  0.02401199]),
            np.array([ 0.62148369,  0.32887677, -0.32946879, -0.63011777]))
        self.rightHandGrabFrame = vis.updateFrame(t,'right hand grab bar', scale=0.2, visible=True, parent=self.pointcloudAffordance)

        self.frameSync = vis.FrameSync()
        self.frameSync.addFrame(self.originFrame)
        self.frameSync.addFrame(self.pointcloudAffordance.getChildFrame(), ignoreIncoming=True)
        self.frameSync.addFrame(self.leftFootEgressStartFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.leftFootEgressInsideFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.leftFootEgressMidFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.leftFootEgressOutsideFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.steeringWheelAffordance.getChildFrame(), ignoreIncoming=True)
        self.frameSync.addFrame(self.leftFootPedalSwingFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.leftFootDrivingFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.rightHandGrabFrame, ignoreIncoming=True)

    def onAprilTag(self, msg):
        t = vtk.vtkTransform()
        cameraview.imageManager.queue.getTransform('april_tag_car_beam', 'local', msg.utime, t)
        self.originFrame.copyFrame(transformUtils.concatenateTransforms([self.originToAprilTransform, t]))

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


    def planEgressArms(self):
        startPose = self.getPlanningStartPose()
        endPose = self.robotSystem.ikPlanner.getMergedPostureFromDatabase(startPose, 'driving', 'egress-arms')
        return self.robotSystem.ikPlanner.computePostureGoal(startPose, endPose)

    def planGetWeightOverFeet(self):
        startPose = self.getPlanningStartPose()
        startPoseName = 'q_egress_start'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_egress_end'
        constraints = []
        constraints.append(ik.QuasiStaticConstraint(leftFootEnabled=True, rightFootEnabled=True, pelvisEnabled=False))
        constraints.append(self.robotSystem.ikPlanner.createLockedBasePostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
        #constraints.append(self.robotSystem.ikPlanner.createLockedRightArmPostureConstraint(startPoseName))
        constraintSet = ConstraintSet(self.robotSystem.ikPlanner, constraints, endPoseName, startPoseName)
        constraintSet.ikParameters = IkParameters(usePointwise=False, maxDegreesPerSecond=7)

        constraintSet.runIk()

        keyFramePlan =  constraintSet.planEndPoseGoal(feetOnGround=False)
        poseTimes, poses = planplayback.PlanPlayback.getPlanPoses(keyFramePlan)
        ts = [poseTimes[0]]
        supportsList = [['r_foot', 'l_foot', 'pelvis']]
        plan = self.publishPlanWithSupports(keyFramePlan, supportsList, ts, True)

        return plan

    def planStandUp(self):
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
        constraints.append(ik.QuasiStaticConstraint(leftFootEnabled=True, rightFootEnabled=True, pelvisEnabled=False))
        constraints.append(self.robotSystem.ikPlanner.createXYZMovingBasePostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
        #constraints.append(self.robotSystem.ikPlanner.createLockedRightArmPostureConstraint(startPoseName))
        constraints.extend(self.robotSystem.ikPlanner.createFixedFootConstraints(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createKneePostureConstraint([0.7, 2.5]))
        constraintSet = ConstraintSet(self.robotSystem.ikPlanner, constraints, endPoseName, startPoseName)
        constraintSet.ikParameters = IkParameters(usePointwise=True)

        constraintSet.runIk()
        keyFramePlan = constraintSet.planEndPoseGoal(feetOnGround=False)
        poseTimes, poses = planplayback.PlanPlayback.getPlanPoses(keyFramePlan)
        ts = [poseTimes[0]]
        supportsList = [['r_foot', 'l_foot']]
        plan = self.publishPlanWithSupports(keyFramePlan, supportsList, ts, True)

        return plan

    def planShiftWeightOut(self):

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
        constraints.append(ik.QuasiStaticConstraint(leftFootEnabled=False, rightFootEnabled=True,
                                                    pelvisEnabled=False))
        constraints.append(self.robotSystem.ikPlanner.createXYZMovingBasePostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedRightArmPostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createFixedLinkConstraints(startPoseName, 'l_foot'))
        constraints.append(self.robotSystem.ikPlanner.createFixedLinkConstraints(startPoseName, 'r_foot'))
        constraintSet = ConstraintSet(self.robotSystem.ikPlanner, constraints, endPoseName, startPoseName)
        constraintSet.ikParameters = IkParameters(usePointwise=True)

        constraintSet.runIk()
        keyFramePlan = constraintSet.planEndPoseGoal(feetOnGround=False)
        poseTimes, poses = planplayback.PlanPlayback.getPlanPoses(keyFramePlan)
        ts = [poseTimes[0]]
        supportsList = [['r_foot', 'l_foot']]
        plan = self.publishPlanWithSupports(keyFramePlan, supportsList, ts, True)

        return plan

    def planFootOut(self):

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
        constraints.append(ik.QuasiStaticConstraint(leftFootEnabled=False, rightFootEnabled=True,
                                                    pelvisEnabled=False, shrinkFactor=0.01))
        constraints.append(self.robotSystem.ikPlanner.createMovingBaseSafeLimitsConstraint())
        constraints.append(self.robotSystem.ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedRightArmPostureConstraint(startPoseName))
        #constraints.append(self.robotSystem.ikPlanner.createLockedBackPostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createFixedLinkConstraints(startPoseName, 'r_foot'))
        constraints.extend(self.createLeftFootPoseConstraint(self.polaris.leftFootEgressOutsideFrame, tspan=[1,1]))
        constraintSet = ConstraintSet(self.robotSystem.ikPlanner, constraints, endPoseName, startPoseName)
        constraintSet.ikParameters = IkParameters(usePointwise=True)

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
        keyFramePlan = constraintSet.runIkTraj()
        poseTimes, poses = planplayback.PlanPlayback.getPlanPoses(keyFramePlan)
        ts = [poseTimes[0]]
        supportsList = [['r_foot']]
        plan = self.publishPlanWithSupports(keyFramePlan, supportsList, ts, False)

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

        constraints.append(ik.QuasiStaticConstraint(leftFootEnabled=False, rightFootEnabled=True,
                                                    pelvisEnabled=False, shrinkFactor=0.2))
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
        constraintSet.ikParameters = IkParameters(usePointwise=True)
        cs.seedPoseName = 'q_start'
        cs.nominalPoseName = 'q_start'
        endPose = cs.runIk()
        keyFramePlan = cs.planEndPoseGoal()
        poseTimes, poses = planplayback.PlanPlayback.getPlanPoses(keyFramePlan)
        ts = [poseTimes[0], poseTimes[-1]]
        supportsList = [['r_foot'], ['r_foot','l_foot']]
        self.publishPlanWithSupports(keyFramePlan, supportsList, ts, False)


    def planCenterWeight(self):
        ikPlanner = self.robotSystem.ikPlanner
        startPose = self.getPlanningStartPose()
        startPoseName = 'q_lean_right'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_egress_end'

        constraints.append(ik.QuasiStaticConstraint(leftFootEnabled=True, rightFootEnabled=True,
                                                    pelvisEnabled=False, shrinkFactor=0.2))

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


    def publishPlanWithSupports(self, keyFramePlan, supportsList, ts, isQuasistatic):
        manipPlanner = self.robotSystem.manipPlanner
        msg_robot_plan_t = manipPlanner.convertKeyframePlan(keyFramePlan)
        supports = manipPlanner.getSupportLCMFromListOfSupports(supportsList,ts)
        msg_robot_plan_with_supports_t = manipPlanner.convertPlanToPlanWithSupports(msg_robot_plan_t, supports, ts, isQuasistatic)
        lcmUtils.publish('CANDIDATE_ROBOT_PLAN_WITH_SUPPORTS', msg_robot_plan_with_supports_t)
        return msg_robot_plan_with_supports_t

    def getFrameToOriginTransform(self, t):
        tCopy = transformUtils.copyFrame(t)
        tCopy.PostMultiply()
        tCopy.Concatenate(self.polaris.originFrame.transform.GetLinearInverse())
        print transformUtils.poseFromTransform(tCopy)
        return tCopy



class EgressPanel(TaskUserPanel):

    def __init__(self, robotSystem):

        TaskUserPanel.__init__(self, windowTitle='Egress')

        self.robotSystem = robotSystem
        self.platformPlanner = polarisplatformplanner.PolarisPlatformPlanner(robotSystem.ikServer, robotSystem)
        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()


    def addButtons(self):
        #sit/stand buttons
        self.addManualButton('Start', self.onStart)
        # polaris step down buttons
        self.addManualButton('Fit Platform Affordance', self.platformPlanner.fitRunningBoardAtFeet)
        self.addManualButton('Spawn Ground Affordance', self.platformPlanner.spawnGroundAffordance)
        self.addManualButton('Raycast Terrain', self.platformPlanner.requestRaycastTerrain)
        self.addManualButton('Update Affordance', self.platformPlanner.updateAffordance)
        self.addManualButton('Arms Up',self.onArmsUp)
        self.addManualButton('Plan Step Down', self.onPlanStepDown)
        self.addManualButton('Plan Step Off', self.onPlanStepOff)

    def addDefaultProperties(self):
        self.params.addProperty('Step Off Direction', 0, attributes=om.PropertyAttributes(enumNames=['Forwards','Sideways']))

    def _syncProperties(self):
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
        addTask(rt.UserPromptTask(name="approve footsteps, set support contact group",
         message="Please approve/modify footsteps. Set the support contact group for the left foot step to be Back 2/3"))
        addTask(rt.UserPromptTask(name="execute step down", message="Please execute walking plan"))
        addTask(rt.WaitForWalkExecution(name='wait for step down'))

        folder = addFolder('Step Off')
        addFunc(pp.spawnGroundAffordance, 'spawn ground affordance')
        addFunc(pp.requestRaycastTerrain, 'raycast terrain')
        addFunc(self.onPlanStepOff, 'plan step off')
        addTask(rt.UserPromptTask(name="approve/adjust footsteps", message="Please approve footsteps, modify if necessary"))
        addTask(rt.UserPromptTask(name="execute step off", message="Please execute walking plan"))

