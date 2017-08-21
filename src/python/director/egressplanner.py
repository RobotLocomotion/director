#import director
from director import cameraview
from director import transformUtils
from director import visualization as vis
from director import objectmodel as om
from director import ikconstraints
from director.ikparameters import IkParameters
from director.ikplanner import ConstraintSet
from director import polarisplatformplanner
from director import robotstate
from director import segmentation
from director import sitstandplanner
from director.timercallback import TimerCallback
from director import visualization as vis
from director import planplayback
from director import lcmUtils
from director.uuidutil import newUUID

import os
import functools
import numpy as np
import scipy.io
from . import vtkAll as vtk
import bot_core as lcmbotcore
from director.tasks.taskuserpanel import TaskUserPanel
import director.tasks.robottasks as rt
from director import filterUtils
from director import ioUtils


class PolarisModel(object):

    def __init__(self):
        self.aprilTagSubsciber = lcmUtils.addSubscriber('APRIL_TAG_TO_CAMERA_LEFT', lcmbotcore.rigid_transform_t, self.onAprilTag)
        pose = transformUtils.poseFromTransform(vtk.vtkTransform())
        desc = dict(classname='MeshAffordanceItem', Name='polaris',
                    Filename='software/models/polaris/polaris_cropped.vtp', pose=pose)
        self.pointcloudAffordance = segmentation.affordanceManager.newAffordanceFromDescription(desc)
        self.originFrame = self.pointcloudAffordance.getChildFrame()
        self.originToAprilTransform = transformUtils.transformFromPose(np.array([-0.038508  , -0.00282131, -0.01000079]),
            np.array([  9.99997498e-01,  -2.10472556e-03,  -1.33815696e-04, 7.46246794e-04])) # offset for  . . . who knows why

        # t = transformUtils.transformFromPose(np.array([ 0.14376024,  0.95920689,  0.36655712]), np.array([ 0.28745842,  0.90741428, -0.28822068,  0.10438304]))

        t = transformUtils.transformFromPose(np.array([ 0.10873244,  0.93162364,  0.40509084]),
            np.array([ 0.32997378,  0.88498408, -0.31780588,  0.08318602]))
        self.leftFootEgressStartFrame  = vis.updateFrame(t, 'left foot start', scale=0.2,visible=True, parent=self.pointcloudAffordance)


        t = transformUtils.transformFromPose(np.array([ 0.265,  0.874,  0.274]),
                                             np.array([ 0.35290731,  0.93443693, -0.04181263,  0.02314636]))
        self.leftFootEgressMidFrame  = vis.updateFrame(t, 'left foot mid', scale=0.2,visible=True, parent=self.pointcloudAffordance)

        t = transformUtils.transformFromPose(np.array([ 0.54339115,  0.89436275,  0.26681047]),
                                             np.array([ 0.34635985,  0.93680077, -0.04152008,  0.02674412]))
        self.leftFootEgressOutsideFrame  = vis.updateFrame(t, 'left foot outside', scale=0.2,visible=True, parent=self.pointcloudAffordance)


        # pose = [np.array([-0.78962299,  0.44284877, -0.29539116]), np.array([ 0.54812954,  0.44571517, -0.46063251,  0.53731713])] #old location
        # pose = [np.array([-0.78594663,  0.42026626, -0.23248139]), np.array([ 0.54812954,  0.44571517, -0.46063251,  0.53731713])] # updated location

        pose = [np.array([-0.78594663,  0.42026626, -0.23248139]), np.array([ 0.53047159,  0.46554963, -0.48086192,  0.52022615])] # update orientation

        desc = dict(classname='CapsuleRingAffordanceItem', Name='Steering Wheel', uuid=newUUID(), pose=pose,
                    Color=[1, 0, 0], Radius=float(0.18), Segments=20)
        self.steeringWheelAffordance = segmentation.affordanceManager.newAffordanceFromDescription(desc)


        pose = [np.array([-0.05907324,  0.80460545,  0.45439687]), np.array([ 0.14288327,  0.685944  , -0.703969  ,  0.11615873])]

        desc = dict(classname='BoxAffordanceItem', Name='pedal', Dimensions=[0.12, 0.33, 0.04], pose=pose, Color=[0,1,0])
        self.pedalAffordance = segmentation.affordanceManager.newAffordanceFromDescription(desc)


        # t = transformUtils.transformFromPose(np.array([ 0.04045136,  0.96565326,  0.25810111]),
        #     np.array([ 0.26484648,  0.88360091, -0.37065556, -0.10825996]))

        # t = transformUtils.transformFromPose(np.array([ -4.34908919e-04,   9.24901627e-01,   2.65614116e-01]),
        #     np.array([ 0.25022251,  0.913271  , -0.32136359, -0.00708626]))

        t = transformUtils.transformFromPose(np.array([ 0.0384547 ,  0.89273742,  0.24140762]),
            np.array([ 0.26331831,  0.915796  , -0.28055337,  0.11519963]))

        self.leftFootPedalSwingFrame = vis.updateFrame(t,'left foot pedal swing', scale=0.2, visible=True, parent=self.pointcloudAffordance)

        t = transformUtils.transformFromPose(np.array([-0.9012598 , -0.05709763,  0.34897024]),
            np.array([ 0.03879584,  0.98950919,  0.03820214,  0.13381721]))

        self.leftFootDrivingFrame = vis.updateFrame(t,'left foot driving', scale=0.2, visible=True, parent=self.pointcloudAffordance)
        # t = transformUtils.transformFromPose(np.array([-0.12702725,  0.92068409,  0.27209386]),
        #     np.array([ 0.2062255 ,  0.92155886, -0.30781119,  0.11598529]))


        # t = transformUtils.transformFromPose(np.array([-0.14940375,  0.90347275,  0.23812658]),
        #     np.array([ 0.27150909,  0.91398724, -0.28877386,  0.0867167 ]))

        # t = transformUtils.transformFromPose(np.array([-0.17331227,  0.87879312,  0.25645152]),
        #     np.array([ 0.26344489,  0.91567196, -0.28089824,  0.11505581]))
        # self.leftFootDrivingKneeInFrame = vis.updateFrame(t,'left foot driving knee in', scale=0.2, visible=True, parent=self.pointcloudAffordance)


        t = transformUtils.transformFromPose(np.array([-0.12702725,  0.92068409,  0.27209386]),
            np.array([ 0.2062255 ,  0.92155886, -0.30781119,  0.11598529]))
        self.leftFootDrivingKneeInFrame = vis.updateFrame(t,'left foot driving knee in', scale=0.2, visible=True, parent=self.pointcloudAffordance)

        t = transformUtils.transformFromPose(np.array([-0.13194951,  0.89871423,  0.24956246]),
            np.array([ 0.21589082,  0.91727326, -0.30088849,  0.14651633]))
        self.leftFootOnPedal = vis.updateFrame(t,'left foot on pedal', scale=0.2, visible=True, parent=self.pointcloudAffordance)

        t = transformUtils.transformFromPose(np.array([ 0.17712239,  0.87619935,  0.27001509]),
            np.array([ 0.33484372,  0.88280787, -0.31946488,  0.08044963]))

        self.leftFootUpFrame = vis.updateFrame(t,'left foot up frame', scale=0.2, visible=True, parent=self.pointcloudAffordance)


        t = transformUtils.transformFromPose(np.array([ 0.47214939, -0.04856998,  0.01375837]),
            np.array([  6.10521653e-03,   4.18621358e-04,   4.65520611e-01,
         8.85015882e-01]))
        self.rightHandGrabFrame = vis.updateFrame(t,'right hand grab bar', scale=0.2, visible=True, parent=self.pointcloudAffordance)

        self.frameSync = vis.FrameSync()
        self.frameSync.addFrame(self.originFrame)
        self.frameSync.addFrame(self.pointcloudAffordance.getChildFrame(), ignoreIncoming=True)
        self.frameSync.addFrame(self.leftFootEgressStartFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.leftFootEgressMidFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.leftFootEgressOutsideFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.steeringWheelAffordance.getChildFrame(), ignoreIncoming=True)
        self.frameSync.addFrame(self.pedalAffordance.getChildFrame(), ignoreIncoming=True)
        self.frameSync.addFrame(self.leftFootPedalSwingFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.leftFootDrivingFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.leftFootDrivingKneeInFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.leftFootOnPedal, ignoreIncoming=True)
        self.frameSync.addFrame(self.leftFootUpFrame, ignoreIncoming=True)
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

        self.coneThreshold = np.radians(5)

        self.robotSystem = robotSystem
        self.polaris = None
        self.quasiStaticShrinkFactor = 0.1
        self.maxFootTranslationSpeed = 0.05
        self.maxHandTranslationSpeed = 0.1
        self.plans = []

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

    def addPlan(self, plan):
        self.plans.append(plan)

    def commitManipPlan(self):
        self.robotSystem.manipPlanner.commitManipPlan(self.plans[-1])

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
        constraints.append(ikconstraints.QuasiStaticConstraint(leftFootEnabled=True, rightFootEnabled=True,
                                                    pelvisEnabled=False,
                                                    shrinkFactor=0.5))
        constraints.append(self.robotSystem.ikPlanner.createLockedBasePostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedRightArmPostureConstraint(startPoseName))
        constraintSet = ConstraintSet(self.robotSystem.ikPlanner, constraints, endPoseName, startPoseName)
        constraintSet.ikParameters = IkParameters(usePointwise=False, maxDegreesPerSecond=10)

        constraintSet.runIk()

        keyFramePlan =  constraintSet.planEndPoseGoal(feetOnGround=False)
        poseTimes, poses = planplayback.PlanPlayback.getPlanPoses(keyFramePlan)
        ts = [poseTimes[0]]
        supportsList = [['r_foot', 'l_foot', 'pelvis']]
        plan = self.publishPlanWithSupports(keyFramePlan, supportsList, ts, True)
        self.addPlan(plan)
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
        utorsoFrame = self.robotSystem.ikPlanner.getLinkFrameAtPose('utorso', startPose)
        g = self.createUtorsoGazeConstraints([1.0, 1.0])
        constraints.append(g[1])
        p = ikconstraints.PositionConstraint(linkName='pelvis', referenceFrame=liftFrame,
                                  lowerBound=np.array([0.0, -np.inf, 0.0]),
                                  upperBound=np.array([np.inf, np.inf, 0.0]))
        constraints.append(p)
        constraints.append(ikconstraints.QuasiStaticConstraint(leftFootEnabled=True, rightFootEnabled=True, pelvisEnabled=False,
                                                    shrinkFactor=self.quasiStaticShrinkFactor))
        constraints.append(self.robotSystem.ikPlanner.createXYZMovingBasePostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedRightArmPostureConstraint(startPoseName))
        constraints.extend(self.robotSystem.ikPlanner.createFixedFootConstraints(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createKneePostureConstraint([0.7, 2.5]))
        constraintSet = ConstraintSet(self.robotSystem.ikPlanner, constraints, endPoseName, startPoseName)
        constraintSet.ikParameters = IkParameters(usePointwise=True, maxBaseMetersPerSecond=0.02)

        constraintSet.runIk()
        keyFramePlan = constraintSet.planEndPoseGoal(feetOnGround=True)
        poseTimes, poses = planplayback.PlanPlayback.getPlanPoses(keyFramePlan)
        ts = [poseTimes[0]]
        supportsList = [['r_foot', 'l_foot']]
        plan = self.publishPlanWithSupports(keyFramePlan, supportsList, ts, True)
        self.addPlan(plan)
        return plan

    def createUtorsoGazeConstraints(self, tspan):
        constraints = []
        g = ikconstraints.WorldGazeDirConstraint()
        g.linkName = 'utorso'
        g.targetFrame = vtk.vtkTransform()
        axes = transformUtils.getAxesFromTransform(self.polaris.leftFootEgressOutsideFrame.transform)
        g.targetAxis = axes[0]
        g.bodyAxis = [1,0,0]
        g.coneThreshold = self.coneThreshold
        g.tspan = tspan
        constraints.append(g)

        g = ikconstraints.WorldGazeDirConstraint()
        g.linkName = 'utorso'
        g.targetFrame = vtk.vtkTransform()
        g.targetAxis = [0,0,1]
        g.bodyAxis = [0,0,1]
        g.coneThreshold = self.coneThreshold
        g.tspan = tspan
        constraints.append(g)
        return constraints

    def planFootEgress(self):
        def saveOriginalTraj(name):
            commands = ['%s = qtraj_orig;' % name]
            self.robotSystem.ikServer.comm.sendCommands(commands)

        def concatenateAndRescaleTrajectories(trajectoryNames, concatenatedTrajectoryName, junctionTimesName, ikParameters):
            commands = []
            commands.append('joint_v_max = repmat(%s*pi/180, r.getNumVelocities()-6, 1);' % ikParameters.maxDegreesPerSecond)
            commands.append('xyz_v_max = repmat(%s, 3, 1);' % ikParameters.maxBaseMetersPerSecond)
            commands.append('rpy_v_max = repmat(%s*pi/180, 3, 1);' % ikParameters.maxBaseRPYDegreesPerSecond)
            commands.append('v_max = [xyz_v_max; rpy_v_max; joint_v_max];')
            commands.append("max_body_translation_speed = %r;" % ikParameters.maxBodyTranslationSpeed)
            commands.append("max_body_rotation_speed = %r;" % ikParameters.maxBodyRotationSpeed)
            commands.append('rescale_body_ids = [%s];' % (','.join(['links.%s' % linkName for linkName in ikParameters.rescaleBodyNames])))
            commands.append('rescale_body_pts = reshape(%s, 3, []);' % ikconstraints.ConstraintBase.toColumnVectorString(ikParameters.rescaleBodyPts))
            commands.append("body_rescale_options = struct('body_id',rescale_body_ids,'pts',rescale_body_pts,'max_v',max_body_translation_speed,'max_theta',max_body_rotation_speed,'robot',r);")
            commands.append('trajectories = {};')
            for name in trajectoryNames:
                commands.append('trajectories{end+1} = %s;' % name)
            commands.append('[%s, %s] = concatAndRescaleTrajectories(trajectories, v_max, %s, %s, body_rescale_options);' % (concatenatedTrajectoryName, junctionTimesName, ikParameters.accelerationParam, ikParameters.accelerationFraction))
            commands.append('s.publishTraj(%s, 1);' % concatenatedTrajectoryName)
            self.robotSystem.ikServer.comm.sendCommands(commands)
            return self.robotSystem.ikServer.comm.getFloatArray(junctionTimesName)


        self.planShiftWeightOut()
        shiftWeightName = 'qtraj_shift_weight'
        saveOriginalTraj(shiftWeightName)
        nextStartPose = robotstate.convertStateMessageToDrakePose(self.plans[-1].plan.plan[-1])

        self.planFootOut(startPose=nextStartPose, finalFootHeight=0.0)
        footOutName = 'qtraj_foot_out'
        saveOriginalTraj(footOutName)
        nextStartPose = robotstate.convertStateMessageToDrakePose(self.plans[-1].plan.plan[-1])

        self.planCenterWeight(startPose=nextStartPose)
        centerWeightName = 'qtraj_center_weight'
        saveOriginalTraj(centerWeightName)

        ikParameters = IkParameters(usePointwise=True, maxBaseRPYDegreesPerSecond=10,
                                    rescaleBodyNames=['l_foot'], rescaleBodyPts=[0.0, 0.0, 0.0],
                                    maxBodyTranslationSpeed=self.maxFootTranslationSpeed)

        ikParameters = self.robotSystem.ikPlanner.mergeWithDefaultIkParameters(ikParameters)

        listener = self.robotSystem.ikPlanner.getManipPlanListener()
        supportTimes = concatenateAndRescaleTrajectories([shiftWeightName, footOutName, centerWeightName],
                                                         'qtraj_foot_egress', 'ts', ikParameters)
        keyFramePlan = listener.waitForResponse()
        listener.finish()

        supportsList = []
        supportsList.append(['l_foot', 'r_foot'])
        supportsList.append(['r_foot'])
        supportsList.append(['l_foot', 'r_foot'])
        supportsList.append(['l_foot', 'r_foot'])
        plan = self.publishPlanWithSupports(keyFramePlan, supportsList, supportTimes, True)
        self.addPlan(plan)





    def planShiftWeightOut(self, startPose=None):

        if startPose is None:
            startPose = self.getPlanningStartPose()
        startPoseName = 'q_egress_start'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_egress_end'
        constraints = []

        utorsoFrame = self.robotSystem.ikPlanner.getLinkFrameAtPose('utorso', startPose)
        constraints.extend(self.createUtorsoGazeConstraints([1.0, 1.0]))

        constraints.append(ikconstraints.QuasiStaticConstraint(leftFootEnabled=False, rightFootEnabled=True,
                                                    pelvisEnabled=False,
                                                    shrinkFactor=self.quasiStaticShrinkFactor))
        constraints.append(self.robotSystem.ikPlanner.createXYZMovingBasePostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedRightArmPostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createKneePostureConstraint([0.7, 2.5]))
        constraints.append(self.robotSystem.ikPlanner.createFixedLinkConstraints(startPoseName, 'l_foot'))
        constraints.append(self.robotSystem.ikPlanner.createFixedLinkConstraints(startPoseName, 'r_foot'))
        constraintSet = ConstraintSet(self.robotSystem.ikPlanner, constraints, endPoseName, startPoseName)
        constraintSet.ikParameters = IkParameters(usePointwise=True, maxBaseMetersPerSecond=0.02)

        constraintSet.runIk()
        keyFramePlan = constraintSet.planEndPoseGoal(feetOnGround=False)
        poseTimes, poses = planplayback.PlanPlayback.getPlanPoses(keyFramePlan)
        ts = [poseTimes[0]]
        supportsList = [['r_foot', 'l_foot']]
        plan = self.publishPlanWithSupports(keyFramePlan, supportsList, ts, True)
        self.addPlan(plan)
        return plan

    def computeLeftFootOverPlatformFrame(self, startPose, height):
        lFoot2World = transformUtils.copyFrame(self.polaris.leftFootEgressOutsideFrame.transform)
        rFoot2World = self.robotSystem.ikPlanner.getLinkFrameAtPose('r_foot', startPose)
        lFoot2World = transformUtils.copyFrame(rFoot2World)
        lFoot2World.PreMultiply()
        lFoot2World.Translate([0.05, 0.26, height])
        return lFoot2World

    def planFootOut(self, startPose=None, finalFootHeight=0.05):

        if startPose is None:
            startPose = self.getPlanningStartPose()

        startPoseName = 'q_egress_start'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_egress_end'

        utorsoFrame = self.robotSystem.ikPlanner.getLinkFrameAtPose('utorso', startPose)
        finalLeftFootFrame = self.computeLeftFootOverPlatformFrame(startPose, finalFootHeight)

        constraints = []
        constraints.extend(self.createUtorsoGazeConstraints([0.0, 1.0]))
        constraints.append(ikconstraints.QuasiStaticConstraint(leftFootEnabled=False, rightFootEnabled=True,
                                                    pelvisEnabled=False, shrinkFactor=0.01))
        constraints.append(self.robotSystem.ikPlanner.createMovingBaseSafeLimitsConstraint())
        constraints.append(self.robotSystem.ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedRightArmPostureConstraint(startPoseName))
        #constraints.append(self.robotSystem.ikPlanner.createLockedBackPostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createKneePostureConstraint([0.7, 2.5]))
        constraints.append(self.robotSystem.ikPlanner.createFixedLinkConstraints(startPoseName, 'r_foot'))
        constraints.extend(self.createLeftFootPoseConstraint(finalLeftFootFrame, tspan=[1,1]))

        constraintSet = ConstraintSet(self.robotSystem.ikPlanner, constraints, endPoseName, startPoseName)
        constraintSet.ikParameters = IkParameters(usePointwise=True, maxBaseRPYDegreesPerSecond=10,
                                                  rescaleBodyNames=['l_foot'],
                                                  rescaleBodyPts=[0.0, 0.0, 0.0],
                                                  maxBodyTranslationSpeed=self.maxFootTranslationSpeed)
        #constraintSet.seedPoseName = 'q_start'
        #constraintSet.nominalPoseName = 'q_start'

        constraintSet.runIk()

        footFrame = self.robotSystem.ikPlanner.getLinkFrameAtPose('l_foot', startPose)
        t = transformUtils.frameFromPositionAndRPY([0, 0, self.polaris.leftFootEgressOutsideFrame.transform.GetPosition()[2]-footFrame.GetPosition()[2]], [0, 0, 0])
        liftFrame = transformUtils.concatenateTransforms([footFrame, t])
        vis.updateFrame(liftFrame, 'lift frame')

        c = ikconstraints.WorldFixedOrientConstraint()
        c.linkName = 'l_foot'
        c.tspan = [0.0, 0.1, 0.2]
        constraints.append(c)
        constraints.extend(self.createLeftFootPoseConstraint(liftFrame, tspan=[0.2, 0.2]))
        constraints.extend(self.createLeftFootPoseConstraint(self.polaris.leftFootEgressMidFrame, tspan=[0.5, 0.5]))

        constraints.extend(self.createLeftFootPoseConstraint(self.polaris.leftFootEgressOutsideFrame, tspan=[0.8, 0.8]))

        #plan = constraintSet.planEndPoseGoal(feetOnGround=False)
        keyFramePlan = constraintSet.runIkTraj()
        poseTimes, poses = planplayback.PlanPlayback.getPlanPoses(keyFramePlan)
        ts = [poseTimes[0]]
        supportsList = [['r_foot']]
        plan = self.publishPlanWithSupports(keyFramePlan, supportsList, ts, False)
        self.addPlan(plan)
        return plan

    def planLeftFootDown(self):
        startPose = self.getPlanningStartPose()
        startPoseName = 'q_footdown_start'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_footdown_end'
        utorsoFrame = self.robotSystem.ikPlanner.getLinkFrameAtPose('utorso', startPose)
        finalLeftFootFrame = self.computeLeftFootOverPlatformFrame(startPose, 0.0)

        constraints = []
        constraints.extend(self.createUtorsoGazeConstraints([0.0, 1.0]))
        constraints.append(ikconstraints.QuasiStaticConstraint(leftFootEnabled=False, rightFootEnabled=True,
                                                    pelvisEnabled=False, shrinkFactor=0.01))
        constraints.append(self.robotSystem.ikPlanner.createMovingBaseSafeLimitsConstraint())
        constraints.append(self.robotSystem.ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedRightArmPostureConstraint(startPoseName))
        #constraints.append(self.robotSystem.ikPlanner.createLockedBackPostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createFixedLinkConstraints(startPoseName, 'r_foot'))
        constraints.extend(self.createLeftFootPoseConstraint(finalLeftFootFrame, tspan=[1,1]))

        constraintSet = ConstraintSet(self.robotSystem.ikPlanner, constraints, endPoseName, startPoseName)
        constraintSet.ikParameters = IkParameters(usePointwise=True)
        #constraintSet.seedPoseName = 'q_start'
        #constraintSet.nominalPoseName = 'q_start'

        endPose, _ = constraintSet.runIk()
        keyFramePlan = constraintSet.planEndPoseGoal(feetOnGround=False)
        poseTimes, poses = planplayback.PlanPlayback.getPlanPoses(keyFramePlan)
        ts = [poseTimes[0], poseTimes[-1]]
        supportsList = [['r_foot'], ['r_foot','l_foot']]
        plan = self.publishPlanWithSupports(keyFramePlan, supportsList, ts, True)
        self.addPlan(plan)
        return plan, endPose


    def planCenterWeight(self, startPose=None):
        ikPlanner = self.robotSystem.ikPlanner

        if startPose is None:
            startPose = self.getPlanningStartPose()

        startPoseName = 'q_lean_right'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_egress_end'

        footFixedConstraints = ikPlanner.createFixedFootConstraints(startPoseName)
        backConstraint = ikPlanner.createMovingBackLimitedPostureConstraint()
        armsLocked = ikPlanner.createLockedArmsPostureConstraints(startPoseName)

        constraints = [backConstraint]
        constraints.extend(footFixedConstraints)
        constraints.extend(armsLocked)
        constraints.append(ikconstraints.QuasiStaticConstraint(leftFootEnabled=True, rightFootEnabled=True,
                                                    pelvisEnabled=False,
                                                    shrinkFactor=self.quasiStaticShrinkFactor))
        constraints.append(self.robotSystem.ikPlanner.createKneePostureConstraint([0.7, 2.5]))

        constraintSet = ConstraintSet(ikPlanner, constraints, endPoseName, startPoseName)
        constraintSet.seedPoseName = 'q_start'
        constraintSet.nominalPoseName = 'q_nom'
        endPose = constraintSet.runIk()
        keyFramePlan = constraintSet.planEndPoseGoal()
        poseTimes, poses = planplayback.PlanPlayback.getPlanPoses(keyFramePlan)
        ts = [poseTimes[0]]
        supportsList = [['r_foot','l_foot']]
        plan = self.publishPlanWithSupports(keyFramePlan, supportsList, ts, True)
        self.addPlan(plan)
        return plan


    def planFootDownAndCenterWeight(self):
        leftFootDownPlan, leftFootDownEndPose = self.planLeftFootDown()
        centerWeightPlan = self.planCenterWeight(startPose=leftFootDownEndPose)

        # now we need to combine these plans
        footDownEndTime = leftFootDownPlan.plan.plan[-1].utime

        robotPlan = leftFootDownPlan
        robotPlan.plan.plan_info = list(robotPlan.plan.plan_info)
        for state, info in zip(centerWeightPlan.plan.plan, centerWeightPlan.plan.plan_info):
            state.utime += footDownEndTime
            robotPlan.plan.plan.append(state)
            robotPlan.plan.plan_info.append(info)

        robotPlan.plan.num_states = len(robotPlan.plan.plan)


        # make support sequence
        for support, t in zip(centerWeightPlan.support_sequence.supports, centerWeightPlan.support_sequence.ts):
            t += footDownEndTime
            robotPlan.support_sequence.ts.append(t)
            robotPlan.support_sequence.supports.append(support)

        robotPlan.is_quasistatic = True
        self.addPlan(robotPlan)
        lcmUtils.publish('CANDIDATE_ROBOT_PLAN_WITH_SUPPORTS', robotPlan)
        return robotPlan



    def planArmsForward(self):
        q0 = self.getPlanningStartPose()
        q1 = self.robotSystem.ikPlanner.getMergedPostureFromDatabase(q0, 'General', 'hands-forward', side='left')
        q2 = self.robotSystem.ikPlanner.getMergedPostureFromDatabase(q1, 'General', 'hands-forward', side='right')
        a = 0.25
        q1 = (1-a)*q1 + a*np.array(q2)
        ikParameters = IkParameters(usePointwise=True, maxBaseRPYDegreesPerSecond=10,
                                    rescaleBodyNames=['l_hand', 'r_hand'],
                                    rescaleBodyPts=list(self.robotSystem.ikPlanner.getPalmPoint(side='left')) +
                                                    list(self.robotSystem.ikPlanner.getPalmPoint(side='right')),
                                    maxBodyTranslationSpeed=3*self.maxHandTranslationSpeed)
        plan = self.robotSystem.ikPlanner.computeMultiPostureGoal([q0, q1, q2], ikParameters=ikParameters)
        self.addPlan(plan)
        return plan

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
        print(transformUtils.poseFromTransform(tCopy))
        return tCopy


class EgressPanel(TaskUserPanel):

    def __init__(self, robotSystem):

        TaskUserPanel.__init__(self, windowTitle='Egress')

        self.robotSystem = robotSystem
        self.egressPlanner = EgressPlanner(robotSystem)
        self.platformPlanner = polarisplatformplanner.PolarisPlatformPlanner(robotSystem.ikServer, robotSystem)
        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()


    def addButtons(self):
        # Get onto platform buttons
        self.addManualButton('Spawn Polaris', self.egressPlanner.spawnPolaris)
        self.addManualButton('Get weight over feet', self.egressPlanner.planGetWeightOverFeet)
        self.addManualButton('Stand up', self.egressPlanner.planStandUp)
        self.addManualButton('Step out', self.egressPlanner.planFootEgress)
        self.addManualButton('Arms forward', self.egressPlanner.planArmsForward)
        self.addManualSpacer()
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

    def onStart(self):
        self._syncProperties()
        print('Egress Planner Ready')

    def onUpdateAffordance(self):
        if not self.platformPlanner.initializedFlag:
            self.platformPlanner.initialize()

        self.platformPlanner.updateAffordance()

    def onPlan(self,planType):
        self._syncProperties()

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

        def addManipTask(name, planFunc, userPrompt=False, planner=None):

            if planner is None:
                planner = self.platformPlanner
            prevFolder = self.folder
            addFolder(name, prevFolder)
            addFunc(planFunc, 'plan')
            if not userPrompt:
                addTask(rt.CheckPlanInfo(name='check manip plan info'))
            else:
                addTask(rt.UserPromptTask(name='approve manip plan', message='Please approve manipulation plan.'))
            addFunc(planner.commitManipPlan, name='execute manip plan')
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'))
            self.folder = prevFolder

        pp = self.platformPlanner
        ep = self.egressPlanner

        stepPrep = addFolder('Prep')
        self.folder = stepPrep
        addTask(rt.UserPromptTask(name="Verify SE processes", message="Please confirm that all SE processes have started"))
        addTask(rt.UserPromptTask(name="Run Init Nav", message='Please click "Init Nav"'))
        addTask(rt.UserPromptTask(name="Stop April Tags", message='Please stop the "April Tags" process'))
        addTask(rt.UserPromptTask(name="Confirm pressure", message='Set high pressure for egress'))
        addTask(rt.UserPromptTask(name="Disable recovery and bracing", message="Please disable recovery and bracing"))
        addTask(rt.SetNeckPitch(name='set neck position', angle=60))
        stepOut = addFolder('Step out of car')
        self.folder = stepOut
        addManipTask('Get weight over feet', ep.planGetWeightOverFeet, userPrompt=True, planner=ep)
        addManipTask('Stand up', ep.planStandUp, userPrompt=True, planner=ep)
        addManipTask('Step out', ep.planFootEgress, userPrompt=True, planner=ep)
        addManipTask('Move arms up for walking', ep.planArmsForward, userPrompt=True, planner=ep)

        prep = addFolder('Step down prep')
        addFunc(self.onStart, 'start')
        addFunc(pp.switchToPolarisPlatformParameters, "Switch walking params to 'Polaris Platform")
        addTask(rt.UserPromptTask(name="wait for lidar", message="Please wait for next lidar sweep"))

        self.folder = prep
        addFunc(pp.fitRunningBoardAtFeet, 'fit running board')
        addTask(rt.FindAffordance(name='confirm running board affordance', affordanceName='running board'))
        addFunc(pp.spawnGroundAffordance, 'spawn ground affordance')
        addFunc(pp.requestRaycastTerrain, 'raycast terrain')
        addTask(rt.UserPromptTask(name="wait for raycast terrain", message="wait for raycast terrain"))

        folder = addFolder('Step Down')
        addFunc(self.onPlanStepDown, 'plan step down')
        addTask(rt.UserPromptTask(name="approve footsteps, set support contact group",
         message="Please approve/modify footsteps."))
        addFunc(self.robotSystem.footstepsDriver.onExecClicked, 'commit footstep plan')
        addTask(rt.WaitForWalkExecution(name='wait for walking'))

        folder = addFolder('Step Off')
        # addTask(rt.UserPromptTask(name="wait for lidar sweep", message="wait for lidar sweep before spawning ground affordance"))
        addFunc(pp.spawnFootplaneGroundAffordance, 'spawn footplane ground affordance')
        addFunc(pp.requestRaycastTerrain, 'raycast terrain')
        addTask(rt.UserPromptTask(name="wait for raycast terrain", message="wait for raycast terrain"))
        addFunc(self.onPlanStepOff, 'plan step off')
        addTask(rt.UserPromptTask(name="approve footsteps", message="Please approve footsteps, modify if necessary"))
        addFunc(self.robotSystem.footstepsDriver.onExecClicked, 'commit footstep plan')
        addTask(rt.WaitForWalkExecution(name='wait for walking'))
        addManipTask('plan nominal', pp.planNominal, userPrompt=True)
        addTask(rt.UserPromptTask(name="reset walking parameters", message="Please set walking parameters to drake nominal"))
        addTask(rt.SetNeckPitch(name='set neck position', angle=20))
