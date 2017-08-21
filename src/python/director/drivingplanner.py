import director
import math
import textwrap
import drc as lcmdrc
import bot_core as lcmbotcore
from . import vtkAll as vtk
from director import transformUtils
from director import visualization as vis
from director import objectmodel as om
from director import lcmUtils
from director import ikconstraints
from director import cameraview
from director import affordanceupdater
from director import affordancemanager
from director import segmentation
from director import robotstate
from director.debugVis import DebugData
from director.utime import getUtime
from director.ikplanner import ConstraintSet
import director.tasks.robottasks as rt
from director.ikparameters import IkParameters
from director.timercallback import TimerCallback

import bot_core
import os
import functools
import numpy as np
import scipy.io
from director.tasks.taskuserpanel import TaskUserPanel
from director import drcargs

class DrivingPlanner(object):

    def __init__(self, ikServer, robotSystem):
        self.ikServer = ikServer
        self.robotSystem = robotSystem
        self.ikServer.connectStartupCompleted(self.initialize)
        self.steeringAngleDegrees = 0.0
        self.maxTurningRadius = 9.5
        self.trajectoryX = 0
        self.trajectoryY = 0.3
        self.trajectoryAngle = 0
        self.trajSegments = 25
        self.wheelDistance = 1.4
        self.tagToLocalTransform = transformUtils.transformFromPose([0,0,0],[1,0,0,0])

        self.commandStreamChannel = 'JOINT_POSITION_GOAL'
        self.drivingThrottleJoint = drcargs.getDirectorConfig()['drivingThrottleJoint']
        self.drivingSteeringJoint = drcargs.getDirectorConfig()['drivingSteeringJoint']
        self.akyIdx =  robotstate.getDrakePoseJointNames().index( self.drivingThrottleJoint )
        self.lwyIdx =  robotstate.getDrakePoseJointNames().index( self.drivingSteeringJoint )
        self.anklePositions = np.array([np.nan,np.nan])
        self.jointLimitsMin = np.array([self.robotSystem.teleopRobotModel.model.getJointLimits(jointName)[0] for jointName in robotstate.getDrakePoseJointNames()])
        self.jointLimitsMax = np.array([self.robotSystem.teleopRobotModel.model.getJointLimits(jointName)[1] for jointName in robotstate.getDrakePoseJointNames()])
        self.idleAngleSlack = 10
        self.fineGrainedThrottleTravel = 10
        self.steeringAngleOffset = 0
        self.throttlePublishChannel = 'SINGLE_JOINT_POSITION_GOAL'
        self.steeringPublishChannel = 'SINGLE_JOINT_POSITION_GOAL'
        self.addSubscribers()
        self.graspWheelAngle = None
        self.graspWristAngle = None
        self.kneeInPedal = 0
        self.angleToleranceInDegrees = 10
        self.distanceAbovePedal = 0.05
        self.distanceAboveFootStartPose = 0.2
        self.plans = []

        self.throttleCommandTimer = TimerCallback(targetFps=5)
        self.throttleCommandTimer.callback = self.publishThrottleCommand
        self.throttleCommandMsg = None

        self.steeringCommandTimer = TimerCallback(targetFps=5)
        self.steeringCommandTimer.callback = self.publishSteeringCommand
        self.steeringCommandMsg = None

    @staticmethod
    def isCompatibleWithConfig():
        return 'drivingThrottleJoint' in drcargs.getDirectorConfig()

    def getInitCommands(self):

      commands = [textwrap.dedent('''
        % ------ driving planner startup ------

        addpath([getenv('DRC_BASE'), '/software/control/matlab/planners/driving_planner']);
        clear driving_planner_options;
        driving_planner_options.listen_to_lcm_flag = 0;
        driving_planner_options.qstar = q_nom;
        dp = drivingPlanner(s.robot, driving_planner_options);

        % ------ driving planner startup end ------
      ''')]

      return commands

    def addSubscribers(self):
        lcmUtils.addSubscriber('THROTTLE_COMMAND', lcmdrc.trigger_finger_t , self.onThrottleCommand)
        lcmUtils.addSubscriber('STEERING_COMMAND', lcmdrc.driving_control_cmd_t , self.onSteeringCommand)

    def initialize(self, ikServer, success):
        if ikServer.restarted:
            return

        commands = self.getInitCommands()
        self.ikServer.taskQueue.addTask(functools.partial(self.ikServer.comm.sendCommandsAsync, commands))
        self.ikServer.taskQueue.start()

    # applies the properties to the driving planner object
    def applyProperties(self):
        commands = []
        commands.append("dp.options.quat_tol = %r;" % self.quatTol)
        commands.append("dp.options.tol = %r;" % self.positionTol)
        commands.append("dp.options.seed_with_current = %r;" % self.seedWithCurrent)
        self.ikServer.taskQueue.addTask(functools.partial(self.ikServer.comm.sendCommandsAsync, commands))
        self.ikServer.taskQueue.start()

    def updateWheelTransform(self, xyzquat):

        commands = []
        startPose = self.getPlanningStartPose()
        commands.append("q0 = %s;" % ikconstraints.ConstraintBase.toColumnVectorString(startPose))
        commands.append("xyzquat = %s;" % ikconstraints.ConstraintBase.toColumnVectorString(xyzquat))
        commands.append("dp = dp.updateWheelTransform(xyzquat, q0);")

        self.ikServer.comm.sendCommands(commands)

    def planSafe(self, speed=1):
        commands = []
        commands.append("clear options;")
        commands.append("options.speed = %r;" % speed)
        startPose = self.getPlanningStartPose()
        commands.append("dp.planSafe(options,%s);" % ikconstraints.ConstraintBase.toColumnVectorString(startPose))

        self.ikServer.taskQueue.addTask(functools.partial(self.ikServer.comm.sendCommandsAsync, commands))
        self.ikServer.taskQueue.start()


    def planPreGrasp(self, depth=0.2, xyz_des=None, angle=0, speed=1, graspLocation='center', turnRadius=0.187):
        commands = []
        commands.append("clear options;")
        commands.append("options = struct('depth',{%r});" % depth)
        commands.append("options.turn_radius = %r;" % turnRadius)
        commands.append("options.graspLocation = '%s';" % graspLocation)
        commands.append("options.angle = %r;" % np.radians(angle))
        commands.append("options.speed = %r;" % speed)

        if xyz_des is not None:
            commands.append("options.xyz_des = {%s};",ikconstraints.ConstraintBase.toColumnVectorString(xyz_des))
        startPose = self.getPlanningStartPose()
        commands.append("dp.planPreGrasp(options, %s);" % ikconstraints.ConstraintBase.toColumnVectorString(startPose))

        listener = self.getManipPlanListener()
        self.ikServer.comm.sendCommands(commands)
        plan = listener.waitForResponse()
        listener.finish()
        self.addPlan(plan)

    def planTouch(self, depth=0, xyz_des=None, speed=1):
        commands = []
        commands.append("clear options;")
        commands.append("options = struct('depth',{%r});" % depth)
        commands.append("options.speed = %r;" % speed)
        startPose = self.getPlanningStartPose()
        commands.append("dp.planTouch(options, %s);" % ikconstraints.ConstraintBase.toColumnVectorString(startPose))

        listener = self.getManipPlanListener()
        self.ikServer.comm.sendCommands(commands)
        plan = listener.waitForResponse()
        listener.finish()
        self.addPlan(plan)

    def planRetract(self, depth=0.2, speed=1):
        commands = []
        commands.append("clear options;")
        commands.append("options = struct('depth',{%r});" % depth)
        commands.append("options.speed = %s;" % speed)
        startPose = self.getPlanningStartPose()
        commands.append("dp.planRetract(options, %s);" % ikconstraints.ConstraintBase.toColumnVectorString(startPose))

        listener = self.getManipPlanListener()
        self.ikServer.comm.sendCommands(commands)
        plan = listener.waitForResponse()
        listener.finish()
        self.addPlan(plan)

    def planTurn(self, angle=0, speed=1):
        commands = []
        commands.append("clear options;")
        commands.append("options.turn_angle = %r;" % np.radians(angle))
        commands.append("options.speed = %r;" % speed)
        commands.append("options.use_raw_angle = 1;")
        startPose = self.getPlanningStartPose()
        commands.append("dp.planTurn(options,%s);" % ikconstraints.ConstraintBase.toColumnVectorString(startPose))

        listener = self.getManipPlanListener()
        self.ikServer.comm.sendCommands(commands)
        plan = listener.waitForResponse()
        listener.finish()
        self.addPlan(plan)

    def planSteeringWheelTurn(self, speed=1, knotPoints=20, turnRadius=.187, gazeTol=0.3):
        commands = []
        commands.append("clear options;")
        commands.append("options.speed = %r;" % speed)
        commands.append("options.turn_radius = %r;" % turnRadius)
        commands.append("options.N = %r;" % knotPoints)
        commands.append("options.steering_gaze_tol = %r;" % gazeTol)
        startPose = self.getPlanningStartPose()
        commands.append("dp.planSteeringWheelTurn(options,%s);" % ikconstraints.ConstraintBase.toColumnVectorString(startPose))

        self.ikServer.taskQueue.addTask(functools.partial(self.ikServer.comm.sendCommandsAsync, commands))
        self.ikServer.taskQueue.start()

    def planSeed(self):
        commands = []
        startPose = self.getPlanningStartPose()
        commands.append("dp.planSeed(%s);" % ikconstraints.ConstraintBase.toColumnVectorString(startPose))
        self.ikServer.taskQueue.addTask(functools.partial(self.ikServer.comm.sendCommandsAsync, commands))
        self.ikServer.taskQueue.start()

    def getPlanningStartPose(self):
        return self.robotSystem.robotStateJointController.q


    # move left leg up a bit
    def planLegUp(self):
        ikPlanner = self.robotSystem.ikPlanner
        startPose = self.getPlanningStartPose()
        startPoseName = 'q_start_foot'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_foot_end'
        lFoot2World = self.robotSystem.ikPlanner.getLinkFrameAtPose('l_foot', startPose)

        # targetFrame = transformUtils.copyFrame(lFoot2World)
        # targetFrame.PreMultiply()
        # targetFrame.Translate([0.0,0.0, self.distanceAboveFootStartPose])
        targetFrame = transformUtils.copyFrame(om.findObjectByName('left foot up frame').transform)
        footPoseConstraint = self.createLeftFootPoseConstraint(targetFrame)
        allButLeftLegPostureConstraint = self.createAllButLeftLegPostureConstraint(startPoseName)

        constraints = [allButLeftLegPostureConstraint]
        constraints.extend(footPoseConstraint)

        cs = ConstraintSet(ikPlanner, constraints, endPoseName, startPoseName)
        cs.ikParameters = IkParameters(maxDegreesPerSecond=10, usePointwise=False)
        cs.seedPoseName = 'q_start'
        cs.nominalPoseName = 'q_start'
        endPose = cs.runIk()
        plan = cs.planEndPoseGoal()
        self.plans.append(plan)

        return plan


    def planLegSwingIn(self):
        om.findObjectByName('left foot driving')
        ikPlanner = self.robotSystem.ikPlanner
        startPose = self.getPlanningStartPose()
        startPoseName = 'q_start_foot'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_foot_end'

        if self.kneeInPedal:
            legAbovePedalFrame = transformUtils.copyFrame(om.findObjectByName('left foot driving knee in').transform)
            legAbovePedalFrame.PreMultiply()
            legAbovePedalFrame.Translate([0.0, 0.0, self.distanceAbovePedal])
        else:
            legAbovePedalFrame = transformUtils.copyFrame(om.findObjectByName('left foot driving').transform)
            legAbovePedalFrame.PreMultiply()
            legAbovePedalFrame.Translate([-0.02, 0.0, 0.03])


        identityFrame = vtk.vtkTransform()
        legAbovePedalConstraint = self.createLeftFootPoseConstraint(legAbovePedalFrame, tspan=[1,1], angleToleranceInDegrees=self.angleToleranceInDegrees)
        allButLeftLegPostureConstraint = self.createAllButLeftLegPostureConstraint(startPoseName)

        constraints = [allButLeftLegPostureConstraint]
        constraints.extend(legAbovePedalConstraint)

        seedPoseName = 'q_driving'
        seedPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'driving', 'driving')
        self.robotSystem.ikPlanner.addPose(seedPose, seedPoseName)

        cs = ConstraintSet(ikPlanner, constraints, endPoseName, startPoseName)
        cs.ikParameters = IkParameters(maxDegreesPerSecond=10, usePointwise=False)
        cs.seedPoseName = 'q_driving'
        cs.nominalPoseName = 'q_driving'
        endPose = cs.runIk()

        # add constraint that we hit intermediate frame, maybe doesn't have to be exact???
        legSwingFrame = om.findObjectByName('left foot pedal swing').transform
        cs.constraints.extend(self.createLeftFootPoseConstraint(legSwingFrame, tspan=[0.3,0.3]))
        keyFramePlan = cs.runIkTraj()
        self.plans.append(keyFramePlan)

        return keyFramePlan

    def planLegAbovePedal(self, startPose=None):
        om.findObjectByName('left foot driving')
        ikPlanner = self.robotSystem.ikPlanner
        if startPose is None:
            startPose = self.getPlanningStartPose()
        startPoseName = 'q_start_foot'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_foot_end'

        if self.kneeInPedal:
            legAbovePedalFrame = transformUtils.copyFrame(om.findObjectByName('left foot driving knee in').transform)
            legAbovePedalFrame.PreMultiply()
            legAbovePedalFrame.Translate([0.0, 0, self.distanceAbovePedal])
        else:
            legAbovePedalFrame = transformUtils.copyFrame(om.findObjectByName('left foot driving').transform)
            legAbovePedalFrame.PreMultiply()
            legAbovePedalFrame.Translate([-0.02, 0.0, 0.03])


        identityFrame = vtk.vtkTransform()
        legAbovePedalConstraint = self.createLeftFootPoseConstraint(legAbovePedalFrame, tspan=[1,1], angleToleranceInDegrees=self.angleToleranceInDegrees)
        allButLeftLegPostureConstraint = self.createAllButLeftLegPostureConstraint(startPoseName)

        constraints = [allButLeftLegPostureConstraint]
        constraints.extend(legAbovePedalConstraint)

        seedPoseName = 'q_driving'
        seedPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'driving', 'driving')
        self.robotSystem.ikPlanner.addPose(seedPose, seedPoseName)

        cs = ConstraintSet(ikPlanner, constraints, endPoseName, startPoseName)
        cs.ikParameters = IkParameters(maxDegreesPerSecond=10, usePointwise=False)
        cs.seedPoseName = 'q_driving'
        cs.nominalPoseName = 'q_driving'
        endPose = cs.runIk()
        plan = cs.planEndPoseGoal()
        self.plans.append(plan)

        return plan


    def planLegSwingOut(self, startPose=None):
        om.findObjectByName('left foot driving')
        ikPlanner = self.robotSystem.ikPlanner

        if startPose is None:
            startPose = self.getPlanningStartPose()
        startPoseName = 'q_start_foot'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_foot_end'

        # legUpFrame = transformUtils.copyFrame(om.findObjectByName('left foot start').transform)
        # legUpFrame.PreMultiply()
        # legUpFrame.Translate([0.0,0.0, self.distanceAboveFootStartPose])
        legUpFrame = transformUtils.copyFrame(om.findObjectByName('left foot up frame').transform)
        identityFrame = vtk.vtkTransform()
        legUpConstraint = self.createLeftFootPoseConstraint(legUpFrame, tspan=[1,1], angleToleranceInDegrees=10)
        allButLeftLegPostureConstraint = self.createAllButLeftLegPostureConstraint(startPoseName)

        constraints = [allButLeftLegPostureConstraint]
        constraints.extend(legUpConstraint)

        seedPoseName = 'q_driving'
        seedPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'driving', 'car_entry_new')
        self.robotSystem.ikPlanner.addPose(seedPose, seedPoseName)

        cs = ConstraintSet(ikPlanner, constraints, endPoseName, startPoseName)
        cs.ikParameters = IkParameters(maxDegreesPerSecond=10, usePointwise=False)
        cs.seedPoseName = 'q_driving'
        cs.nominalPoseName = 'q_driving'
        endPose = cs.runIk()


        legSwingFrame = om.findObjectByName('left foot pedal swing').transform
        cs.constraints.extend(self.createLeftFootPoseConstraint(legSwingFrame, tspan=[0.7,0.7]))
        keyFramePlan = cs.runIkTraj()
        self.plans.append(keyFramePlan)

        return keyFramePlan

    def planLegEgressStart(self, startPose=None):
        om.findObjectByName('left foot driving')
        ikPlanner = self.robotSystem.ikPlanner
        if startPose is None:
            startPose = self.getPlanningStartPose()
        startPoseName = 'q_start_foot'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_foot_end'

        lFoot2RFoot = om.findObjectByName('left foot to right foot')
        assert lFoot2RFoot

        rFoot2World = self.robotSystem.ikPlanner.getLinkFrameAtPose('r_foot', startPose)
        lFootGoalFrame = transformUtils.concatenateTransforms([transformUtils.copyFrame(lFoot2RFoot.transform), rFoot2World])

        legDownFrame = transformUtils.copyFrame(lFootGoalFrame)
        identityFrame = vtk.vtkTransform()
        legDownConstraint = self.createLeftFootPoseConstraint(legDownFrame)
        allButLeftLegPostureConstraint = self.createAllButLeftLegPostureConstraint(startPoseName)

        constraints = [allButLeftLegPostureConstraint]
        constraints.extend(legDownConstraint)

        seedPoseName = 'q_driving'
        seedPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'driving', 'driving')
        self.robotSystem.ikPlanner.addPose(seedPose, seedPoseName)

        cs = ConstraintSet(ikPlanner, constraints, endPoseName, startPoseName)
        cs.ikParameters = IkParameters(maxDegreesPerSecond=10, usePointwise=False)
        cs.seedPoseName = 'q_driving'
        cs.nominalPoseName = 'q_driving'
        endPose = cs.runIk()
        plan = cs.planEndPoseGoal()
        self.plans.append(plan)

        return plan


    def planLegEgressFull(self):
        legAbovePedalName = 'qtraj_leg_above_pedal'
        self.planLegAbovePedal()
        self.saveOriginalTraj(legAbovePedalName)

        nextStartPose = robotstate.convertStateMessageToDrakePose(self.plans[-1].plan[-1])
        self.planLegSwingOut(startPose=nextStartPose)
        legSwingOutName = 'qtraj_leg_swing_out'
        self.saveOriginalTraj(legSwingOutName)

        nextStartPose = robotstate.convertStateMessageToDrakePose(self.plans[-1].plan[-1])
        self.planLegEgressStart(startPose=nextStartPose)
        legEgressStartName = 'qtraj_leg_egress_start'
        self.saveOriginalTraj(legEgressStartName)


        ikParameters = IkParameters(usePointwise=False, maxDegreesPerSecond=10)
        ikParameters = self.robotSystem.ikPlanner.mergeWithDefaultIkParameters(ikParameters)

        listener = self.getManipPlanListener()
        _ = self.concatenateAndRescaleTrajectories([legAbovePedalName, legSwingOutName, legEgressStartName], 'qtraj_foot_egress_start', 'ts', ikParameters)

        plan = listener.waitForResponse()
        listener.finish()
        self.addPlan(plan)


    def planLegPedal(self):
        ikPlanner = self.robotSystem.ikPlanner
        startPose = self.getPlanningStartPose()
        startPoseName = 'q_start_foot'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_foot_end'

        if self.kneeInPedal:
            lfootConstraintFrame = transformUtils.copyFrame(om.findObjectByName('left foot on pedal').transform)
        else:
            lfootConstraintFrame = transformUtils.copyFrame(om.findObjectByName('left foot driving').transform)
        identityFrame = vtk.vtkTransform()
        lfootPositionOrientationConstraint = ikPlanner.createPositionOrientationConstraint('l_foot', lfootConstraintFrame, identityFrame)
        allButLeftLegPostureConstraint = self.createAllButLeftLegPostureConstraint(startPoseName)

        constraints = [allButLeftLegPostureConstraint]
        constraints.extend(lfootPositionOrientationConstraint)

        seedPoseName = 'q_driving'
        seedPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'driving', 'driving')
        self.robotSystem.ikPlanner.addPose(seedPose, seedPoseName)

        cs = ConstraintSet(ikPlanner, constraints, endPoseName, startPoseName)
        cs.ikParameters = IkParameters(quasiStaticShrinkFactor=1, maxDegreesPerSecond=10, usePointwise=False)
        cs.seedPoseName = 'q_driving'
        cs.nominalPoseName = 'q_driving'
        endPose = cs.runIk()
        keyFramePlan = cs.planEndPoseGoal()
        self.plans.append(keyFramePlan)

        return keyFramePlan

    def captureHandPose(self):
        startPose = self.getPlanningStartPose()
        self.wheelAngleBeforeReGrasp = self.getSteeringWheelAngle()
        ikPlanner = self.robotSystem.ikPlanner

        handName = 'left'
        palmToHand = ikPlanner.getPalmToHandLink(handName)
        palmToWorld = ikPlanner.newGraspToWorldFrame(startPose, handName, palmToHand)
        self.palmToWorldBeforeRegrasp = palmToWorld

    def planSteeringWheelReGrasp(self, useLineConstraint=True):
        ikPlanner = self.robotSystem.ikPlanner
        startPose = self.getPlanningStartPose()
        startPoseName = 'q_regrasp_start'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_regrasp_end'
        handName = 'left'
        handLinkName = 'l_hand'
        maxMetersPerSecond = 0.1
        retractDepth = 0.15

        palmToHand = ikPlanner.getPalmToHandLink(handName)
        palmToWorldBeforeRegrasp = self.palmToWorldBeforeRegrasp
        finalTargetFrame = transformUtils.copyFrame(palmToWorldBeforeRegrasp)
        finalTargetFrame.PreMultiply()
        finalTargetFrame.RotateY(180)
        finalPoseConstraint = self.createLeftPalmPoseConstraints(finalTargetFrame, tspan=[1,1])

        palmToWorld = ikPlanner.newGraspToWorldFrame(startPose, handName, palmToHand)
        palmPosition = palmToWorld.GetPosition()
        _, finalPose = transformUtils.poseFromTransform(finalTargetFrame)
        preGraspTargetFrame = transformUtils.transformFromPose(palmPosition, finalPose)
        preGraspPoseConstraint = self.createLeftPalmPoseConstraints(preGraspTargetFrame, tspan=[0.5, 0.5])

        allButLeftArmPostureConstraint = self.createAllButLeftArmPostureConstraint(startPoseName)
        lockedBaseConstraint = ikPlanner.createLockedBasePostureConstraint(startPoseName)
        lockedRightArmConstraint = ikPlanner.createLockedRightArmPostureConstraint(startPoseName)
        lockedTorsoConstraint = ikPlanner.createLockedTorsoPostureConstraint(startPoseName)
        constraints = [allButLeftArmPostureConstraint]
        constraints.extend(finalPoseConstraint)

        seedPoseName = 'q_regrasp_seed'

        seedPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'driving', 'driving')
        self.robotSystem.ikPlanner.addPose(seedPose, seedPoseName)

        constraintSet = ConstraintSet(ikPlanner, constraints, endPoseName, startPoseName)
        constraintSet.ikParameters = IkParameters(quasiStaticShrinkFactor=10, usePointwise=False, maxDegreesPerSecond=60,
            maxBodyTranslationSpeed=maxMetersPerSecond, rescaleBodyNames=[handLinkName], rescaleBodyPts=list(ikPlanner.getPalmPoint()))
        constraintSet.seedPoseName = seedPoseName
        constraintSet.nominalPoseName = seedPoseName


        endPose = constraintSet.runIk()

        # move on line constraint
        motionVector = np.array(preGraspTargetFrame.GetPosition()) - np.array(finalTargetFrame.GetPosition())
        motionTargetFrame = transformUtils.getTransformFromOriginAndNormal(np.array(preGraspTargetFrame.GetPosition()), motionVector)

        p = ikPlanner.createLinePositionConstraint(handLinkName, palmToHand, motionTargetFrame, lineAxis=2, bounds=[-np.linalg.norm(motionVector)*1, 0], positionTolerance=0.001)
        p.tspan = np.linspace(0,1,5)


        endPose = constraintSet.runIk()
        constraintSet.constraints.extend(preGraspPoseConstraint)

        # orientation constraint for 0.5, 1
        _, orientationConstraint = ikPlanner.createPositionOrientationGraspConstraints(handName, finalTargetFrame, graspToHandLinkFrame=palmToHand, positionTolerance=0.0, angleToleranceInDegrees=0.0)
        orientationConstraint.tspan = np.array([0.5,0.6,0.8,1])
        constraintSet.constraints.append(orientationConstraint)


        if useLineConstraint:
            constraintSet.constraints.append(p)
            plan = constraintSet.runIkTraj()
        else:
            plan = constraintSet.runIkTraj()

        self.plans.append(plan)
        return plan

    def createLeftFootPoseConstraint(self, targetFrame, tspan=[-np.inf, np.inf], angleToleranceInDegrees=0.0):
        positionConstraint, orientationConstraint = self.robotSystem.ikPlanner.createPositionOrientationConstraint('l_foot', targetFrame, vtk.vtkTransform(), angleToleranceInDegrees=angleToleranceInDegrees)
        positionConstraint.tspan = tspan
        orientationConstraint.tspan = tspan
        return positionConstraint, orientationConstraint

    def createLeftPalmPoseConstraints(self, targetFrame, tspan=[-np.inf, np.inf]):
        ikPlanner = self.robotSystem.ikPlanner
        positionConstraint, orientationConstraint = ikPlanner.createPositionOrientationGraspConstraints('left', targetFrame)
        positionConstraint.tspan = tspan
        orientationConstraint.tspan = tspan
        return positionConstraint, orientationConstraint

    def createLeftPalmPoseConstraints(self, targetFrame, tspan=[-np.inf, np.inf]):
        ikPlanner = self.robotSystem.ikPlanner
        positionConstraint, orientationConstraint = ikPlanner.createPositionOrientationGraspConstraints('left', targetFrame)
        positionConstraint.tspan = tspan
        orientationConstraint.tspan = tspan
        return positionConstraint, orientationConstraint


    def createPalmPoseConstraints(self, side, targetFrame, tspan=[-np.inf, np.inf]):
        ikPlanner = self.robotSystem.ikPlanner
        positionConstraint, orientationConstraint = ikPlanner.createPositionOrientationGraspConstraints(side, targetFrame)
        positionConstraint.tspan = tspan
        orientationConstraint.tspan = tspan
        return positionConstraint, orientationConstraint

    def createLeftHandPoseConstraintOnWheel(self, depth=0.12, tspan=[-np.inf, np.inf]):
        targetFrame = self.getSteeringWheelPalmFrame()
        targetFrame.PreMultiply()
        targetFrame.Translate([0.0, depth, 0.0])
        positionConstraint, orientationConstraint = self.robotSystem.ikPlanner.createPositionOrientationConstraint('l_hand_face', targetFrame, vtk.vtkTransform())
        positionConstraint.tspan = tspan
        orientationConstraint.tspan = tspan
        return positionConstraint, orientationConstraint

    def getSteeringWheelPalmFrame(self):
        frame = transformUtils.copyFrame(om.findObjectByName('Steering Wheel frame').transform)
        frame.PreMultiply()
        frame.RotateX(90)
        frame.PreMultiply()
        frame.RotateZ(-90)
        return frame


    def planBarGrasp(self,depth=0.03, useLineConstraint=False):
        ikPlanner = self.robotSystem.ikPlanner
        handSide = 'right'
        handLinkName = 'r_hand'
        startPose = self.getPlanningStartPose()
        startPoseName = 'q_grasp_start'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_end_grasp'

        palmToHand = ikPlanner.getPalmToHandLink(handSide)
        palmToWorld = transformUtils.copyFrame(ikPlanner.newGraspToWorldFrame(startPose, handSide, palmToHand))

        targetFrame = transformUtils.copyFrame(om.findObjectByName('right hand grab bar').transform)
        targetFrame.PreMultiply()
        targetFrame.Translate([0.0,-depth,0.0])

        finalPoseConstraints = self.createPalmPoseConstraints(handSide, targetFrame, tspan=[1,1])
        allButRightArmPostureConstraint = self.createAllButRightArmPostureConstraint(startPoseName)

        seedPoseName = 'q_bar_grab'
        seedPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'driving', 'bar_pre_grab', side=handSide)
        self.robotSystem.ikPlanner.addPose(seedPose, seedPoseName)

        constraints = [allButRightArmPostureConstraint]
        constraints.extend(finalPoseConstraints)
        constraintSet = ConstraintSet(ikPlanner, constraints, endPoseName, startPoseName)
        constraintSet.ikParameters = IkParameters(quasiStaticShrinkFactor=10, usePointwise=False)
        constraintSet.seedPoseName = seedPoseName
        constraintSet.nominalPoseName = seedPoseName

        # move on line constraint
        motionVector = np.array(targetFrame.GetPosition()) - np.array(palmToWorld.GetPosition())
        motionTargetFrame = transformUtils.getTransformFromOriginAndNormal(np.array(targetFrame.GetPosition()), motionVector)

        # vis.updateFrame(motionTargetFrame,'motion frame')
        # vis.updateFrame(targetFrame, 'target')
        # vis.updateFrame(currentFrame, 'current')

        p = ikPlanner.createLinePositionConstraint(handLinkName, palmToHand, motionTargetFrame, lineAxis=2, bounds=[-np.linalg.norm(motionVector)*1, 0], positionTolerance=0.001)
        p.tspan = np.linspace(0.2,0.8,5)

        endPose = constraintSet.runIk()
        if useLineConstraint:
            constraintSet.constraints.append(p)
            plan = constraintSet.runIkTraj()
        else:
            plan = constraintSet.planEndPoseGoal()

        self.plans.append(plan)
        return plan

    def planBarRetract(self, depth=0.3, useLineConstraint=False):
        ikPlanner = self.robotSystem.ikPlanner
        handSide = 'right'
        handLinkName = 'r_hand'
        startPose = self.getPlanningStartPose()
        startPoseName = 'q_grasp_start'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)
        endPoseName = 'q_end_grasp'
        maxBodyTranslationSpeed = 0.3

        palmToHand = ikPlanner.getPalmToHandLink(handSide)
        palmToWorld = transformUtils.copyFrame(ikPlanner.newGraspToWorldFrame(startPose, handSide, palmToHand))

        targetFrame = transformUtils.copyFrame(palmToWorld)
        targetFrame.PreMultiply()
        targetFrame.Translate([0.0,-depth,0.0])

        finalPoseConstraints = self.createPalmPoseConstraints(handSide, targetFrame, tspan=[1,1])
        allButRightArmPostureConstraint = self.createAllButRightArmPostureConstraint(startPoseName)



        seedPoseName = 'q_bar_grab'
        seedPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'driving', 'bar_pre_grab', side=handSide)
        self.robotSystem.ikPlanner.addPose(seedPose, seedPoseName)



        constraints = [allButRightArmPostureConstraint]
        constraints.extend(finalPoseConstraints)
        constraintSet = ConstraintSet(ikPlanner, constraints, endPoseName, startPoseName)
        constraintSet.ikParameters = IkParameters(quasiStaticShrinkFactor=10, usePointwise=False, maxBodyTranslationSpeed=0.3)
        constraintSet.seedPoseName = 'q_bar_grab'
        constraintSet.nominalPoseName = 'q_bar_grab'

        # move on line constraint
        motionVector = np.array(targetFrame.GetPosition()) - np.array(palmToWorld.GetPosition())
        motionTargetFrame = transformUtils.getTransformFromOriginAndNormal(np.array(targetFrame.GetPosition()), motionVector)

        # vis.updateFrame(motionTargetFrame,'motion frame')
        # vis.updateFrame(targetFrame, 'target')
        # vis.updateFrame(currentFrame, 'current')

        p = ikPlanner.createLinePositionConstraint(handLinkName, palmToHand, motionTargetFrame, lineAxis=2, bounds=[-np.linalg.norm(motionVector)*1, 0.0], positionTolerance=0.02)
        p.tspan = np.linspace(0,1,5)

        endPose = constraintSet.runIk()
        if useLineConstraint:
            constraintSet.constraints.append(p)
            plan = constraintSet.runIkTraj()
        else:
            plan = constraintSet.planEndPoseGoal()

        self.plans.append(plan)
        return plan

    def commitManipPlan(self):
        self.robotSystem.manipPlanner.commitManipPlan(self.plans[-1])

    def createAllButLeftLegPostureConstraint(self, poseName):
        joints = robotstate.matchJoints('^(?!l_leg)')
        return self.robotSystem.ikPlanner.createPostureConstraint(poseName, joints)

    def createAllButLeftArmPostureConstraint(self, poseName):
        joints = robotstate.matchJoints('^(?!l_arm)')
        return self.robotSystem.ikPlanner.createPostureConstraint(poseName, joints)

    def createAllButRightArmPostureConstraint(self, poseName):
        joints = robotstate.matchJoints('^(?!r_arm)')
        return self.robotSystem.ikPlanner.createPostureConstraint(poseName, joints)


    def captureLeftFootToRightFootTransform(self):
        startPose = self.getPlanningStartPose()

        lFoot2World = self.robotSystem.ikPlanner.getLinkFrameAtPose('l_foot', startPose)
        rFoot2World = self.robotSystem.ikPlanner.getLinkFrameAtPose('r_foot', startPose)

        lFoot2RFoot = transformUtils.concatenateTransforms([lFoot2World, rFoot2World.GetLinearInverse()])

        vis.showFrame(lFoot2RFoot, 'left foot to right foot', scale=0.2, visible=False)


    def computeDrivingTrajectories(self, steeringAngleDegrees, maxTurningRadius = 10, numTrajPoints = 50):

        angle = -steeringAngleDegrees

        if abs(angle) < 0.1:
            angle = 1e-8

        turningRadius = 1.0 / (angle * (1 / (maxTurningRadius * 170.0)))
        turningCenter = [0, turningRadius, 0]
        trajPoints = list()

        for i in range(0, numTrajPoints):
            theta = math.radians((40 / turningRadius) * i - 90)
            trajPoint = np.asarray(turningCenter)+turningRadius*np.asarray([math.cos(theta), math.sin(theta), 0])
            trajPoints.append(trajPoint)

        leftTraj = list()
        rightTraj = list()

        for i in range(0, numTrajPoints - 1):
                v1 = trajPoints[i + 1] - trajPoints[i]
                v2 = np.cross(v1, [0, 0, 1])
                v2 /= np.linalg.norm(v2)
                leftTraj.append(trajPoints[i] - 0.5 * self.wheelDistance * v2)
                rightTraj.append(trajPoints[i] + 0.5 * self.wheelDistance * v2)

        return leftTraj, rightTraj

    def transformDrivingTrajectory(self, drivingTraj):
        transformedDrivingTraj = list()
        transform = vtk.vtkTransform()

        z_axis = self.tagToLocalTransform.TransformVector([0,0,1])
        tag_origin = self.tagToLocalTransform.TransformPoint([0,0,0])

        z_norm = np.linalg.norm(z_axis[0:2])
        if z_norm > 1e-6:
            z_axis_proj = z_axis[0:2] / z_norm
            angle = math.degrees(math.atan2(z_axis_proj[1], z_axis_proj[0]))
        else:
            angle = 0


        transform.Translate([tag_origin[0] , tag_origin[1], 0])
        transform.RotateZ(self.trajectoryAngle + angle)
        transform.Translate([self.trajectoryX, self.trajectoryY, 0])

        for p in drivingTraj:
            transformedPoint = np.asarray(transform.TransformPoint(p))
            transformedDrivingTraj.append(transformedPoint)

        return transformedDrivingTraj


    def onThrottleCommand(self, msg):

        # slider 0 is the coarse grained slider, slider 1 is for fine grained adjustment
        slider = self.decodeThrottleMessage(msg)
        const = np.rad2deg(self.jointLimitsMin[self.akyIdx])
        ankleGoalPosition = const + slider[0]*self.coarseGrainedThrottleTravel + (slider[1]-1/2.0)*self.fineGrainedThrottleTravel
        ankleGoalPositionRadians = np.deg2rad(ankleGoalPosition)

        # trip the safety if slider[3] is < 1/2, emergency come off the throttle
        if slider[3] < 0.5:
            print('Emergency stop, coming off the throttle')
            print("setting l_leg_aky to it's min value")
            ankleGoalPositionRadians = self.jointLimitsMin[self.akyIdx]


        msg = lcmdrc.joint_position_goal_t()
        msg.utime = getUtime()
        msg.joint_position = ankleGoalPositionRadians
        msg.joint_name = drcargs.getDirectorConfig()['drivingThrottleJoint']
        self.throttleCommandMsg = msg

    def publishThrottleCommand(self):
        if not self.throttleStreaming:
            return

        if self.throttleCommandMsg is None:
            return

        lcmUtils.publish(self.throttlePublishChannel, self.throttleCommandMsg)

    def publishSteeringCommand(self):
        if not self.steeringStreaming:
            return

        if self.steeringCommandMsg is None:
            return

        lcmUtils.publish(self.steeringPublishChannel, self.steeringCommandMsg)

    def onSteeringCommand(self, msg):

        steeringAngle = -msg.steering_angle
        lwyPositionGoal = steeringAngle + self.steeringAngleOffset
        msg = lcmdrc.joint_position_goal_t()
        msg.utime = getUtime()
        msg.joint_position = lwyPositionGoal
        msg.joint_name = self.drivingThrottleJoint
        self.steeringCommandMsg = msg


    def decodeThrottleMessage(self,msg):
        slider = np.zeros(4)
        slider[0] = msg.slider1
        slider[1] = msg.slider2
        slider[2] = msg.slider3
        slider[3] = msg.slider4

        return slider

    def captureRobotPoseFromStreaming(self):
        helper = lcmUtils.MessageResponseHelper(self.commandStreamChannel, bot_core.robot_state_t)
        msg = helper.waitForResponse(timeout=1000, keepAlive=False)

        if msg is None:
            print("Didn't receive a JOINT_POSITION_GOAL message")
            print("Are you streaming?")
            return None

        pose = robotstate.convertStateMessageToDrakePose(msg)
        return pose

    def planCarEntryPose(self):
        ikPlanner = self.robotSystem.ikPlanner
        startPose = self.getPlanningStartPose()
        endPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'driving', 'car_entry_new')
        plan = ikPlanner.computePostureGoal(startPose, endPose, feetOnGround=False)
        self.addPlan(plan)

    def planArmsEgressPrep(self):
        startPose = self.getPlanningStartPose()
        ikPlanner = self.robotSystem.ikPlanner
        ikParameters = IkParameters(maxDegreesPerSecond=60)
        midPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'driving', 'pre_egress_left_arm', side='left')
        midPose = ikPlanner.getMergedPostureFromDatabase(midPose, 'driving', 'pre_egress_right_arm', side='right')

        # endPose = ikPlanner.getMergedPostureFromDatabase(midPose, 'driving', 'egress-arms')

        # plan = ikPlanner.computeMultiPostureGoal([startPose, midPose, endPose], feetOnGround=False, ikParameters=ikParameters)

        plan = ikPlanner.computePostureGoal(startPose, midPose, feetOnGround=False, ikParameters=ikParameters)
        self.addPlan(plan)
    def planArmsEgressStart(self, startPose=None):
        if startPose is None:
            startPose = self.getPlanningStartPose()

        ikPlanner = self.robotSystem.ikPlanner
        endPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'driving', 'egress-left-arm', side='left')
        endPose = ikPlanner.getMergedPostureFromDatabase(endPose, 'General', 'crane', side='right')
        ikParameters = IkParameters(maxDegreesPerSecond=60)
        plan = ikPlanner.computePostureGoal(startPose, endPose, feetOnGround=False, ikParameters=ikParameters)
        self.addPlan(plan)

    def planArmsEgress(self):
        self.planArmsEgressPrep()
        armsEgressPrepName = 'qtraj_arms_prep'
        self.saveOriginalTraj(armsEgressPrepName)

        nextStartPose = robotstate.convertStateMessageToDrakePose(self.plans[-1].plan[-1])

        self.planArmsEgressStart(startPose=nextStartPose)
        armsEgressStartName = 'qtraj_arms_egress_start'
        self.saveOriginalTraj(armsEgressStartName)


        ikParameters = IkParameters(usePointwise=False, maxDegreesPerSecond=60)
        ikParameters = self.robotSystem.ikPlanner.mergeWithDefaultIkParameters(ikParameters)

        listener = self.getManipPlanListener()
        _ = self.concatenateAndRescaleTrajectories([armsEgressPrepName, armsEgressStartName], 'qtraj_arms_egress', 'ts', ikParameters)

        plan = listener.waitForResponse()
        listener.finish()
        self.addPlan(plan)


    def setSteeringWheelAndWristGraspAngles(self):
        self.graspWheelAngle = np.deg2rad(self.userSpecifiedGraspWheelAngleInDegrees)
        pose = self.getPlanningStartPose()
        self.graspWristAngle = pose[self.lwyIdx]

    def getSteeringWheelAngle(self):
        if self.graspWristAngle is None or self.graspWheelAngle is None:
            # this means wrist and hand haven't been initialized yet
            return 0

        pose = self.getPlanningStartPose()
        lwyAngle = pose[self.lwyIdx]

        wheelAngle = self.graspWheelAngle + lwyAngle - self.graspWristAngle
        return wheelAngle


    # executes regrasp plan, updates graspWristAngle, graspWheelAngle
    def updateGraspOffsets(self):
        pose = self.getPlanningStartPose()
        #now that plan has finished update our graspWristAngle
        self.graspWristAngle = pose[self.lwyIdx]
        self.graspWheelAngle = self.wheelAngleBeforeReGrasp

    def printSteeringWheelAngleInDegrees(self):
        print(np.rad2deg(self.getSteeringWheelAngle()))

    def addPlan(self, plan):
        self.plans.append(plan)

    def getManipPlanListener(self):
        responseChannel = 'CANDIDATE_MANIP_PLAN'
        responseMessageClass = lcmdrc.robot_plan_w_keyframes_t
        return lcmUtils.MessageResponseHelper(responseChannel, responseMessageClass)

    def saveOriginalTraj(self, name):
            commands = ['%s = qtraj_orig;' % name]
            self.robotSystem.ikServer.comm.sendCommands(commands)

    def concatenateAndRescaleTrajectories(self, trajectoryNames, concatenatedTrajectoryName, junctionTimesName, ikParameters):
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


class DrivingPlannerPanel(TaskUserPanel):

    def __init__(self, robotSystem):

        TaskUserPanel.__init__(self, windowTitle='Driving Task')

        self.robotSystem = robotSystem
        self.drivingPlanner = DrivingPlanner(robotSystem.ikServer, robotSystem)
        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()
        self.apriltagSub = lcmUtils.addSubscriber('APRIL_TAG_TO_CAMERA_LEFT', lcmbotcore.rigid_transform_t, self.onAprilTag)
        self.imageView = cameraview.CameraImageView(cameraview.imageManager, 'CAMERACHEST_RIGHT', 'right image view')
        self.imageViewLeft = cameraview.CameraImageView(cameraview.imageManager, 'MULTISENSE_CAMERA_LEFT', 'left image view')

        self.imageView.view.orientationMarkerWidget().Off()
        self.imageView.view.backgroundRenderer().SetBackground([0,0,0])
        self.imageView.view.backgroundRenderer().SetBackground2([0,0,0])

        self.imageViewLeft.view.orientationMarkerWidget().Off()
        self.imageViewLeft.view.backgroundRenderer().SetBackground([0,0,0])
        self.imageViewLeft.view.backgroundRenderer().SetBackground2([0,0,0])

        self.affordanceUpdater = affordanceupdater.AffordanceInCameraUpdater(segmentation.affordanceManager, self.imageView)
        self.affordanceUpdaterLeft = affordanceupdater.AffordanceInCameraUpdater(segmentation.affordanceManager, self.imageViewLeft)
        self.affordanceUpdater.prependImageName = True
        self.affordanceUpdaterLeft.prependImageName = True
        self.affordanceUpdater.projectAffordances = False
        self.affordanceUpdaterLeft.projectAffordances = False

        self.imageViewLayout.addWidget(self.imageView.view)
        self.imageViewLayout.addWidget(self.imageViewLeft.view)

        self.timer = TimerCallback(targetFps=10)
        self.timer.callback = self.updateAndDrawTrajectory


    def onAprilTag(self, msg):
        cameraview.imageManager.queue.getTransform('april_tag_car_beam', 'local', msg.utime, self.drivingPlanner.tagToLocalTransform)

    def addButtons(self):
        self.addManualButton('Start', self.onStart)
        self.addManualButton('Update Wheel Location', self.onUpdateWheelLocation)
        self.addManualButton('Plan Safe', self.onPlanSafe)
        self.addManualButton('Plan Pre Grasp', self.onPlanPreGrasp)
        self.addManualButton('Plan Touch', self.onPlanTouch)
        self.addManualButton('Plan Retract', self.onPlanRetract)
        self.addManualButton('Plan Turn', self.onPlanTurn)
        self.addManualButton('Plan Wheel Re-Grasp', self.drivingPlanner.planSteeringWheelReGrasp)
        self.addManualButton('Plan Bar Grab', self.onPlanBarGrasp)
        self.addManualButton('Plan Bar Retract', self.onPlanBarRetract)
        # self.addManualButton('Plan Steering Wheel Turn', self.onPlanSteeringWheelTurn)
        # self.addManualButton('Plan Seed', self.drivingPlanner.planSeed)
        # self.addManualButton('Capture Ankle Angle Low', functools.partial(self.drivingPlanner.captureAnklePosition, 0))
        # self.addManualButton('Capture Ankle Angle High', functools.partial(self.drivingPlanner.captureAnklePosition, 1))
        self.addManualButton('Capture Wheel and Wrist grasp angles', self.drivingPlanner.setSteeringWheelAndWristGraspAngles)
        self.addManualButton('Print Steering Wheel Angle', self.drivingPlanner.printSteeringWheelAngleInDegrees)

        self.addManualSpacer()

        self.addManualButton('Arms Egress Prep', self.drivingPlanner.planArmsEgressPrep)
        self.addManualButton('Arms Egress Start', self.drivingPlanner.planArmsEgressStart)
        self.addManualButton('Plan Left Leg Egress Start', self.drivingPlanner.planLegEgressStart)

    def addDefaultProperties(self):
        self.params.addProperty('PreGrasp/Retract Depth', 0.2, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Touch Depth', 0.0, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('PreGrasp Angle', 0, attributes=om.PropertyAttributes(singleStep=10))
        self.params.addProperty('Turn Angle', 0, attributes=om.PropertyAttributes(singleStep=10))
        # self.params.addProperty('Steering Wheel Radius (meters)', 0.1873, attributes=om.PropertyAttributes(singleStep=0.01))
        # self.params.addProperty('Knot Points', 20, attributes=om.PropertyAttributes(singleStep=1))
        # self.params.addProperty('Gaze Constraint Tol', 0.3, attributes=om.PropertyAttributes(singleStep=0.1, decimals=2))
        self.params.addProperty('Position Constraint Tol', 0.0, attributes=om.PropertyAttributes(singleStep=0.01, decimals=2))
        self.params.addProperty('Quat Constraint Tol', 0.0, attributes=om.PropertyAttributes(singleStep=0.01, decimals=2))
        self.params.addProperty('Grasp Location', 0, attributes=om.PropertyAttributes(enumNames=['Center','Rim']))
        self.params.addProperty('Seed with current posture', 0, attributes=om.PropertyAttributes(enumNames=['False','True']))
        self.params.addProperty('Speed', 0.75, attributes=om.PropertyAttributes(singleStep=0.1, decimals=2))
        # self.params.addProperty('Throttle Idle Angle Slack', 10, attributes=om.PropertyAttributes(singleStep=1))
        self.params.addProperty('Coarse Grained Throttle Travel', 100, attributes=om.PropertyAttributes(singleStep=10))
        self.params.addProperty('Fine Grained Throttle Travel', 30, attributes=om.PropertyAttributes(singleStep=1))
        self.params.addProperty('Throttle Streaming', False)
        self.params.addProperty('Steering Streaming', False)
        self.params.addProperty('Bar Grasp/Retract Depth', 0.1, attributes=om.PropertyAttributes(singleStep=0.01, decimals=2))
        self.params.addProperty('Pedal Foot Location', 1, attributes=om.PropertyAttributes(enumNames=['Standard','Knee In']))


        self.params.addProperty('Steering Wheel Angle when Grasped', 0, attributes=om.PropertyAttributes(singleStep=10))
        self.params.addProperty('Turning Radius', 9.5, attributes=om.PropertyAttributes(singleStep=0.01, decimals=2))
        self.params.addProperty('Wheel Separation', 1.4, attributes=om.PropertyAttributes(singleStep=0.01, decimals=2))
        self.params.addProperty('Trajectory Segments', 25, attributes=om.PropertyAttributes(singleStep=1, decimals=0))
        self.params.addProperty('Trajectory X Offset', 0.0, attributes=om.PropertyAttributes(singleStep=0.01, decimals=2)),
        self.params.addProperty('Trajectory Y Offset', 0.30, attributes=om.PropertyAttributes(singleStep=0.01, decimals=2))
        self.params.addProperty('Trajectory Angle Offset', 0.0, attributes=om.PropertyAttributes(singleStep=1, decimals=0)),
        self.params.addProperty('Show Trajectory', False)

        self.params.addProperty('Show Driving/Regrasp Tasks',0, attributes=om.PropertyAttributes(enumNames=['Ingress','Regrasp', 'Egress']))
        self._syncProperties()

    def _syncProperties(self):
        self.preGraspDepth = self.params.getProperty('PreGrasp/Retract Depth')
        self.touchDepth = self.params.getProperty('Touch Depth')
        self.preGraspAngle = self.params.getProperty('PreGrasp Angle')
        self.turnAngle = self.params.getProperty('Turn Angle')
        self.speed = self.params.getProperty('Speed')
        self.turnRadius = 0.18 #self.params.getProperty('Steering Wheel Radius (meters)')
        self.knotPoints = 20
        self.gazeTol = 0.3
        self.drivingPlanner.positionTol = 0.0
        self.drivingPlanner.quatTol = 0.0
        self.graspLocation = 'center'
        self.drivingPlanner.seedWithCurrent = self.params.getProperty('Seed with current posture')
        # self.drivingPlanner.throttleIdleAngleSlack = self.params.getProperty('Throttle Idle Angle Slack')
        self.drivingPlanner.fineGrainedThrottleTravel = self.params.getProperty('Fine Grained Throttle Travel')
        self.drivingPlanner.coarseGrainedThrottleTravel = self.params.getProperty('Coarse Grained Throttle Travel')
        self.drivingPlanner.throttleStreaming = self.params.getProperty('Throttle Streaming')
        self.drivingPlanner.steeringStreaming = self.params.getProperty('Steering Streaming')
        self.barGraspDepth = self.params.getProperty('Bar Grasp/Retract Depth')
        self.drivingPlanner.maxTurningRadius = self.params.getProperty('Turning Radius')
        self.drivingPlanner.userSpecifiedGraspWheelAngleInDegrees = self.params.getProperty('Steering Wheel Angle when Grasped')
        self.drivingPlanner.trajSegments = self.params.getProperty('Trajectory Segments')
        self.drivingPlanner.wheelDistance = self.params.getProperty('Wheel Separation')
        self.drivingPlanner.trajectoryX = self.params.getProperty('Trajectory X Offset')
        self.drivingPlanner.trajectoryY = self.params.getProperty('Trajectory Y Offset')
        self.drivingPlanner.trajectoryAngle = self.params.getProperty('Trajectory Angle Offset')
        self.drivingPlanner.kneeInPedal = self.params.getProperty('Pedal Foot Location')
        self.taskToShow = self.params.getProperty('Show Driving/Regrasp Tasks')
        self.drivingPlanner.applyProperties()


    def onSteeringCommand(self, msg):
        if msg.type == msg.TYPE_DRIVE_DELTA_STEERING:
            self.drivingPlanner.steeringAngleDegrees = math.degrees(msg.steering_angle)

    def onStart(self):
        self.onUpdateWheelLocation()
        print('Driving Planner Ready')

    def onUpdateWheelLocation(self):
        f = om.findObjectByName('Steering Wheel').getChildFrame().transform
        xyzquat = transformUtils.poseFromTransform(f)
        xyzquat = np.concatenate(xyzquat)
        self.drivingPlanner.updateWheelTransform(xyzquat)

    def onPlanSafe(self):
        self.drivingPlanner.planSafe()

    def onPlanPreGrasp(self, depth=None):
        self.drivingPlanner.planPreGrasp(depth=self.preGraspDepth, speed=self.speed, angle=self.preGraspAngle,
            graspLocation=self.graspLocation, turnRadius=self.turnRadius)

    def onPlanTouch(self):
        self._syncProperties()
        self.drivingPlanner.planTouch(depth=self.touchDepth, speed=self.speed)

    def onPlanRetract(self):
        self._syncProperties()
        self.drivingPlanner.planRetract(depth=self.preGraspDepth, speed=self.speed)

    def onPlanTurn(self):
        self._syncProperties()
        self.drivingPlanner.planTurn(angle=self.turnAngle, speed=self.speed)

    def onPlanSteeringWheelTurn(self):
        self._syncProperties()
        self.drivingPlanner.planSteeringWheelTurn(speed=self.speed, turnRadius=self.turnRadius, knotPoints=self.knotPoints, gazeTol=self.gazeTol)

    def onPropertyChanged(self, propertySet, propertyName):
        taskToShowOld = self.taskToShow
        self._syncProperties()

        if not taskToShowOld == self.taskToShow:
            self.addTasks()


        if propertyName == 'Throttle Streaming':
            if self.params.getProperty(propertyName):
                self.drivingPlanner.throttleCommandTimer.start()
            else:
                self.drivingPlanner.throttleCommandTimer.stop()

        elif propertyName == 'Steering Streaming':
            if self.params.getProperty(propertyName):
                self.drivingPlanner.steeringCommandTimer.start()
            else:
                self.drivingPlanner.steeringCommandTimer.stop()

        elif propertyName == 'Show Trajectory':

            if self.params.getProperty(propertyName):
                self.timer.start()
                self.affordanceUpdater.timer.start()
                self.affordanceUpdaterLeft.timer.start()
            else:
                self.timer.stop()
                self.affordanceUpdater.cleanUp()
                self.affordanceUpdaterLeft.cleanUp()
                self.affordanceUpdater.extraObjects = []
                self.affordanceUpdaterLeft.extraObjects = []
                om.removeFromObjectModel(om.findObjectByName('driving trajectory'))


    def onPlanBarRetract(self):
        self.drivingPlanner.planBarRetract(depth=self.barGraspDepth, useLineConstraint=True)

    def onPlanBarGrasp(self):
        self.drivingPlanner.planBarGrasp(depth=self.barGraspDepth, useLineConstraint=True)

    def setParamsPreGrasp1(self):
        self.params.setProperty('PreGrasp/Retract Depth', 0.22)

    def setParamsPreGrasp2(self):
        self.params.setProperty('PreGrasp/Retract Depth', 0.12)

    def setParamsWheelRetract(self):
        self.params.setProperty('PreGrasp/Retract Depth', 0.3)

    def setParamsBarRetract(self):
        self.params.setProperty('Bar Grasp/Retract Depth', 0.3)

    def setParamsBarGrasp(self):
        self.params.setProperty('Bar Grasp/Retract Depth', -0.015)

    def startSteering(self):
        self.params.setProperty('Steering Streaming', 1)

    def stopSteering(self):
        self.params.setProperty('Steering Streaming', 0)

    def stopStreaming(self):
        self.params.setProperty('Steering Streaming', 0)
        self.params.setProperty('Throttle Streaming', 0)

    def addTasks(self):
        self.taskTree.removeAllTasks()

        if self.taskToShow == 0:
            self.addIngressTasks()
        elif self.taskToShow == 1:
            self.addRegraspTasks()
        elif self.addEgressTasks() == 2:
            self.addEgressTasks()
        else:
            return

    def addIngressTasks(self):

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

        def addManipTaskMatlab(name, planFunc, userPrompt=False, parentFolder=None):

            prevFolder = self.folder
            addFolder(name, prevFolder)
            addFunc(planFunc, 'plan')
            addTask(rt.UserPromptTask(name='approve manip plan', message='Please approve and commit manipulation plan.'))
            addTask(rt.UserPromptTask(name='wait for plan execution', message='Continue when plan finishes.'))

        def addManipTask(name, planFunc, userPrompt=False):

            prevFolder = self.folder
            addFolder(name, prevFolder)
            addFunc(planFunc, 'plan')
            if not userPrompt:
                addTask(rt.CheckPlanInfo(name='check manip plan info'))
            else:
                addTask(rt.UserPromptTask(name='approve manip plan', message='Please approve manipulation plan.'))
            addFunc(dp.commitManipPlan, name='execute manip plan')
            addTask(rt.UserPromptTask(name='wait for plan execution', message='Continue when plan finishes.'))

        dp = self.drivingPlanner

        prep = addFolder('Prep')
        addTask(rt.UserPromptTask(name="confirm user mode", message="Please go to User mode"))
        addTask(rt.UserPromptTask(name="Confirm pressure", message='Confirm that pressure is set for ingress (2000 psi)'))
        addTask(rt.UserPromptTask(name="start streaming", message="Please start streaming"))
        addManipTask('car entry posture', self.drivingPlanner.planCarEntryPose, userPrompt=True)
        self.folder = prep
        addTask(rt.SetNeckPitch(name='set neck position', angle=30))
        addFunc(self.drivingPlanner.captureLeftFootToRightFootTransform, 'capture lfoot to rfoot transform')
        addTask(rt.UserPromptTask(name="spawn polaris model", message="launch egress planner and spawn polaris model"))
        addFunc(self.onStart, 'update wheel location')

        graspWheel = addFolder('Grasp Steering Wheel')
        addTask(rt.OpenHand(name='open left hand', side='Left'))
        addFunc(self.setParamsPreGrasp1, 'set params')
        addManipTask('Pre Grasp 1', self.onPlanPreGrasp, userPrompt=True)
        self.folder = graspWheel
        addTask(rt.UserPromptTask(name="check alignment", message="Please ask field team for hand location relative to wheel, adjust wheel affordance if necessary"))
        addFunc(self.setParamsPreGrasp2, 'set params')
        addManipTask('Pre Grasp 2', self.onPlanPreGrasp, userPrompt=True)
        self.folder = graspWheel
        addTask(rt.UserPromptTask(name="check alignment", message="Please make any manual adjustments if necessary"))
        addTask(rt.CloseHand(name='close left hand', side='Left'))
        addTask(rt.UserPromptTask(name="set true steering wheel angle", message="Set true steering wheel angle in spin box"))
        addFunc(self.drivingPlanner.setSteeringWheelAndWristGraspAngles, 'capture true wheel angle and current wrist angle')

        graspBar = addFolder('Grasp Bar')
        addTask(rt.OpenHand(name='open right hand', side='Right'))
        addFunc(self.setParamsBarGrasp, 'set params')
        addManipTask('Bar Grasp', self.onPlanBarGrasp, userPrompt=True)
        self.folder = graspBar
        addTask(rt.UserPromptTask(name="check alignment and depth", message="Please check alignment and depth, make any manual adjustments"))
        addTask(rt.CloseHand(name='close Right hand', side='Right'))

        footToDriving = addFolder('Foot to Driving Pose')
        addManipTask('Foot Up', self.drivingPlanner.planLegUp, userPrompt=True)
        self.folder = footToDriving
        addManipTask('Swing leg in', self.drivingPlanner.planLegSwingIn , userPrompt=True)
        self.folder = footToDriving
        addManipTask('Foot On Pedal', self.drivingPlanner.planLegPedal, userPrompt=True)

        driving = addFolder('Driving')
        addTask(rt.UserPromptTask(name="launch drivers", message="Please launch throttle and steering drivers"))
        addTask(rt.UserPromptTask(name="switch to regrasp tasks", message="Switch to regrasp task set"))

    def addEgressTasks(self):

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

            prevFolder = self.folder
            addFolder(name, prevFolder)
            addFunc(planFunc, 'plan')
            if not userPrompt:
                addTask(rt.CheckPlanInfo(name='check manip plan info'))
            else:
                addTask(rt.UserPromptTask(name='approve manip plan', message='Please approve manipulation plan.'))
            addFunc(dp.commitManipPlan, name='execute manip plan')
            addTask(rt.UserPromptTask(name='wait for plan execution', message='Continue when plan finishes.'))


        dp = self.drivingPlanner

        addFolder('Stop throttle and steering')
        addFunc(self.stopStreaming, 'stop steering and throttle streaming')
        addTask(rt.UserPromptTask(name="Confirm steering and throttle streaming is off", message='Confirm steering and throttle streaming is off, move sliders and wheel to check'))

        # footToEgress = addFolder('Foot to Egress Pose')
        self.folder = None
        addManipTask('Foot to Egress Start', self.drivingPlanner.planLegEgressFull, userPrompt=True)
        # addManipTask('Foot Off Pedal', self.drivingPlanner.planLegAbovePedal, userPrompt=True)
        # self.folder = footToEgress
        # addManipTask('Swing leg out', self.drivingPlanner.planLegSwingOut , userPrompt=True)
        # self.folder = footToEgress
        # addManipTask('Foot Down', self.drivingPlanner.planLegEgressStart, userPrompt=True)

        addFunc(self.onUpdateWheelLocation, 'Update wheel location')

        ungraspWheel = addFolder('Ungrasp Steering Wheel')
        addTask(rt.UserPromptTask(name="Confirm pressure", message='Confirm that pressure is set for prep-for-egress (2200 psi)'))
        addTask(rt.OpenHand(name='open left hand', side='Left'))
        addTask(rt.UserPromptTask(name="confirm hand is open", message="Confirm the left hand has opened"))
        addFunc(self.setParamsWheelRetract, 'set params')
        addManipTask('Retract hand', self.onPlanRetract, userPrompt=True)
        self.folder = ungraspWheel
        addTask(rt.CloseHand(name='close left hand', side='Left'))

        ungraspBar = addFolder('Ungrasp Bar')
        addTask(rt.OpenHand(name='open right hand', side='Right'))
        addTask(rt.UserPromptTask(name="confirm hand is open", message="Confirm the right hand has opened"))
        addFunc(self.setParamsBarRetract, 'set params')
        addManipTask('Retract hand', self.onPlanBarRetract, userPrompt=True)
        self.folder = ungraspBar
        addTask(rt.CloseHand(name='close Right hand', side='Right'))

        # armsToEgressStart = addFolder('Arms to Egress Position')
        self.folder = None
        addManipTask('Arms to Egress', self.drivingPlanner.planArmsEgress, userPrompt=True)
        # addManipTask('Arms To Egress Prep', self.drivingPlanner.planArmsEgressPrep, userPrompt=True)
        # self.folder = armsToEgressStart
        # addManipTask('Arms To Egress Start', self.drivingPlanner.planArmsEgressStart, userPrompt=True)

        prep = addFolder('Stop Streaming')
        addTask(rt.UserPromptTask(name='stop streaming base side', message='stop streaming base side'))



    def addRegraspTasks(self):
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

            prevFolder = self.folder
            addFolder(name, prevFolder)
            addFunc(planFunc, 'plan')
            if not userPrompt:
                addTask(rt.CheckPlanInfo(name='check manip plan info'))
            else:
                addTask(rt.UserPromptTask(name='approve manip plan', message='Please approve manipulation plan.'))
            addFunc(dp.commitManipPlan, name='execute manip plan')
            addTask(rt.UserPromptTask(name='wait for plan execution', message='Continue when plan finishes.'))

        dp = self.drivingPlanner
        regrasp = addFolder('Regrasp')
        addFunc(self.stopSteering, 'stop steering commands')
        addTask(rt.UserPromptTask(name="high pressure", message="set pump to 2400 psi"))
        addFunc(self.onUpdateWheelLocation, 'update wheel location')
        addFunc(self.drivingPlanner.captureHandPose, 'capture hand pose')
        addTask(rt.UserPromptTask(name="approve open left hand", message="Check ok to open left hand"))
        addTask(rt.OpenHand(name='open left hand', side='Left'))
        addTask(rt.UserPromptTask(name="confirm hand is open", message="Confirm the left hand has opened"))
        addFunc(self.setParamsWheelRetract, 'set params')
        addManipTask('Retract hand', self.onPlanRetract, userPrompt=True)
        self.folder = regrasp
        addManipTask('Plan Regrasp', self.drivingPlanner.planSteeringWheelReGrasp, userPrompt=True)
        self.folder = regrasp
        addTask(rt.UserPromptTask(name="approve close left hand", message="Check ok to close left hand"))
        addTask(rt.CloseHand(name='close left hand', side='Left'))
        addFunc(self.drivingPlanner.updateGraspOffsets, 'update steering wheel grasp offsets')
        addTask(rt.UserPromptTask(name="driving pressure", message="set pump to 1500 psi"))
        addTask(rt.UserPromptTask(name="reset steering wheel", message="Set the steering wheel to approximate lwy angle"))
        addFunc(self.startSteering, 'start steering commands')


    def updateAndDrawTrajectory(self):
        if not self.params.getProperty('Show Trajectory') or om.findObjectByName('Steering Wheel') is None:
            return None

        steeringAngleDegrees = np.rad2deg(self.drivingPlanner.getSteeringWheelAngle())
        leftTraj, rightTraj = self.drivingPlanner.computeDrivingTrajectories(steeringAngleDegrees, self.drivingPlanner.maxTurningRadius, self.drivingPlanner.trajSegments + 1)

        d = DebugData()

        for traj in [leftTraj, rightTraj]:
            traj = self.drivingPlanner.transformDrivingTrajectory(traj)
            numTrajPoints = len(traj)

            for i in range(numTrajPoints):
                rgb = [(numTrajPoints - i) / float(numTrajPoints), 1 - (numTrajPoints - i) / float(numTrajPoints), 1]
                d.addSphere(traj[i], 0.05, rgb, resolution=12)

        obj = vis.updatePolyData(d.getPolyData(), 'driving trajectory', colorByName='RGB255', parent='planning')

        for updater in [self.affordanceUpdater, self.affordanceUpdaterLeft]:
            updater.extraObjects = [obj]

        return obj
