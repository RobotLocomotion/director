import ddapp
import math
import textwrap
import drc as lcmdrc
import bot_core as lcmbotcore
import vtkAll as vtk
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp import objectmodel as om
from ddapp import lcmUtils
from ddapp import ik
from ddapp import cameraview
from ddapp import affordanceupdater
from ddapp import segmentation
from ddapp import robotstate
from ddapp.debugVis import DebugData
from ddapp.utime import getUtime


import os
import functools
import numpy as np
import scipy.io
from ddapp.tasks.taskuserpanel import TaskUserPanel

class DrivingPlanner(object):

    def __init__(self, ikServer, robotSystem):
        self.ikServer = ikServer
        self.robotSystem = robotSystem
        self.ikServer.connectStartupCompleted(self.initialize)
        self.steeringAngleDegrees = 0.0
        self.maxTurningRadius = 9.5
        self.trajectoryX = 0
        self.trajectoryY = 0
        self.trajectoryAngle = 0
        self.trajSegments = 25
        self.wheelDistance = 1.4
        self.tagToLocalTransform = transformUtils.transformFromPose([0,0,0],[1,0,0,0])

        self.commandStreamChannel = 'JOINT_POSITION_GOAL'
        self.akyIdx = robotstate.getDrakePoseJointNames().index('l_leg_aky')
        self.lwyIdx = robotstate.getDrakePoseJointNames().index('l_arm_lwy')
        self.anklePositions = np.array([np.nan,np.nan])
        self.jointLimitsMin = np.array([self.robotSystem.teleopRobotModel.model.getJointLimits(jointName)[0] for jointName in robotstate.getDrakePoseJointNames()])
        self.jointLimitsMax = np.array([self.robotSystem.teleopRobotModel.model.getJointLimits(jointName)[1] for jointName in robotstate.getDrakePoseJointNames()])
        self.idleAngleSlack = 10
        self.fineGrainedThrottleTravel = 10
        self.steeringAngleOffset = 0
        self.throttlePublishChannel = 'THROTTLE_COMMAND_POSITION_GOAL'
        self.steeringPublishChannel = 'STEERING_COMMAND_POSITION_GOAL'
        self.addSubscribers()

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
        lcmUtils.addSubscriber('STEERING_COMMAND', lcmdrc.driving_control_cmd_t, self.onSteeringCommand)

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
        commands.append("q0 = %s;" % ik.ConstraintBase.toColumnVectorString(startPose))
        commands.append("xyzquat = %s;" % ik.ConstraintBase.toColumnVectorString(xyzquat))
        commands.append("dp = dp.updateWheelTransform(xyzquat, q0);")

        self.ikServer.taskQueue.addTask(functools.partial(self.ikServer.comm.sendCommandsAsync, commands))
        self.ikServer.taskQueue.start()

    def planSafe(self, speed=1):
        commands = []
        commands.append("clear options;")
        commands.append("options.speed = %r;" % speed)
        startPose = self.getPlanningStartPose()
        commands.append("dp.planSafe(options,%s);" % ik.ConstraintBase.toColumnVectorString(startPose))

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
            commands.append("options.xyz_des = {%s};",ik.ConstraintBase.toColumnVectorString(xyz_des))
        startPose = self.getPlanningStartPose()
        commands.append("dp.planPreGrasp(options, %s);" % ik.ConstraintBase.toColumnVectorString(startPose))

        self.ikServer.taskQueue.addTask(functools.partial(self.ikServer.comm.sendCommandsAsync, commands))
        self.ikServer.taskQueue.start()

    def planTouch(self, depth=0, xyz_des=None, speed=1):
        commands = []
        commands.append("clear options;")
        commands.append("options = struct('depth',{%r});" % depth)
        commands.append("options.speed = %r;" % speed)
        startPose = self.getPlanningStartPose()
        commands.append("dp.planTouch(options, %s);" % ik.ConstraintBase.toColumnVectorString(startPose))

        self.ikServer.taskQueue.addTask(functools.partial(self.ikServer.comm.sendCommandsAsync, commands))
        self.ikServer.taskQueue.start()

    def planRetract(self, depth=0.2, speed=1):
        commands = []
        commands.append("clear options;")
        commands.append("options = struct('depth',{%r});" % depth)
        commands.append("options.speed = %s;" % speed)
        startPose = self.getPlanningStartPose()
        commands.append("dp.planRetract(options, %s);" % ik.ConstraintBase.toColumnVectorString(startPose))

        self.ikServer.taskQueue.addTask(functools.partial(self.ikServer.comm.sendCommandsAsync, commands))
        self.ikServer.taskQueue.start()

    def planTurn(self, angle=0, speed=1):
        commands = []
        commands.append("clear options;")
        commands.append("options.turn_angle = %r;" % np.radians(angle))
        commands.append("options.speed = %r;" % speed)
        commands.append("options.use_raw_angle = 1;")
        startPose = self.getPlanningStartPose()
        commands.append("dp.planTurn(options,%s);" % ik.ConstraintBase.toColumnVectorString(startPose))

        self.ikServer.taskQueue.addTask(functools.partial(self.ikServer.comm.sendCommandsAsync, commands))
        self.ikServer.taskQueue.start()

    def planSteeringWheelTurn(self, speed=1, knotPoints=20, turnRadius=.187, gazeTol=0.3):
        commands = []
        commands.append("clear options;")
        commands.append("options.speed = %r;" % speed)
        commands.append("options.turn_radius = %r;" % turnRadius)
        commands.append("options.N = %r;" % knotPoints)
        commands.append("options.steering_gaze_tol = %r;" % gazeTol)
        startPose = self.getPlanningStartPose()
        commands.append("dp.planSteeringWheelTurn(options,%s);" % ik.ConstraintBase.toColumnVectorString(startPose))

        self.ikServer.taskQueue.addTask(functools.partial(self.ikServer.comm.sendCommandsAsync, commands))
        self.ikServer.taskQueue.start()

    def planSeed(self):
        commands = []
        startPose = self.getPlanningStartPose()
        commands.append("dp.planSeed(%s);" % ik.ConstraintBase.toColumnVectorString(startPose))
        self.ikServer.taskQueue.addTask(functools.partial(self.ikServer.comm.sendCommandsAsync, commands))
        self.ikServer.taskQueue.start()

    def getPlanningStartPose(self):
        return self.robotSystem.robotStateJointController.q

    def computeDrivingTrajectories(self, steeringAngleDegrees, maxTurningRadius = 10, numTrajPoints = 50):

        angle = steeringAngleDegrees
        
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

    # capture the ankle position, 0 is no throttle, 1 is just starting to move
    def captureAnklePosition(self, typeIdx):
        pose = self.captureRobotPoseFromStreaming()
        if pose is None:
            return

        print 'captured ankle position'
        anklePos = np.rad2deg(pose[self.akyIdx])
        self.anklePositions[typeIdx] = anklePos

    def onThrottleCommand(self, msg):
        if np.isnan(self.anklePositions).any():
            # print 'you must initialize the LOW/HIGH ankle positions before streaming throttle commands'
            # print 'use the Capture Ankle Angle Low/High Buttons'
            return

        # slider 0 is the coarse grained slider, slider 1 is for fine grained adjustment
        slider = self.decodeThrottleMessage(msg)
        slope = self.anklePositions[1] - (self.anklePositions[0] - self.throttleIdleAngleSlack)
        const = self.anklePositions[0] - self.throttleIdleAngleSlack
        ankleGoalPosition = const + slider[0]*slope + (slider[1]-1/2.0)*self.fineGrainedThrottleTravel
        ankleGoalPositionRadians = np.deg2rad(ankleGoalPosition)

        # trip the safety if slider[3] is < 1/2, emergency come off the throttle
        if slider[3] < 0.5:
            print 'Emergency stop, coming off the throttle'
            print "setting l_leg_aky to it's min value"
            ankleGoalPositionRadians = self.jointLimitsMin[self.akyIdx]


        msg = lcmdrc.joint_position_goal_t()
        msg.utime = getUtime()
        msg.joint_position = ankleGoalPositionRadians
        msg.joint_name = 'l_leg_aky'
        lcmUtils.publish(self.throttlePublishChannel, msg)

    def onSteeringCommand(self, msg):
        steeringAngle = -msg.steeringAngle
        lwyPositionGoal = steeringAngle + self.steeringAngleOffset
        msg = lcmdrc.joint_position_goal_t()
        msg.utime = getUtime()
        msg.joint_position = lwyPositionGoal
        msg.joint_name = 'l_arm_lwy'
        lcmUtils.publish(self.steeringPublishChannel, msg)

    def setSteeringOffset(self):
        pose = self.captureRobotPoseFromStreaming()
        if pose is None:
            return

        print 'captured steering offset'
        self.steeringAngleOffset = pose[self.lwyIdx]

    def decodeThrottleMessage(self,msg):
        slider = np.zeros(4)
        slider[0] = msg.slider1
        slider[1] = msg.slider2
        slider[2] = msg.slider3
        slider[3] = msg.slider4

        return slider

    def captureRobotPoseFromStreaming(self):
        helper = lcmUtils.MessageResponseHelper(self.commandStreamChannel, lcmdrc.robot_state_t)
        msg = helper.waitForResponse(timeout=1000, keepAlive=False)

        if msg is None:
            print "Didn't receive a JOINT_POSITION_GOAL message"
            print "Are you streaming?"
            return None

        pose = robotstate.convertStateMessageToDrakePose(msg)
        return pose


class DrivingPlannerPanel(TaskUserPanel):

    def __init__(self, robotSystem):

        TaskUserPanel.__init__(self, windowTitle='Driving Task')

        self.robotSystem = robotSystem
        self.drivingPlanner = DrivingPlanner(robotSystem.ikServer, robotSystem)
        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()
        self.showTrajectory = False
        self.steeringSub = lcmUtils.addSubscriber('STEERING_COMMAND', lcmdrc.driving_control_cmd_t, self.onSteeringCommand)
        self.apriltagSub = lcmUtils.addSubscriber('APRIL_TAG_TO_CAMERA_LEFT', lcmbotcore.rigid_transform_t, self.onAprilTag)
        self.imageView = cameraview.CameraImageView(cameraview.imageManager, 'CAMERA_LEFT', 'image view')
        self.affordanceUpdater = affordanceupdater.AffordanceInCameraUpdater(segmentation.affordanceManager, self.imageView)
        self.affordanceUpdater.timer.start()
        self.imageViewLayout.addWidget(self.imageView.view)

    def onAprilTag(self, msg):
        cameraview.imageManager.queue.getTransform('april_tag_car_beam', 'local', msg.utime, self.drivingPlanner.tagToLocalTransform)
        self.updateAndDrawTrajectory()

    def addButtons(self):
        self.addManualButton('Start', self.onStart)
        self.addManualButton('Update Wheel Location', self.onUpdateWheelLocation)
        self.addManualButton('Plan Safe', self.onPlanSafe)
        self.addManualButton('Plan Pre Grasp', self.onPlanPreGrasp)
        self.addManualButton('Plan Touch', self.onPlanTouch)
        self.addManualButton('Plan Retract', self.onPlanRetract)
        self.addManualButton('Plan Turn', self.onPlanTurn)
        self.addManualButton('Plan Steering Wheel Turn', self.onPlanSteeringWheelTurn)
        self.addManualButton('Plan Seed', self.drivingPlanner.planSeed)
        self.addManualButton('Capture Ankle Angle Low', functools.partial(self.drivingPlanner.captureAnklePosition, 0))
        self.addManualButton('Capture Ankle Angle High', functools.partial(self.drivingPlanner.captureAnklePosition, 1))
        self.addManualButton('Capture Steering Offset', self.drivingPlanner.setSteeringOffset)

    def addDefaultProperties(self):
        self.params.addProperty('PreGrasp/Retract Depth', 0.2, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Touch Depth', 0.0, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('PreGrasp Angle', 0, attributes=om.PropertyAttributes(singleStep=10))
        self.params.addProperty('Turn Angle', 0, attributes=om.PropertyAttributes(singleStep=10))
        self.params.addProperty('Steering Wheel Radius (meters)', 0.1873, attributes=om.PropertyAttributes(singleStep=0.01))
        self.params.addProperty('Knot Points', 20, attributes=om.PropertyAttributes(singleStep=1))
        self.params.addProperty('Gaze Constraint Tol', 0.3, attributes=om.PropertyAttributes(singleStep=0.1, decimals=2))
        self.params.addProperty('Position Constraint Tol', 0.0, attributes=om.PropertyAttributes(singleStep=0.01, decimals=2))
        self.params.addProperty('Quat Constraint Tol', 0.0, attributes=om.PropertyAttributes(singleStep=0.01, decimals=2))
        self.params.addProperty('Grasp Location', 0, attributes=om.PropertyAttributes(enumNames=['Center','Rim']))
        self.params.addProperty('Seed with current posture', 0, attributes=om.PropertyAttributes(enumNames=['False','True']))
        self.params.addProperty('Speed', 1.0, attributes=om.PropertyAttributes(singleStep=0.1, decimals=2))
        self.params.addProperty('Throttle Idle Angle Slack', 10, attributes=om.PropertyAttributes(singleStep=1))
        self.params.addProperty('Fine Grained Throttle Travel', 10, attributes=om.PropertyAttributes(singleStep=1))
        self.params.addProperty('Turning Radius', 9.5, attributes=om.PropertyAttributes(singleStep=0.01, decimals=2))
        self.params.addProperty('Wheel Separation', 1.4, attributes=om.PropertyAttributes(singleStep=0.01, decimals=2))
        self.params.addProperty('Trajectory Segments', 25, attributes=om.PropertyAttributes(singleStep=1, decimals=0))
        self.params.addProperty('Trajectory X Offset', 0.0, attributes=om.PropertyAttributes(singleStep=0.01, decimals=2)),
        self.params.addProperty('Trajectory Y Offset', 0.0, attributes=om.PropertyAttributes(singleStep=0.01, decimals=2))
        self.params.addProperty('Trajectory Angle Offset', 0.0, attributes=om.PropertyAttributes(singleStep=1, decimals=0)),
        self.params.addProperty('Show Trajectory', False)
        self._syncProperties()

    def _syncProperties(self):
        self.preGraspDepth = self.params.getProperty('PreGrasp/Retract Depth')
        self.touchDepth = self.params.getProperty('Touch Depth')
        self.preGraspAngle = self.params.getProperty('PreGrasp Angle')
        self.turnAngle = self.params.getProperty('Turn Angle')
        self.speed = self.params.getProperty('Speed')
        self.turnRadius = self.params.getProperty('Steering Wheel Radius (meters)')
        self.knotPoints = self.params.getProperty('Knot Points')
        self.gazeTol = self.params.getProperty('Gaze Constraint Tol')
        self.drivingPlanner.positionTol = self.params.getProperty('Position Constraint Tol')
        self.drivingPlanner.quatTol = self.params.getProperty('Quat Constraint Tol')
        self.graspLocation = self.params.getPropertyEnumValue('Grasp Location').lower()
        self.drivingPlanner.seedWithCurrent = self.params.getProperty('Seed with current posture')
        self.drivingPlanner.throttleIdleAngleSlack = self.params.getProperty('Throttle Idle Angle Slack')
        self.drivingPlanner.fineGrainedThrottleTravel = self.params.getProperty('Fine Grained Throttle Travel')
        self.drivingPlanner.maxTurningRadius = self.params.getProperty('Turning Radius')
        self.drivingPlanner.trajSegments = self.params.getProperty('Trajectory Segments')
        self.drivingPlanner.wheelDistance = self.params.getProperty('Wheel Separation')
        self.showTrajectory = self.params.getProperty('Show Trajectory')
        self.drivingPlanner.trajectoryX = self.params.getProperty('Trajectory X Offset')
        self.drivingPlanner.trajectoryY = self.params.getProperty('Trajectory Y Offset')
        self.drivingPlanner.trajectoryAngle = self.params.getProperty('Trajectory Angle Offset')
        
        if hasattr(self, 'affordanceUpdater'):
            if self.showTrajectory:
                self.updateAndDrawTrajectory()
                leftTraj = om.findObjectByName('LeftDrivingTrajectory')
                rightTraj = om.findObjectByName('RightDrivingTrajectory')        
                leftTraj.setProperty('Visible', self.showTrajectory)
                rightTraj.setProperty('Visible', self.showTrajectory)
                self.affordanceUpdater.extraObjects = [leftTraj, rightTraj]
            else:
                self.affordanceUpdater.extraObjects = []

        self.drivingPlanner.applyProperties()
      
    def onSteeringCommand(self, msg):
        if msg.type == msg.TYPE_DRIVE_DELTA_STEERING:
            self.drivingPlanner.steeringAngleDegrees = math.degrees(msg.steering_angle)
            self.updateAndDrawTrajectory()

    def onStart(self):
        self.onUpdateWheelLocation()
        print('Driving Planner Ready')

    def onUpdateWheelLocation(self):
        f = om.findObjectByName('ring frame').transform
        xyzquat = transformUtils.poseFromTransform(f)
        xyzquat = np.concatenate(xyzquat)
        self.drivingPlanner.updateWheelTransform(xyzquat)

    def onPlanSafe(self):
        self.drivingPlanner.planSafe()

    def onPlanPreGrasp(self):
        self._syncProperties()
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
        self._syncProperties()


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

    def updateAndDrawTrajectory(self):
        leftTraj, rightTraj = self.drivingPlanner.computeDrivingTrajectories(self.drivingPlanner.steeringAngleDegrees, self.drivingPlanner.maxTurningRadius, self.drivingPlanner.trajSegments + 1)        
        self.drawDrivingTrajectory(self.drivingPlanner.transformDrivingTrajectory(leftTraj), 'LeftDrivingTrajectory')
        self.drawDrivingTrajectory(self.drivingPlanner.transformDrivingTrajectory(rightTraj), 'RightDrivingTrajectory')

    def drawDrivingTrajectory(self, drivingTraj, name):
        if not self.showTrajectory:
            return

        d = DebugData()

        numTrajPoints = len(drivingTraj)

        if numTrajPoints > 1:
            for i in range(0, numTrajPoints):
                rgb = [(numTrajPoints - i) / float(numTrajPoints), 1 - (numTrajPoints - i) / float(numTrajPoints), 1]            
                d.addSphere(drivingTraj[i], 0.05, rgb)

        vis.updatePolyData(d.getPolyData(), name)
        obj = om.findObjectByName(name)
        obj.setProperty('Color By', 1)
