import ddapp
import math
import drc as lcmdrc
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp import objectmodel as om
from ddapp import lcmUtils
from ddapp import ik
from ddapp.debugVis import DebugData

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
        self.steeringAngleDegrees = 0.0;
        self.maxTurningRadius = 10.0
        self.trajSegments = 50
        self.wheelDistance = 1.0

    def getInitCommands(self):

      commands = ['''
        addpath([getenv('DRC_BASE'), '/software/control/matlab/planners/driving_planner']);
        dpRobot = s.robot;
        clear options;
        options = struct('listen_to_lcm_flag',{0});
      ''']
      commands.append('options.qstar = q_nom;')
      commands.append("dp = drivingPlanner(r,options);")

      return commands

    def initialize(self, ikServer, success):
        commands = self.getInitCommands()
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
        commands.append("dp.planSafe(options,%s)" % ik.ConstraintBase.toColumnVectorString(startPose))

        self.ikServer.taskQueue.addTask(functools.partial(self.ikServer.comm.sendCommandsAsync, commands))
        self.ikServer.taskQueue.start()


    def planPreGrasp(self, depth=0.2, xyz_des=None, angle=0, speed=1):
        commands = []
        commands.append("clear options;")
        commands.append("options = struct('depth',{%r});" % depth)
        commands.append("options.angle = %r;" % np.radians(angle))
        commands.append("options.speed = %r;" % speed)

        if xyz_des is not None:
            commands.append("options.xyz_des = {%s};",ik.ConstraintBase.toColumnVectorString(xyz_des))
        startPose = self.getPlanningStartPose()
        commands.append("dp.planPreGrasp(options, %s)" % ik.ConstraintBase.toColumnVectorString(startPose))

        self.ikServer.taskQueue.addTask(functools.partial(self.ikServer.comm.sendCommandsAsync, commands))
        self.ikServer.taskQueue.start()

    def planTouch(self, depth=0, xyz_des=None, speed=1):
        commands = []
        commands.append("clear options;")
        commands.append("options = struct('depth',{%r});" % depth)
        commands.append("options.speed = %r;" % speed)
        startPose = self.getPlanningStartPose()
        commands.append("dp.planTouch(options, %s)" % ik.ConstraintBase.toColumnVectorString(startPose))

        self.ikServer.taskQueue.addTask(functools.partial(self.ikServer.comm.sendCommandsAsync, commands))
        self.ikServer.taskQueue.start()

    def planRetract(self, depth=0.2, speed=1):
        commands = []
        commands.append("clear options;")
        commands.append("options = struct('depth',{%r});" % depth)
        commands.append("options.speed = %r;" % speed)
        startPose = self.getPlanningStartPose()
        commands.append("dp.planRetract(options, %s)" % ik.ConstraintBase.toColumnVectorString(startPose))

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

    def getPlanningStartPose(self):
        return self.robotSystem.robotStateJointController.q

    def computeDrivingTrajectory(self, steeringAngleDegrees, maxTurningRadius = 10, numTrajPoints = 50):
        
        angle = steeringAngleDegrees
        
        if abs(angle) < 0.1:
            angle = 1e-8

        turningRadius = 1.0 / (angle * (1 / (maxTurningRadius * 170.0)))
        turningCenter = [0, turningRadius, 0]
        trajPoints = list()

        for i in range(0, numTrajPoints):
            theta = math.radians((10 / turningRadius) * i - 90)
            trajPoint = np.asarray(turningCenter)+turningRadius*np.asarray([math.cos(theta), math.sin(theta), 0])
            trajPoints.append(trajPoint)

        return trajPoints

    def updateAndDrawTrajectory(self):
        trajPoints = self.computeDrivingTrajectory(self.steeringAngleDegrees, self.maxTurningRadius, self.trajSegments + 1)
        self.drawDrivingTrajectory(trajPoints)            

    def drawDrivingTrajectory(self, drivingTraj):
        d = DebugData()

        numTrajPoints = len(drivingTraj)

        if numTrajPoints > 1:
            for i in range(0, numTrajPoints - 1):
                rgb = [(numTrajPoints - i) / float(numTrajPoints), 1 - (numTrajPoints - i) / float(numTrajPoints), 1]            
                v1 = drivingTraj[i + 1] - drivingTraj[i]
                v2 = np.cross(v1, [0, 0, 1])
                v2 /= np.linalg.norm(v2)
                d.addLine(drivingTraj[i] + 0.5 * self.wheelDistance * v2, drivingTraj[i + 1] + 0.5 * self.wheelDistance * v2,  0.01, rgb)
                d.addLine(drivingTraj[i] - 0.5 * self.wheelDistance * v2, drivingTraj[i + 1] - 0.5 * self.wheelDistance * v2,  0.01, rgb)
                if i == numTrajPoints - 2:
                    d.addArrow(drivingTraj[i] + 0.5 * self.wheelDistance * v2, drivingTraj[i+1] + 0.5 * self.wheelDistance * v2, 0.02, 0.01, rgb)
                    d.addArrow(drivingTraj[i] - 0.5 * self.wheelDistance * v2, drivingTraj[i+1] - 0.5 * self.wheelDistance * v2, 0.02, 0.01, rgb)

        vis.updatePolyData(d.getPolyData(), 'DrivingTrajectory')
        om.findObjectByName('DrivingTrajectory').setProperty('Color By', 1)

class DrivingPlannerPanel(TaskUserPanel):

    def __init__(self, robotSystem):

        TaskUserPanel.__init__(self, windowTitle='Driving Task')

        self.robotSystem = robotSystem
        self.drivingPlanner = DrivingPlanner(robotSystem.ikServer, robotSystem)
        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()
        self.showTrajectory = False
        self.sub = lcmUtils.addSubscriber('STEERING_COMMAND', lcmdrc.driving_control_cmd_t, self.onSteeringCommand)

    def addButtons(self):
        self.addManualButton('Start', self.onStart)
        self.addManualButton('Update Wheel Location', self.onUpdateWheelLocation)
        self.addManualButton('Plan Safe', self.onPlanSafe)
        self.addManualButton('Plan Pre Grasp', self.onPlanPreGrasp)
        self.addManualButton('Plan Touch', self.onPlanTouch)
        self.addManualButton('Plan Retract', self.onPlanRetract)
        self.addManualButton('Plan Turn', self.onPlanTurn)

    def addDefaultProperties(self):
        self.params.addProperty('PreGrasp/Retract Depth', 0.2, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Touch Depth', 0.0, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('PreGrasp Angle', 0, attributes=om.PropertyAttributes(singleStep=10))
        self.params.addProperty('Turn Angle', 0, attributes=om.PropertyAttributes(singleStep=10))
        self.params.addProperty('Speed', 1.0, attributes=om.PropertyAttributes(singleStep=0.1, decimals=2))
        self.params.addProperty('Turning Radius', 10.0, attributes=om.PropertyAttributes(singleStep=0.01, decimals=2))
        self.params.addProperty('Wheel Separation', 1.0, attributes=om.PropertyAttributes(singleStep=0.01, decimals=2))
        self.params.addProperty('Trajectory Segments', 50, attributes=om.PropertyAttributes(singleStep=1, decimals=0))
        self.params.addProperty('Show Trajectory', False)
        self._syncProperties()

    def _syncProperties(self):
        self.preGraspDepth = self.params.getProperty('PreGrasp/Retract Depth')
        self.touchDepth = self.params.getProperty('Touch Depth')
        self.preGraspAngle = self.params.getProperty('PreGrasp Angle')
        self.turnAngle = self.params.getProperty('Turn Angle')
        self.speed = self.params.getProperty('Speed')
        self.drivingPlanner.maxTurningRadius = self.params.getProperty('Turning Radius')
        self.drivingPlanner.trajSegments = self.params.getProperty('Trajectory Segments')
        self.drivingPlanner.wheelDistance = self.params.getProperty('Wheel Separation')
        self.showTrajectory = self.params.getProperty('Show Trajectory')
        self.drivingPlanner.updateAndDrawTrajectory()
        traj = om.findObjectByName('DrivingTrajectory')
        if traj is not None:
            traj.setProperty('Visible', self.showTrajectory)
        

    def onSteeringCommand(self, msg):
        if msg.type == msg.TYPE_DRIVE_DELTA_STEERING:
            self.drivingPlanner.steeringAngleDegrees = math.degrees(msg.steering_angle)
            self.drivingPlanner.updateAndDrawTrajectory()

    def onStart(self):
        self.onUpdateWheelLocation()

    def onUpdateWheelLocation(self):
        f = om.findObjectByName('ring frame').transform
        xyzquat = transformUtils.poseFromTransform(f);
        xyzquat = np.concatenate(xyzquat)
        self.drivingPlanner.updateWheelTransform(xyzquat);

    def onPlanSafe(self):
        self.drivingPlanner.planSafe()

    def onPlanPreGrasp(self):
        self._syncProperties()
        self.drivingPlanner.planPreGrasp(depth=self.preGraspDepth, speed=self.speed, angle=self.preGraspAngle)

    def onPlanTouch(self):
        self._syncProperties()
        self.drivingPlanner.planTouch(depth=self.touchDepth, speed=self.speed)

    def onPlanRetract(self):
        self._syncProperties()
        self.drivingPlanner.planRetract(depth=self.preGraspDepth, speed=self.speed)

    def onPlanTurn(self):
        self._syncProperties()
        self.drivingPlanner.planTurn(angle=self.turnAngle, speed=self.speed)

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
