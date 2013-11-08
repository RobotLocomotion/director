import math
import types
import functools
import numpy as np
from ddapp.timercallback import TimerCallback
from ddapp import matlab
from ddapp import botpy
from ddapp.jointcontrol import JointController
from ddapp.transformUtils import transformFromPose, poseFromTransform

import vtk


class AsyncIKCommunicator(TimerCallback):

    def __init__(self, jointController):
        TimerCallback.__init__(self)
        self.targetFps = 60
        self.comm = None
        self.outputConsole = None
        self.controller = jointController
        self.positionOffset = {}
        self.positionOffsetBounds = {}
        self.positionTargets = {}
        self.pointInLink = {}
        self.orientationOffset = {}
        self.orientationTargets = {}
        self.gazeAxes = {}
        self.activePositionConstraint = 'l_hand'
        self.quasiStaticConstraintName = 'both_feet_qsc'
        self.seedName = 'q_end'
        self.nominalName = 'q_nom'
        self.infoFunc = None
        self.poses = None

        #self.setPointInLink('l_hand', [0, 0.11516, 0.015]) #irobot
        #self.setPointInLink('r_hand', [0, -0.11516, -0.015]) #irobot

        #self.setPointInLink('l_hand', [0.00179, 0.13516, 0.01176]) #sandia
        #self.setPointInLink('r_hand', [-0.00179, -0.13516, -0.01176]) #sandia

        self.setGazeAxes('utorso', [0,0,1], [0,0,1])

        self.constraintNames = [
           'self_collision_constraint',

           'l_foot_position_constraint',
           'r_foot_position_constraint',
           'l_hand_position_constraint',
           'r_hand_position_constraint',
           'pelvis_position_constraint',
           'utorso_position_constraint',

           'l_foot_orient_constraint',
           'r_foot_orient_constraint',
           'l_hand_orient_constraint',
           'r_hand_orient_constraint',
           'pelvis_orient_constraint',
           'utorso_orient_constraint',

           'l_hand_gaze_constraint',
           'r_hand_gaze_constraint',
           'pelvis_gaze_constraint',
           'utorso_gaze_constraint',
        ]
        self.activeConstraintNames = [
           'l_foot_position_constraint',
           'r_foot_position_constraint',
           'l_foot_orient_constraint',
           'r_foot_orient_constraint',
           'utorso_gaze_constraint',
          ]

        self.tasks = []

    def _startupCommands(self):

        commands = []
        commands.append('format long e')
        commands.append('addpath_control')
        commands.append("addpath('%s')" % matlab.getAppMatlabDir())
        commands.append('runIKServer')
        return commands

    def startServer(self):

        proc = matlab.startMatlab()
        self.comm = matlab.MatlabCommunicator(proc)
        self.comm.outputConsole = self.outputConsole
        self.comm.waitForResult()
        self.comm.printResult()
        self.comm.sendCommands(self._startupCommands())

    def startServerAsync(self):

        proc = matlab.startMatlab()
        self.comm = matlab.MatlabCommunicator(proc)
        self.comm.outputConsole = self.outputConsole
        self.tasks.append(self.comm.waitForResultAsync())
        self.tasks.append(functools.partial(self.comm.printResult))
        self.tasks.append(self.comm.sendCommandsAsync(self._startupCommands()))
        self.tasks.append(functools.partial(self.fetchPoseFromServer, 'q_start'))
        self.tasks.append(functools.partial(self.fetchPoseFromServer, 'q_end'))

    def handleAsyncTasks(self):

        for i in xrange(10):

            if not self.tasks:
                break

            task = self.tasks[0]

            if isinstance(task, functools.partial):
                task()
                self.tasks.remove(task)
            elif isinstance(task, types.GeneratorType):
                try:
                    task.next()
                except StopIteration:
                    self.tasks.remove(task)

        return len(self.tasks)

    def waitForAsyncTasks(self):
        while self.handleAsyncTasks() > 0:
            pass

    def fetchPoseFromServer(self, poseName):
        pose = self.comm.getFloatArray(poseName)
        self.controller.addPose(poseName, pose)
        self.controller.setPose(poseName)

    def sendPoseToServer(self, pose, poseName):
        self.comm.assignFloatArray(pose, poseName)
        self.controller.addPose(poseName, pose)

    def forcePose(self, poseName):

        commands = []
        commands.append('q_end = %s;' % poseName)
        self.comm.sendCommands(commands)

        self.controller.setPose(poseName)

        for linkName in ['l_hand', 'r_hand', 'l_foot', 'r_foot', 'pelvis', 'utorso']:
            self.grabCurrentLinkPose(linkName)

    def interact(self):
        self.comm.interact()

    def getPositionTarget(self, linkName):
        if linkName not in self.positionTargets:
            self.grabCurrentLinkPose(linkName)
        return self.positionTargets[linkName]

    def setPositionTarget(self, linkName, position):
        self.positionTargets[linkName] = np.array(position)

    def getPositionOffset(self, linkName):
        return self.positionOffset.get(linkName, np.array([0.0, 0.0, 0.0]))

    def getPositionOffsetBounds(self, linkName):
        return self.positionOffsetBounds.get(linkName, [(0,0), (0,0), (0,0)])

    def setPositionOffset(self, linkName, offset):
        self.positionOffset[linkName] = np.array(offset)

    def setPositionOffsetBounds(self, linkName, bounds):
        self.positionOffsetBounds[linkName] = bounds

    def getOrientationTarget(self, linkName):
        if linkName not in self.orientationTargets:
            self.grabCurrentLinkPose(linkName)
        return self.orientationTargets[linkName]

    def setOrientationTarget(self, linkName, orientation):
        self.orientationTargets[linkName] = np.array(orientation)

    def getOrientationOffset(self, linkName):
        return self.orientationOffset.get(linkName, np.array([0.0, 0.0, 0.0]))

    def getGazeAxes(self, linkName):
        return self.gazeAxes.get(linkName, (np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])))

    def setGazeAxes(self, linkName, bodyAxis, worldAxis):
        self.gazeAxes[linkName] = (np.array(bodyAxis), np.array(worldAxis))

    def setOrientationOffset(self, linkName, offset):
        self.orientationOffset[linkName] = np.array(offset)

    def setPointInLink(self, linkName, point):
        self.pointInLink[linkName] = point

    def setLinkConstraintsWithFrame(self, linkName, frame):
        pos, quat = poseFromTransform(frame)

        self.setPositionTarget(linkName, pos)
        self.setOrientationTarget(linkName, quat)


    def grabCurrentLinkPose(self, linkName):

        #pointInLink = self.pointInLink.get(linkName, (0,0,0))
        pointInLink = (0,0,0)
        pointInLink = '[%s]' % (';'.join([repr(x) for x in pointInLink]))

        commands = []
        commands.append('kinsol = doKinematics(r, q_end);')
        commands.append("%s = r.findLinkInd('%s');" % (linkName, linkName))
        commands.append('%s_pose = r.forwardKin(kinsol, %s, %s, 2);' % (linkName, linkName, pointInLink))
        commands.append('%s_position_target_start = %s_pose(1:3);' % (linkName, linkName))
        commands.append('%s_orient_target_start = %s_pose(4:7);' % (linkName, linkName))
        self.comm.sendCommands(commands)

        linkPosition = np.array(self.comm.getFloatArray('%s_position_target_start' % linkName))
        linkQuaternion = np.array(self.comm.getFloatArray('%s_orient_target_start' % linkName))

        self.setPositionTarget(linkName, linkPosition)
        self.setOrientationTarget(linkName, linkQuaternion)
        self.setPositionOffset(linkName, [0.0, 0.0, 0.0])
        self.setPositionOffsetBounds(linkName, [(0,0), (0,0), (0,0)])
        self.setOrientationOffset(linkName, [0.0, 0.0, 0.0])

        return transformFromPose(linkPosition, linkQuaternion)

    def updatePositionConstraint(self, linkName, execute=True):

        commands = []
        positionTarget = self.getPositionTarget(linkName)
        positionOffset = self.getPositionOffset(linkName)
        offsetBounds = self.getPositionOffsetBounds(linkName)
        positionConstraint = positionTarget + positionOffset

        pointInLink = self.pointInLink.get(linkName, (0,0,0))
        pointInLink = '[%s]' % (';'.join([repr(x) for x in pointInLink]))

        t = vtk.vtkTransform()
        elements = [[t.GetMatrix().GetElement(r, c) for c in xrange(4)] for r in xrange(4)]
        elements = ';'.join([','.join([repr(x) for x in row]) for row in elements])

        formatArgs = dict(name=linkName, x=positionConstraint[0], y=positionConstraint[1], z=positionConstraint[2], ref_frame=elements, link_pt=pointInLink)
        commands.append('{name}_position_target = [{x}; {y}; {z}];'.format(**formatArgs))
        commands.append('%s_position_bounds = [%r; %r; %r];' % (linkName, offsetBounds[0][1], offsetBounds[1][1], offsetBounds[2][1]))
        commands.append('{name}_point_in_body_frame = {link_pt};'.format(**formatArgs))
        commands.append('{name}_ref_frame = [{ref_frame}];'.format(**formatArgs))

        #commands.append('{name}_position_constraint = WorldPositionInFrameConstraint(r, {name}, {name}_point_in_body_frame, {name}_ref_frame, {name}_position_target - {name}_position_bounds, {name}_position_target + {name}_position_bounds, tspan);'.format(**formatArgs))

        commands.append('{name}_position_constraint = WorldPositionConstraint(r, {name}, {name}_point_in_body_frame, {name}_position_target - {name}_position_bounds, {name}_position_target + {name}_position_bounds, tspan);'.format(**formatArgs))

        if execute:
            self.comm.sendCommands(commands)
        else:
            return commands

    def updateOrientationConstraint(self, linkName, execute=True):

        commands = []
        orientationTarget = self.getOrientationTarget(linkName)
        orientationOffset = self.getOrientationOffset(linkName)
        orientationConstraint = orientationTarget
        formatArgs = dict(name=linkName, w=orientationConstraint[0], x=orientationConstraint[1], y=orientationConstraint[2], z=orientationConstraint[3])
        commands.append('{name}_orient_target = [{w}; {x}; {y}; {z}];'.format(**formatArgs))
        commands.append('{name}_orient_constraint = WorldQuatConstraint(r, {name}, {name}_orient_target, 0.0, tspan);'.format(**formatArgs))
        if execute:
            self.comm.sendCommands(commands)
        else:
            return commands

    def updateGazeConstraint(self, linkName, execute=True):

        commands = []
        bodyAxis, worldAxis = self.getGazeAxes(linkName)
        theta = 0.02

        formatArgs = dict(name=linkName)
        commands.append('%s_gaze_theta = %s;' % (linkName, repr(theta)))
        commands.append('%s_gaze_body_axis = [%s];' % (linkName, ';'.join([repr(x) for x in bodyAxis])))
        commands.append('%s_gaze_world_axis = [%s];' % (linkName, ';'.join([repr(x) for x in worldAxis])))
        commands.append('{name}_gaze_constraint = WorldGazeDirConstraint(r, {name}, {name}_gaze_body_axis, {name}_gaze_world_axis, {name}_gaze_theta, tspan);'.format(**formatArgs))
        if execute:
            self.comm.sendCommands(commands)
        else:
            return commands


    def updateIk(self):

        commands = []

        #commands.extend(self.updatePositionConstraint(self.activePositionConstraint, execute=False))

        for constraintName in self.activeConstraintNames:
            if constraintName.endswith('position_constraint'):
                commands.extend(self.updatePositionConstraint(constraintName.replace('_position_constraint', ''), execute=False))
            elif constraintName.endswith('orient_constraint'):
                commands.extend(self.updateOrientationConstraint(constraintName.replace('_orient_constraint', ''), execute=False))
            elif constraintName.endswith('gaze_constraint'):
                commands.extend(self.updateGazeConstraint(constraintName.replace('_gaze_constraint', ''), execute=False))


        activeConstraintNames = list(self.activeConstraintNames)
        if self.quasiStaticConstraintName:
            activeConstraintNames.insert(0, self.quasiStaticConstraintName)

        commands.append('q_seed = %s;' % self.seedName)
        commands.append('active_constraints = {%s};' % ', '.join(activeConstraintNames))
        commands.append('[q_end, info] = inverseKin(r, q_seed, %s, active_constraints{:}, s.ikoptions);' % self.nominalName)

        #self.tasks.append(self.comm.sendCommandsAsync(commands))
        #self.tasks.append(functools.partial(self.fetchPoseFromServer, 'q_end'))
        #self.waitForAsyncTasks()

        self.comm.sendCommands(commands)
        self.fetchPoseFromServer('q_end')
        info = self.comm.getFloatArray('info')[0]

        if self.infoFunc:
            self.infoFunc(info)


    def sampleTraj(self, t):

        commands = []
        commands.append('q_trajPose = eval(qtraj, %f);' % t)

        self.comm.sendCommands(commands)
        self.fetchPoseFromServer('q_trajPose')

    def storeCurrentPose(self, poseName):
        commands = []
        commands.append('%s = q_end;' % poseName)

        self.comm.sendCommands(commands)
        self.fetchPoseFromServer(poseName)

    def makePoseTraj(self, poseStart='q_start', poseEnd='q_end'):

        commands = []
        commands.append('qtraj = PPTrajectory(foh(tspan, [%s, %s]));' % (poseStart, poseEnd))

        self.comm.sendCommands(commands)

    def runIkTraj(self, poseStart='q_start', poseEnd='q_end', nt=5):

        commands = []

        commands.append('nt = %d;' % nt)
        commands.append('t = linspace(tspan(1), tspan(end), nt);')
        commands.append('q_nom_traj = PPTrajectory(foh(t, repmat(q_nom, 1, nt)));')
        commands.append('q_seed_traj = PPTrajectory(foh(tspan, [%s, %s]));' % (poseStart, poseEnd))

        '''
        % args:
        % RigidBodyManipulator
        % starting joint position
        % starting joint velocity
        % time points
        % join position seeds
        % joint nominal positions
        % constraints
        % ikoptions
        '''

        commands.append('[xtraj, info] = inverseKinTraj(r, t, q_seed_traj, q_nom_traj, active_constraints{:}, s.ikoptions);')
        commands.append('qtraj = xtraj(1:nq);')
        self.comm.sendCommands(commands)

        info = self.comm.getFloatArray('info')[0]
        if self.infoFunc:
            self.infoFunc(info)

    def resetQSeed(self):
        commands = []
        commands.append('q_end = q_nom;')
        self.comm.sendCommands(commands)
        self.updateIk()

    def tick(self):

        if self.handleAsyncTasks() > 0:
            return
