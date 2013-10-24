import math
import types
import functools
import numpy as np
from ddapp.timercallback import TimerCallback
from ddapp import matlab
from ddapp import botpy
from ddapp.jointcontrol import JointController

import vtk


class AsyncIKCommunicator(TimerCallback):

    def __init__(self, jointController):
        TimerCallback.__init__(self)
        self.targetFps = 60
        self.comm = None
        self.outputConsole = None
        self.controller = jointController
        self.positionOffset = {}
        self.positionTargets = {}
        self.orientationOffset = {}
        self.orientationTargets = {}
        self.activePositionConstraint = 'l_hand'
        self.quasiStaticConstraintName = 'both_feet_qsc'
        self.seedName = 'q_end'
        self.nominalName = 'q_nom'
        self.infoFunc = None
        self.poses = None

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

    def setPositionOffset(self, linkName, offset):
        self.positionOffset[linkName] = np.array(offset)

    def getOrientationTarget(self, linkName):
        if linkName not in self.orientationTargets:
            self.grabCurrentLinkPose(linkName)
        return self.orientationTargets[linkName]

    def setOrientationTarget(self, linkName, orientation):
        self.orientationTargets[linkName] = np.array(orientation)

    def getOrientationOffset(self, linkName):
        return self.orientationOffset.get(linkName, np.array([0.0, 0.0, 0.0]))

    def setOrientationOffset(self, linkName, offset):
        self.orientationOffset[linkName] = np.array(offset)

    def getLinkFrame(self, linkName):

        quat = self.getOrientationTarget(linkName)
        pos = self.getPositionTarget(linkName)

        rotationMatrix = np.zeros((3,3))
        vtk.vtkMath.QuaternionToMatrix3x3(quat, rotationMatrix)

        mat = np.eye(4)
        mat[:3,:3] = rotationMatrix
        mat[:3,3] = pos

        t = vtk.vtkTransform()
        t.SetMatrix(mat.flatten())
        return t

    def setLinkConstraintsWithFrame(self, linkName, frame):
        angleAxis = range(4)
        frame.GetOrientationWXYZ(angleAxis)
        angleAxis[0] = math.radians(angleAxis[0])
        pos = frame.GetPosition()
        quat = botpy.angle_axis_to_quat(angleAxis[0], angleAxis[1:])
        self.setPositionTarget(linkName, pos)
        self.setOrientationTarget(linkName, quat)


    def grabCurrentLinkPose(self, linkName):
        commands = []
        commands.append('kinsol = doKinematics(r, q_end);')
        commands.append('%s_pose = r.forwardKin(kinsol, %s, [0;0;0], 2);' % (linkName, linkName))
        commands.append('%s_position_target_start = %s_pose(1:3);' % (linkName, linkName))
        commands.append('%s_orient_target_start = %s_pose(4:7);' % (linkName, linkName))
        self.comm.sendCommands(commands)

        linkPosition = np.array(self.comm.getFloatArray('%s_position_target_start' % linkName))
        linkOrientation = np.array(self.comm.getFloatArray('%s_orient_target_start' % linkName))

        self.setPositionTarget(linkName, linkPosition)
        self.setOrientationTarget(linkName, linkOrientation)
        self.setPositionOffset(linkName, [0.0, 0.0, 0.0])
        self.setOrientationOffset(linkName, [0.0, 0.0, 0.0])


    def updatePositionConstraint(self, linkName, execute=True):

        commands = []
        positionTarget = self.getPositionTarget(linkName)
        positionOffset = self.getPositionOffset(linkName)
        positionConstraint = positionTarget + positionOffset
        formatArgs = dict(name=linkName, x=positionConstraint[0], y=positionConstraint[1], z=positionConstraint[2])
        commands.append('{name}_position_target = [{x}; {y}; {z}];'.format(**formatArgs))
        commands.append('{name}_position_constraint = WorldPositionConstraint(r, {name}, [0;0;0], {name}_position_target, {name}_position_target, tspan);'.format(**formatArgs))
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


    def updateIk(self):

        commands = []

        #commands.extend(self.updatePositionConstraint(self.activePositionConstraint, execute=False))

        for constraintName in self.activeConstraintNames:
            if constraintName.endswith('position_constraint'):
                commands.extend(self.updatePositionConstraint(constraintName.replace('_position_constraint', ''), execute=False))
            elif constraintName.endswith('orient_constraint'):
                commands.extend(self.updateOrientationConstraint(constraintName.replace('_orient_constraint', ''), execute=False))


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
        commands.append('qtraj = PPTrajectory(foh(tspan, [%s, %s]));' % (poseStart, poseEnd))
        commands.append('nt = %d;' % nt)
        commands.append('t = linspace(0, 1, nt);')

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

        commands.append('[xtraj, info] = inverseKinTraj(r, %s, zeros(nq,1), t, squeeze(eval(qtraj, t(2:end))), repmat(q_nom, 1, nt-1), active_constraints{:}, s.ikoptions);' % poseStart)
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
