import math
import types
import functools
import numpy as np
from ddapp.timercallback import TimerCallback
from ddapp import matlab
from ddapp.asynctaskqueue import AsyncTaskQueue
from ddapp.jointcontrol import JointController
from ddapp.ikconstraints import *


class AsyncIKCommunicator():

    def __init__(self):

        self.comm = None
        self.outputConsole = None

        self.seedName = 'q_nom'
        self.nominalName = 'q_nom'
        self.infoFunc = None

        self.maxDegreesPerSecond = 30.0
        self.usePointwise = True


    def getStartupCommands(self):

        commands = []
        commands.append('\n%-------- startup --------\n')
        commands.append('format long e')
        commands.append('addpath_control')
        commands.append("addpath('%s')" % matlab.getAppMatlabDir())
        commands.append('runIKServer')
        commands.append('\n%------ startup end ------\n')
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

        taskQueue = AsyncTaskQueue()
        taskQueue.addTask(self.comm.waitForResultAsync)
        taskQueue.addTask(self.comm.printResult)
        taskQueue.addTask(functools.partial(self.comm.sendCommandsAsync, self.getStartupCommands()))

        self.taskQueue = taskQueue
        self.taskQueue.start()



    def fetchPoseFromServer(self, poseName):
        return self.comm.getFloatArray(poseName)


    def sendPoseToServer(self, pose, poseName):
        self.comm.assignFloatArray(pose, poseName)


    def interact(self):
        self.comm.interact()


    def getConstraintCommands(self, constraintNames):

        commands = []

        for constraintName in constraintNames:
            if constraintName.endswith('position_constraint'):
                commands.extend(self.updatePositionConstraint(constraintName.replace('_position_constraint', ''), execute=False))
            elif constraintName.endswith('orient_constraint'):
                commands.extend(self.updateOrientationConstraint(constraintName.replace('_orient_constraint', ''), execute=False))
            elif constraintName.endswith('gaze_constraint'):
                commands.extend(self.updateGazeConstraint(constraintName.replace('_gaze_constraint', ''), execute=False))

        commands.append('active_constraints = {%s};' % ', '.join(constraintNames))
        return commands


    def plotJoints(self, jointNames, filename):
        commands = []
        commands.append('\n%-------- plot joints --------\n')
        commands.append('plot_joint_ids = [%s];' % ', '.join(['joints.%s' % name for name in jointNames]))
        commands.append("s.saveJointTrajPlot(%s, plot_joint_ids, '%s');" % ('xtraj_pw' if self.usePointwise else 'xtraj', filename))
        commands.append('\n%-------- plot joints end --------\n')
        self.comm.sendCommands(commands)


    def runIk(self, constraints, nominalPostureName=None, seedPostureName=None):

        commands = []
        commands.append('\n%-------- runIk --------\n')
        constraintNames = []
        for constraintId, constraint in enumerate(constraints):
            constraint.getCommands(commands, constraintNames, suffix='_%d' % constraintId)
            commands.append('\n')

        nominalPostureName = nominalPostureName or self.nominalName
        seedPostureName = seedPostureName or self.seedName

        commands.append('active_constraints = {%s};' % ', '.join(constraintNames))
        commands.append('q_seed = %s;' % seedPostureName)
        commands.append('clear q_end;')
        commands.append('clear info;')
        commands.append('clear infeasible_constraint;')
        commands.append('\n')
        commands.append('[q_end, info, infeasible_constraint] = inverseKin(r, q_seed, %s, active_constraints{:}, s.ikoptions);' % nominalPostureName)
        commands.append('\n')
        commands.append('if (info > 10) display(infeasibleConstraintMsg(infeasible_constraint)); end;')
        commands.append('\n%-------- runIk end --------\n')

        self.comm.sendCommands(commands)
        endPose = self.comm.getFloatArray('q_end')
        info = self.comm.getFloatArray('info')[0]

        return endPose, info


    def sampleTraj(self, t):

        commands = []
        commands.append('tdelta = qtraj.tspan(end) - qtraj.tspan(1);')
        commands.append('q_trajPose = eval(qtraj, qtraj.tspan(1) + %f*tdelta);' % t)

        self.comm.sendCommands(commands)
        self.fetchPoseFromServer('q_trajPose')


    def runIkTraj(self, constraints, poseStart='q_start', poseEnd='q_end', timeSamples=None, additionalTimeSamples=0):

        if timeSamples is None:
            timeSamples = np.hstack([constraint.tspan for constraint in constraints])
            timeSamples = [x for x in timeSamples if x not in [-np.inf, np.inf]]
            timeSamples.append(0.0)
            timeSamples = np.unique(timeSamples)

        commands = []
        commands.append('\n%-------- runIkTraj --------\n')

        constraintNames = []
        for constraintId, constraint in enumerate(constraints):
            constraint.getCommands(commands, constraintNames, suffix='_%d' % constraintId)
            commands.append('\n')

        commands.append('active_constraints = {%s};' % ', '.join(constraintNames))
        commands.append('t = [%s];' % ', '.join([repr(x) for x in timeSamples]))
        commands.append('nt = size(t, 2);')
        commands.append('q_nom_traj = PPTrajectory(foh(t, repmat(q_nom, 1, nt)));')
        #commands.append('q_seed_traj = PPTrajectory(foh([t(1), t(end)], [%s, %s]));' % (poseStart, poseEnd))
        commands.append('q_seed_traj = PPTrajectory(spline([t(1), t(end)], [zeros(nq,1), %s, %s, zeros(nq,1)]));' % (poseStart, poseEnd))
        commands.append('clear xtraj;')
        commands.append('clear info;')
        commands.append('clear infeasible_constraint;')
        if additionalTimeSamples:
            commands.append('additionalTimeSamples = linspace(t(1), t(end), %d);' % additionalTimeSamples)
        else:
            commands.append('additionalTimeSamples = [];')

        commands.append('ikoptions = s.ikoptions.setAdditionaltSamples(additionalTimeSamples);')
        #commands.append('ikoptions = ikoptions.setSequentialSeedFlag(true);')

        commands.append('\n')
        commands.append('[xtraj, info, infeasible_constraint] = inverseKinTraj(r, t, q_seed_traj, q_nom_traj, active_constraints{:}, ikoptions);')
        commands.append('\n')
        commands.append('if (info > 10) display(infeasibleConstraintMsg(infeasible_constraint)); end;')
        commands.append('qtraj = xtraj(1:nq);')
        commands.append('max_degrees_per_second = %f;' % self.maxDegreesPerSecond)
        commands.append('plan_time = s.getPlanTimeForJointVelocity(xtraj, max_degrees_per_second);')


        if self.usePointwise:
            commands.append('\n%--- pointwise ik --------\n')
            commands.append('num_pointwise_time_points = 20;')
            commands.append('pointwise_time_points = linspace(t(1), t(end), num_pointwise_time_points);')
            #commands.append('spline_traj = PPTrajectory(spline(t, [ zeros(size(xtraj, 1),1), xtraj.eval(t), zeros(size(xtraj, 1),1)]));')
            #commands.append('q_seed_pointwise = spline_traj.eval(pointwise_time_points);')
            commands.append('q_seed_pointwise = xtraj.eval(pointwise_time_points);')
            commands.append('q_seed_pointwise = q_seed_pointwise(1:nq,:);')
            commands.append('[xtraj_pw, info_pw] = inverseKinPointwise(r, pointwise_time_points, q_seed_pointwise, q_seed_pointwise, active_constraints{:}, ikoptions);')
            commands.append('xtraj_pw = PPTrajectory(foh(pointwise_time_points, xtraj_pw));')
            commands.append('info = info_pw(end);')
            commands.append('if (any(info_pw > 10)) disp(\'pointwise info:\'); disp(info_pw); end;')
            commands.append('\n%--- pointwise ik end --------\n')


        publish = True
        if publish:
            commands.append('s.publishTraj(plan_publisher, r, %s, info, plan_time);' % ('xtraj_pw' if self.usePointwise else 'xtraj'))

        commands.append('\n%--- runIKTraj end --------\n')
        self.comm.sendCommands(commands)

        info = self.comm.getFloatArray('info')[0]
        if self.infoFunc:
            self.infoFunc(info)

        return info


    def tick(self):

        if self.handleAsyncTasks() > 0:
            return
