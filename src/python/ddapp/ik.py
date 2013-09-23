import math
import types
import functools
from ddapp.timercallback import TimerCallback
from ddapp import matlab
from ddapp.jointcontrol import JointController


class AsyncIKCommunicator(TimerCallback):

    def __init__(self, jointController):
        TimerCallback.__init__(self)
        self.targetFps = 60
        self.comm = None
        self.outputConsole = None
        self.controller = jointController
        self.positionOffset = {}
        self.activePositionConstraint = 'l_foot'
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
           'l_hand_gaze_constraint',
           'r_hand_gaze_constraint',
           'pelvis_gaze_constraint',
           'utorso_gaze_constraint',
        ]
        self.activeConstraintNames = [
           'l_foot_position_constraint',
           'r_foot_position_constraint',
           'utorso_gaze_constraint',
          ]

        self.tasks = []

    def _startupCommands(self):

        commands = []
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

    def interact(self):
        self.comm.interact()

    def getPositionOffset(self, linkName):
        return self.positionOffset.get(linkName, [0.0, 0.0, 0.0])

    def updateIk(self):

        commands = []

        positionOffset = self.getPositionOffset(self.activePositionConstraint)
        formatArgs = dict(name=self.activePositionConstraint, x=positionOffset[0], y=positionOffset[1], z=positionOffset[2])
        commands.append('{name}_target = vertcat({name}_target_start(1,:)+{x}, {name}_target_start(2,:)+{y}, {name}_target_start(3,:)+{z});'.format(**formatArgs))
        commands.append('{name}_position_constraint = WorldPositionConstraint(r, {name}, {name}_pts, {name}_target, {name}_target, tspan);'.format(**formatArgs))
        commands.append('q_seed = %s;' % self.seedName)
        commands.append('active_constraints = {%s};' % ', '.join(self.activeConstraintNames))
        commands.append('[q_end, info] = inverseKin(r, q_seed, %s, %s, active_constraints{:}, s.ikoptions);' % (self.nominalName, self.quasiStaticConstraintName))

        #self.tasks.append(self.comm.sendCommandsAsync(commands))
        #self.tasks.append(functools.partial(self.fetchPoseFromServer, 'q_end'))
        #self.waitForAsyncTasks()

        #print '\n'.join(commands)

        self.comm.sendCommands(commands)
        self.fetchPoseFromServer('q_end')
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
