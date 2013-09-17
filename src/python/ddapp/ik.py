import math
import types
import functools
from ddapp.timercallback import TimerCallback
from ddapp import midi
from ddapp import matlab
from ddapp.jointcontrol import JointController



class AsyncIKCommunicator(TimerCallback):

    def __init__(self, model):
        TimerCallback.__init__(self)
        self.targetFps = 60
        self.reader = midi.MidiReader()
        self.controller = JointController(model)
        self.channelsX = [midi.TriggerFinger.faders[0], midi.TriggerFinger.pads[0], midi.TriggerFinger.dials[0] ]
        self.channelsY = [midi.TriggerFinger.faders[1], midi.TriggerFinger.pads[1], midi.TriggerFinger.dials[1] ]
        self.channelsZ = [midi.TriggerFinger.faders[3], midi.TriggerFinger.pads[2], midi.TriggerFinger.dials[2] ]
        self.footOffsets = [0.0, 0.0, 0.0]
        self.seedWithNominal = False
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
        self.comm.waitForResult()
        self.comm.printResult()
        self.comm.sendCommands(self._startupCommands())

    def startServerAsync(self):

        proc = matlab.startMatlab()
        self.comm = matlab.MatlabCommunicator(proc)
        self.tasks.append(self.comm.waitForResultAsync())
        self.tasks.append(functools.partial(self.comm.printResult))
        self.tasks.append(self.comm.sendCommandsAsync(self._startupCommands()))
        self.tasks.append(functools.partial(self.fetchPoseFromServer, 'q_start'))

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

    def updateIk(self):

        commands = []
        commands.append('l_foot_target = vertcat(l_foot_target_start(1,:)+%s, l_foot_target_start(2,:)+%s, l_foot_target_start(3,:)+%s);' % (self.footOffsets[0], self.footOffsets[1], self.footOffsets[2]) )
        commands.append('kc2l = WorldPositionConstraint(r, l_foot, l_foot_pts, l_foot_target, l_foot_target, tspan);')

        if self.seedWithNominal:
            commands.append('q_seed = q_nom;')
        else:
            commands.append('q_seed = q_end;')

        commands.append('[q_end, info] = inverseKin(r, q_seed, q_nom, qsc, kc2l, kc2r, kc4, s.ikoptions)')

        self.tasks.append(self.comm.sendCommandsAsync(commands))
        self.tasks.append(functools.partial(self.fetchPoseFromServer, 'q_end'))
        self.waitForAsyncTasks()


    def resetQSeed(self):
        commands = []
        commands.append('q_seed = q_nom;')
        self.comm.sendCommands(commands)


    def _scaleMidiValue(self, midiValue):
        scaledValue = midiValue * 0.5/127.0
        return scaledValue

    def handleMidiEvents(self):

        messages = self.reader.getMessages()
        if not messages:
            return

        targets = {}
        for message in messages:
            channel = message[2]
            value = message[3]
            targets[channel] = value

        shouldUpdate = False
        for channel, value in targets.iteritems():

            if channel in self.channelsX:
                self.footOffsets[0] = self._scaleMidiValue(value)
                shouldUpdate = True
            elif channel in self.channelsY:
                self.footOffsets[1] = self._scaleMidiValue(value)
                shouldUpdate = True
            elif channel in self.channelsZ:
                self.footOffsets[2] = self._scaleMidiValue(value)
                shouldUpdate = True

        if shouldUpdate:
            self.updateIk()

    def tick(self):

        if self.handleAsyncTasks() > 0:
            return

        self.handleMidiEvents()
