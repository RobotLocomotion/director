import math
from ddapp.timercallback import TimerCallback
from ddapp.simpletimer import SimpleTimer
from ddapp import midi


class JointController(object):

    def __init__(self, model):
        #self.numberOfJoints = model.numberOfJoints()
        self.numberOfJoints = 34
        self.models = [model]
        self.poses = {}
        self.poses['zero'] = [0.0 for i in xrange(self.numberOfJoints)]

    def addModel(self, model):
        self.models.append(model)

    def setJointPosition(self, jointId, position):
        '''
        Set joint position in degrees.
        '''
        assert jointId >= 0 and jointId < len(self.q)
        self.q[jointId] = math.radians(position % 360.0)
        self.push()

    def push(self):
        for model in self.models:
            model.setJointPositions(self.q)

    def reset(self):
        self.q = [0.0 for i in xrange(self.numberOfJoints)]

    def setPose(self, poseName):
        if poseName not in self.poses:
            raise Exception('Pose %r has not been defined.' % poseName)
        self.q = self.poses[poseName]
        self.push()

    def setZeroPose(self):
        self.setPose('zero')

    def setNominalPose(self):
        self.setPose('nominal')

    def addPose(self, poseName, poseData):
        assert len(poseData) == self.numberOfJoints
        self.poses[poseName] = poseData

    def addNominalPoseFromFile(self, filename):
        import scipy.io
        matData = scipy.io.loadmat(filename)
        #xstar = matData['xstar'][:self.numberOfJoints]
        xstar = matData['xstar'][:34]
        self.addPose('nominal', xstar.flatten().tolist())


class MidiJointControl(TimerCallback):

    def __init__(self, jointController):
        TimerCallback.__init__(self)
        self.reader = midi.MidiReader()
        self.controller = jointController
        self.channelToJoint = { 21: 13 }


    def _scaleMidiValue(self, midiValue):
        degrees = midiValue * 180.0/127.0
        return degrees


    def tick(self):
        messages = self.reader.getMessages()
        if not messages:
            return

        targets = {}
        for message in messages:
            channel = message[2]
            value = message[3]
            targets[channel] = value

        for channel, value in targets.iteritems():
            jointId = self.channelToJoint.get(channel)
            position = self._scaleMidiValue(value)

            if jointId is not None:
                self.controller.setJointPosition(jointId, position)


class JointControlTestRamp(TimerCallback):

    def __init__(self, jointController):
        TimerCallback.__init__(self)
        self.controller = jointController
        self.testTime = 2.0

    def testJoint(self, jointId):
        self.jointId = jointId
        self.testTimer = SimpleTimer()
        self.start()

    def tick(self):

        if self.testTimer.elapsed() > self.testTime:
            self.stop()
            return

        jointPosition = math.sin( (self.testTimer.elapsed() / self.testTime) * math.pi) * math.pi
        self.controller.setJointPosition(self.jointId, math.degrees(jointPosition))
