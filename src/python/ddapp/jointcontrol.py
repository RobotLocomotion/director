import math
from ddapp.timercallback import TimerCallback
from ddapp import midi


class JointController(object):

    def __init__(self, model):
        self.model = model
        self.poses = {}
        self.poses['zero'] = [0.0 for i in xrange(self.model.numberOfJoints())]
        self.setZeroPose()

    def setJointPosition(self, jointId, position):
        assert jointId >= 0 and jointId < len(self.q)
        self.q[jointId] = math.radians(position % 360.0)
        self.push()

    def push(self):
        self.model.setJointPositions(self.q)

    def reset(self):
        self.q = [0.0 for i in xrange(self.model.numberOfJoints())]

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
        assert len(poseData) == self.model.numberOfJoints()
        self.poses[poseName] = poseData

    def addNominalPoseFromFile(self, filename):
        import scipy.io
        matData = scipy.io.loadmat(filename)
        xstar = matData['xstar'][:self.model.numberOfJoints()]
        self.addPose('nominal', xstar.flatten().tolist())


class MidiJointControl(TimerCallback):

    def __init__(self, model):
        TimerCallback.__init__(self)
        self.reader = midi.MidiReader()
        self.controller = JointController(model)
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
