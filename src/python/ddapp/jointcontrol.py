import os
import math
from ddapp.timercallback import TimerCallback
from ddapp.simpletimer import SimpleTimer
from ddapp import midi


class JointController(object):

    def __init__(self, models, poseCollection=None):
        self.numberOfJoints = 34
        self.models = models
        self.poses = {}
        self.poseCollection = poseCollection
        self.currentPoseName = None
        self.addPose('q_zero', [0.0 for i in xrange(self.numberOfJoints)])

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

    def setPose(self, poseName, poseData=None):
        if poseData is not None:
            self.addPose(poseName, poseData)
        if poseName not in self.poses:
            raise Exception('Pose %r has not been defined.' % poseName)
        self.q = self.poses[poseName]
        self.currentPoseName = poseName
        self.push()

    def setZeroPose(self):
        self.setPose('q_zero')

    def setNominalPose(self, poseData=None):
        self.setPose('q_nom', poseData)

    def getPose(self, poseName):
        return self.poses.get(poseName)

    def addPose(self, poseName, poseData):
        assert len(poseData) == self.numberOfJoints
        self.poses[poseName] = poseData
        if self.poseCollection is not None:
            self.poseCollection.setItem(poseName, poseData)

    def loadPoseFromFile(self, filename):
        assert os.path.splitext(filename)[1] == '.mat'
        import scipy.io
        matData = scipy.io.loadmat(filename)
        return matData['xstar'][:self.numberOfJoints]


class MidiJointControl(TimerCallback):

    def __init__(self, jointController):
        TimerCallback.__init__(self)
        self.reader = midi.MidiReader()
        self.controller = jointController
        self.channelToJoint = { 112: 13 }


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
