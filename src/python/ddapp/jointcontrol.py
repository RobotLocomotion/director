import os
import math
from ddapp.timercallback import TimerCallback
from ddapp.simpletimer import SimpleTimer
from ddapp import robotstate
from ddapp import getDRCBaseDir
from ddapp import lcmUtils
import drc as lcmdrc
import numpy as np


class JointController(object):

    def __init__(self, models, nominalPoseMatFile, poseCollection=None, jointNames=None):
        self.jointNames = jointNames or robotstate.getDrakePoseJointNames()
        self.numberOfJoints = len(self.jointNames)
        self.models = list(models)
        self.poses = {}
        self.poseCollection = poseCollection
        self.currentPoseName = None
        self.lastRobotStateMessage = None
        self.ignoreOldStateMessages = False

        self.addPose('q_zero', [0.0 for i in xrange(self.numberOfJoints)])
        self.addPose('q_nom', self.loadPoseFromFile(nominalPoseMatFile))

    def setJointPosition(self, jointId, position):
        '''
        Set joint position in degrees.
        '''
        assert jointId >= 0 and jointId < len(self.q)
        self.q[jointId] = math.radians(position % 360.0)
        self.push()

    def push(self):
        for model in self.models:
            model.model.setJointPositions(self.q, self.jointNames)

    def setPose(self, poseName, poseData=None, pushToModel=True):
        if poseData is not None:
            self.addPose(poseName, poseData)
        if poseName not in self.poses:
            raise Exception('Pose %r has not been defined.' % poseName)
        self.q = self.poses[poseName]
        self.currentPoseName = poseName
        if pushToModel:
            self.push()

    def setZeroPose(self):
        self.setPose('q_zero')

    def setNominalPose(self, poseData=None):
        self.setPose('q_nom', poseData)

    def getPose(self, poseName):
        return self.poses.get(poseName)

    def addPose(self, poseName, poseData):
        assert len(poseData) == self.numberOfJoints
        self.poses[poseName] = np.asarray(poseData)
        if self.poseCollection is not None:
            self.poseCollection.setItem(poseName, poseData)

    def loadPoseFromFile(self, filename):
        assert os.path.splitext(filename)[1] == '.mat'
        import scipy.io
        matData = scipy.io.loadmat(filename)
        return np.array(matData['xstar'][:self.numberOfJoints].flatten(), dtype=float)

    def addLCMUpdater(self, channelName):
        '''
        adds an lcm subscriber to update the joint positions from
        lcm robot_state_t messages
        '''

        def onRobotStateMessage(msg):
            if self.ignoreOldStateMessages and self.lastRobotStateMessage is not None and msg.utime < self.lastRobotStateMessage.utime:
                return
            poseName = channelName
            pose = robotstate.convertStateMessageToDrakePose(msg)
            self.lastRobotStateMessage = msg

            # use joint name/positions from robot_state_t and append base_{x,y,z,roll,pitch,yaw}
            jointPositions = np.hstack((msg.joint_position, pose[:6]))
            jointNames = msg.joint_name + robotstate.getDrakePoseJointNames()[:6]

            self.setPose(poseName, pose, pushToModel=False)
            for model in self.models:
                model.model.setJointPositions(jointPositions, jointNames)

        self.subscriber = lcmUtils.addSubscriber(channelName, lcmdrc.robot_state_t, onRobotStateMessage)
        self.subscriber.setSpeedLimit(60)

    def removeLCMUpdater(self):
        lcmUtils.removeSubscriber(self.subscriber)
        self.subscriber = None


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
