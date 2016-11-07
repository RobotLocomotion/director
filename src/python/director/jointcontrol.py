import os
import math
from director.timercallback import TimerCallback
from director.simpletimer import SimpleTimer
from director import robotstate
from director import getDRCBaseDir
from director import lcmUtils
import bot_core
import numpy as np


class JointController(object):

    def __init__(self, models, poseCollection=None, jointNames=None):
        self.jointNames = jointNames or robotstate.getDrakePoseJointNames()
        self.numberOfJoints = len(self.jointNames)
        self.models = list(models)
        self.poses = {}
        self.poseCollection = poseCollection
        self.currentPoseName = None
        self.lastRobotStateMessage = None
        self.ignoreOldStateMessages = False
        self.setPose('q_zero', np.zeros(self.numberOfJoints))

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

    def getPose(self, poseName):
        return self.poses.get(poseName)

    def addPose(self, poseName, poseData):
        assert len(poseData) == self.numberOfJoints
        self.poses[poseName] = np.asarray(poseData)
        if self.poseCollection is not None:
            self.poseCollection.setItem(poseName, poseData)

    def loadPoseFromFile(self, filename):
        ext = os.path.splitext(filename)[1].lower()

        if ext == '.mat':
            import scipy.io
            matData = scipy.io.loadmat(filename)
            pose = np.array(matData['xstar'][:self.numberOfJoints].flatten(), dtype=float)
        elif ext == '.csv':
            pose = np.loadtxt(filename, delimiter=',', dtype=float).flatten()
        else:
            raise Exception('Unsupported pose file format: %s' % filename)

        assert pose.shape[0] == self.numberOfJoints
        return pose

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

        self.subscriber = lcmUtils.addSubscriber(channelName, bot_core.robot_state_t, onRobotStateMessage)
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
