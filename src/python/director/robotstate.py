import drc as lcmdrc
import numpy as np
import time
import re
import json
from ddapp import drcargs
from ddapp import transformUtils

_robotStateToDrakePoseJointMap = None
_drakePoseToRobotStateJointMap = None
_drakePoseJointNames = None
_robotStateJointNames = None
_numPositions = None


def getRollPitchYawFromRobotState(robotState):
    return transformUtils.quaternionToRollPitchYaw(robotState[3:7])


def getPositionFromRobotState(robotState):
    return robotState[0:4]


def getRobotStateToDrakePoseJointMap():

    global _robotStateToDrakePoseJointMap

    if _robotStateToDrakePoseJointMap is None:
        robotStateJointNames = getRobotStateJointNames()
        drakePoseJointNames = getDrakePoseJointNames()

        _robotStateToDrakePoseJointMap = dict()

        for robotStateJointIdx, robotStateJointName in enumerate(robotStateJointNames):
            drakeJointIdx = drakePoseJointNames.index(robotStateJointName)
            _robotStateToDrakePoseJointMap[robotStateJointIdx] = drakeJointIdx

    return _robotStateToDrakePoseJointMap


def getDrakePoseToRobotStateJointMap():

    global _drakePoseToRobotStateJointMap

    if _drakePoseToRobotStateJointMap is None:
        _drakePoseToRobotStateJointMap = dict()
        for key, value in getRobotStateToDrakePoseJointMap().iteritems():
            _drakePoseToRobotStateJointMap[value] = key

    return _drakePoseToRobotStateJointMap


def convertStateMessageToDrakePose(msg):

    jointMap = {}
    for name, position in zip(msg.joint_name, msg.joint_position):
        jointMap[name] = position

    jointPositions = []
    for name in getDrakePoseJointNames()[6:]:
        jointPositions.append(jointMap[name])

    trans = msg.pose.translation
    quat = msg.pose.rotation
    trans = [trans.x, trans.y, trans.z]
    quat = [quat.w, quat.x, quat.y, quat.z]
    rpy = transformUtils.quaternionToRollPitchYaw(quat)

    pose = np.hstack((trans, rpy, jointPositions))
    assert len(pose) == getNumPositions()
    return pose

def atlasCommandToDrakePose(msg):
    jointIndexMap = getRobotStateToDrakePoseJointMap()
    drakePose = np.zeros(len(getDrakePoseJointNames()))
    for jointIdx, drakeIdx in jointIndexMap.iteritems():
        drakePose[drakeIdx] = msg.position[jointIdx]
    return drakePose.tolist()


def asRobotPlan(msg):
    if isinstance(msg, lcmdrc.robot_plan_with_supports_t):
        return msg.plan
    return msg


def robotStateToDrakePose(robotState):

    drakePose = range(getNumPositions())
    jointIndexMap = getRobotStateToDrakePoseJointMap()

    pos = getPositionFromRobotState(robotState)
    rpy = getRollPitchYawFromRobotState(robotState)
    robotState = robotState[7:]

    assert len(jointIndexMap) == getNumJoints()
    assert len(robotState) >= len(jointIndexMap)

    for jointIdx in xrange(len(jointIndexMap)):
        drakePose[jointIndexMap[jointIdx]] = robotState[jointIdx]

    drakePose[0] = pos[0]
    drakePose[1] = pos[1]
    drakePose[2] = pos[2]
    drakePose[3] = rpy[0]
    drakePose[4] = rpy[1]
    drakePose[5] = rpy[2]

    return drakePose


def getPoseLCMFromXYZRPY(xyz, rpy):

    wxyz = transformUtils.rollPitchYawToQuaternion(rpy)

    trans = lcmdrc.vector_3d_t()
    trans.x, trans.y, trans.z = xyz

    quat = lcmdrc.quaternion_t()
    quat.w, quat.x, quat.y, quat.z = wxyz

    pose = lcmdrc.position_3d_t()
    pose.translation = trans
    pose.rotation = quat

    return pose


def drakePoseToRobotState(drakePose):

    robotState = range(getNumJoints())
    jointIndexMap = getDrakePoseToRobotStateJointMap()

    for jointIdx, jointAngle in enumerate(drakePose):
        jointIdx =jointIndexMap.get(jointIdx)
        if jointIdx is not None:
            robotState[jointIdx] = jointAngle

    xyz = drakePose[:3]
    rpy = drakePose[3:6]

    m = lcmdrc.robot_state_t()
    m.utime = int(time.time() * 1e6)
    m.pose = getPoseLCMFromXYZRPY(xyz, rpy)
    m.twist = lcmdrc.twist_t()
    m.twist.linear_velocity = lcmdrc.vector_3d_t()
    m.twist.angular_velocity = lcmdrc.vector_3d_t()
    m.num_joints = getNumJoints()
    m.joint_name = getRobotStateJointNames()
    m.joint_position = robotState
    m.joint_velocity = np.zeros(getNumJoints())
    m.joint_effort = np.zeros(getNumJoints())
    m.force_torque = lcmdrc.force_torque_t()
    m.force_torque.l_hand_force = np.zeros(3)
    m.force_torque.l_hand_torque = np.zeros(3)
    m.force_torque.r_hand_force = np.zeros(3)
    m.force_torque.r_hand_torque = np.zeros(3)

    return m

    
def matchJoints(regex):
    search = re.compile(regex).search
    return [name for name in getDrakePoseJointNames() if search(name)]


def getDrakePoseJointNames():
    global _drakePoseJointNames

    if _drakePoseJointNames:
        return _drakePoseJointNames
    else:
        with open(drcargs.args().directorConfigFile) as directorConfigFile:
            _drakePoseJointNames = json.load(directorConfigFile)['drakeJointNames']

        return _drakePoseJointNames

def getRobotStateJointNames():
    global _robotStateJointNames

    if _robotStateJointNames:
        return _robotStateJointNames
    else:
        with open(drcargs.args().directorConfigFile) as directorConfigFile:
            _robotStateJointNames = json.load(directorConfigFile)['robotStateJointNames']

        return _robotStateJointNames

def getNumPositions():
    global _numPositions

    if _numPositions is None:
        _numPositions = len(getDrakePoseJointNames())

    return _numPositions

def getNumJoints():
    return getNumPositions() - 6
