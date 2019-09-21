import bot_core
import numpy as np
import time
import re
from director import drcargs
from director import transformUtils

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
        for key, value in getRobotStateToDrakePoseJointMap().items():
            _drakePoseToRobotStateJointMap[value] = key

    return _drakePoseToRobotStateJointMap


def convertStateMessageToDrakePose(msg, strict=True):
    '''
    If strict is true, then the state message must contain a joint_position
    for each named joint in the drake pose joint names.  If strict is false,
    then a default value of 0.0 is used to fill joint positions that are
    not specified in the robot state msg argument.
    '''

    jointMap = {}
    for name, position in zip(msg.joint_name, msg.joint_position):
        jointMap[name] = position

    jointPositions = []
    for name in getDrakePoseJointNames()[6:]:
        if strict:
            jointPositions.append(jointMap[name])
        else:
            jointPositions.append(jointMap.get(name, 0.0))

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
    for jointIdx, drakeIdx in jointIndexMap.items():
        drakePose[drakeIdx] = msg.position[jointIdx]
    return drakePose.tolist()


def robotStateToDrakePose(robotState):

    drakePose = list(range(getNumPositions()))
    jointIndexMap = getRobotStateToDrakePoseJointMap()

    pos = getPositionFromRobotState(robotState)
    rpy = getRollPitchYawFromRobotState(robotState)
    robotState = robotState[7:]

    assert len(jointIndexMap) == getNumJoints()
    assert len(robotState) >= len(jointIndexMap)

    for jointIdx in range(len(jointIndexMap)):
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

    trans = bot_core.vector_3d_t()
    trans.x, trans.y, trans.z = xyz

    quat = bot_core.quaternion_t()
    quat.w, quat.x, quat.y, quat.z = wxyz

    pose = bot_core.position_3d_t()
    pose.translation = trans
    pose.rotation = quat

    return pose


def drakePoseToRobotState(drakePose):

    robotState = list(range(getNumJoints()))
    jointIndexMap = getDrakePoseToRobotStateJointMap()

    for jointIdx, jointAngle in enumerate(drakePose):
        jointIdx =jointIndexMap.get(jointIdx)
        if jointIdx is not None:
            robotState[jointIdx] = jointAngle

    xyz = drakePose[:3]
    rpy = drakePose[3:6]

    m = bot_core.robot_state_t()
    m.utime = int(time.time() * 1e6)
    m.pose = getPoseLCMFromXYZRPY(xyz, rpy)
    m.twist = bot_core.twist_t()
    m.twist.linear_velocity = bot_core.vector_3d_t()
    m.twist.angular_velocity = bot_core.vector_3d_t()
    m.num_joints = getNumJoints()
    m.joint_name = getRobotStateJointNames()
    m.joint_position = robotState
    m.joint_velocity = np.zeros(getNumJoints())
    m.joint_effort = np.zeros(getNumJoints())
    m.force_torque = bot_core.force_torque_t()
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
        _drakePoseJointNames = drcargs.getDirectorConfig()['drakeJointNames']

        return _drakePoseJointNames

def getRobotStateJointNames():
    global _robotStateJointNames

    if _robotStateJointNames:
        return _robotStateJointNames
    else:
        _robotStateJointNames = drcargs.getDirectorConfig()['robotStateJointNames']

        return _robotStateJointNames

def getNumPositions():
    global _numPositions

    if _numPositions is None:
        _numPositions = len(getDrakePoseJointNames())

    return _numPositions

def getNumJoints():
    return getNumPositions() - 6
