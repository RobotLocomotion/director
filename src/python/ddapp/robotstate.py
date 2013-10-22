import botpy


_robotStateToDrakePoseJointMap = None
_drakePoseToRobotStateJointMap = None


def getRollPitchYawFromRobotState(robotState):
    return botpy.quat_to_roll_pitch_yaw(robotState[3:7])


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


def robotStateToDrakePose(robotState):

    drakePose = range(34)
    jointIndexMap = getRobotStateToDrakePoseJointMap()

    pos = getPositionFromRobotState(robotState)
    rpy = getRollPitchYawFromRobotState(robotState)
    robotState = robotState[7:]

    assert len(jointIndexMap) == 28
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


def drakePoseToRobotState(drakePose):

    robotState = range(28)
    jointIndexMap = getDrakePoseToRobotStateJointMap()

    for jointIdx, jointAngle in enumerate(drakePose):
        jointIdx =jointIndexMap.get(jointIdx)
        if jointIdx is not None:
            robotState[jointIdx] = jointAngle

    return robotState



def getDrakePoseJointNames():
    return [
      'base_x',
      'base_y',
      'base_z',
      'base_roll',
      'base_pitch',
      'base_yaw',
      'back_bkz',
      'back_bky',
      'back_bkx',
      'l_arm_usy',
      'l_arm_shx',
      'l_arm_ely',
      'l_arm_elx',
      'l_arm_uwy',
      'l_leg_hpz',
      'l_leg_hpx',
      'l_leg_hpy',
      'l_leg_kny',
      'l_leg_aky',
      'l_leg_akx',
      'l_arm_mwx',
      'r_arm_usy',
      'r_arm_shx',
      'r_arm_ely',
      'r_arm_elx',
      'r_arm_uwy',
      'r_leg_hpz',
      'r_leg_hpx',
      'r_leg_hpy',
      'r_leg_kny',
      'r_leg_aky',
      'r_leg_akx',
      'r_arm_mwx',
      'neck_ay',
      ]

def getRobotStateJointNames():
    return [
      'back_bkz',
      'back_bky',
      'back_bkx',
      'neck_ay',
      'l_leg_hpz',
      'l_leg_hpx',
      'l_leg_hpy',
      'l_leg_kny',
      'l_leg_aky',
      'l_leg_akx',
      'r_leg_hpz',
      'r_leg_hpx',
      'r_leg_hpy',
      'r_leg_kny',
      'r_leg_aky',
      'r_leg_akx',
      'l_arm_usy',
      'l_arm_shx',
      'l_arm_ely',
      'l_arm_elx',
      'l_arm_uwy',
      'l_arm_mwx',
      'r_arm_usy',
      'r_arm_shx',
      'r_arm_ely',
      'r_arm_elx',
      'r_arm_uwy',
      'r_arm_mwx'
      ]
