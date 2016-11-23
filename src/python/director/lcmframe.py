from director import transformUtils
import bot_core

def frameFromPositionMessage(msg):
    '''
    Given an bot_core.position_t message, returns a vtkTransform
    '''
    trans = msg.translation
    quat = msg.rotation

    trans = [trans.x, trans.y, trans.z]
    quat = [quat.w, quat.x, quat.y, quat.z]

    return transformUtils.transformFromPose(trans, quat)


def frameFromRigidTransformMessage(msg):
    '''
    Given an bot_core.rigid_transform_t message, returns a vtkTransform
    '''
    trans = msg.trans
    quat = msg.quat
    return transformUtils.transformFromPose(trans, quat)


def positionMessageFromFrame(transform):
    '''
    Given a vtkTransform, returns a bot_core.position_t message
    '''

    pos, wxyz = transformUtils.poseFromTransform(transform)

    trans = bot_core.vector_3d_t()
    trans.x, trans.y, trans.z = pos

    quat = bot_core.quaternion_t()
    quat.w, quat.x, quat.y, quat.z = wxyz

    pose = bot_core.position_3d_t()
    pose.translation = trans
    pose.rotation = quat
    return pose


def rigidTransformMessageFromFrame(transform):
    '''
    Given a vtkTransform, returns a bot_core.rigid_transform_t message
    '''
    msg = bot_core.rigid_transform_t()
    msg.trans, msg.quat = transformUtils.poseFromTransform(transform)
    return msg
