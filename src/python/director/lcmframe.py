from director import transformUtils
import bot_core

def frameFromPositionMessage(positionMessage):
    '''
    Given an bot_core.position_t message, returns a vtkTransform
    '''
    trans = positionMessage.translation
    quat = positionMessage.rotation

    trans = [trans.x, trans.y, trans.z]
    quat = [quat.w, quat.x, quat.y, quat.z]

    return transformUtils.transformFromPose(trans, quat)


def positionMessageFromFrame(transform):
    '''
    Given a vtkTransform, returns an bot_core.position_t message
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
