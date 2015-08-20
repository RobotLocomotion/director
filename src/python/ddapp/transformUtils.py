import vtkAll as vtk
#from ddapp import botpy
import math
import numpy as np
#import drc as lcmdrc


def getTransformFromNumpy(mat):
    '''
    Given a numpy 4x4 array, return a vtkTransform.
    '''
    m = vtk.vtkMatrix4x4()
    for r in xrange(4):
        for c in xrange(4):
            m.SetElement(r, c, mat[r][c])

    t = vtk.vtkTransform()
    t.SetMatrix(m)
    return t


def getNumpyFromTransform(transform):
    '''
    Given a vtkTransform, return a numpy 4x4 array
    '''
    mat = transform.GetMatrix()
    a = np.zeros((4,4))

    for r in xrange(4):
        for c in xrange(4):
            a[r][c] = mat.GetElement(r, c)

    return a


def getTransformFromAxes(xaxis, yaxis, zaxis):

    t = vtk.vtkTransform()
    m = vtk.vtkMatrix4x4()

    axes = np.array([xaxis, yaxis, zaxis]).transpose().copy()
    vtk.vtkMath.Orthogonalize3x3(axes, axes)

    for r in xrange(3):
        for c in xrange(3):
            m.SetElement(r, c, axes[r][c])

    t.SetMatrix(m)
    return t


def getTransformFromAxesAndOrigin(xaxis, yaxis, zaxis, origin):
    t = getTransformFromAxes(xaxis, yaxis, zaxis)
    t.PostMultiply()
    t.Translate(origin)
    return t


def getAxesFromTransform(t):
    xaxis = np.array(t.TransformNormal(1,0,0))
    yaxis = np.array(t.TransformNormal(0,1,0))
    zaxis = np.array(t.TransformNormal(0,0,1))
    return xaxis, yaxis, zaxis


def concatenateTransforms(transformList):
    '''
    Given a list of vtkTransform objects, returns a new vtkTransform
    which is a concatenation of the whole list using vtk post multiply.
    See documentation for vtkTransform::PostMultiply.
    '''
    result = vtk.vtkTransform()
    result.PostMultiply()
    for t in transformList:
        result.Concatenate(t)
    return result


def findTransformAxis(transform, referenceVector):
    '''
    Given a vtkTransform and a reference vector, find a +/- axis of the transform
    that most closely matches the reference vector.  Returns the matching axis
    index, axis, and sign.
    '''
    refAxis = referenceVector / np.linalg.norm(referenceVector)
    axes = getAxesFromTransform(transform)

    axisProjections = np.array([np.abs(np.dot(axis, refAxis)) for axis in axes])
    matchIndex = axisProjections.argmax()
    matchAxis = axes[matchIndex]
    matchSign = np.sign(np.dot(matchAxis, refAxis))
    return matchIndex, matchAxis, matchSign


def getTransformFromOriginAndNormal(origin, normal, normalAxis=2):

    normal = np.array(normal)
    normal /= np.linalg.norm(normal)

    axes = [[0,0,0],
            [0,0,0],
            [0,0,0]]

    axes[normalAxis] = normal

    vtk.vtkMath.Perpendiculars(axes[normalAxis], axes[(normalAxis+1) % 3], axes[(normalAxis+2) % 3], 0)
    t = getTransformFromAxes(*axes)
    t.PostMultiply()
    t.Translate(origin)
    return t


def orientationFromNormal(normal):
    '''
    Creates a frame where the Z axis points in the direction of the given normal.
    '''

    zaxis = normal
    xaxis = [0,0,0]
    yaxis = [0,0,0]

    vtk.vtkMath.Perpendiculars(zaxis, xaxis, yaxis, 0)

    return orientationFromAxes(xaxis, yaxis, zaxis)


def orientationFromAxes(xaxis, yaxis, zaxis):
    t = getTransformFromAxes(xaxis, yaxis, zaxis)
    return rollPitchYawFromTransform(t)


def rollPitchYawFromTransform(t):
    pos, quat = poseFromTransform(t)
    return quaternionToRollPitchYaw(quat)


def frameInterpolate(trans_a, trans_b, weight_b):
    '''
    Interpolate two frames where weight_b=[0,1]
    '''
    [pos_a, quat_a] = poseFromTransform(trans_a)
    [pos_b, quat_b] = poseFromTransform(trans_b)
    pos_c = pos_a *(1-weight_b) + pos_b * weight_b;
    quat_c = botpy.quat_interpolate(quat_a,quat_b, weight_b)
    return transformFromPose(pos_c, quat_c)


def transformFromPose(position, quaternion):
    '''
    Returns a vtkTransform
    '''
    rotationMatrix = np.zeros((3,3))
    vtk.vtkMath.QuaternionToMatrix3x3(quaternion, rotationMatrix)

    mat = np.eye(4)
    mat[:3,:3] = rotationMatrix
    mat[:3,3] = position

    t = vtk.vtkTransform()
    t.SetMatrix(mat.flatten())
    return t


def poseFromTransform(transform):
    '''
    Returns position, quaternion
    '''
    angleAxis = range(4)
    transform.GetOrientationWXYZ(angleAxis)
    angleAxis[0] = math.radians(angleAxis[0])
    pos = transform.GetPosition()
    quat = botpy.angle_axis_to_quat(angleAxis[0], angleAxis[1:])
    return np.array(pos), np.array(quat)


def frameFromPositionAndRPY(position, rpy):
    '''
    rpy specified in degrees
    '''

    rpy = [math.radians(deg) for deg in rpy]

    angle, axis = botpy.roll_pitch_yaw_to_angle_axis(rpy)

    t = vtk.vtkTransform()
    t.PostMultiply()
    t.RotateWXYZ(math.degrees(angle), axis)
    t.Translate(position)
    return t


def rollPitchYawToQuaternion(rpy):
    return botpy.roll_pitch_yaw_to_quat(rpy)


def quaternionToRollPitchYaw(quat):
    return botpy.quat_to_roll_pitch_yaw(quat)


def frameFromPositionMessage(positionMessage):
    '''
    Given an lcmdrc.position_t message, returns a vtkTransform
    '''
    trans = positionMessage.translation
    quat = positionMessage.rotation

    trans = [trans.x, trans.y, trans.z]
    quat = [quat.w, quat.x, quat.y, quat.z]

    return transformFromPose(trans, quat)


def positionMessageFromFrame(transform):
    '''
    Given a vtkTransform, returns an lcmdrc.position_t message
    '''

    pos, wxyz = poseFromTransform(transform)

    trans = lcmdrc.vector_3d_t()
    trans.x, trans.y, trans.z = pos

    quat = lcmdrc.quaternion_t()
    quat.w, quat.x, quat.y, quat.z = wxyz

    pose = lcmdrc.position_3d_t()
    pose.translation = trans
    pose.rotation = quat
    return pose


def copyFrame(transform):
    t = vtk.vtkTransform()
    t.PostMultiply()
    t.SetMatrix(transform.GetMatrix())
    return t
