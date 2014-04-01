import vtkAll as vtk
from ddapp import botpy
import math
import numpy as np
import drc as lcmdrc

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
    rpy = [0.0, 0.0, 0.0]
    vtk.vtkMultisenseSource.GetBotRollPitchYaw(t, rpy)
    return rpy


def rollPitchYawFromTransform(t):
    rpy = np.zeros(3)
    vtk.vtkMultisenseSource.GetBotRollPitchYaw(t, rpy)
    return rpy



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
