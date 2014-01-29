import vtkAll as vtk
from ddapp import botpy
import math
import numpy as np
import drc as lcmdrc

def getTransformFromAxes(xaxis, yaxis, zaxis):

    t = vtk.vtkTransform()
    m = vtk.vtkMatrix4x4()

    axes = [xaxis, yaxis, zaxis]
    for r in xrange(3):
        for c in xrange(3):
            # transpose on assignment
            m.SetElement(r, c, axes[c][r])

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


def positionMessageFromFrame(transform):

    pos, wxyz = poseFromTransform(transform)

    trans = lcmdrc.vector_3d_t()
    trans.x, trans.y, trans.z = pos

    quat = lcmdrc.quaternion_t()
    quat.w, quat.x, quat.y, quat.z = wxyz

    pose = lcmdrc.position_3d_t()
    pose.translation = trans
    pose.rotation = quat
    return pose

