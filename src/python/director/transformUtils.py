from director.thirdparty import transformations
import director.vtkAll as vtk
import math
import numpy as np


def getTransformFromNumpy(mat):
    '''
    Given a numpy 4x4 array, return a vtkTransform.
    '''
    assert mat.shape == (4,4)
    t = vtk.vtkTransform()
    t.SetMatrix(mat.flatten())
    return t


def getNumpyFromTransform(transform):
    '''
    Given a vtkTransform, return a numpy 4x4 array
    '''
    mat = transform.GetMatrix()
    a = np.zeros((4,4))

    for r in range(4):
        for c in range(4):
            a[r][c] = mat.GetElement(r, c)

    return a


def getTransformFromAxes(xaxis, yaxis, zaxis):

    t = vtk.vtkTransform()
    m = vtk.vtkMatrix4x4()

    axes = np.array([xaxis, yaxis, zaxis]).transpose().copy()
    vtk.vtkMath.Orthogonalize3x3(axes, axes)

    for r in range(3):
        for c in range(3):
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


def getLookAtTransform(lookAtPosition, lookFromPosition, viewUp=[0.0, 0.0, 1.0]):

    xaxis = np.array(lookAtPosition) - np.array(lookFromPosition)
    if np.linalg.norm(xaxis) < 1e-8:
        xaxis = [1.0, 0.0, 0.0]
    zaxis = np.array(viewUp)
    xaxis /= np.linalg.norm(xaxis)
    zaxis /= np.linalg.norm(zaxis)
    yaxis = np.cross(zaxis, xaxis)
    yaxis /= np.linalg.norm(yaxis)
    zaxis = np.cross(xaxis, yaxis)
    return getTransformFromAxesAndOrigin(xaxis, yaxis, zaxis, lookFromPosition)


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
    quat_c = transformations.quaternion_slerp(quat_a, quat_b, weight_b)
    return transformFromPose(pos_c, quat_c)


def transformFromPose(position, quaternion):
    '''
    Returns a vtkTransform
    '''
    mat = transformations.quaternion_matrix(quaternion)
    mat[:3,3] = position
    return getTransformFromNumpy(mat)


def poseFromTransform(transform):
    '''
    Returns position, quaternion
    '''
    mat = getNumpyFromTransform(transform)
    return np.array(mat[:3,3]), transformations.quaternion_from_matrix(mat, isprecise=True)


def frameFromPositionAndRPY(position, rpy):
    '''
    rpy specified in degrees
    '''

    rpy = np.radians(rpy)
    mat = transformations.euler_matrix(rpy[0], rpy[1], rpy[2])
    mat[:3,3] = position
    return getTransformFromNumpy(mat)


def frameWithoutRollAndPitch(transform):
    pos, quat = poseFromTransform(transform)
    rpy = quaternionToRollPitchYaw(quat)
    quat = rollPitchYawToQuaternion([0.0, 0.0, rpy[2]])
    return transformFromPose(pos, quat)


def rollPitchYawToQuaternion(rpy):
    return transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])


def quaternionToRollPitchYaw(quat):
    return transformations.euler_from_quaternion(quat)


def copyFrame(transform):
    t = vtk.vtkTransform()
    t.PostMultiply()
    t.SetMatrix(transform.GetMatrix())
    return t


def forceMomentTransformation(inputFrame, outputFrame):
    """
    Utility getting the force-moment transformation for converting wrenches from
    one frame to another. Ff wrench is 6 x 1 (moment, force) expressed in input frame,
    then corresponding wrench in output frame would be numpy.dot(FM, wrench) = wrench_in_output_frame.
    :param inputFrame: A vtkTransform defining the input frame
    :param outputFrame: A vtkTransform defining the output frame
    :return: 4x4 matrix that transform a wrench from the input frame to the output frame
    """
    
    FM = np.zeros((6,6))

    inputToOutputFrame = copyFrame(inputFrame)
    inputToOutputFrame.PostMultiply()
    inputToOutputFrame.Concatenate(outputFrame.GetLinearInverse())

    position, quaternion = poseFromTransform(inputToOutputFrame)
    inputToOutputRotationMatrix = transformations.quaternion_matrix(quaternion)[:3,:3]

    FM[0:3, 0:3] = inputToOutputRotationMatrix
    FM[3:,3:] = inputToOutputRotationMatrix
    cross = crossProductMatrix(position)
    FM[0:3,3:] = np.dot(cross,inputToOutputRotationMatrix)

    return FM

def crossProductMatrix(x):
    """
    Computes the matrix P such that P*y = cross product of x and y
    :param x:
    :return: P, a 3x3 matrix
    """
    cross = np.zeros((3,3))
    cross[0,1] = -x[2]
    cross[0,2] = x[1]
    cross[1,0] = x[2]
    cross[1,2] = -x[0]
    cross[2,0] = -x[1]
    cross[2,1] = x[0]

    return cross


