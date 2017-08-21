from director import transformUtils
from director.thirdparty import transformations
import numpy as np


def testTransform():
    '''
    test transformFromPose --> getAxesFromTransform is same as quat --> matrix
    '''

    quat = transformations.random_quaternion()
    pos = np.random.rand(3)

    frame = transformUtils.transformFromPose(pos, quat)
    axes = transformUtils.getAxesFromTransform(frame)
    mat = transformUtils.getNumpyFromTransform(frame)

    assert np.allclose(mat[:3,:3], np.array(axes).transpose())

    mat2 = transformations.quaternion_matrix(quat)
    mat2[:3,3] = pos

    print mat
    print mat2
    assert np.allclose(mat, mat2)


def testEuler():
    '''
    Test some euler conversions
    '''
    quat = transformations.random_quaternion()
    pos = np.random.rand(3)
    frame = transformUtils.transformFromPose(pos, quat)
    mat = transformUtils.getNumpyFromTransform(frame)

    rpy = transformUtils.rollPitchYawFromTransform(frame)
    rpy2 = transformations.euler_from_matrix(mat)

    print rpy
    print rpy2
    assert np.allclose(rpy, rpy2)



def testEulerToFrame():
    '''
    Test some euler converions
    '''
    rpy = transformations.euler_from_quaternion(transformations.random_quaternion())

    frame = transformUtils.frameFromPositionAndRPY([0,0,0], np.degrees(rpy))
    mat = transformUtils.getNumpyFromTransform(frame)

    mat2 = transformations.euler_matrix(rpy[0], rpy[1], rpy[2])

    print mat
    print mat2
    assert np.allclose(mat, mat2)


testTransform()
testEuler()
testEulerToFrame()
