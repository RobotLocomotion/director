from ddapp import transformUtils
from ddapp.thirdparty import transformations
import numpy as np

try:
  from ddapp import botpy
except ImportError:
  botpy = None

'''
This tests the routines in ddapp.transformUtils, botpy, and
ddapp.thirdparty.transformations to make sure the modules
agree about conversions between matrices, quaternions, and
euler angles.
'''


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


def isQuatEqual(quatA, quatB):
    matA = transformations.quaternion_matrix(quatA)
    matB = transformations.quaternion_matrix(quatB)
    return transformations.is_same_transform(matA, matB)


def testQuaternionInterpolate():
    '''
    Test quaternion interpolation in botpy
    '''

    q1 = transformations.random_quaternion()
    q2 = transformations.random_quaternion()

    print q1
    print q2

    for weight in np.linspace(0, 1, 10):
      qi = botpy.quat_interpolate(q1, q2, weight)
      qi2 = transformations.quaternion_slerp(q1, q2, weight)

      print weight, qi, qi2
      assert isQuatEqual(qi, qi2)

      if weight == 0.0:
        assert isQuatEqual(qi, q1)

      if weight == 1.0:
        assert isQuatEqual(qi, q2)


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


def testEulerBotpy():
    '''
    Test some quaternion and euler conversions with botpy
    '''

    quat = transformations.random_quaternion()
    rpy = transformations.euler_from_quaternion(quat)

    rpy2 = botpy.quat_to_roll_pitch_yaw(quat)
    quat2 = botpy.roll_pitch_yaw_to_quat(rpy)

    mat = transformations.quaternion_matrix(quat)
    frame = transformUtils.getTransformFromNumpy(mat)
    rpy3 = transformUtils.rollPitchYawFromTransform(frame)

    print quat, quat2
    print rpy, rpy2, rpy3

    assert isQuatEqual(quat, quat2)
    assert np.allclose(rpy, rpy2)
    assert np.allclose(rpy2, rpy3)


testTransform()
testEuler()
testEulerToFrame()

if botpy:
    testQuaternionInterpolate()
    testEulerBotpy()
else:
    print 'skipped botpy tests because botpy module is not available'

