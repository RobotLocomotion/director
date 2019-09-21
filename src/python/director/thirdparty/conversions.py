

import numbers

import numpy
from director.thirdparty import transformations
from .geometry_msgs_mock import Pose



def rounded(val):
  if isinstance(val, str):
    return rounded(float(val))
  elif isinstance(val, numbers.Number):
    return int(round(val,6) * 1e5) / 1.0e5
  else:
    return numpy.array([rounded(v) for v in val])


def homogeneous2translation_quaternion(homogeneous):
  """
  Translation: [x, y, z]
  Quaternion: [x, y, z, w]
  """
  translation = transformations.translation_from_matrix(homogeneous)
  quaternion = transformations.quaternion_from_matrix(homogeneous)
  return translation, quaternion


def homogeneous2translation_rpy(homogeneous):
  """
  Translation: [x, y, z]
  RPY: [sx, sy, sz]
  """
  translation = transformations.translation_from_matrix(homogeneous)
  rpy = transformations.euler_from_matrix(homogeneous)
  return translation, rpy


def homogeneous2pose_msg(homogeneous):
  pose = Pose()
  translation, quaternion = homogeneous2translation_quaternion(homogeneous)
  pose.position.x = translation[0]
  pose.position.y = translation[1]
  pose.position.z = translation[2]
  pose.orientation.x = quaternion[0]
  pose.orientation.y = quaternion[1]
  pose.orientation.z = quaternion[2]
  pose.orientation.w = quaternion[3]
  return pose


def pose_msg2homogeneous(pose):
  trans = transformations.translation_matrix((pose.position.x, pose.position.y, pose.position.z))
  rot = transformations.quaternion_matrix((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
  return transformations.concatenate_matrices(trans, rot)


def array2string(array):
  return numpy.array_str(array).strip('[]. ').replace('. ', ' ')


def homogeneous2tq_string(homogeneous):
  return 't=%s q=%s' % homogeneous2translation_quaternion(homogeneous)


def homogeneous2tq_string_rounded(homogeneous):
  return 't=%s q=%s' % tuple(rounded(o) for o in homogeneous2translation_quaternion(homogeneous))


def string2float_list(s):
  return [float(i) for i in s.split()]


def pose_string2homogeneous(pose):
  pose_float = string2float_list(pose)
  translate = pose_float[:3]
  angles = pose_float[3:]
  homogeneous = transformations.compose_matrix(None, None, angles, translate)
  #print('pose_string=%s; translate=%s angles=%s homogeneous:\n%s' % (pose, translate, angles, homogeneous))
  return homogeneous


def rotation_only(homogeneous):
  euler = transformations.euler_from_matrix(homogeneous)
  return transformations.euler_matrix(euler[0], euler[1], euler[2])

