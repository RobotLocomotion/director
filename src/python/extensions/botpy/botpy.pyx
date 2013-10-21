# botpy
__version__="0.1"

import numpy as np

cdef extern from "bot_core/rotations.h":

    int bot_quat_to_matrix(const double quat[4], double rot[9])
    int bot_matrix_to_quat(const double rot[9], double quat[4])
    int bot_quat_pos_to_matrix(const double quat[4], const double pos[3], double m[16])
    void bot_angle_axis_to_quat (double theta, const double axis[3], double q[4])
    void bot_quat_to_angle_axis (const double q[4], double *theta, double axis[3])
    void bot_angle_axis_to_roll_pitch_yaw (double angle, const double axis[3], double rpy[3])


######################


cdef double* to_vec2d(data) except NULL:
    array_check(data, 2, float)
    return [data[0], data[1]]

cdef double* to_vec3d(data) except NULL:
    array_check(data, 3, float)
    return [data[0], data[1], data[2]]

cdef double* to_vec4d(data) except NULL:
    array_check(data, 4, float)
    return [data[0], data[1], data[2], data[3]]

cdef double* to_vec9d(data) except NULL:
    return [data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]]

cdef from_vecd(double* data, size):
    x = np.zeros(size)
    for i in xrange(size):
        x[i] = data[i]
    return x

cdef from_vec2d(double* data):
    return from_vecd(data, 2)

cdef from_vec3d(double* data):
    return from_vecd(data, 3)

cdef from_vec4d(double* data):
    return from_vecd(data, 4)

cdef from_vec9d(double* data):
    return from_vecd(data, 9)

cdef from_vec16d(double* data):
    return from_vecd(data, 16)

def array_check(sequence, size, intype):
    n = len(sequence)
    if n != size:
        raise ValueError('expected sequence of length %d, but got %d' % (size, n))
    for x in sequence:
        try:
            intype(x)
        except (TypeError, ValueError):
            raise TypeError('need sequence with item type %s, could not convert item with type %s' % (intype.__name__, type(x).__name__))


#####################



def quat_to_matrix(quat):
    cdef double rot[9]
    bot_quat_to_matrix(to_vec4d(quat), rot)
    return from_vec9d(rot)

def matrix_to_quat(rot):
    cdef double[4] quat
    bot_matrix_to_quat(to_vec9d(rot), quat)
    return from_vec4d(quat)

def quat_pos_to_matrix(quat, pos):
    cdef double[16] m
    bot_quat_pos_to_matrix(to_vec4d(quat), to_vec3d(pos), m)
    return from_vec16d(m)

def angle_axis_to_quat(theta, axis):
    cdef double[4] q
    bot_angle_axis_to_quat(theta, to_vec3d(axis), q)
    return from_vec4d(q)

def quat_to_angle_axis(quat):
    cdef double theta
    cdef double axis[3]
    bot_quat_to_angle_axis(to_vec4d(quat), &theta, axis)
    return theta, from_vec3d(axis)

def angle_axis_to_roll_pitch_yaw(angle, axis):
    cdef double rpy[3]
    bot_angle_axis_to_roll_pitch_yaw(angle, to_vec3d(axis), rpy)
    return from_vec3d(rpy)
