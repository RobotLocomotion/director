import numpy as np
import drc as lcmdrc


def encodeLinCon(A, b):
    A = np.asarray(A)
    b = np.asarray(b)
    msg = lcmdrc.lin_con_t()
    msg.m = A.shape[0]
    msg.n = A.shape[1]
    msg.m_times_n = msg.m * msg.n
    msg.A = A.flatten(order='F') # column-major
    msg.b = b
    return msg


def decodeLinCon(msg):
    A = np.asarray(msg.A)
    b = np.asarray(msg.b)
    A = A.reshape((msg.m, msg.n), order='F')
    return A, b
