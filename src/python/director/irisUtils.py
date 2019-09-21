import numpy as np
import drc as lcmdrc
from irispy.utils import lcon_to_vert
from director.lcmframe import frameFromPositionMessage, positionMessageFromFrame
from scipy.spatial import ConvexHull


class SafeTerrainRegion:
    __slots__ = ['A', 'b', 'C', 'd','pose']
    """
    Container for describing an IRIS safe region of terrain.
    Format:
        A, b: linear constraints on [x,y,z,yaw] of the center of the foot, A[x;y;z;yaw] <= b
        C, d: ellipsoidal constraints on [x,y,z,yaw] of the center of the foot. Not currently used.
        tform: a pose on the terrain within the region, from which we extract a plane.
    """
    def __init__(self, A, b, C, d, tform):
        self.A = A
        self.b = b
        self.C = C
        self.d = d
        self.tform = tform

    def to_iris_region_t(self):
        msg = lcmdrc.iris_region_t()
        msg.lin_con = self.to_lin_con_t()
        msg.seed_pose = positionMessageFromFrame(self.tform)
        return msg

    def to_lin_con_t(self):
        msg = encodeLinCon(self.A, self.b)
        return msg

    def xy_polytope(self):
        """
        Project the 3D c-space polytope Ax <= b to a list of vertices in the xy plane
        """
        V = lcon_to_vert(self.A, self.b)
        if V is not None and V.size > 0:
            hull = ConvexHull(V[:2,:].T)
            return V[:2,hull.vertices]
        else:
            # print "Infeasible polytope"
            return np.zeros((2,0))

    @staticmethod
    def from_iris_region_t(msg):
        A, b = decodeLinCon(msg.lin_con)
        C = []
        d = []
        tform = frameFromPositionMessage(msg.seed_pose)
        return SafeTerrainRegion(A, b, C, d, tform)

    @property
    def point(self):
        point = self.tform.GetPosition()
        print("point", point)
        return self.tform.GetPosition()

    @property
    def normal(self):
        return self.tform.TransformNormal(np.array([0,0,1]))


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
