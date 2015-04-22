from __future__ import division

import numpy as np
from scipy.spatial import ConvexHull

from ddapp.irisUtils import SafeTerrainRegion
from ddapp import transformUtils
from irispy.utils import sample_convex_polytope
import polyhedron._cdd
from polyhedron import Vrep, Hrep
from py_drake_utils.utils import rpy2rotmat




DEFAULT_FOOT_CONTACTS = np.array([[-0.13, -0.13, 0.13, 0.13],
                                  [0.0562, -0.0562, 0.0562, -0.0562]])
DEFAULT_BOUNDING_BOX_WIDTH = 1


class PolygonSegmentationNonIRIS():
    def __init__(self, polygon_vertices, bot_pts=DEFAULT_FOOT_CONTACTS,
                 bounding_box_width=DEFAULT_BOUNDING_BOX_WIDTH):

        polygon_vertices = np.asarray(polygon_vertices)
        self.planar_polyhedron = Vrep(polygon_vertices[:2,:].T)
        self.bot_pts = bot_pts

    def getBoundingPolytope(self, start):
        """
        Return A, b describing a bounding box on [x, y, yaw] into which the IRIS region must be contained.
        The format is A [x;y;yaw] <= b
        """
        start = np.array(start).reshape((3,))
        lb = np.hstack((start[:2] - self.bounding_box_width / 2, start[2] - np.pi))
        ub = np.hstack((start[:2] + self.bounding_box_width / 2, start[2] + np.pi))
        A_bounds = np.vstack((-np.eye(3), np.eye(3)))
        b_bounds = np.hstack((-lb, ub))
        return A_bounds, b_bounds

    def findSafeRegion(self, pose):
        pose = np.asarray(pose)
        tformForProjection = transformUtils.frameFromPositionAndRPY([0,0,0], pose[3:] * 180 / np.pi)
        tform = transformUtils.frameFromPositionAndRPY(pose[:3], pose[3:] * 180 / np.pi)

        contact_pts_on_plane = np.zeros((2, self.bot_pts.shape[1]))
        for j in range(self.bot_pts.shape[1]):
            contact_pts_on_plane[:,j] = tformForProjection.TransformPoint([self.bot_pts[0,j], self.bot_pts[1,j], 0])[:2]

        Rdot = np.array([[0, -1], [1, 0]])
        contact_vel_in_world = Rdot.dot(contact_pts_on_plane)

        c_region = {'A': [], 'b': []}

        for i in range(self.planar_polyhedron.A.shape[0]):
            ai = self.planar_polyhedron.A[i,:]
            n = np.linalg.norm(ai)
            ai = ai / n
            bi = self.planar_polyhedron.b[i] / n

            p = ai.dot(contact_pts_on_plane)
            v = ai.dot(contact_vel_in_world)

            mask = np.logical_or(p >= 0, v >= 0)
            for j, tf in enumerate(mask):
                if tf:
                    c_region['A'].append(np.hstack((ai, v[j])))
                    c_region['b'].append([bi - p[j]])

        A = np.vstack(c_region['A'])
        b = np.hstack(c_region['b'])

        b = b + A.dot(np.array([0,0,pose[5]]))

        self.c_space_polyhedron = Hrep(A, b)
        return SafeTerrainRegion(A, b, [], [], tform)

    def drawSamples(self, nsamples):
        import matplotlib.pyplot as plt
        plt.figure(1)
        plt.clf()
        plt.hold(True)
        k = ConvexHull(self.bot_pts.T).vertices
        k = np.hstack((k, k[0]))
        n = self.planar_polyhedron.generators.shape[0]
        plt.plot(self.planar_polyhedron.generators.T[0,range(n) + [0]],
                 self.planar_polyhedron.generators.T[1,range(n) + [0]], 'r.-')
        samples = sample_convex_polytope(self.c_space_polyhedron.A,
                                         self.c_space_polyhedron.b,
                                         500)
        for i in range(samples.shape[1]):
            R = np.array([[np.cos(samples[2,i]), -np.sin(samples[2,i])],
                          [np.sin(samples[2,i]), np.cos(samples[2,i])]])
            V = R.dot(self.bot_pts[:,k])
            V = V + samples[:2, i].reshape((2,1))
            plt.plot(V[0,:], V[1,:], 'k-')
        plt.show()


def get_point_and_normal(pose):
    point = pose[:3]
    normal = rpy2rotmat(pose[3:]).dot([0,0,1])
    return point, normal

