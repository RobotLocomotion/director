from __future__ import division

import sys
import numpy as np
from scipy import ndimage
from scipy.spatial import ConvexHull
from scipy.io import loadmat, savemat

import drc
from irispy.cspace import rotmat
from irispy.iris import inflate_region
from irispy.utils import lcon_to_vert

DEFAULT_FOOT_CONTACTS = np.array([[-0.13, -0.13, 0.13, 0.13],
                                  [0.0562, -0.0562, 0.0562, -0.0562]])
DEFAULT_BOUNDING_BOX_WIDTH = 1
DEFAULT_CONTACT_SLICES = {(0.05, 0.35): np.array([[-0.13, -0.13, 0.13, 0.13],
                                          [0.0562, -0.0562, 0.0562, -0.0562]]),
                          (0.35, .75): np.array([[-0.13, -0.13, 0.25, 0.25],
                                          [.25, -.25, .25, -.25]]),
                          (0.75, 1.15): np.array([[-0.2, -0.2, 0.25, 0.25],
                                          [.4, -.4, .4, -.4]]),
                          (1.15, 1.85): np.array([[-0.35, -0.35, 0.25, 0.25],
                                          [.4, -.4, .4, -.4]])
                          }

def classify_terrain(heights, px2world):
    # heights[np.isnan(heights)] = np.min(heights) # TODO: make sure these are filled properly
    # TODO: actually fill in missing height data
    heights[np.isnan(heights)] = 0
    sx = ndimage.sobel(heights, axis=0, mode='constant')
    sy = ndimage.sobel(heights, axis=1, mode='constant')
    sob = np.hypot(sx, sy)
    # sob[np.isnan(sob)] = np.inf
    # sob[np.isnan(sob)] = 0
    edges = sob > 0.5 # TODO: maybe not just a magic constant?
    edges[np.isnan(heights)] = False
    feas = np.logical_not(edges)

    return feas


def terrain_edge_obs(feas, px2world_2x3):
    C, R = np.meshgrid(range(feas.shape[1]), range(feas.shape[0]))
    mask = np.logical_not(feas)
    cr = np.vstack((C[mask], R[mask]))
    obs_flat = px2world_2x3.dot(np.vstack((cr, np.ones(cr.shape[1]))))
    obs = obs_flat.reshape((2,1,-1))
    return obs

def terrain_body_obs(heights, px2world_2x3, pose, ht_range):
    # TODO: use a plane fit, rather than just a horizontal plane
    C, R = np.meshgrid(range(heights.shape[1]), range(heights.shape[0]))
    delta = heights - pose[2]
    obs_mask = np.logical_and(ht_range[0] <= delta, delta <= ht_range[1])
    cr = np.vstack((C[obs_mask], R[obs_mask]))
    obs_flat = px2world_2x3.dot(np.vstack((cr, np.ones(cr.shape[1]))))
    obs_flat = np.vstack((obs_flat, heights[obs_mask]))
    obs = obs_flat.reshape((3,1,-1))
    return obs


class TerrainSegmentation:
    def __init__(self, bot_pts=DEFAULT_FOOT_CONTACTS,
                 bounding_box_width=DEFAULT_BOUNDING_BOX_WIDTH,
                 contact_slices=DEFAULT_CONTACT_SLICES):
        self.bot_pts = bot_pts
        self.bounding_box_width = bounding_box_width
        self.contact_slices = contact_slices
        self.heights = None
        self.px2world = None
        self.feas = None
        self.edge_pts_xy = None
        self.obs_pts_xyz = None
        self.last_obs_mask = None

    def setHeights(self, heights, px2world=None):
        if px2world is not None:
            self.px2world = px2world
        C, R = np.meshgrid(range(heights.shape[1]), range(heights.shape[0]))
        self.heights = self.px2world[2,:].dot(np.vstack((C.flatten(), R.flatten(), heights.flatten(), np.ones((1,C.flatten().shape[0]))))).reshape(heights.shape)
        self.feas = classify_terrain(self.heights, self.px2world)
        self.edge_pts_xy = terrain_edge_obs(self.feas, self.px2world_2x3)

    def findSafeRegion(self, pose, **kwargs):
        start = pose[[0,1,5]] # x y yaw
        A_bounds, b_bounds = self.getBoundingPolytope(start)
        c_obs = self.getCObs(start, self.edge_pts_xy, A_bounds, b_bounds)

        for hts, bot in self.contact_slices.iteritems():
            body_obs_xyz = terrain_body_obs(self.heights, self.px2world_2x3, pose, hts)
            # print hts, body_obs_xyz
            if body_obs_xyz.size > 0:
                c_obs = np.dstack((c_obs, self.getCObs(start, body_obs_xyz[:2,:], A_bounds, b_bounds, bot=bot)))
                # b = body_obs_xyz
                # c = self.getCObs(start, body_obs_xyz[:2,:], A_bounds, b_bounds, bot=bot)
                # print c
                # print "=========="
                # import pdb; pdb.set_trace()

        A, b, C, d, results = inflate_region(c_obs, A_bounds, b_bounds, start, **kwargs)
        return SafeTerrainRegion(A, b, C, d, pose)

    def getBoundingPolytope(self, start):
        start = np.array(start).reshape((3,))
        lb = np.hstack((start[:2] - self.bounding_box_width / 2, start[2] - np.pi))
        ub = np.hstack((start[:2] + self.bounding_box_width / 2, start[2] + np.pi))
        A_bounds = np.vstack((-np.eye(3), np.eye(3)))
        b_bounds = np.hstack((-lb, ub))
        return A_bounds, b_bounds

    def getCObs(self, start, obs_pts, A_bounds, b_bounds, bot=None, theta_steps=8):

        start = np.array(start).reshape((3,))

        if bot is None:
            bot = self.bot_pts
        Ax = A_bounds.dot(np.vstack((obs_pts.reshape((2,-1)),
                                     start[2] + np.zeros(obs_pts.shape[1]*obs_pts.shape[2]))))
        obs_pt_mask = np.all(Ax - b_bounds.reshape((-1,1)) - np.max(np.abs(bot)) < 0,
                             axis=0).reshape(obs_pts.shape[1:])
        obs_mask = np.any(obs_pt_mask, axis=0)
        self.last_obs_mask = obs_mask

        # Fast c-obs computation for all point obstacles
        assert(obs_pts.shape[1] == 1)
        n_active_obs = np.sum(obs_mask)
        thetas = np.linspace(start[2]-np.pi, start[2]+np.pi, num=theta_steps)
        c_bot = -np.array(bot)
        c_obs = []
        for j, _ in enumerate(thetas[:-1]):
            th0 = thetas[j]
            th1 = thetas[j+1]
            c_bot0 = rotmat(th0).dot(c_bot)
            c_bot1 = rotmat(th1).dot(c_bot)
            c_obs0 = np.vstack((c_bot0.reshape(2,-1,1) + obs_pts[:,:,obs_mask],
                                th0 + np.zeros((1, c_bot0.shape[1], n_active_obs))))
            c_obs1 = np.vstack((c_bot1.reshape(2,-1,1) + obs_pts[:,:,obs_mask],
                                th1 + np.zeros((1, c_bot1.shape[1], n_active_obs))))
            c_obs.append(np.hstack((c_obs0, c_obs1)))
        return np.dstack(c_obs)


    @property
    def world2px(self):
        return np.linalg.inv(self.px2world)

    @property
    def world2px_2x3(self):
        return self.world2px[:2,[0,1,3]]

    @property
    def px2world_2x3(self):
        return self.px2world[:2,[0,1,3]]

class SafeTerrainRegion:
    __slots__ = ['A', 'b', 'C', 'd']
    def __init__(self, A, b, C, d, pose):
        self.A = A
        self.b = b
        self.C = C
        self.d = d
        self.pose = pose

    def to_iris_region_t(self):
        msg = drc.iris_region_t()
        msg.lin_con = self.to_lin_con_t()
        msg.point = self.pose[:3]
        print "Warning: normal not set"
        msg.normal = [0,0,1] # TODO: set normal from pose
        return msg

    def to_lin_con_t(self):
        msg = drc.lin_con_t()
        msg.m = self.A.shape[0]
        msg.n = self.A.shape[1]
        msg.m_times_n = msg.m * msg.n
        msg.A = self.A.flatten(order='F') # column-major
        msg.b = self.b
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


def mat_interface(fname):
    """
    Use a .mat file to take in the heights and seed pose, and then return the resulting A, b, C, d in the same file. This allows us to call this code from external programs like Matlab
    """
    matvars = loadmat(fname, mat_dtype=True)
    heights = matvars['heights']
    pose = np.reshape(matvars['pose'], (-1,))
    px2world = matvars['px2world']
    bounding_box_width = matvars['bounding_box_width'][0]
    t = TerrainSegmentation(bounding_box_width=bounding_box_width)
    t.setHeights(heights, px2world)
    r = t.findSafeRegion(pose, iter_limit=2, require_containment=True)
    savemat(fname, {'A': r.A,
                    'b': r.b,
                    'C': r.C,
                    'd': r.d,
                    'V': r.xy_polytope()})

if __name__ == '__main__':
    mat_interface(sys.argv[1])
