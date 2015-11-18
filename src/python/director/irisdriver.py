from __future__ import division

import numpy as np
import drc as lcmdrc
from ddapp import applogic as app
from ddapp import objectmodel as om
from ddapp import transformUtils
from ddapp import irisUtils
from ddapp import lcmUtils
from ddapp import robotstate
from ddapp.terrainitem import TerrainRegionItem
from ddapp.utime import getUtime


class IRISDriver(object):
    def __init__(self, jointController,
                 footstepParams,
                 request_channel='IRIS_REGION_REQUEST',
                 response_channel='IRIS_REGION_RESPONSE'):
        self.jointController = jointController
        self.params = footstepParams
        self.map_mode_map = [
                             lcmdrc.footstep_plan_params_t.FOOT_PLANE,
                             lcmdrc.footstep_plan_params_t.TERRAIN_HEIGHTS_AND_NORMALS,
                             lcmdrc.footstep_plan_params_t.TERRAIN_HEIGHTS_Z_NORMALS,
                             lcmdrc.footstep_plan_params_t.HORIZONTAL_PLANE
                             ]
        self.request_channel = request_channel
        self.response_channel = response_channel
        self.regions = {}
        self.sub = lcmUtils.addSubscriber(self.response_channel, lcmdrc.iris_region_response_t, self.onIRISRegionResponse)
        self.segmentation_sub = lcmUtils.addSubscriber('IRIS_SEGMENTATION_RESPONSE', lcmdrc.iris_region_response_t, self.onIRISRegionResponse)

    def getNewUID(self):
        return max(self.regions.keys() + [0]) + 1

    def getNewUIDs(self, num_ids=1):
        return [max(self.regions.keys() + [0]) + 1 + i for i in range(num_ids)]

    def newTerrainItem(self, tform, uid=None, region=None):
        if uid is None:
            uid = self.getNewUID()
        elif uid in self.regions:
            return self.regions[uid]

        view = app.getCurrentRenderView()
        item = TerrainRegionItem(uid, view, tform, self, region)
        parentObj = om.getOrCreateContainer('Safe terrain regions')
        om.addToObjectModel(item, parentObj)
        self.regions[uid] = item
        return item

    def requestIRISRegion(self, tform, uid, bounding_box_width=2):
        start = np.asarray(tform.GetPosition())
        A_bounds, b_bounds = self.getXYBounds(start, bounding_box_width)
        msg = lcmdrc.iris_region_request_t()
        msg.utime = getUtime()
        msg.initial_state = robotstate.drakePoseToRobotState(self.jointController.q)
        msg.map_mode = self.map_mode_map[self.params.properties.map_mode]
        msg.num_seed_poses = 1
        msg.seed_poses = [transformUtils.positionMessageFromFrame(tform)]
        msg.region_id = [uid]
        msg.xy_bounds = [irisUtils.encodeLinCon(A_bounds, b_bounds)]
        lcmUtils.publish(self.request_channel, msg)

    def onIRISRegionResponse(self, msg):
        for i in range(msg.num_iris_regions):
            new_region = irisUtils.SafeTerrainRegion.from_iris_region_t(msg.iris_regions[i])
            uid = msg.region_id[i]
            if uid not in self.regions:
                tform = new_region.tform
                item = self.newTerrainItem(tform, uid, new_region)
            else:
                item = self.regions[uid]
                item.setRegion(new_region)

    def autoIRISSegmentation(self,
                             xy_lb=[-2, -2],
                             xy_ub=[2, 2],
                             max_num_regions=10,
                             default_yaw=np.nan,
                             max_slope_angle=np.nan,
                             max_height_variation=np.nan,
                             plane_distance_tolerance=np.nan,
                             plane_angle_tolerance=np.nan):
        msg = lcmdrc.auto_iris_segmentation_request_t()
        msg.utime = getUtime()
        A = np.vstack((np.eye(2), -np.eye(2)))
        b = np.hstack((xy_ub, -np.asarray(xy_lb)))
        msg.xy_bounds = irisUtils.encodeLinCon(A, b)

        msg.initial_state = robotstate.drakePoseToRobotState(self.jointController.q)
        msg.map_mode = self.map_mode_map[self.params.properties.map_mode]

        msg.num_seed_poses = 0

        msg.max_num_regions = max_num_regions
        msg.region_id = self.getNewUIDs(max_num_regions)
        msg.default_yaw = default_yaw
        msg.max_slope_angle = max_slope_angle
        msg.max_height_variation = max_height_variation
        msg.plane_distance_tolerance = plane_distance_tolerance
        msg.plane_angle_tolerance = plane_angle_tolerance
        lcmUtils.publish('AUTO_IRIS_SEGMENTATION_REQUEST', msg)

    def getXYBounds(self, start, bounding_box_width=2):
        start = np.asarray(start)
        lb = start[:2] - bounding_box_width/2
        ub = start[:2] + bounding_box_width/2
        A = np.vstack((-np.eye(2), np.eye(2)))
        b = np.hstack((-lb, ub))
        return A, b
