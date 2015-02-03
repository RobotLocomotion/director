from __future__ import division

import numpy as np
import drc as lcmdrc
from ddapp import applogic as app
from ddapp import objectmodel as om
from ddapp import transformUtils
from ddapp import irisUtils
from ddapp import lcmUtils
from ddapp.terrainitem import TerrainRegionItem
from ddapp.utime import getUtime


class IRISDriver(object):
    def __init__(self, depth_provider,
                 callback=None,
                 request_channel='IRIS_REGION_REQUEST',
                 response_channel='IRIS_REGION_RESPONSE'):
        self.depth_provider = depth_provider
        self.request_channel = request_channel
        self.response_channel = response_channel
        self.callback = callback
        self.sub = lcmUtils.addSubscriber(self.response_channel, lcmdrc.iris_region_response_t, self.onIRISRegionResponse)
        self.segmentation_sub = lcmUtils.addSubscriber('IRIS_SEGMENTATION_RESPONSE', lcmdrc.iris_region_response_t, self.onIRISSegmentation)

    def newIRISRegion(self, tform, region=None):
        view = app.getCurrentRenderView()
        item = TerrainRegionItem('IRIS region', view, tform, self, region)
        parentObj = om.getOrCreateContainer('Safe terrain regions')
        om.addToObjectModel(item, parentObj)

    def requestIRISRegion(self, tform, callback, bounding_box_width=2):
        start = np.asarray(tform.GetPosition())
        A_bounds, b_bounds = self.getXYBounds(start, bounding_box_width)
        msg = lcmdrc.iris_region_request_t()
        msg.utime = getUtime()
        msg.view_id = lcmdrc.data_request_t.HEIGHT_MAP_SCENE
        msg.map_id = self.depth_provider.reader.GetCurrentMapId(msg.view_id)
        msg.num_seed_poses = 1
        msg.seed_poses = [transformUtils.positionMessageFromFrame(tform)]
        msg.xy_bounds = [irisUtils.encodeLinCon(A_bounds, b_bounds)]
        lcmUtils.publish(self.request_channel, msg)
        self.callback = callback

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
        msg.view_id = lcmdrc.data_request_t.HEIGHT_MAP_SCENE
        msg.map_id = self.depth_provider.reader.GetCurrentMapId(msg.view_id)
        A = np.vstack((np.eye(2), -np.eye(2)))
        b = np.hstack((xy_ub, -np.asarray(xy_lb)))
        msg.xy_bounds = irisUtils.encodeLinCon(A, b)

        msg.num_seed_poses = 0

        msg.max_num_regions = max_num_regions
        msg.default_yaw = default_yaw
        msg.max_slope_angle = max_slope_angle
        msg.max_height_variation = max_height_variation
        msg.plane_distance_tolerance = plane_distance_tolerance
        msg.plane_angle_tolerance = plane_angle_tolerance
        lcmUtils.publish('AUTO_IRIS_SEGMENTATION_REQUEST', msg)

    def onIRISSegmentation(self, msg):
        for region_msg in msg.iris_regions:
            region = irisUtils.SafeTerrainRegion.from_iris_region_t(region_msg)
            tform = transformUtils.getTransformFromOriginAndNormal(region.point, region.normal)
            self.newIRISRegion(tform, region)


    def getXYBounds(self, start, bounding_box_width=2):
        start = np.asarray(start)
        lb = start[:2] - bounding_box_width/2
        ub = start[:2] + bounding_box_width/2
        A = np.vstack((-np.eye(2), np.eye(2)))
        b = np.hstack((-lb, ub))
        return A, b

    def onIRISRegionResponse(self, msg):
        if self.callback is not None:
            self.callback(irisUtils.SafeTerrainRegion.from_iris_region_t(msg.iris_regions[0]))

