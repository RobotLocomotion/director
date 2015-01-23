from __future__ import division

import numpy as np
import drc as lcmdrc
from ddapp import transformUtils
from ddapp import irisUtils
from ddapp import lcmUtils
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

    def requestIRISRegion(self, tform, callback, bounding_box_width=2):
        start = np.asarray(tform.GetPosition())
        A_bounds, b_bounds = self.getXYBounds(start, bounding_box_width)
        msg = lcmdrc.iris_region_request_t()
        msg.utime = getUtime();
        msg.view_id = lcmdrc.data_request_t.HEIGHT_MAP_SCENE
        msg.map_id = self.depth_provider.reader.GetCurrentMapId(msg.view_id)
        msg.num_seed_poses = 1
        msg.seed_poses = [transformUtils.positionMessageFromFrame(tform)]
        msg.xy_bounds = [irisUtils.encodeLinCon(A_bounds, b_bounds)]
        lcmUtils.publish(self.request_channel, msg)
        self.callback = callback


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

