import drc as lcmdrc
from ddapp import lcmUtils
from ddapp import affordanceurdf


class RaycastDriver(object):
    def __init__(self, request_channel='TERRAIN_RAYCAST_REQUEST'):
        self.request_channel = request_channel

    def requestRaycast(self, affordances):
        urdfStr = affordanceurdf.urdfStringFromAffordances(affordances)
        msg = lcmdrc.terrain_raycast_request_t()
        msg.urdf = lcmdrc.robot_urdf_t()
        msg.urdf.robot_name = "collision_environment"
        msg.urdf.urdf_xml_string = urdfStr
        msg.x_min = -2
        msg.x_step = 0.02
        msg.x_max = 2
        msg.y_min = -2
        msg.y_step = 0.02
        msg.y_max = 2

        max_height = None
        for aff in affordances:
            bounds = aff.actor.GetBounds()
            zmax = bounds[-1]
            max_height = max(max_height, zmax)
        if max_height is not None:
            msg.scanner_height = 10 + max_height
        else:
            msg.scanner_height = 10
        lcmUtils.publish(self.request_channel, msg)
