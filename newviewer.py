import hashlib
import json
import threading
import time
import math
import sys
sys.path.append("build/install/lib/python2.7/site-packages")

import lcm
import bot_core


class Visualizer:
    def __init__(self, path, geometries):
        self.path = path
        self.geometries = geometries
        self.lcm = lcm.LCM()
        self.lcm.subscribe("DRAKE_VIEWER2_RESPONSE", self.onResponse)
        self.listener = threading.Thread(target=self.listen)
        self.listener.daemon = True
        self.listener.start()

    def listen(self):
        while True:
            self.lcm.handle_timeout(10)

    def load(self):
        link_data = bot_core.viewer2_link_data_t()
        link_data.num_geometries = 1
        link_data.geometries = self.geometries
        link_data.path = self.path

        load_msg = bot_core.viewer2_load_links_t()
        load_msg.num_links = 1
        load_msg.link_data = [link_data]
        self.lcm.publish("DRAKE_VIEWER2_LOAD_LINKS", load_msg.encode())

    def draw(self, pose):
        draw_msg = bot_core.viewer2_draw_t()
        draw_msg.num_links = 1
        draw_msg.poses = [pose]
        draw_msg.paths = [self.path]
        self.lcm.publish("DRAKE_VIEWER2_DRAW", draw_msg.encode())

    def onResponse(self, channel, data):
        msg = bot_core.viewer2_response_t.decode(data)
        if msg.status == msg.STATUS_OK:
            return
        elif msg.status == msg.STATUS_NO_SUCH_LINK:
            print "Loading model"
            self.load()
        else:
            print "Warning: unhandled failure: ", msg.status, json.loads(msg.json)


if __name__ == '__main__':
    geometry_data = bot_core.viewer_geometry_data_t()
    geometry_data.type = geometry_data.BOX
    geometry_data.position = [0, 0, 0]
    geometry_data.quaternion = [1, 0, 0, 0]
    geometry_data.color = [1, 0, 0, 0.5]
    geometry_data.num_float_data = 3
    geometry_data.float_data = [1, 1, 1]
    vis = Visualizer("robot1/link1", [geometry_data])
    while True:
        for i in range(1000):
            trans = bot_core.vector_3d_t()
            trans.x = math.sin(math.pi * 2 * i / 1000.0)
            quat = bot_core.quaternion_t()
            quat.w = 1
            pose = bot_core.position_3d_t()
            pose.translation = trans
            pose.rotation = quat
            vis.draw(pose)
            time.sleep(0.001)
