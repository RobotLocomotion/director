import hashlib
import json
import threading
import time
import math
import sys
sys.path.append("build/install/lib/python2.7/site-packages")

import lcm
import bot_core


# def to_pose_3d(trans, quat):
#     msg = bot_core.position_3d_t()
#     msg.translation = bot_core.vector_3d_t()
#     msg.translation.x, msg.translation.y, msg.translation.z = trans
#     msg.rotation = bot_core.quaternion_t()
#     msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z = quat
#     return msg


def comms_msg(timestamp, data):
    msg = bot_core.viewer2_comms_t()
    msg.format = "viewer2_json"
    msg.format_version_major = 1
    msg.format_version_minor = 0
    encoded = json.dumps(data)
    msg.num_bytes = len(encoded)
    msg.data = encoded
    return msg


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
        timestamp = 0
        data = {
            "timestamp": timestamp,
            "type": "load",
            "data": {
                "links": [
                    {
                        "path": self.path,
                        "geometries": self.geometries
                    }
                ]
            }
        }
        msg = comms_msg(timestamp, data)
        self.lcm.publish("DRAKE_VIEWER2_REQUEST", msg.encode())

    def draw(self, pose):
        timestamp = 0
        data = {
            "timestamp": timestamp,
            "type": "draw",
            "data": {
                "commands": [
                    {
                        "path": self.path,
                        "pose": pose
                    }
                ]
            }
        }
        msg = comms_msg(timestamp, data)
        self.lcm.publish("DRAKE_VIEWER2_REQUEST", msg.encode())

    def onResponse(self, channel, raw_data):
        msg = bot_core.viewer2_comms_t.decode(raw_data)
        print "response data:", msg.data
        data = json.loads(msg.data)
        print "decoded"
        if data["status"] == 0:
            return
        elif data["status"] == 1:
            print "Loading model"
            self.load()
        else:
            print "Warning: unhandled failure: ", data


if __name__ == '__main__':
    geometry = {
        "name": "box",
        "type": "box",
        "pose": {
            "translation": [0, 0, 0],
            "quaternion": [1, 0, 0, 0]
        },
        "color": [1, 0, 0, 0.5],
        "parameters": {
            "lengths": [1, 1, 1]
        }
    }
    # geometry_data = bot_core.viewer2_geometry_data_t()
    # geometry_data.name = "box"
    # geometry_data.pose = to_pose_3d([0, 0, 0], [1, 0, 0, 0])
    # geometry_data.color = [1.0, 0, 0, 0.5]
    # geometry_data.format = "json"
    # geometry_data.format_version = 1

    # geom = {
    #     "type": "box",
    #     "parameters": {
    #         "lengths": [1, 1, 1]
    #     }
    # }
    # data = json.dumps(geom)
    # geometry_data.num_bytes = len(data)
    # geometry_data.data = data
    # geometry_data = bot_core.viewer_geometry_data_t()
    # geometry_data.type = geometry_data.BOX
    # geometry_data.position = [0, 0, 0]
    # geometry_data.quaternion = [1, 0, 0, 0]
    # geometry_data.color = [1, 0, 0, 0.5]
    # geometry_data.num_float_data = 3
    # geometry_data.float_data = [1, 1, 1]
    vis = Visualizer(["robot1", "link1"], [geometry])
    # vis.load()
    while True:
        for i in range(1000):
            x = math.sin(math.pi * 2 * i / 1000.0)
            pose = {
                "translation": [x, 0, 0],
                "quaternion": [1, 0, 0, 0]
            }
            vis.draw(pose)
            time.sleep(0.001)
