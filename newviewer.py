import json
import threading
import time
import math
import sys
import numpy as np
sys.path.append("build/install/lib/python2.7/dist-packages")

import lcm
import bot_core


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
        print "Loading model"
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

    def delete(self):
        timestamp = 0
        data = {
            "timestamp": timestamp,
            "type": "delete",
            "data": {
                "paths": [self.path]
            }
        }
        msg = comms_msg(timestamp, data)
        self.lcm.publish("DRAKE_VIEWER2_REQUEST", msg.encode())

    def onResponse(self, channel, raw_data):
        msg = bot_core.viewer2_comms_t.decode(raw_data)
        data = json.loads(msg.data)
        if data["status"] == 0:
            return
        elif data["status"] == 1:
            self.load()
        else:
            print "Warning: unhandled failure: ", data


if __name__ == '__main__':
    geometries = [{
            "name": "box",
            "pose": {
                "translation": [0, 0, 0],
                "quaternion": [1, 0, 0, 0]
            },
            "parameters": {
                "color": [0, 1, 0, 0.5],
                "type": "box",
                "lengths": [1, 1, 1]
            },
        },
        {
            "name": "box2",
            "pose": {
                "translation": [1, 0, 0],
                "quaternion": [1, 0, 0, 0]
            },
            "parameters": {
                "color": [0, 0, 1, 0.5],
                "type": "box",
                "lengths": [1, 1, 1]
            }
        },
        {
            "name": "points",
            "pose": {
                "translation": [1, 0, 0],
                "quaternion": [1, 0, 0, 0]
            },
            "parameters": {
                "type": "pointcloud",
                "points": [[0, 0, 2 + x / 100.] for x in range(100)],
                "channels": {
                    "rgb": [[x / 100., 1 - x / 100., x / 100.] for x in range(100)]
                }
            }
        },
        {
            "name": "planar lidar",
            "pose": {
                "translation": [0, 2, 0],
                "quaternion": [1, 0, 0, 0]
            },
            "parameters": {
                "type": "planar_lidar",
                "angle_start": -np.pi/2,
                "angle_step": np.pi / 100,
                "ranges": [1 for i in range(100)],
                "channels": {
                    "intensity": [i / 100. for i in range(100)]
                }
            }
        },
        ]
    vis = Visualizer(["robot1", "link1"], geometries)
    vis.load()
    try:
        while True:
            for i in range(1000):
                x = math.sin(math.pi * 2 * i / 1000.0)
                pose = {
                    "translation": [x, 0, 0],
                    "quaternion": [1, 0, 0, 0]
                }
                vis.draw(pose)
                time.sleep(0.001)
    except:
        # vis.delete()
        raise

