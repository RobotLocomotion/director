import json
import threading
import time
import math
import sys
import numpy as np
sys.path.append("build/install/lib/python2.7/site-packages")

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
    def __init__(self, geometries={}):
        self.geometries = {}
        self.poses = {}
        self.queue = {"load": [], "draw": [], "delete": []}
        for (path, geom) in geometries.items():
            self.load(path, geom)
        self.lcm = lcm.LCM()
        self.lcm.subscribe("DRAKE_VIEWER2_RESPONSE", self.onResponse)
        self.listener = threading.Thread(target=self.listen)
        self.listener.daemon = True
        self.listener.start()

    def listen(self):
        while True:
            self.lcm.handle_timeout(10)

    def publish(self):
        timestamp = 0
        data = {
            "timestamp": timestamp,
            "delete": self.queue["delete"],
            "load": self.queue["load"],
            "draw": self.queue["draw"]
        }
        msg = comms_msg(timestamp, data)
        self.lcm.publish("DRAKE_VIEWER2_REQUEST", msg.encode())
        self.queue["load"] = []
        self.queue["delete"] = []
        self.queue["draw"] = []

    def load(self, path, geometry):
        self.geometries[path] = geometry
        self.queue["load"].append({
            "path": path.split("/"),
            "geometry": geometry
        })

    def draw(self, path, pose):
        self.poses[path] = pose
        self.queue["draw"].append({
            "path": path.split("/"),
            "transform": pose
        })

    def delete(self, path):
        if path in self.poses:
            del self.poses[path]
        if path in self.geometries:
            del self.geometries[path]
        self.queue["delete"].append({
            "path": path.split("/")
        })

    def onResponse(self, channel, raw_data):
        msg = bot_core.viewer2_comms_t.decode(raw_data)
        response = json.loads(msg.data)
        if response["status"] == 0:
            print("ok")
        elif response["status"] == 1:
            for path, geom in self.geometries.items():
                self.load(path, geom)
            for path, pose in self.poses.items():
                self.draw(path, pose)
        else:
            print("unhandled:", response)

if __name__ == '__main__':
    geometries = {
        "robot1/link1/box1": {
            "type": "box",
            "color": [0, 1, 0, 0.5],
            "lengths": [1, 1, 1]
        },
        "robot1/link1/box2": {
            "type": "box",
            "color": [0, 0, 1, 0.5],
            "lengths": [1, 1, 1]
        },
        "robot1/link1/points": {
            "type": "pointcloud",
            "points": [[0, 0, 2 + x / 100.] for x in range(100)],
            "channels": {
                "rgb": [[x / 100., 1 - x / 100., x / 100.] for x in range(100)]
            }
        },
        "robot1/link1/planar lidar": {
            "type": "planar_lidar",
            "angle_start": -np.pi/2,
            "angle_step": np.pi / 100,
            "ranges": [1 for i in range(100)],
            "channels": {
                "intensity": [i / 100. for i in range(100)]
            }
        }
    }
    vis = Visualizer(geometries)

    vis.draw("robot1/link1/box2", {"translation": [1, 0, 0], "quaternion": [1, 0, 0, 0]})
    vis.draw("robot1/link1/points", {"translation": [0, 1, 0], "quaternion": [1, 0, 0, 0]})
    vis.draw("robot1/link1/planar lidar", {"translation": [0, 2, 0], "quaternion": [1, 0, 0, 0]})
    try:
        while True:
            for i in range(1000):
                x1 = math.sin(math.pi * 2 * i / 1000.0)
                pose = {
                    "translation": [x1, 0, 0],
                    "quaternion": [1, 0, 0, 0]
                }
                vis.draw("robot1/link1", pose)

                x2 = math.sin(math.pi * 2 * i / 500.0)
                pose = {
                    "translation": [x2, 0, 0],
                    "quaternion": [1, 0, 0, 0]
                }
                vis.draw("robot1/link1/box1", pose)

                vis.publish()
                time.sleep(0.001)
    except:
        # print "deleting"
        # paths = vis.geometries.keys()
        # for path in paths:
        #     vis.delete(path)
        # vis.publish()
        raise

