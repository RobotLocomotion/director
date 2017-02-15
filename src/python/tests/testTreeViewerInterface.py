import director
import os
import subprocess
import json
import threading
import time
import math
import sys
import numpy as np
import lcm
import robotlocomotion as lcmrl


def comms_msg(timestamp, data):
    msg = lcmrl.viewer2_comms_t()
    msg.format = "treeviewer_json"
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
        self.queue = {"setgeometry": [], "settransform": [], "delete": []}
        for (path, geom) in geometries.items():
            self.setgeometry(path, geom)
        self.lcm = lcm.LCM()
        self.lcm.subscribe("DIRECTOR_TREE_VIEWER_RESPONSE_<foo>", self.onResponse)
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
            "setgeometry": self.queue["setgeometry"],
            "settransform": self.queue["settransform"]
        }
        msg = comms_msg(timestamp, data)
        self.lcm.publish("DIRECTOR_TREE_VIEWER_REQUEST_<foo>", msg.encode())
        self.queue["setgeometry"] = []
        self.queue["delete"] = []
        self.queue["settransform"] = []

    def setgeometry(self, path, geometry):
        self.geometries[path] = geometry
        self.queue["setgeometry"].append({
            "path": path.split("/"),
            "geometry": geometry
        })

    def settransform(self, path, pose):
        self.poses[path] = pose
        self.queue["settransform"].append({
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
        msg = lcmrl.viewer2_comms_t.decode(raw_data)
        response = json.loads(msg.data)
        if response["status"] == 0:
            print("ok")
        elif response["status"] == 1:
            for path, geom in self.geometries.items():
                self.setgeometry(path, geom)
            for path, pose in self.poses.items():
                self.settransform(path, pose)
        else:
            print("unhandled:", response)

if __name__ == '__main__':
    # We'll open the visualizer by spawning it as a subprocess. See
    # testDrakeVisualizer.py for an example of how to spawn it within Python
    # instead.
    vis_binary = os.path.join(os.path.dirname(sys.executable),
                              "drake-visualizer")

    # The viewer will take some time to load before it is ready to receive
    # messages, so we'll wait until it sends its first status message.
    print "waiting for viewer to initialize"
    lc = lcm.LCM()
    lc.subscribe("DIRECTOR_TREE_VIEWER_RESPONSE", lambda c, d: None)
    vis_process = subprocess.Popen([vis_binary, '--testing', '--interactive'])

    # Wait for one LCM message to be received.
    lc.handle()

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
        },
        "triads/triad1": {
            "type": "triad"
        }
    }
    vis = Visualizer(geometries)

    vis.settransform("robot1/link1/box2", {"translation": [1, 0, 0], "quaternion": [1, 0, 0, 0]})
    vis.settransform("robot1/link1/points", {"translation": [0, 1, 0], "quaternion": [1, 0, 0, 0]})
    vis.settransform("robot1/link1/planar lidar", {"translation": [0, 2, 0], "quaternion": [1, 0, 0, 0]})
    for j in range(2):
        for i in range(1000):
            x1 = math.sin(math.pi * 2 * i / 1000.0)
            pose = {
                "translation": [x1, 0, 0],
                "quaternion": [1, 0, 0, 0]
            }
            vis.settransform("robot1/link1", pose)

            x2 = math.sin(math.pi * 2 * i / 500.0)
            pose = {
                "translation": [x2, 0, 0],
                "quaternion": [1, 0, 0, 0]
            }
            vis.settransform("robot1/link1/box1", pose)

            vis.publish()
            time.sleep(0.001)

    vis_process.terminate()
