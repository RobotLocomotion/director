import time
import os
import subprocess

import numpy as np
import lcm
from director.thirdparty import transformations
from director.viewerclient import Visualizer, Box, GeometryData, Sphere


if __name__ == '__main__':
    # We'll open the visualizer by spawning it as a subprocess. See
    # testDrakeVisualizer.py for an example of how to spawn it within Python
    # instead.
    vis_binary = os.path.join(os.path.dirname(sys.executable),
                              "drake-visualizer")

    # The viewer will take some time to load before it is ready to receive
    # messages, so we'll wait until it sends its first status message.
    print("waiting for viewer to initialize")
    lc = lcm.LCM()
    lc.subscribe("DIRECTOR_TREE_VIEWER_RESPONSE", lambda c, d: None)
    vis_process = subprocess.Popen([vis_binary, '--testing', '--interactive'])

    # Wait for one LCM message to be received.
    lc.handle()

    # We can provide an initial path if we want
    vis = Visualizer(path="/root/folder1")

    vis["boxes"].setgeometry(
        [GeometryData(Box([1, 1, 1]),
                      color=np.hstack((np.random.rand(3), 0.5)),
                      transform=transformations.translation_matrix([x, -2, 0]))
         for x in range(10)])

    # Index into the visualizer to get a sub-tree. vis.__getitem__ is lazily
    # implemented, so these sub-visualizers come into being as soon as they're
    # asked for
    vis = vis["group1"]

    box_vis = vis["box"]
    sphere_vis = vis["sphere"]

    box = Box([1, 1, 1])
    geom = GeometryData(box, color=[0, 1, 0, 0.5])
    box_vis.setgeometry(geom)

    sphere_vis.setgeometry(Sphere(1.0))
    sphere_vis.settransform(transformations.translation_matrix([1, 0, 0]))

    for theta in np.linspace(0, 2 * np.pi, 100):
        vis.settransform(transformations.rotation_matrix(theta, [0, 0, 1]))
        time.sleep(0.01)
    sphere_vis.delete()

    vis_process.terminate()
