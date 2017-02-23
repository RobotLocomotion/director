import time
import os
import subprocess

import numpy as np
import lcm
from director.thirdparty import transformations
from director.viewerclient import Visualizer, PolyLine, GeometryData


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

    vis = Visualizer("lines")


    vis["basic_line"].setgeometry(PolyLine([[0, 0, 0], [0, 0, 1]]))

    vis["arrow_end"].setgeometry(PolyLine([[.1, 0, 0], [.1, 0, 1]], end_head=True))

    vis["arrow_start"].setgeometry(PolyLine([[-.1, 0, 0], [-.1, 0, 1]], start_head=True))

    vis["loop"].setgeometry(PolyLine([[1, 0, 0], [1, 1, 0.5], [1, 0, 1]], closed=True))

    vis["poly_arrow"].setgeometry(PolyLine([[0, 1, 0], [0, 1, 1], [-1, 1, 1]], start_head=True, end_head=True))

    vis["colored_hairline"].setgeometry(GeometryData(PolyLine([[0, -1, 0], [0, -1, 1]], radius=0), color=[0, 0.2, 0.8, 0.5]))

    vis["sharp_arrow"].setgeometry(PolyLine([[-1, 0, 0.5], [-0.5, 0, 0.5]], end_head=True, head_length=0.2))

    vis_process.terminate()
