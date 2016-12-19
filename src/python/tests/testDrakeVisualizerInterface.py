# import the director module first to setup python paths
# which may be required on certain platforms to find the
# bot_core lcmtypes python module which is imported next.
# see https://github.com/RobotLocomotion/libbot/issues/20
import director
import bot_core as lcmbot
import lcm
import time
import math
import threading
import subprocess
import os
import sys

"""
This script demonstrates interacting with the drake-visualizer application
from an external script over LCM . It doesn't require the directorPython
executable; instead, it can be run from a standard Python interpreter.

To demonstrate the visualizer, we construct a robot consisting of two links.
Each link has a single geometry attached: a rectangular prism for the first
and a sphere for the second. To spawn a model of the robot in the visualizer
app, we publish a viewer_load_robot_t() LCM message. To draw the robot in a
particular configuration, we publish a viewer_draw_t() message with the
position and orientation of each link in the robot.
"""

def rectangle_geometry_msg():
    """
    Constructs a viewer_geometry_data_t describing a red rectangular prism
    """
    msg = lcmbot.viewer_geometry_data_t()
    msg.type = msg.BOX

    # Position and orientation of the center of the rectangle relative to the origin of the link which contains it:
    msg.position = [0, 0, 0]
    msg.quaternion = [1, 0, 0, 0]

    msg.color = [1, 0, 0, 0.5] # R G B A
    msg.string_data = ""
    msg.float_data = [1.0, 0.5, 0.5] # Length along x, y, z axes
    msg.num_float_data = 3
    return msg

def sphere_geometry_msg():
    """
    Constructs a viewer_geometry_data_t() describing a single green sphere.
    """
    msg = lcmbot.viewer_geometry_data_t()
    msg.type = msg.SPHERE

    # Position and orientation of the center of the sphere relative to the origin of the link which contains it:
    msg.position = [0, 0, 0]
    msg.quaternion = [1, 0, 0, 0]

    msg.color = [0, 1, 0, 0.5] # R G B A
    msg.string_data = ""
    msg.float_data = [0.5] # Radius
    msg.num_float_data = 1
    return msg

def robot_load_msg():
    """
    Constructs the viewer_load_robot_t() message to spawn our particular
    robot. To do that, it constructs a viewer_link_data_t() message for each
    link. Each viewer_link_data_t itself contains a description of the
    geometry attached to that link.
    """
    rectangle = rectangle_geometry_msg()
    sphere = sphere_geometry_msg()
    links = []
    for (i, geometry) in enumerate((rectangle, sphere)):
        link = lcmbot.viewer_link_data_t()
        link.name = "link_{:d}".format(i)

        # The visualizer can distinguish multiple robots by giving them
        # different robot_num values. We'll attach both of our links to robot
        # 1.
        link.robot_num = 1
        link.num_geom = 1
        link.geom = [geometry]
        links.append(link)
    load_msg = lcmbot.viewer_load_robot_t()
    load_msg.num_links = 2
    load_msg.link = links
    return load_msg

def spawn_robot():
    lc = lcm.LCM()
    lc.publish("DRAKE_VIEWER_LOAD_ROBOT", robot_load_msg().encode())

def animate_robot():
    """
    Animate the robot by publishing lcm messages to move the two links.
    """
    start = time.time()
    animationTime = 2.0
    lc = lcm.LCM()
    while True:
        elapsed = time.time() - start
        rectangle_x = math.sin(elapsed)
        sphere_z = math.cos(elapsed)

        # The draw message includes the name, robot_num, position, and
        # orientation of the origin of each link to be drawn. The link names
        # and robot numbers must match those given in the viewer_load_robot_t
        # message
        draw_msg = lcmbot.viewer_draw_t()
        draw_msg.timestamp = int(elapsed * 1e6)
        draw_msg.num_links = 2
        draw_msg.link_name = ["link_0", "link_1"]
        draw_msg.robot_num = [1, 1]
        draw_msg.position = [[rectangle_x, 0, 0], [0, 0, sphere_z]]
        draw_msg.quaternion = [[1, 0, 0, 0], [1, 0, 0, 0]]
        lc.publish("DRAKE_VIEWER_DRAW", draw_msg.encode())

        if elapsed > animationTime:
            break

def publish_robot_data():
    print "spawning robot"
    spawn_robot()

    print "animating robot"
    animate_robot()

def main():
    # We'll open the visualizer by spawning it as a subprocess. See
    # testDrakeVisualizer.py for an example of how to spawn it within Python
    # instead.
    vis_binary = os.path.join(os.path.dirname(sys.executable),
                              "drake-visualizer")
    print "vis_binary:", vis_binary
    vis_process = subprocess.Popen(vis_binary)

    # The viewer will take some time to load before it is ready to receive
    # messages, so we'll wait until it sends its first status message.
    print "waiting for viewer to initialize"
    lc = lcm.LCM()
    lc.subscribe("DRAKE_VIEWER_STATUS", lambda c, d: None)
    # Wait for one LCM message to be received.
    lc.handle()

    # Load and animate the robot
    publish_robot_data()

    # Shut down the viewer.
    vis_process.terminate()

if __name__ == '__main__':
    main()
