import sys
print sys.path

import bot_core as lcmbot
import lcm
import time
import math
import threading
import subprocess

def rectangle_geometry_msg():
    msg = lcmbot.viewer_geometry_data_t()
    msg.type = msg.BOX
    msg.position = [0, 0, 0]
    msg.quaternion = [1, 0, 0, 0]
    msg.color = [1, 0, 0, 0.5]
    msg.string_data = ""
    msg.float_data = [1.0, 0.5, 0.5]
    msg.num_float_data = 3
    return msg

def sphere_geometry_msg():
    msg = lcmbot.viewer_geometry_data_t()
    msg.type = msg.SPHERE
    msg.position = [0, 0, 0]
    msg.quaternion = [1, 0, 0, 0]
    msg.color = [0, 1, 0, 0.5]
    msg.string_data = ""
    msg.float_data = [0.5]
    msg.num_float_data = 1
    return msg

def robot_load_msg():
    rectangle = rectangle_geometry_msg()
    sphere = sphere_geometry_msg()
    links = []
    for (i, geometry) in enumerate((rectangle, sphere)):
        link = lcmbot.viewer_link_data_t()
        link.name = "link_{:d}".format(i)
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
    start = time.time()
    lc = lcm.LCM()
    while True:
        elapsed = time.time() - start
        rectangle_x = math.sin(elapsed)
        sphere_z = math.cos(elapsed)

        draw_msg = lcmbot.viewer_draw_t()
        draw_msg.timestamp = int(elapsed * 1e9)
        draw_msg.num_links = 2
        draw_msg.link_name = ["link_0", "link_1"]
        draw_msg.robot_num = [1, 1]
        draw_msg.position = [[rectangle_x, 0, 0], [0, 0, sphere_z]]
        draw_msg.quaternion = [[1, 0, 0, 0], [1, 0, 0, 0]]
        lc.publish("DRAKE_VIEWER_DRAW", draw_msg.encode())

        if elapsed > 10:
            break

def publish_robot_data():
    print "spawning robot"
    spawn_robot()

    print "animating robot"
    animate_robot()

def main():
    vis_process = subprocess.Popen("drake-visualizer")
    print "waiting for viewer to initialize"
    lc = lcm.LCM()
    lc.subscribe("DRAKE_VIEWER_STATUS", lambda c, d: None)
    lc.handle()

    publish_robot_data()
    vis_process.terminate()

if __name__ == '__main__':
    main()
