import os
import sys
import time
from collections import OrderedDict

import PythonQt
import director
from director import mainwindowapp
from director import vtkAll as vtk
from director.simpletimer import FPSCounter
from director.timercallback import TimerCallback
from director import visualization as vis

import rospy
from std_msgs.msg import String, Bool


class RosSubscriberManager(object):
    """A helper class for managing rospy subscribers and dispatching callbacks."""

    def __init__(self):
        self.msgs = {}
        self.pending_msgs = {}
        self.counters = {}
        self.subs = OrderedDict()
        self.callbacks = OrderedDict()

    def init_node(self):
        rospy.init_node('director_rospy_node', anonymous=True)
        def on_idle():
            time.sleep(0.0001)
        self.idle_timer = TimerCallback(callback=on_idle, targetFps=100)
        self.idle_timer.start()

        self.dispatch_timer = TimerCallback(targetFps=30)
        self.dispatch_timer.callback = self.on_dispatch_timer
        self.dispatch_timer.start()

    def get_estimated_topic_hz(self, topic_name):
        assert topic_name in self.counters
        return self.counters[topic_name].getAverageFPS()

    def get_latest_msg(self, topic_name):
        return self.msgs.get(topic_name)

    def subscribe(self, topic_name, message_type, message_function=None, call_on_thread=False):
        def on_message(msg):
            self.msgs[topic_name] = msg
            self.counters[topic_name].tick()
            if call_on_thread and message_function:
                message_function(topic_name, msg)
            else:
                self.pending_msgs[topic_name] = msg

        self.counters[topic_name] = FPSCounter()
        self.callbacks[topic_name] = message_function
        self.subs[topic_name] = rospy.Subscriber(topic_name, message_type, on_message)

    def handle_pending_message(self, topic_name):
        msg = self.pending_msgs.pop(topic_name, None)
        if msg is not None:
            callback = self.callbacks.get(topic_name)
            if callback:
                callback(topic_name, msg)

    def handle_pending_messages(self):
        for topic_name in list(self.pending_msgs.keys()):
            self.handle_pending_message(topic_name)

    def on_dispatch_timer(self):
        self.handle_pending_messages()



def onVtkObject(name):
    obj = objectMap.take(name)
    if not obj:
        return

    if isinstance(obj, vtk.vtkPolyData):
        vis.updatePolyData(obj, name)


def loadRosPlugin():
    filename = os.path.join(os.path.dirname(director.__file__), '../../..', 'libddROSPlugin.so')
    if not os.path.exists(filename):
      raise Exception('plugin file not found: ' + filename)
    _pythonManager.loadPlugin(filename, 'init_ddROSPlugin')


def startRos():
    print 'starting ros c++ node'
    rosInterface.initNode()
    rosInterface.addPointcloudSubscriber('/pointcloud')
    rosInterface.addImageSubscriber('/image')


def onInitTimer():
    if not rosInterface.rosCheckMaster():
        return True
    startRos()

    # return False to stop the timer
    return False


if __name__ == '__main__':

    loadRosPlugin()

    rosInterface = PythonQt.ddROSPlugin.ddROSInterface()
    objectMap = rosInterface.vtkObjectMap()
    objectMap.connect('objectAssigned(const QString&)', onVtkObject)

    initTimer = TimerCallback(targetFps=3, callback=onInitTimer)
    initTimer.start()

    app = mainwindowapp.construct()
    app.app.start()
