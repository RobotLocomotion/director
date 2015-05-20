import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
import ddapp.objectmodel as om
from ddapp import lcmUtils
from ddapp import applogic as app
from ddapp.utime import getUtime
from ddapp.timercallback import TimerCallback
from ddapp import visualization as vis
import multisense as lcmmultisense
import numpy as np
import math
from time import time
from time import sleep

class BlackoutMonitor(object):
    UPDATE_RATE = 5
    def __init__(self, robotStateJointController):

        self.robotStateJointController = robotStateJointController

        lcmUtils.addSubscriber('SCAN', lcmmultisense.planar_lidar_t, self.resetCounter)

        self.last_message_time = 0

        self.updateTimer = TimerCallback(self.UPDATE_RATE)
        self.updateTimer.callback = self.update
        self.updateTimer.start()

    def resetCounter(self, msg):
        self.last_message_time = msg.utime

    def update(self):
        if self.robotStateJointController.lastRobotStateMessage:
            elapsed = (self.robotStateJointController.lastRobotStateMessage.utime - self.last_message_time) / (1000*1000)
            if elapsed > 1.0:
                # blackout!
                vis.updateText("BLACKOUT: " + str(elapsed), "blackout text", fontSize=24, position=(500, 50)).setProperty('Bold', True)
                om.findObjectByName('blackout text').setProperty('Visible', True)
            else:
                # not blackout!
                om.findObjectByName('blackout text').setProperty('Visible', False)