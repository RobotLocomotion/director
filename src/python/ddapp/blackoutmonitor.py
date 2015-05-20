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
    def __init__(self, robotStateJointController, view, cameraview):

        self.robotStateJointController = robotStateJointController
        self.view = view
        self.cameraview = cameraview

        self.last_message_time = 0

        self.updateTimer = TimerCallback(self.UPDATE_RATE)
        self.updateTimer.callback = self.update
        self.updateTimer.start()

    def update(self):
        self.last_message_time = self.cameraview.imageManager.queue.getCurrentImageTime('CAMERA_LEFT')
        if self.robotStateJointController.lastRobotStateMessage:
            elapsed = (self.robotStateJointController.lastRobotStateMessage.utime - self.last_message_time) / (1000*1000)
            if elapsed > 1.0:
                # blackout!
                ssize = self.view.size
                textstr = "BLACKOUT: " + str(elapsed) + " sec"
                txt = vis.updateText(textstr, "blackout text", view=self.view)
                txt.setProperty('Font Size', 24)
                txt.setProperty('Position', (25, ssize.height()-50))
                txt.setProperty('Bold', True)
                txt.setProperty('Visible', True)
            else:
                # not blackout!
                txt = om.findObjectByName('blackout text')
                if (txt):
                    txt.setProperty('Visible', False)
