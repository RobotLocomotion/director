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
    AVERAGE_N = 10
    def __init__(self, robotStateJointController, view, cameraview):

        self.robotStateJointController = robotStateJointController
        self.view = view
        self.cameraview = cameraview

        self.lastMessageTime = 0

        self.lastBlackoutLengths = []
        self.lastBlackoutLength = 0
        self.inBlackout = False
        self.averageBlackoutLength = 30.0

        self.updateTimer = TimerCallback(self.UPDATE_RATE)
        self.updateTimer.callback = self.update
        self.updateTimer.start()

    def update(self):
        #self.lastMessageTime = self.cameraview.imageManager.queue.getCurrentImageTime('CAMERA_LEFT')
        if self.robotStateJointController.lastRobotStateMessage:
            elapsed = (self.robotStateJointController.lastRobotStateMessage.utime - self.lastMessageTime) / (1000*1000)
            if elapsed > 1.0:
                self.inBlackout = True
                self.lastBlackoutLength = elapsed
                # blackout!
                ssize = self.view.size
                textstr = "BLACKOUT: %3.2f / %3.2f sec" % (elapsed, self.averageBlackoutLength)
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
                if (self.inBlackout):
                    if (self.lastBlackoutLength < 100000):
                        self.lastBlackoutLengths.append(self.lastBlackoutLength)
                    if len(self.lastBlackoutLengths) > self.AVERAGE_N:
                        self.lastBlackoutLengths.pop(0)
                    if len(self.lastBlackoutLengths) > 0:
                        self.averageBlackoutLength = sum(self.lastBlackoutLengths) / float(len(self.lastBlackoutLengths))
                    self.inBlackout = False
