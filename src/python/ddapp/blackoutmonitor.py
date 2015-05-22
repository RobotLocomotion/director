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
    AVERAGE_N = 5
    def __init__(self, robotStateJointController, view, cameraview):

        self.robotStateJointController = robotStateJointController
        self.view = view
        self.cameraview = cameraview

        self.lastMessageTime = 0

        self.lastBlackoutLengths = []
        self.lastBlackoutLength = 0
        self.inBlackout = False
        self.averageBlackoutLength = 0.0

        self.txt = vis.updateText("DATA AGE: 0 sec", "Data Age Text", view=self.view)
        self.txt.addProperty('Show Avg Duration', False)
        self.txt.setProperty('Visible', False)

        self.updateTimer = TimerCallback(self.UPDATE_RATE)
        self.updateTimer.callback = self.update
        self.updateTimer.start()

    def update(self):
        self.lastMessageTime = self.cameraview.imageManager.queue.getCurrentImageTime('CAMERA_LEFT')
        if self.robotStateJointController.lastRobotStateMessage:
            elapsed = max((self.robotStateJointController.lastRobotStateMessage.utime - self.lastMessageTime) / (1000*1000), 0.0)
            # can't be deleted, only hidden, so this is ok
            if (self.txt.getProperty('Visible')):
                if (self.txt.getProperty('Show Avg Duration')):
                    textstr = "DATA AGE: %d / %d sec" % (math.floor(elapsed), math.floor(self.averageBlackoutLength))
                else:
                    textstr = "DATA AGE: %d sec" % math.floor(elapsed)
                ssize = self.view.size
                self.txt.setProperty('Text', textstr)
                self.txt.setProperty('Position', [10, 10])

            # count out blackouts
            if elapsed > 1.0:
                self.inBlackout = True
                self.lastBlackoutLength = elapsed
            else:
                if (self.inBlackout):
                    # Don't count huge time jumps due to init
                    if (self.lastBlackoutLength < 100000):
                        self.lastBlackoutLengths.append(self.lastBlackoutLength)
                    if len(self.lastBlackoutLengths) > self.AVERAGE_N:
                        self.lastBlackoutLengths.pop(0)
                    if len(self.lastBlackoutLengths) > 0:
                        self.averageBlackoutLength = sum(self.lastBlackoutLengths) / float(len(self.lastBlackoutLengths))
                    self.inBlackout = False

