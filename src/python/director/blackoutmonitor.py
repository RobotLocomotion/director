import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
import director.objectmodel as om
from director import lcmUtils
from director import applogic as app
from director.utime import getUtime
from director.timercallback import TimerCallback
from director import visualization as vis
import multisense as lcmmultisense
import numpy as np
import math
from time import time
from time import sleep

class BlackoutMonitor(object):
    UPDATE_RATE = 5
    AVERAGE_N = 5
    def __init__(self, robotStateJointController, view, cameraview, mapServerSource):

        self.robotStateJointController = robotStateJointController
        self.view = view
        self.cameraview = cameraview
        self.mapServerSource = mapServerSource

        self.lastCameraMessageTime = 0
        self.lastScanBundleMessageTime = 0

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
        self.lastCameraMessageTime = self.cameraview.imageManager.queue.getCurrentImageTime('MULTISENSE_CAMERA_LEFT')
        self.lastScanBundleMessageTime = self.mapServerSource.reader.GetLastScanBundleUTime()
        if self.robotStateJointController.lastRobotStateMessage:
            elapsedCam = max((self.robotStateJointController.lastRobotStateMessage.utime - self.lastCameraMessageTime) / (1000*1000), 0.0)
            elapsedScan = max((self.robotStateJointController.lastRobotStateMessage.utime - self.lastScanBundleMessageTime) / (1000*1000), 0.0)
            # can't be deleted, only hidden, so this is ok
            if (self.txt.getProperty('Visible')):
                if (self.txt.getProperty('Show Avg Duration')):
                    textstr = "CAM  AGE: %02d sec\nSCAN AGE: %02d sec    AVG: %02d sec" % (math.floor(elapsedCam), math.floor(elapsedScan), math.floor(self.averageBlackoutLength))
                else:
                    textstr = "CAM  AGE: %02d sec\nSCAN AGE: %02d sec" % (math.floor(elapsedCam), math.floor(elapsedScan))
                ssize = self.view.size
                self.txt.setProperty('Text', textstr)
                self.txt.setProperty('Position', [10, 10])

            # count out blackouts
            if elapsedCam > 1.0:
                self.inBlackout = True
                self.lastBlackoutLength = elapsedCam
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

