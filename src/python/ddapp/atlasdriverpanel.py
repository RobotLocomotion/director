import PythonQt
from PythonQt import QtCore, QtGui
from ddapp import lcmUtils
from ddapp import applogic as app
from ddapp.utime import getUtime
from ddapp.timercallback import TimerCallback

import numpy as np
import math


def _makeButton(text, func):

    b = QtGui.QPushButton(text)
    b.connect('clicked()', func)
    return b





class AtlasDriverPanel(object):

    def __init__(self, driver):

        self.driver = driver

        self.widget = QtGui.QWidget()
        self.widget.setWindowTitle('Atlas Driver')

        l = QtGui.QVBoxLayout(self.widget)

        l.addWidget(_makeButton('calibrate encoders', self.onCalibrateEncoders))
        l.addWidget(_makeButton('calibrate bdi', self.onCalibrateBdi))
        l.addWidget(_makeButton('prep', self.onPrep))
        l.addWidget(_makeButton('user', self.onUser))
        l.addWidget(QtGui.QLabel(''))
        l.addWidget(_makeButton('stand', self.onStand))
        l.addWidget(_makeButton('manip', self.onManip))
        l.addWidget(QtGui.QLabel(''))
        l.addWidget(_makeButton('freeze', self.onFreeze))
        l.addWidget(_makeButton('stop', self.onStop))
        l.addWidget(QtGui.QLabel(''))

        self.behaviorLabel = QtGui.QLabel('current behavior: <unknown>')
        l.addWidget(self.behaviorLabel)
        l.addStretch()

        self.updateTimer = TimerCallback()
        self.updateTimer.callback = self.updatePanel
        self.updateTimer.start()

        self.updatePanel()

    def updatePanel(self):
        self.behaviorLabel.text = 'current behavior: %s' % self.driver.getCurrentBehaviorName()


    def onFreeze(self):
        self.driver.sendFreezeCommand()

    def onStop(self):
        self.driver.sendStopCommand()

    def onCalibrateEncoders(self):
        self.driver.sendCalibrateEncodersCommand()

    def onCalibrateBdi(self):
        self.driver.sendCalibrateCommand()

    def onPrep(self):
        self.driver.sendPrepCommand()

    def onStand(self):
        self.driver.sendStandCommand()

    def onManip(self):
        self.driver.sendManipCommand()

    def onUser(self):
        self.driver.sendUserCommand()



def init(driver):

    global panel
    global dock

    panel = AtlasDriverPanel(driver)
    dock = app.addWidgetToDock(panel.widget)
    #dock.hide()

    return panel
