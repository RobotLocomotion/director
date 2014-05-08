import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from ddapp import lcmUtils
from ddapp import applogic as app
from ddapp.utime import getUtime
from ddapp.timercallback import TimerCallback

import numpy as np
import math
from time import time

def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)

class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


class HandControlPanel(object):

    def __init__(self, lDriver, rDriver):

        self.drivers = {}
        self.drivers['left'] = lDriver
        self.drivers['right'] = rDriver

        self.storedCommand = False
        self.position = None
        self.force = None
        self.velocity = None
        self.mode = None

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddHandControl.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)

        self.ui = WidgetDict(self.widget.children())
        self._updateBlocked = True

        self.widget.advanced.sendButton.setEnabled(True)

        # connect the callbacks
        self.widget.basic.openButton.clicked.connect(self.openClicked)
        self.widget.basic.closeButton.clicked.connect(self.closeClicked)
        self.widget.advanced.sendButton.clicked.connect(self.sendClicked)
        self.widget.advanced.calibrateButton.clicked.connect(self.calibrateClicked)
        self.widget.advanced.setModeButton.clicked.connect(self.setModeClicked)

        # create a timer to repeat commands
        self.updateTimer = TimerCallback()
        self.updateTimer.callback = self.updatePanel
        self.updateTimer.targetFps = 5
        self.updateTimer.start()


    def getModeInt(self, inputStr):
        if inputStr == 'Basic':
            return 0
        if inputStr == 'Pinch':
            return 1
        if inputStr == 'Wide':
            return 2
        if inputStr == 'Scissor':
            return 3
        return 0

    def openClicked(self):
        if self.widget.handSelect.leftButton.checked:
            side = 'left'
        else:
            side = 'right'

        self.widget.advanced.closePercentSpinner.setValue(0.0)

        self.position = 0.0
        self.force = float(self.widget.advanced.forcePercentSpinner.value)
        self.velocity = float(self.widget.advanced.velocityPercentSpinner.value)

        self.mode = self.getModeInt(self.widget.advanced.modeBox.currentText)

        self.drivers[side].sendCustom(self.position, self.force, self.velocity, self.mode)
        self.storedCommand = True

    def closeClicked(self):
        if self.widget.handSelect.leftButton.checked:
            side = 'left'
        else:
            side = 'right'

        self.widget.advanced.closePercentSpinner.setValue(100.0)

        self.position = 100.0
        self.force = float(self.widget.advanced.forcePercentSpinner.value)
        self.velocity = float(self.widget.advanced.velocityPercentSpinner.value)

        self.mode = self.getModeInt(self.widget.advanced.modeBox.currentText)

        self.drivers[side].sendCustom(self.position, self.force, self.velocity, self.mode)
        self.storedCommand = True

    def sendClicked(self):
        if self.widget.handSelect.leftButton.checked:
            side = 'left'
        else:
            side = 'right'

        self.position = float(self.widget.advanced.closePercentSpinner.value)
        self.force = float(self.widget.advanced.forcePercentSpinner.value)
        self.velocity = float(self.widget.advanced.velocityPercentSpinner.value)

        self.mode = self.getModeInt(self.widget.advanced.modeBox.currentText)

        self.drivers[side].sendCustom(self.position, self.force, self.velocity, self.mode)
        self.storedCommand = True

    def setModeClicked(self):
        if self.widget.handSelect.leftButton.checked:
            side = 'left'
        else:
            side = 'right'

        self.mode = self.getModeInt(self.widget.advanced.modeBox.currentText)

        self.drivers[side].setMode(self.mode)

    def calibrateClicked(self):
        if self.widget.handSelect.leftButton.checked:
            side = 'left'
        else:
            side = 'right'

        self.drivers[side].sendCalibrate()

    def updatePanel(self):
        if self.ui.repeaterCheckBox.checked and self.storedCommand:
            self.drivers[side].sendCustom(self.position, self.force, self.velocity, self.mode)


def _getAction():
    return app.getToolBarActions()['ActionHandControlPanel']


def init(driverL, driverR):

    global panel
    global dock

    panel = HandControlPanel(driverL, driverR)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel
