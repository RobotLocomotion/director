import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from director import lcmUtils
from director import applogic as app
from director.utime import getUtime
from director.timercallback import TimerCallback
from director import visualization as vis
from director.debugVis import DebugData
from director.wristforcetorquevisualizer import WristForceTorqueVisualizer

import numpy as np
import math
from time import time
from time import sleep

def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)

class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


class HandControlPanel(object):

    def __init__(self, lDriver, rDriver, robotStateModel, robotStateJointController, view):

        self.robotStateModel = robotStateModel
        self.robotStateJointController = robotStateJointController
        self.drivers = {}
        self.drivers['left'] = lDriver
        self.drivers['right'] = rDriver

        self.storedCommand = {'left': None, 'right': None}

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddHandControl.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)

        self.ui = WidgetDict(self.widget.children())
        self._updateBlocked = True

        self.widget.advanced.sendButton.setEnabled(True)

        # Store calibration for wrist f/t sensors
        self.wristftvis = WristForceTorqueVisualizer(robotStateModel, robotStateJointController, view)

        # connect the callbacks
        self.widget.basic.openButton.clicked.connect(self.openClicked)
        self.widget.basic.closeButton.clicked.connect(self.closeClicked)
        self.widget.basic.waitOpenButton.clicked.connect(self.waitOpenClicked)
        self.widget.basic.waitCloseButton.clicked.connect(self.waitCloseClicked)
        self.widget.advanced.sendButton.clicked.connect(self.sendClicked)
        self.widget.advanced.calibrateButton.clicked.connect(self.calibrateClicked)
        self.widget.advanced.setModeButton.clicked.connect(self.setModeClicked)
        self.widget.advanced.regraspButton.clicked.connect(self.regraspClicked)
        self.widget.advanced.dropButton.clicked.connect(self.dropClicked)
        self.widget.advanced.repeatRateSpinner.valueChanged.connect(self.rateSpinnerChanged)
        self.ui.fingerControlButton.clicked.connect(self.fingerControlButton)

        self.widget.sensors.rightTareButton.clicked.connect(self.wristftvis.tareRightFT)
        self.widget.sensors.leftTareButton.clicked.connect(self.wristftvis.tareLeftFT)
        self.widget.sensors.rightCalibButton.clicked.connect(self.wristftvis.calibRightFT)
        self.widget.sensors.leftCalibButton.clicked.connect(self.wristftvis.calibLeftFT)
        self.widget.sensors.rightCalibClearButton.clicked.connect(self.wristftvis.calibRightClearFT)
        self.widget.sensors.leftCalibClearButton.clicked.connect(self.wristftvis.calibLeftClearFT)
        self.widget.sensors.rightVisCheck.clicked.connect(self.updateWristFTVis)
        self.widget.sensors.leftVisCheck.clicked.connect(self.updateWristFTVis)
        self.widget.sensors.torqueVisCheck.clicked.connect(self.updateWristFTVis)

        PythonQt.dd.ddGroupBoxHider(self.ui.sensors)
        PythonQt.dd.ddGroupBoxHider(self.ui.fingerControl)

        # Bug fix... for some reason one slider is set as disabled
        self.ui.fingerASlider.setEnabled(True)

        # create a timer to repeat commands
        self.updateTimer = TimerCallback()
        self.updateTimer.callback = self.updatePanel
        self.updateTimer.targetFps = 3
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

        position = 0.0
        force = float(self.widget.advanced.forcePercentSpinner.value)
        velocity = float(self.widget.advanced.velocityPercentSpinner.value)

        mode = self.getModeInt(self.widget.advanced.modeBox.currentText)

        self.drivers[side].sendCustom(position, force, velocity, mode)
        self.storedCommand[side] = (position, force, velocity, mode)

    def closeClicked(self):
        if self.widget.handSelect.leftButton.checked:
            side = 'left'
        else:
            side = 'right'

        self.widget.advanced.closePercentSpinner.setValue(100.0)

        position = 100.0
        force = float(self.widget.advanced.forcePercentSpinner.value)
        velocity = float(self.widget.advanced.velocityPercentSpinner.value)

        mode = self.getModeInt(self.widget.advanced.modeBox.currentText)

        self.drivers[side].sendCustom(position, force, velocity, mode)
        self.storedCommand[side] = (position, force, velocity, mode)

    def waitOpenClicked(self):
        sleep(10)
        self.openClicked()

    def waitCloseClicked(self):
        sleep(10)
        self.closeClicked()

    def sendClicked(self):
        if self.widget.handSelect.leftButton.checked:
            side = 'left'
        else:
            side = 'right'

        position = float(self.widget.advanced.closePercentSpinner.value)
        force = float(self.widget.advanced.forcePercentSpinner.value)
        velocity = float(self.widget.advanced.velocityPercentSpinner.value)

        mode = self.getModeInt(self.widget.advanced.modeBox.currentText)

        self.drivers[side].sendCustom(position, force, velocity, mode)
        self.storedCommand[side] = (position, force, velocity, mode)

    def setModeClicked(self):
        if self.widget.handSelect.leftButton.checked:
            side = 'left'
        else:
            side = 'right'

        mode = self.getModeInt(self.widget.advanced.modeBox.currentText)

        self.drivers[side].setMode(mode)
        self.storedCommand[side] = None

    def calibrateClicked(self):
        if self.widget.handSelect.leftButton.checked:
            side = 'left'
        else:
            side = 'right'

        self.drivers[side].sendCalibrate()
        self.storedCommand[side] = None

    def dropClicked(self):
        if self.widget.handSelect.leftButton.checked:
            side = 'left'
        else:
            side = 'right'

        self.drivers[side].sendDrop()
        self.storedCommand[side] = None

    def regraspClicked(self):
        if self.widget.handSelect.leftButton.checked:
            side = 'left'
        else:
            side = 'right'

        position = float(self.widget.advanced.closePercentSpinner.value)
        force = float(self.widget.advanced.forcePercentSpinner.value)
        velocity = float(self.widget.advanced.velocityPercentSpinner.value)

        mode = self.getModeInt(self.widget.advanced.modeBox.currentText)

        self.drivers[side].sendRegrasp(position, force, velocity, mode)
        self.storedCommand[side] = (position, force, velocity, mode)

    def rateSpinnerChanged(self):
        self.updateTimer.targetFps = self.ui.repeatRateSpinner.value

    def fingerControlButton(self):
        if self.widget.handSelect.leftButton.checked:
            side = 'left'
        else:
            side = 'right'

        if not self.ui.scissorControl.isChecked():
            self.drivers[side].sendFingerControl(int(self.ui.fingerAValue.text),
                                                 int(self.ui.fingerBValue.text),
                                                 int(self.ui.fingerCValue.text),
                                                 float(self.widget.advanced.forcePercentSpinner.value),
                                                 float(self.widget.advanced.velocityPercentSpinner.value),
                                                 None,
                                                 self.getModeInt(self.widget.advanced.modeBox.currentText))
        else:
            self.drivers[side].sendFingerControl(int(self.ui.fingerAValue.text),
                                                 int(self.ui.fingerBValue.text),
                                                 int(self.ui.fingerCValue.text),
                                                 float(self.widget.advanced.forcePercentSpinner.value),
                                                 float(self.widget.advanced.velocityPercentSpinner.value),
                                                 int(self.ui.scissorValue.text),
                                                 0)  # can ignore mode because scissor will override
        self.storedCommand[side] = None

    def updateWristFTVis(self):
        self.wristftvis.updateDrawSettings(self.ui.leftVisCheck.checked, self.ui.rightVisCheck.checked, self.ui.torqueVisCheck.checked);

    def updatePanel(self):
        if self.ui.repeaterCheckBox.checked and self.storedCommand['left']:
            position, force, velocity, mode = self.storedCommand['left']
            self.drivers['left'].sendCustom(position, force, velocity, mode)
        if self.ui.repeaterCheckBox.checked and self.storedCommand['right']:
            position, force, velocity, mode = self.storedCommand['right']
            self.drivers['right'].sendCustom(position, force, velocity, mode)



def _getAction():
    return app.getToolBarActions()['ActionHandControlPanel']


def init(driverL, driverR, model, jc, view):

    global panel
    global dock

    panel = HandControlPanel(driverL, driverR, model, jc, view)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel
