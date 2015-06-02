import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from ddapp import lcmUtils
from ddapp import applogic as app
from ddapp.utime import getUtime
from ddapp.timercallback import TimerCallback

import numpy as np
import math
from time import time
from copy import deepcopy

def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)

class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


class MultisensePanel(object):

    def __init__(self, multisenseDriver, neckDriver):

        self.multisenseDriver = multisenseDriver
        self.neckDriver = neckDriver
        self.neckPitchChanged = False
        self.multisenseChanged = False

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddMultisense.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)

        self.ui = WidgetDict(self.widget.children())

        self.updateTimer = TimerCallback(targetFps=2)
        self.updateTimer.callback = self.updatePanel
        self.updateTimer.start()

        self.widget.headCamGainSpinner.setEnabled(False)
        self.widget.headCamExposureSpinner.setEnabled(False)

        #connect the callbacks
        self.widget.neckPitchSpinner.valueChanged.connect(self.neckPitchChange)
        self.widget.spinRateSpinner.valueChanged.connect(self.spinRateChange)
        self.widget.scanDurationSpinner.valueChanged.connect(self.scanDurationChange)
        self.widget.headCamFpsSpinner.valueChanged.connect(self.headCamFpsChange)
        self.widget.headCamGainSpinner.valueChanged.connect(self.headCamGainChange)
        self.widget.headCamExposureSpinner.valueChanged.connect(self.headCamExposureChange)
        self.widget.headAutoGainCheck.clicked.connect(self.headCamAutoGainChange)
        self.widget.ledOnCheck.clicked.connect(self.ledOnCheckChange)
        self.widget.ledBrightnessSpinner.valueChanged.connect(self.ledBrightnessChange)

        self.widget.sendButton.clicked.connect(self.sendButtonClicked)

        self.updatePanel()


    def getCameraFps(self):
        return self.widget.headCamFpsSpinner.value

    def getCameraGain(self):
        return self.widget.headCamGainSpinner.value

    def getCameraExposure(self):
        return self.widget.headCamExposureSpinner.value

    def getCameraLedOn(self):
        return self.widget.ledOnCheck.isChecked()

    def getCameraLedBrightness(self):
        return self.widget.ledBrightnessSpinner.value

    def getCameraAutoGain(self):
        return self.widget.headAutoGainCheck.isChecked()

    def getSpinRate(self):
        return self.widget.spinRateSpinner.value

    def getScanDuration(self):
        return self.widget.scanDurationSpinner.value

    def getNeckPitch(self):
        return self.widget.neckPitchSpinner.value

    def ledBrightnessChange(self, event):
        self.multisenseChanged = True

    def ledOnCheckChange(self, event):
        self.multisenseChanged = True

    def headCamExposureChange(self, event):
        self.multisenseChanged = True

    def headCamAutoGainChange(self, event):
        self.multisenseChanged = True
        self.widget.headCamGainSpinner.setEnabled(not self.getCameraAutoGain())
        self.widget.headCamExposureSpinner.setEnabled(not self.getCameraAutoGain())

    def neckPitchChange(self, event):
        self.neckPitchChanged = True
        self.updateTimer.stop()

    def headCamFpsChange(self, event):
        self.multisenseChanged = True

    def headCamGainChange(self, event):
        self.multisenseChanged = True

    def spinRateChange(self, event):
        self.multisenseChanged = True
        spinRate = self.getSpinRate()

        if spinRate == 0.0:
            scanDuration = 240.0
        else:
            scanDuration = abs(60.0 / (spinRate * 2))
        if scanDuration > 240.0:
            scanDuration = 240.0

        self.widget.scanDurationSpinner.blockSignals(True)
        self.widget.scanDurationSpinner.value = scanDuration
        self.widget.scanDurationSpinner.blockSignals(False)

    def scanDurationChange(self, event):
        self.multisenseChanged = True
        scanDuration = self.getScanDuration()

        spinRate = abs(60.0 / (scanDuration * 2))

        self.widget.spinRateSpinner.blockSignals(True)
        self.widget.spinRateSpinner.value = spinRate
        self.widget.spinRateSpinner.blockSignals(False)


    def sendButtonClicked(self, event):
        self.publishCommand()

    def updatePanel(self):
        if not self.widget.isVisible():
            return

        if not self.neckPitchChanged:
            self.widget.neckPitchSpinner.blockSignals(True)
            self.widget.neckPitchSpinner.setValue(self.neckDriver.getNeckPitchDegrees())
            self.widget.neckPitchSpinner.blockSignals(False)

    def publishCommand(self):

        fps = self.getCameraFps()
        camGain = self.getCameraGain()
        exposure = 1000*self.getCameraExposure()
        ledFlash = self.getCameraLedOn()
        ledDuty = self.getCameraLedBrightness()
        spinRate = self.getSpinRate()
        autoGain = 1 if self.getCameraAutoGain() else 0

        self.multisenseDriver.sendMultisenseCommand(fps, camGain, exposure, autoGain, spinRate, ledFlash, ledDuty)
        self.neckDriver.setNeckPitch(self.getNeckPitch())

        self.multisenseChanged = False
        self.neckPitchChanged = False
        self.updateTimer.start()


def _getAction():
    return app.getToolBarActions()['ActionMultisensePanel']


def init(driver, neckDriver):

    global panel
    global dock

    panel = MultisensePanel(driver, neckDriver)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel
