import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from ddapp import lcmUtils
from ddapp import applogic as app
from ddapp.utime import getUtime
from ddapp.timercallback import TimerCallback

import numpy as np
import math


def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)

class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


class AtlasStatusPanel(object):

    def __init__(self, driver):

        self.driver = driver

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddRobotStatus.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)

        self.ui = WidgetDict(self.widget.children())
        self._updateBlocked = True

        self.updateTimer = TimerCallback()
        self.updateTimer.callback = self.updatePanel
        self.updateTimer.start()

        self.updatePanel()


    def updatePanel(self):
        self.widget.behaviorValue.text = self.driver.getCurrentBehaviorName()

        self.widget.inletPressureValue.value = self.driver.getCurrentInletPressure()
        self.widget.supplyPressureValue.value = self.driver.getCurrentSupplyPressure()
        self.widget.returnPressureValue.value = self.driver.getCurrentReturnPressure()

        self.widget.sumpPressureValue.value = self.driver.getCurrentAirSumpPressure()

        self.widget.pumpRpmValue.value = self.driver.getCurrentPumpRpm()


def init(driver):


    box = AtlasStatusPanel(driver)

    box.widget.show()

