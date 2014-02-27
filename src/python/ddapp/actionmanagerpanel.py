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


class ActionManagerPanel(object):

    def __init__(self):

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddActionManager.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)

        self.ui = WidgetDict(self.widget.children())
        self._updateBlocked = True

        self.updateTimer = TimerCallback()
        self.updateTimer.callback = self.updatePanel
        self.updateTimer.start()

	self.updatePanel()

    def updatePanel(self):
    	return


def toggleWidgetShow():

    if dock.isVisible():
        dock.hide()
    else:
        dock.show()

def init():

    global dock

    panel = ActionManagerPanel()
    dock = app.addWidgetToDock(panel.widget)
    dock.hide()

    actionName = 'ActionActionManagerPanel'
    action = app.getToolBarActions()[actionName]
    action.triggered.connect(toggleWidgetShow)


    return panel
