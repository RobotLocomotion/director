import PythonQt
from PythonQt import QtCore, QtGui
from ddapp import lcmUtils
from ddapp import applogic as app
from ddapp.utime import getUtime
from ddapp import objectmodel as om
from ddapp.timercallback import TimerCallback

import numpy as np
import math


def _makeButton(text, func):

    b = QtGui.QPushButton(text)
    b.connect('clicked()', func)
    return b


def getDefaultRobotModel():
    return om.findObjectByName('robot state model')


class FootstepsPanel(object):

    def __init__(self, driver):

        self.driver = driver

        self.widget = QtGui.QWidget()
        self.widget.setWindowTitle('Footsteps Panel')

        l = QtGui.QVBoxLayout(self.widget)

        l.addWidget(_makeButton('new walking goal', self.onNewWalkingGoal))
        l.addWidget(QtGui.QLabel(''))
        l.addWidget(_makeButton('goal steps', self.onGoalSteps))
        l.addWidget(QtGui.QLabel(''))
        l.addWidget(_makeButton('execute footstep plan', self.onExecute))
        l.addWidget(QtGui.QLabel(''))
        l.addWidget(_makeButton('stop walking', self.onStop))
        l.addStretch()

    def onNewWalkingGoal(self):
        model = getDefaultRobotModel()
        self.driver.createWalkingGoal(model)

    def onGoalSteps(self):
        model = getDefaultRobotModel()
        self.driver.createGoalSteps(model)

    def onExecute(self):
        self.driver.commitFootstepPlan(self.driver.lastFootstepPlan)

    def onStop(self):
        self.driver.sendStopWalking()

def toggleWidgetShow():

    if dock.isVisible():
        dock.hide()
    else:
        dock.show()

def init(driver):

    global panel
    global dock

    panel = FootstepsPanel(driver)
    dock = app.addWidgetToDock(panel.widget)
    dock.hide()

    actionName = 'ActionFootstepPanel'
    action = app.getToolBarActions()[actionName]
    action.triggered.connect(toggleWidgetShow)


    return panel
