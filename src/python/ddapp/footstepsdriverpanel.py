import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
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

def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)
    
class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)    
    
class FootstepsPanel(object):

    def __init__(self, driver):

        self.driver = driver

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddFootsteps.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)

        ui = WidgetDict(self.widget.children())

        ui.walkingGoalButton.connect("clicked()", self.onNewWalkingGoal)
        ui.goalStepsButton.connect("clicked()", self.onGoalSteps)
        ui.executeButton.connect("clicked()", self.onExecute)
        ui.stopButton.connect("clicked()", self.onStop)       
        
        ### BDI frame logic        
        ui.hideBDIButton.connect("clicked()", self.onHideBDIButton)
        ui.showBDIButton.connect("clicked()", self.onShowBDIButton)


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
        
        
    ### BDI frame logic    
    def onHideBDIButton(self):
        print "hide bdi"
        self.driver.showBDIPlan = False
        self.driver.bdiRobotModel.setProperty('Visible', False)
        folder = om.getOrCreateContainer("BDI footstep plan")
        om.removeFromObjectModel(folder)
        folder = om.getOrCreateContainer("BDI adj footstep plan")
        om.removeFromObjectModel(folder)

    def onShowBDIButton(self):
        print "show bdi"
        self.driver.showBDIPlan = True
        self.driver.bdiRobotModel.setProperty('Visible', True)
        self.driver.drawBDIFootstepPlan()
        self.driver.drawBDIFootstepPlanAdjusted()            

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
