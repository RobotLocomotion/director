import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from ddapp import lcmUtils
from ddapp import applogic as app
from ddapp.utime import getUtime
from ddapp import objectmodel as om
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp.timercallback import TimerCallback

import numpy as np
import math


def _makeButton(text, func):

    b = QtGui.QPushButton(text)
    b.connect('clicked()', func)
    return b

def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)

class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)

class FootstepsPanel(object):

    def __init__(self, driver, robotModel, jointController):

        self.driver = driver
        self.robotModel = robotModel
        self.jointController = jointController

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddFootsteps.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)

        self.ui = WidgetDict(self.widget.children())

        self.ui.walkingGoalButton.connect("clicked()", self.onNewWalkingGoal)
        self.ui.executeButton.connect("clicked()", self.onExecute)
        self.ui.stopButton.connect("clicked()", self.onStop)

        ### BDI frame logic
        self.ui.hideBDIButton.connect("clicked()", self.onHideBDIButton)
        self.ui.showBDIButton.connect("clicked()", self.onShowBDIButton)
        self._setupPropertiesPanel()

    def _setupPropertiesPanel(self):
        l = QtGui.QVBoxLayout(self.ui.paramsContainer)
        l.setMargin(0)
        propertiesPanel = PythonQt.dd.ddPropertiesPanel()
        propertiesPanel.setBrowserModeToWidget()
        om.addPropertiesToPanel(self.driver.params, propertiesPanel)
        l.addWidget(propertiesPanel)
        propertiesPanel.connect('propertyValueChanged(QtVariantProperty*)', self.onPropertyChanged)
        PythonQt.dd.ddGroupBoxHider(self.ui.paramsContainer)

    def onPropertyChanged(self, prop):
        self.driver.params.setProperty(prop.propertyName(), prop.value())
        self.driver.updateRequest()


    def newWalkingGoalFrame(self, robotModel, distanceForward=1.0):
        t = self.driver.getFeetMidPoint(robotModel)
        t = transformUtils.frameFromPositionAndRPY(t.GetPosition(), [0.0, 0.0, t.GetOrientation()[2]])
        t.PreMultiply()
        t.Translate(distanceForward, 0.0, 0.0)
        t.PostMultiply()
        return t


    def onNewWalkingGoal(self):

        t = self.newWalkingGoalFrame(self.robotModel)
        frameObj = vis.updateFrame(t, 'walking goal', parent='planning', scale=0.25)
        frameObj.setProperty('Edit', True)
        frameObj.connectFrameModified(self.onWalkingGoalModified)
        self.onWalkingGoalModified(frameObj)

    def onWalkingGoalModified(self, frame):

        request = self.driver.constructFootstepPlanRequest(self.jointController.q, frame.transform)
        self.driver.sendFootstepPlanRequest(request)

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

def init(driver, robotModel, jointController):

    global panel
    global dock

    panel = FootstepsPanel(driver, robotModel, jointController)
    dock = app.addWidgetToDock(panel.widget)
    #dock.hide()

    actionName = 'ActionFootstepPanel'
    action = app.getToolBarActions()[actionName]
    action.triggered.connect(toggleWidgetShow)


    return panel
