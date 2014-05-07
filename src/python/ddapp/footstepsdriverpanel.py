import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from ddapp import lcmUtils
from ddapp import applogic as app
from ddapp.utime import getUtime
from ddapp import objectmodel as om
from ddapp import transformUtils
from ddapp import roboturdf
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
        self.ui.walkingPlanButton.connect("clicked()", self.onShowWalkingPlan)
        self.ui.BDIExecuteButton.connect("clicked()", self.onBDIExecute)
        self.ui.drakeExecuteButton.connect("clicked()", self.onDrakeExecute)
        self.ui.stopButton.connect("clicked()", self.onStop)
        self.ui.BDIDefaultsButton.connect("clicked()", lambda: self.applyDefaults('BDI'))
        self.ui.drakeDefaultsButton.connect("clicked()", lambda: self.applyDefaults('drake'))

        ### BDI frame logic
        self.ui.hideBDIButton.connect("clicked()", self.onHideBDIButton)
        self.ui.showBDIButton.connect("clicked()", self.onShowBDIButton)
        self._setupPropertiesPanel()

    def _setupPropertiesPanel(self):
        l = QtGui.QVBoxLayout(self.ui.paramsContainer)
        l.setMargin(0)
        self.propertiesPanel = PythonQt.dd.ddPropertiesPanel()
        self.propertiesPanel.setBrowserModeToWidget()
        om.PropertyPanelHelper.addPropertiesToPanel(self.driver.params.properties, self.propertiesPanel)
        l.addWidget(self.propertiesPanel)
        self.propertiesPanel.connect('propertyValueChanged(QtVariantProperty*)', self.onPropertyChanged)
        PythonQt.dd.ddGroupBoxHider(self.ui.paramsContainer)

    def onPropertyChanged(self, prop):
        self.driver.params.setProperty(prop.propertyName(), prop.value())
        self.driver.updateRequest()

    def applyDefaults(self, set_name):
        for k, v in self.driver.default_step_params[set_name].iteritems():
            self.driver.params.setProperty(k, v)
            om.PropertyPanelHelper.onPropertyValueChanged(self.propertiesPanel, self.driver.params.properties, k)

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

    def onBDIExecute(self):
        self.driver.commitFootstepPlan(self.driver.lastFootstepPlan)

    def onDrakeExecute(self):
        startPose = self.jointController.getPose('EST_ROBOT_STATE')
        self.driver.sendWalkingControllerRequest(self.driver.lastFootstepPlan, startPose, waitForResponse=True)

    def onShowWalkingPlan(self):
        startPose = self.jointController.getPose('EST_ROBOT_STATE')
        self.driver.sendWalkingPlanRequest(self.driver.lastFootstepPlan, startPose, waitForResponse=True)

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

def _getAction():
    return app.getToolBarActions()['ActionFootstepPanel']

def init(driver, robotModel, jointController):

    global panel
    global dock

    panel = FootstepsPanel(driver, robotModel, jointController)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel
