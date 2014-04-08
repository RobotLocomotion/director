import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from ddapp import lcmUtils
from ddapp import applogic as app
from ddapp.utime import getUtime
from ddapp.timercallback import TimerCallback

import numpy as np
import math

import vtkAll as vtk

from time import time
from copy import deepcopy
from ddapp import transformUtils
import ddapp.visualization as vis
import ddapp.objectmodel as om
from ddapp import robotstate
from ddapp import botpy

import drc as lcmdrc

def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)

class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


class NavigationPanel(object):

    def __init__(self, jointController, footstepDriver, playbackRobotModel, playbackJointController):

        self.jointController = jointController
        self.footstepDriver = footstepDriver
        self.playbackRobotModel = playbackRobotModel
        self.playbackJointController = playbackJointController

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddNavigation.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)

        self.ui = WidgetDict(self.widget.children())

        self.ui.captureButton.connect("clicked()", self.onCaptureButton)
        self.ui.visualizeButton.connect("clicked()", self.onVisualizeButton)
        self.ui.planButton.connect("clicked()", self.onPlanButton)

        self.goal = dict()
        
        
    ###############################
    def getSelectedGoalName(self):
        goal_name = self.ui.comboBox.currentText
        return goal_name
    
    def getFrameFromCombo(self):
        pose = self.goal[ self.getSelectedGoalName() ]
        #frame = transformUtils.frameFromPositionAndRPY(pose[0:3], np.degrees(pose[3:6]) )
        return pose
    
    def printTransform(self,t,message):
        p = t.GetPosition()
        q = t.GetOrientation()
        print "%f %f %f | %f %f %f | %s" % (p[0],p[1],p[2],q[0],q[1],q[2],message)
    
    
    def onCaptureButton(self):
        print "capture" #,self.jointController.q
        # body frame:
        #goal = self.jointController.q[0:6]

        # mid point of feet (as used by robin)
        # this assumes model publisher exists - not always true currently
        model = om.findObjectByName("robot state model")
        t_feet_mid = self.footstepDriver.getFeetMidPoint(model)
        #vis.updateFrame(t_feet_mid, "Current Goal New", parent="navigation")

        goal_name = "Goal %d" % len(self.goal)
        self.goal[goal_name] = t_feet_mid
        self.updateComboBox()

    def updateComboBox(self):
        self.ui.comboBox.clear()
        self.ui.comboBox.addItems(self.goal.keys())

    def onVisualizeButton(self):
        print "viz",self.ui.comboBox.currentText
        frame = self.getFrameFromCombo()

        #vis.showFrame(frame, self.getSelectedGoalName(), parent="navigation", scale=0.35, visible=True)
        #vis.updateFrame(frame, self.getSelectedGoalName(), parent="navigation")
        vis.updateFrame(frame, "Current Goal", parent="navigation")

    def onPlanButton(self):
        print "plan",self.ui.comboBox.currentText

        goalFrame = self.getFrameFromCombo()
        startPose = self.jointController.q

        request = self.footstepDriver.constructFootstepPlanRequest(startPose, goalFrame)
        self.footstepDriver.sendFootstepPlanRequest(request)


def init(jointController, footstepDriver, playbackRobotModel, playbackJointController):

    global dock

    panel = NavigationPanel(jointController, footstepDriver, playbackRobotModel, playbackJointController)
    dock = app.addWidgetToDock(panel.widget)
    dock.hide()

    return panel

