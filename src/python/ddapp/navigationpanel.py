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
from bot_core.pose_t import pose_t
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
        self.ui.executeButton.connect("clicked()", self.onExecuteButton)
        self.ui.hideBDIButton.connect("clicked()", self.onHideBDIButton)
        self.ui.showBDIButton.connect("clicked()", self.onShowBDIButton)

        self.goal = dict()

        lcmUtils.addSubscriber('POSE_BDI', pose_t, self.onPoseBDI)
        lcmUtils.addSubscriber('POSE_BODY', pose_t, self.onPoseBody)
        lcmUtils.addSubscriber('FOOTSTEP_PLAN_RESPONSE', lcmdrc.footstep_plan_t, self.onFootStepPlanResponse)
        self.pose_bdi = None
        self.pose_body = None
        self.bdi_plan = None

        sub = lcmUtils.addSubscriber('EST_ROBOT_STATE_BDI', lcmdrc.robot_state_t, self.onERSBDI)
        sub.setSpeedLimit(60)

        self.playbackRobotModel.setProperty('Visible', True)
        self.showBDIPlan = True # show the BDI plans when created



    #############################
    def onPoseBDI(self,msg):
        self.pose_bdi = msg
        
    def onPoseBody(self,msg):
        self.pose_body = msg

    def onERSBDI(self,msg):
        pose = robotstate.convertStateMessageToDrakePose(msg)
        self.playbackJointController.setPose("ERS BDI", pose)
        

    def onFootStepPlanResponse(self,msg):
        self.transformPlanToBDIFrame(msg)


        
    #################################
    def printTransform(self,t,message):
        p = t.GetPosition()
        q = t.GetOrientation()
        print "%f %f %f | %f %f %f | %s" % (p[0],p[1],p[2],q[0],q[1],q[2],message)

    def getSelectedGoalName(self):
        goal_name = self.ui.comboBox.currentText
        return goal_name

    def getFrameFromCombo(self):
        pose = self.goal[ self.getSelectedGoalName() ]
        #frame = transformUtils.frameFromPositionAndRPY(pose[0:3], np.degrees(pose[3:6]) )

        return pose

    def transformPlanToBDIFrame(self, plan):
        if ((self.pose_bdi is None) or (self.pose_body is None)):
            print "haven't received POSE_BDI and POSE_BODY"
            return

        t_bodybdi  = transformUtils.transformFromPose(self.pose_bdi.pos,self.pose_bdi.orientation)
        t_bodybdi.PostMultiply()
        t_bodymain = transformUtils.transformFromPose(self.pose_body.pos,self.pose_body.orientation)
        t_bodymain.PostMultiply()
        #self.printTransform(t_bodybdi,"t_bodybdi")
        #self.printTransform(t_bodymain,"t_bodymain")

        # iterate and transform
        self.bdi_plan = plan.decode( plan.encode() ) # decode and encode ensures deepcopy
        for i, footstep in enumerate(self.bdi_plan.footsteps):
            step = footstep.pos
            
            #print i
            t_step = transformUtils.frameFromPositionMessage(step)
            #self.printTransform(t_step,"t_step")
            #self.printTransform(t_bodymain,"t_bodymain")

            t_body_to_step = vtk.vtkTransform()
            t_body_to_step.DeepCopy(t_step)
            t_body_to_step.PostMultiply()
            t_body_to_step.Concatenate(t_bodymain.GetLinearInverse())
            #self.printTransform(t_body_to_step,"t_body_to_step")

            t_stepbdi = vtk.vtkTransform()
            t_stepbdi.DeepCopy(t_body_to_step)
            t_stepbdi.PostMultiply()
            t_stepbdi.Concatenate(t_bodybdi)
            footstep.pos = transformUtils.positionMessageFromFrame(t_stepbdi)

        if (self.showBDIPlan is True):
            self.drawBDIFootstepPlan()
        else:
            print "not showing bdi plan"

    def drawBDIFootstepPlan(self):
        folder = om.getOrCreateContainer("BDI footstep plan")
        om.removeFromObjectModel(folder)
        self.footstepDriver.drawFootstepPlan(self.bdi_plan, om.getOrCreateContainer("BDI footstep plan"), [0.0, 0.0, 1.0] , [1.0, 0.0, 0.0])



    ###############################
    def onCaptureButton(self):
        print "capture" #,self.jointController.q
        # body frame:
        #goal = self.jointController.q[0:6]

        # mid point of feet (as used by robin)
        # this assumes model publisher exists - not always true currently
        model = om.findObjectByName("model publisher")
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

        frame = self.getFrameFromCombo()
        self.footstepDriver.sendFootstepPlanRequest(frame)

    def onHideBDIButton(self):
        print "hide bdi"
        self.showBDIPlan = False
        self.playbackRobotModel.setProperty('Visible', False)
        folder = om.getOrCreateContainer("BDI footstep plan")
        om.removeFromObjectModel(folder)

    def onShowBDIButton(self):
        print "show bdi"
        self.showBDIPlan = True
        self.playbackRobotModel.setProperty('Visible', True)
        self.drawBDIFootstepPlan()

    def onExecuteButton(self):
        if (self.bdi_plan is None):
            print "No BDI plan calculated, cannot execute"
            return

        print "committing bdi plan"
        self.footstepDriver.commitFootstepPlan(self.bdi_plan)

def init(jointController, footstepDriver, playbackRobotModel, playbackJointController):

    global dock

    panel = NavigationPanel(jointController, footstepDriver, playbackRobotModel, playbackJointController)
    dock = app.addWidgetToDock(panel.widget)
    #dock.hide()

    return panel

