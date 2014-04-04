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

    def pointPickerDemo(self,p1, p2):
        print "mezo"
        print p1, p2
        
        yaw = math.atan2( p2[1] - p1[1] , p2[0] - p1[0] )*180/math.pi + 90
        
        print yaw*180/math.pi
        frame_p1 = transformUtils.frameFromPositionAndRPY(p1, [0,0,yaw])
        
        blockl = 0.3937
        blockh = 0.142875
        
        frame_pt_to_centerline = transformUtils.frameFromPositionAndRPY( [0, -blockl/2, 0], [0,0,0])
         
        frame_pt_to_centerline.PostMultiply()
        frame_pt_to_centerline.Concatenate(frame_p1)
         
        vis.updateFrame(frame_pt_to_centerline, "p1", parent="navigation")
        
        flist = np.array( [[ blockl*-0.5 , .1  , 0      , 0 , 0 , 0] ,
                           [ blockl*-0.5 , -.1 , 0      , 0 , 0 , 0] ,
	                   [ blockl*0.5  , .1  , blockh , 0 , 0 , 0] ,
                           [ blockl*0.5  ,-.1  , blockh , 0 , 0 , 0] ,
                           [ blockl*1.5  , .1  , 0      , 0 , 0 , 0] ,
                           [ blockl*1.5  ,-.1  , 0      , 0 , 0 , 0]])
        #print flist
        
        numGoalSteps = 3
        is_right_foot = True
        self.goalSteps = []
        for i in range(numGoalSteps):

            #print flist[i,:]
            #print flist[i,0:3] 
            #print flist[i,3:6]
            
            step_t= transformUtils.frameFromPositionAndRPY(flist[i,0:3] , flist[i,3:6])

            step_t.PostMultiply()
            step_t.Concatenate(frame_pt_to_centerline)
            
 
            is_right_foot = not is_right_foot
            step = lcmdrc.footstep_t()
            step.pos = transformUtils.positionMessageFromFrame(step_t)
            step.is_right_foot = is_right_foot
            step.params = self.footstepDriver.getDefaultStepParams()
            
            vis.updateFrame(step_t, str(i), parent="navigation")
            
            self.goalSteps.append(step)
        request = self.footstepDriver.constructFootstepPlanRequest()
        request.num_goal_steps = len(self.goalSteps)
        request.goal_steps = self.goalSteps
        self.lastFootstepRequest = request
        lcmUtils.publish('FOOTSTEP_PLAN_REQUEST', request)        

def init(jointController, footstepDriver, playbackRobotModel, playbackJointController):

    global dock

    panel = NavigationPanel(jointController, footstepDriver, playbackRobotModel, playbackJointController)
    dock = app.addWidgetToDock(panel.widget)
    dock.hide()

    return panel


     
