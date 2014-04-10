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
        self.ui.reversePlanButton.connect("clicked()", self.onReversePlanButton)

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

    def onReversePlanButton(self):
        print "reverse me"
        
        #self.clearFootstepPlan()
        lastPlan = self.footstepDriver.lastFootstepPlan
        reversedPlan = lastPlan.decode( lastPlan.encode() ) # decode and encode ensures deepcopy

        #planFolder = getFootstepsFolder()
        #self.drawFootstepPlan(msg, planFolder)

        for i, footstep in enumerate(lastPlan.footsteps):
          j = lastPlan.num_steps - 1 - i 
          reversedPlan.footsteps[j] = footstep
        
        self.footstepDriver.onFootstepPlan(reversedPlan)
        
    def pointPickerDemo(self,p1, p2):
        #print "mezo"
        #print p1, p2
        
        yaw = math.atan2( p2[1] - p1[1] , p2[0] - p1[0] )*180/math.pi + 90
        
        print yaw*180/math.pi
        frame_p1 = transformUtils.frameFromPositionAndRPY(p1, [0,0,yaw])
        
        blockl = 0.3937
        blockh = 0.142875
        
        frame_pt_to_centerline = transformUtils.frameFromPositionAndRPY( [0, -blockl/2, 0], [0,0,0])
         
        frame_pt_to_centerline.PostMultiply()
        frame_pt_to_centerline.Concatenate(frame_p1)
         
        vis.updateFrame(frame_pt_to_centerline, "p1", parent="navigation")

# Original: 1 up/down
#        flist = np.array( [[ blockl*-0.5 , .1  , 0      , 0 , 0 , 0] ,
#                           [ blockl*-0.5 , -.1 , 0      , 0 , 0 , 0] ,
#                           [ blockl*0.5 - 0.03 , .1  , blockh , 0 , 0 , 0] ,
#                           [ blockl*0.5 + 0.06  ,-.1  , blockh , 0 , 0 , 0] ,
#                           [ blockl*1.5        , .1  , 0      , 0 , 0 , 0] ,
#                           [ blockl*1.5 +0.03  ,-.1  , 0      , 0 , 0 , 0]])


# Newer: 1 up/down
#        flist = np.array( [[ blockl*-0.5       , .1  , 0      , 0 , 0 , 0] ,
#                           [ blockl*-0.5       , -.1 , 0      , 0 , 0 , 0] ,
#	                   [ blockl*0.5 - 0.03 , .1  , blockh , 0 , 0 , 0] ,
#                           [ blockl*0.5 + 0.04 ,-.1  , blockh , 0 , 0 , 0] ,
#                           [ blockl*1.5 - 0.03 , .1  , 0      , 0 , 0 , 0] ,
#                           [ blockl*1.5 + 0.00 ,-.1  , 0      , 0 , 0 , 0]])

# 3 up
#        flist = np.array( [[ blockl*-0.5       , .1  , 0      , 0 , 0 , 0] ,
#                           [ blockl*-0.5       , -.1 , 0      , 0 , 0 , 0] ,
#	                   [ blockl*0.5 - 0.03 , .1  , blockh , 0 , 0 , 0] ,
#                           [ blockl*0.5 + 0.0  ,-.1  , blockh , 0 , 0 , 0] ,
#                           [ blockl*1.5 - 0.03 , .1  , 2*blockh, 0 , 0 , 0] ,
#                           [ blockl*1.5 + 0.0  ,-.1  , 2*blockh, 0 , 0 , 0],
#                           [ blockl*2.5 - 0.03 , .1  , 3*blockh, 0 , 0 , 0] ,
#                           [ blockl*2.5 + 0.0  ,-.1  , 3*blockh, 0 , 0 , 0]])
                           
# 3 up and down (original)
#        flist = np.array( [[ blockl*-0.5       , .1  , 0       , 0 , 0 , 0] ,
#                           [ blockl*-0.5       , -.1 , 0       , 0 , 0 , 0] ,
#	                   [ blockl*0.5 - 0.03 , .1  , blockh  , 0 , 0 , 0] ,
#                           [ blockl*0.5 + 0.0  ,-.1  , blockh  , 0 , 0 , 0] ,
#                           [ blockl*1.5 - 0.03 , .1  , 2*blockh, 0 , 0 , 0] ,
#                           [ blockl*1.5 + 0.0  ,-.1  , 2*blockh, 0 , 0 , 0],
#                           [ blockl*2.5 - 0.03 , .1  , 3*blockh, 0 , 0 , 0] ,
#                           [ blockl*2.5 + 0.03  ,-.1 , 3*blockh, 0 , 0 , 0],
#                           [ blockl*3.5 - 0.03 , .1  , 2*blockh, 0 , 0 , 0] ,
#                           [ blockl*3.5 + 0.03  ,-.1 , 2*blockh, 0 , 0 , 0],
#                           [ blockl*4.5 - 0.03 , .1  , 1*blockh, 0 , 0 , 0] ,
#                           [ blockl*4.5 + 0.03  ,-.1 , 1*blockh, 0 , 0 , 0],
#                           [ blockl*5.5 - 0.03 , .1  , 0       , 0 , 0 , 0] ,
#                           [ blockl*5.5 + 0.03  ,-.1 , 0       , 0 , 0 , 0]])
                           

# 3 up and down
        sep = 0.11
        r =1
        flist = np.array( [[ blockl*-0.5       , sep  , 0       , 0 , 0 , 0, 0],
                           [ blockl*-0.5       , -sep , 0       , 0 , 0 , 0, r],
	                   [ blockl*0.5 - 0.03 , sep  , blockh  , 0 , 0 , 0, 0],
                           [ blockl*0.5 + 0.0  ,-sep  , blockh  , 0 , 0 , 0, r],
                           [ blockl*1.5 - 0.03 , sep  , 2*blockh, 0 , 0 , 0, 0],
                           [ blockl*1.5 + 0.0  ,-sep  , 2*blockh, 0 , 0 , 0, r],
                           [ blockl*2.5 - 0.03 , sep  , 3*blockh, 0 , 0 , 0, 0],
                           [ blockl*2.5 + 0.03  ,-sep , 3*blockh, 0 , 0 , 0, r],
                           [ blockl*3.5 - 0.03 , sep  , 2*blockh, 0 , 0 , 0, 0],
                           [ blockl*3.5 + 0.03  ,-sep , 2*blockh, 0 , 0 , 0, r],
                           [ blockl*4.5 - 0.03 , sep  , 1*blockh, 0 , 0 , 0, 0],
                           [ blockl*4.5 + 0.03  ,-sep , 1*blockh, 0 , 0 , 0, r],
                           [ blockl*5.5 - 0.03 , sep  , 0       , 0 , 0 , 0, 0],
                           [ blockl*5.5 + 0.03  ,-sep , 0       , 0 , 0 , 0, r], # half 
                           [ blockl*5.5 - 0.03 , sep  , 0       , 0 , 0 , 0, 0], # extra step for planner
                           [ blockl*4.5 + 0.03  ,-sep , 1*blockh, 0 , 0 , 0, r], # invert order
                           [ blockl*4.5 - 0.03 , sep  , 1*blockh, 0 , 0 , 0, 0],
                           [ blockl*3.5 + 0.03  ,-sep , 2*blockh, 0 , 0 , 0, r],
                           [ blockl*3.5 - 0.03 , sep  , 2*blockh, 0 , 0 , 0, 0],
                           [ blockl*2.5 + 0.03  ,-sep , 3*blockh, 0 , 0 , 0, r], # top
                           [ blockl*2.5 - 0.06 , sep  , 3*blockh, 0 , 0 , 0, 0], # top
                           [ blockl*1.5 + 0.04 ,-sep  , 2*blockh, 0 , 0 , 0, r],
                           [ blockl*1.5 - 0.06 , sep  , 2*blockh, 0 , 0 , 0, 0],
                           [ blockl*0.5 + 0.04 ,-sep  , blockh  , 0 , 0 , 0, r],
	                   [ blockl*0.5 - 0.06 , sep  , blockh  , 0 , 0 , 0, 0],
                           [ blockl*-0.5+ 0.04 , -sep , 0       , 0 , 0 , 0, r],
                           [ blockl*-0.5 - 0.03, sep  , 0       , 0 , 0 , 0, 0]])
                           
                           
        contact_pts = self.footstepDriver.getContactPts()
        contact_pts_mid = np.mean(contact_pts, axis=0) # mid point on foot relative to foot frame
        foot_to_sole = transformUtils.frameFromPositionAndRPY( contact_pts_mid, [0,0,0]).GetLinearInverse()
        
        flist_shape = flist.shape
        #is_right_foot = True
        self.goalSteps = []
        for i in range(flist_shape[0]):

            #print flist[i,:]
            #print flist[i,0:3] 
            #print flist[i,3:6]
            
            step_t = vtk.vtkTransform()
            step_t.PostMultiply()
            step_t.Concatenate(transformUtils.frameFromPositionAndRPY(flist[i,0:3] , flist[i,3:6]))
            step_t.Concatenate(foot_to_sole)
            step_t.Concatenate(frame_pt_to_centerline)

 
            #is_right_foot = not is_right_foot
            
            step = lcmdrc.footstep_t()
            step.pos = transformUtils.positionMessageFromFrame(step_t)
            step.is_right_foot =  flist[i,6] # is_right_foot
            step.params = self.footstepDriver.getDefaultStepParams()
            
            vis.updateFrame(step_t, str(i), parent="navigation")
            
            self.goalSteps.append(step)

        startPose = self.jointController.q
        request = self.footstepDriver.constructFootstepPlanRequest(startPose)
        request.num_goal_steps = len(self.goalSteps)
        request.goal_steps = self.goalSteps
        lcmUtils.publish('FOOTSTEP_PLAN_REQUEST', request)        

def init(jointController, footstepDriver, playbackRobotModel, playbackJointController):

    global dock

    panel = NavigationPanel(jointController, footstepDriver, playbackRobotModel, playbackJointController)
    dock = app.addWidgetToDock(panel.widget)
    #dock.hide()

    return panel


     
