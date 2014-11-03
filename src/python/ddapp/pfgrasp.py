from copy import deepcopy, copy
from math import sqrt
import math
import os
import subprocess
from threading import Thread
import time
from vtk import vtkTransform
from PythonQt import QtCore, QtGui

import bot_frames
from ddapp import botpy
from ddapp import lcmUtils
from ddapp import planplayback
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp.simpletimer import SimpleTimer

import drc as lcmdrc
import numpy as np


class Timer(object):
    def __init__(self, name=None):
        self.name = name

    def __enter__(self):
        self.tstart = time.time()

    def __exit__(self, type, value, traceback):
        if self.name:
            print '[%s]' % self.name,
        print 'Elapsed: %s' % (time.time() - self.tstart)

class FTContactSensor(object):    
    # for zero offsetting
    nvalue = 12
    offset = np.zeros(nvalue)
    count = 0
    ncount = 10
    resetting = True # mode
    threshold = 0.1
    
    onContactCallback = None
    
    lcmsub = None
    
    def __init__(self, onContactCallback):
        lcmUtils.addSubscriber('WRIST_STRAIN_GAUGES', lcmdrc.atlas_strain_gauges_t, self.handleFTSignal)
        self.onContactCallback = onContactCallback
    
    def handleFTSignal(self, data):
        if self.resetting:
            #print 'resetting'
            self.count = self.count + 1
            self.offset = np.asarray(data.strain_gauges) / float(self.ncount) + self.offset
            if self.count == self.ncount:
                self.resetting = False
        else:  # detecting mode
            filtered = data.strain_gauges - self.offset
            #print 'detecting mode', filtered[0:3]
            if filtered[2] < -0.05:  # this is the threshold for detecting hitting a drill on a table, [2] means the z axis which is parallel to hand direction
                if self.onContactCallback is not None:
                    self.onContactCallback() 
                    self.onContactCallback = None                
            
    def reset(self):
        self.count = 0
        self.resetting = True
        self.offset = np.zeros(12)
        
    def __exit__(self):
        if self.lcmsub is not None:
            lcmUtils.removeSubscriber(self.lcmsub)
        
        

class PFGrasp(object):
    side = 'L'
    planFrame = ''
    cameraChannel = ''
    graspingHand = 'left'
    TLDResultMsg = None
    ui = None
    contactDetector = None

    def __init__(self, om, robotModel, playbackRobotModel, teleopRobotModel, footstepPlanner, manipPlanner, ikPlanner,
                 lhandDriver, rhandDriver, atlasDriver, multisenseDriver, affordanceFitFunction, sensorJointController,
                 planPlaybackFunction, showPoseFunction, cameraView, segmentationpanel):
        self.om = om
        self.robotModel = robotModel
        self.playbackRobotModel = playbackRobotModel # not used inside the demo
        self.teleopRobotModel = teleopRobotModel # not used inside the demo
        self.footstepPlanner = footstepPlanner
        self.manipPlanner = manipPlanner
        self.ikPlanner = ikPlanner
        self.lhandDriver = lhandDriver
        self.rhandDriver = rhandDriver
        self.atlasDriver = atlasDriver
        self.multisenseDriver = multisenseDriver
        self.affordanceFitFunction = affordanceFitFunction
        self.sensorJointController = sensorJointController
        self.planPlaybackFunction = planPlaybackFunction
        self.showPoseFunction = showPoseFunction
        self.cameraView = cameraView
        self.segmentationpanel = segmentationpanel
        self.pointerTracker = None
        self.projectCallback = None
        self.drillYawSliderValue = 0.0
        self.segmentationpanel.init() # TODO: check with Pat. I added dependency on segmentationpanel, but am sure its appropriate

        self.defaultGraspingHand = "left"
        #self.setGraspingHand(defaultGraspingHand)
        
        self.planFrame = 'CAMERALHAND'
        self.cameraChannel = 'CAMERALHAND'
        self.TLDResultMsg = lcmdrc.image_roi_t()
        self.tldsub = lcmUtils.addSubscriber('TLD_OBJECT_ROI_RESULT', lcmdrc.image_roi_t, self.TLDReceived)
        self.targetsub = lcmUtils.addSubscriber('REACH_TARGET_POSE', bot_frames.update_t, self.TargetReceived)
        self.autoMode = False

    def log(self, str):
        if self.ui is not None:
            self.ui.statusTextEdit.plainText = self.ui.statusTextEdit.plainText + '\n' + str 
            cursor = self.ui.statusTextEdit.textCursor()
            cursor.movePosition(QtGui.QTextCursor.End, QtGui.QTextCursor.MoveAnchor)
            self.ui.statusTextEdit.setTextCursor(cursor)
            self.ui.statusTextEdit.moveCursor(QtGui.QTextCursor.End)
    
    def getEstimatedRobotStatePose(self):
        return self.sensorJointController.getPose('EST_ROBOT_STATE')
    
    def getPlanningStartPose(self):
        return self.getEstimatedRobotStatePose()

    def start(self, autoMode):

        # start pfgrasp c++ program  
        startPfgraspHere = False 
        if startPfgraspHere == True:
            FNULL = open(os.devnull, 'w')
            self.pfgraspCpp = subprocess.Popen(["drc-pfgrasp","-c","CAMERALHAND","-s","0.25","-g","LHAND_FORCE_TORQUE"],stdout=FNULL,stderr=FNULL)
            #pfgraspCpp = subprocess.Popen(["drc-pfgrasp","-c","CAMERALHAND","-s","0.25","-g","LHAND_FORCE_TORQUE"])
            time.sleep(1)
            
        # initialize pfgrasp particles
        msg = lcmdrc.pfgrasp_command_t()
        msg.command = lcmdrc.pfgrasp_command_t.START
        lcmUtils.publish('PFGRASP_CMD', msg)
        
        self.autoMode = autoMode
        
        if self.autoMode:
            self.runoneiter()
            
    def runoneiter(self):
        msg = lcmdrc.pfgrasp_command_t()
        msg.command = lcmdrc.pfgrasp_command_t.RUN_ONE_ITER
        lcmUtils.publish('PFGRASP_CMD', msg)
    
    def planDeltaMove(self, Direction, LocalOrWorld, Amount):
        linkMap = { 'left' : 'l_hand_face', 'right': 'r_hand_face'}
        linkName = linkMap[self.graspingHand]

        handToWorld = self.robotModel.getLinkFrame(linkName)

        if Direction == 'X':
            delta = transformUtils.frameFromPositionAndRPY([Amount,0,0],[0,0,0])
        elif Direction == 'Y':
            delta = transformUtils.frameFromPositionAndRPY([0,Amount,0],[0,0,0])
        else:
            delta = transformUtils.frameFromPositionAndRPY([0,0,Amount],[0,0,0])

        startPose = self.getPlanningStartPose() 
        constraintSet = self.ikPlanner.planEndEffectorDelta(startPose, self.graspingHand, 
                        delta.GetPosition(), constraints=None, LocalOrWorldDelta=LocalOrWorld)
        
        endPose, info = constraintSet.runIk()        
        graspPlan = constraintSet.runIkTraj()
        return graspPlan
        
    def delay(self, delayTimeInSeconds):
        yield
        t = SimpleTimer()
        while t.elapsed() < delayTimeInSeconds:
            yield
    
    def waitForPlanExecution(self, plan):
        self.log('in waitForPlanExecution')
        planElapsedTime = planplayback.PlanPlayback.getPlanElapsedTime(plan)
        self.log('waiting for plan execution: %f' % planElapsedTime)
    
        time.sleep(planElapsedTime + 1.0)
        #return self.delay(planElapsedTime + 1.0)
    
    def onRetreatPlanCommitted(self, plan):
        self.log('in onRetreatPlanCommitted')
        self.manipPlanner.disconnectPlanCommitted(self.onRetreatConnector)
        self.waitForPlanExecution(plan)
        self.log('in onRetreatPlanCommitted:PlanExecuted')
        
        self.log('done')
        
    def onHoldPlanCommitted(self, plan):
        self.log('in onHoldPlanCommitted')
        self.manipPlanner.disconnectPlanCommitted(self.onHoldConnector)
        self.waitForPlanExecution(plan)
        self.log('in onHoldPlanCommitted:PlanExecuted')
        
        # retreat
        plan = self.planDeltaMove('Y', 'Local', -0.20)
        self.onRetreatConnector = self.manipPlanner.connectPlanCommitted(self.onRetreatPlanCommitted)
        
        if self.autoMode:
            self.manipPlanner.commitManipPlan(plan)   
            
    def grasp(self):
        # close the hand
        if self.graspingHand == 'left':
            self.lhandDriver.sendClose(100)
            self.delay(1.5)
            self.lhandDriver.sendClose(100)
        else:
            self.rhandDriver.sendClose(100)
            self.delay(1.5)
            self.rhandDriver.sendClose(100)
    
    def stop(self):
        self.manipPlanner.sendPlanPause()
    
    def onContactCallback(self):
        self.log('in onContactCallback')
        self.manipPlanner.sendPlanPause()
        self.grasp()
        
        # hold it by moving up
        plan = self.planDeltaMove('Z', 'World', 0.10)
        
        self.onHoldConnector = self.manipPlanner.connectPlanCommitted(self.onHoldPlanCommitted)
        if self.autoMode:
            self.manipPlanner.commitManipPlan(plan)        
        
        self.contactDetector = None
        
    def guardedMoveForwardAndGraspHoldRetreat(self):
        self.log('in guardedMoveForwardAndGraspHoldRetreat')
        plan = self.planDeltaMove('Y', 'Local', 0.25)
        
        self.contactDetector = FTContactSensor(self.onContactCallback)
        if self.autoMode:
            self.manipPlanner.commitManipPlan(plan)
                        
    def TLDReceived(self, data):
        #print 'TLD received', data
        self.TLDResultMsg = deepcopy(data)
        
    def TargetReceived(self, data):
        self.log( 'Target received %s, %s' % (data.trans, data.quat) )
        if math.isnan(data.trans[0]):
            self.log('Getting NaN target, stop')
            return
        self.TargetMsg = deepcopy(data)
        
        targetToWorld = transformUtils.frameFromPositionAndRPY(self.TargetMsg.trans,
                                                   np.degrees(botpy.quat_to_roll_pitch_yaw(self.TargetMsg.quat)))
    
        startPose = self.getPlanningStartPose()            
        
        handToWorld= self.ikPlanner.getLinkFrameAtPose( 'l_hand_face', startPose)
        goalFrame = vis.updateFrame(handToWorld, 'OriginalFrame', parent='Pfgrasp', visible=True, scale=0.25)
        goalFrame2 = vis.updateFrame(targetToWorld, 'PeterFrame', parent='Pfgrasp', visible=True, scale=0.25)
        
        handToWorld_XYZ = handToWorld.GetPosition() 
        targetToWorld_XYZ = targetToWorld.GetPosition()
        dist = sqrt( (handToWorld_XYZ[0]-targetToWorld_XYZ[0])**2 + (handToWorld_XYZ[1]-targetToWorld_XYZ[1])**2 + (handToWorld_XYZ[2]-targetToWorld_XYZ[2])**2 )
        
        self.log( "dist %f" % dist )
        threshold = float(self.ui.criterionEdit.text)
        if(dist < threshold):
            #easygui.msgbox("The correction movement is less than 0.015, you can go grasp it", title="Done")
            self.log("The correction movement is %f less than %.3f, you can go grasp it" % (dist, threshold))
            
            if self.autoMode: self.guardedMoveForwardAndGraspHoldRetreat()
        else:
            #print "startPose", startPose
            #print "targetToWorld", targetToWorld
            #print "graspingHand", self.graspingHand
            constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, targetToWorld, lockBase=False, lockBack=True)
            
            endPose, info = constraintSet.runIk()
            if info > 10:
                self.log("in Target received: Bad movement")
                return
                
            graspPlan = constraintSet.runIkTraj()
        
            if self.autoMode:
                self.manipPlanner.commitManipPlan(graspPlan)
                self.waitForPlanExecution(graspPlan) 
                self.runoneiter()

            
         