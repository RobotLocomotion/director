from copy import deepcopy, copy
from math import sqrt
import os
import subprocess
from threading import Thread
import time
from vtk import vtkTransform

import bot_frames
from ddapp import botpy
from ddapp import lcmUtils
from ddapp import transformUtils
from ddapp import visualization as vis

import drc as lcmdrc
import easygui
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

class PFGraspDemo(object):
    side = 'L'
    planFrame = ''
    cameraChannel = ''
    graspingHand = 'left'
    TLDResultMsg = None

    def __init__(self, robotModel, playbackRobotModel, teleopRobotModel, footstepPlanner, manipPlanner, ikPlanner,
                 lhandDriver, rhandDriver, atlasDriver, multisenseDriver, affordanceFitFunction, sensorJointController,
                 planPlaybackFunction, showPoseFunction, cameraView, segmentationpanel):
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
        self.targetsub = lcmUtils.addSubscriber('REACH_TARGET_POSE', bot_frames.update_t) #, self.TargetReceived)

    def getEstimatedRobotStatePose(self):
        return self.sensorJointController.getPose('EST_ROBOT_STATE')
    
    def getPlanningStartPose(self):
        return self.getEstimatedRobotStatePose()

    def start(self):
        self.thread = Thread(target = PFGraspDemo.start_, args = (self, ))
        self.thread.start()

    
    def TLDReceived(self, data):
        #print 'TLD received', data
        self.TLDResultMsg = deepcopy(data)
        
    def TargetReceived(self, data):
        #print 'Target received', data
        self.TargetMsg = deepcopy(data)
        
    def start_(self):
        
        # start pfgrasp c++ program   
        
        FNULL = open(os.devnull, 'w')
        pfgraspCpp = subprocess.Popen(["drc-pfgrasp","-c","CAMERALHAND","-s","0.25","-g","LHAND_FORCE_TORQUE"],stdout=FNULL,stderr=FNULL)
        #pfgraspCpp = subprocess.Popen(["drc-pfgrasp","-c","CAMERALHAND","-s","0.25","-g","LHAND_FORCE_TORQUE"])
        # initialize pfgrasp particles
        time.sleep(1)
        msg = lcmdrc.pfgrasp_command_t()
        msg.command = lcmdrc.pfgrasp_command_t.START
        lcmUtils.publish('PFGRASP_CMD', msg)
        
        
     
        #wait = input("PRESS ENTER TO CONTINUE.")
        
        #for i in xrange(1,10000):
        #    time.sleep(0.01)
            
        while True:
            #print self.TLDResultMsg.roi.width
            #print "start1"
            if self.TLDResultMsg.roi.width < 1e-8 and self.TLDResultMsg.roi.height < 1e-8:
                time.sleep(0.1)
                continue
            msg = lcmdrc.pfgrasp_command_t()
            msg.command = lcmdrc.pfgrasp_command_t.RUN_ONE_ITER
            lcmUtils.publish('PFGRASP_CMD', msg)
            
            msgTarget = lcmUtils.getNextMessage(self.targetsub, bot_frames.update_t, 1)
            
            if msgTarget is None:
                continue
            
            with Timer('start2'):
                print "start2"
                #msgTarget.trans
                #msgTarget.quat
                
                targetToWorld = transformUtils.frameFromPositionAndRPY(msgTarget.trans,
                                                           np.degrees(botpy.quat_to_roll_pitch_yaw(msgTarget.quat)))
            
            with Timer('start3'):
                print "start3"
                startPose = self.getPlanningStartPose()
                #handFrame = self.om.findObjectByName('cameras')
                
                
                handToWorld= self.ikPlanner.getLinkFrameAtPose( 'l_hand_face', startPose)
                goalFrame = vis.updateFrame(handToWorld, 'OriginalFrame', parent='Pfgrasp', visible=True, scale=0.25)
                goalFrame2 = vis.updateFrame(targetToWorld, 'PeterFrame', parent='Pfgrasp', visible=True, scale=0.25)
                
                handToWorld_XYZ = handToWorld.GetPosition() 
                targetToWorld_XYZ = targetToWorld.GetPosition()
                dist = sqrt( (handToWorld_XYZ[0]-targetToWorld_XYZ[0])**2 + (handToWorld_XYZ[1]-targetToWorld_XYZ[1])**2 + (handToWorld_XYZ[2]-targetToWorld_XYZ[2])**2 )
                
                print "dist", dist
                if(dist < 0.015):
                    easygui.msgbox("The correction movement is less than 0.015, stop updating", title="Pause")
                    break
            
            with Timer('start4'):
                print "start4"
                #print "startPose", startPose
                #print "targetToWorld", targetToWorld
                #print "graspingHand", self.graspingHand
                constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, targetToWorld, lockBase=False, lockBack=True)
                
            with Timer('start5'):
                print "start5"
                endPose, info = constraintSet.runIk()
                print info
            
            with Timer('start6'):
                print "start6"
                graspPlan = constraintSet.runIkTraj()
                
            easygui.msgbox("Please review the plan", title="Pause")
            if not easygui.ynbox("Do next iteration?", title=" "):
                break 
            
#             print "start7"
#             if graspPlan.plan_info[-1] > 10:
#                 print "PLANNER REPORTS ERROR!"
#     
#                 self.planPlaybackFunction([graspPlan])
#     
#                 #self.fail()
#                 break
#             else:
#                 print "Planner reports success!"
#     
#                 # Plan was successful, save it to be animated and update output data
#                 self.planPlaybackFunction([graspPlan])
#                 self.manipPlanner.commitManipPlan(graspPlan)
            
            
            
        #while True:
        #    time.sleep(1)
        # while(1) {
         
        #   ask user to select the object in the image (if no tracking) 
        
        #   prompt for confirmation if in step-by-step mode
        
        #   run one iteration of pfgrasp
        
        #   get the goal of the hand 
        
        #   if delta movement (goal-current hand pose) < threshold :
      
        #     plan arm motion to goal
        
        #     if planning success
        
        #       prompt for confirmation if in step-by-step mode
        
        #       execute the plan
        
        #     else
        
        #       error can't move due to failed planning
      
        #   else:
        
        #     break
        
        # } while  
        
        # Go on to grasp??
        
        # kill pfgrasp c++ program
        
        pfgraspCpp.kill()
        #lcmUtils.removeSubscriber(tldsub)
        #hideImageOverlay()
