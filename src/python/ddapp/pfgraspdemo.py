import subprocess
import time
from ddapp import lcmUtils

import drc as lcmdrc

class PFGraspDemo(object):
    side = 'L'
    planFrame = ''
    cameraChannel = ''



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

        defaultGraspingHand = "left"
        #self.setGraspingHand(defaultGraspingHand)
        
        self.planFrame = 'CAMERALHAND'
        self.cameraChannel = 'CAMERALHAND'



    def start(self):
        # start pfgrasp c++ program   
        pfgraspCpp = subprocess.Popen(["drc-pfgrasp","-c","CAMERALHAND","-s","0.25","-g","CAMERALHAND"])
        # initialize pfgrasp particles
        msg = lcmdrc.pfgrasp_command_t()
        msg.command = lcmdrc.pfgrasp_command_t.START
        lcmUtils.publish('PFGRASP_CMD', msg)
        
        #self.cameraView.addCameraView('CAMERALHAND')
        #showImageOverlay(viewName='CAMERALHAND')
        
        #wait = input("PRESS ENTER TO CONTINUE.")
        
        time.sleep(10)
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
        hideImageOverlay()
