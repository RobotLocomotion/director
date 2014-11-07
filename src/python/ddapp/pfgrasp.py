from copy import deepcopy, copy
from math import sqrt
import math
import os
import subprocess
from threading import Thread
import time
from vtk import vtkTransform
from ddapp import objectmodel as om
from ddapp import segmentation
import vtk
from ddapp import affordanceitems

import bot_frames

from PythonQt import QtCore, QtGui
from ddapp import botpy, filterUtils
from ddapp import lcmUtils
from ddapp import planplayback
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp.simpletimer import SimpleTimer
from ddapp.drilldemo import Drill
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
    graspingHand = 'left'
    TLDResultMsg = None
    ui = None
    contactDetector = None

    def __init__(self, drillDemo, robotModel, playbackRobotModel, teleopRobotModel, footstepPlanner, manipPlanner, ikPlanner,
                 lhandDriver, rhandDriver, atlasDriver, multisenseDriver, affordanceFitFunction, sensorJointController,
                 planPlaybackFunction, showPoseFunction, cameraView, segmentationpanel):
        self.drillDemo = drillDemo
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
        
        self.TLDResultMsg = lcmdrc.image_roi_t()
        self.tldsub = lcmUtils.addSubscriber('TLD_OBJECT_ROI_RESULT', lcmdrc.image_roi_t, self.TLDReceived)
        self.targetsub = lcmUtils.addSubscriber('REACH_TARGET_POSE', bot_frames.update_t, self.TargetReceived)
        self.autoMode = False
        
        self.drill = Drill()

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

        #handToWorld = self.robotModel.getLinkFrame(linkName)

        if Direction == 'X':
            delta = transformUtils.frameFromPositionAndRPY([Amount,0,0],[0,0,0])
        elif Direction == 'Y':
            delta = transformUtils.frameFromPositionAndRPY([0,Amount,0],[0,0,0])
        else:
            delta = transformUtils.frameFromPositionAndRPY([0,0,Amount],[0,0,0])

        startPose = self.getPlanningStartPose() 
        constraintSet = self.ikPlanner.planEndEffectorDelta(startPose, self.graspingHand, 
                        delta.GetPosition(), constraints=None, LocalOrWorldDelta=LocalOrWorld)
        handfaceToWorld = self.ikPlanner.getLinkFrameAtPose(linkName, self.getPlanningStartPose())
        # constraint orientation
        p,q = self.ikPlanner.createPositionOrientationGraspConstraints('left',handfaceToWorld)
        q.tspan=[0.1,np.inf]
        
        constraintSet.constraints.append(q)
        ##
        endPose, info = constraintSet.runIk()    
        if info>10:
            return None    
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
    
    def spawnDrillAffordance(self):
        if om.findObjectByName('drill') is None: 
            self.drillDemo.spawnDrillAffordance()
        
        self.moveDrill()
        
    def apply3DFit(self):
        if om.findObjectByName('drill') is None: 
            self.log('No 3D fit of drill. Click Spawn Drill button to provide a fit.')
        
        msg = lcmdrc.pfgrasp_command_t()
        msg.command = lcmdrc.pfgrasp_command_t.RUN_ONE_ITER_W_3D_PRIOR
        affordanceReach = om.findObjectByName('grasp frame')
        affordanceReach.actor.GetUserTransform().GetPosition(msg.pos)
        lcmUtils.publish('PFGRASP_CMD', msg)
    
    def moveDrill(self,Pos=[0.1,0,0],RPY=[0,0,0],Style='Local'):
        linkMap = { 'left' : 'l_hand_face', 'right': 'r_hand_face'}
        linkName = linkMap[self.graspingHand]
        
        affordance = om.findObjectByName('drill')
        affordanceReach = om.findObjectByName('reach frame')
        frame = om.findObjectByName('drill frame')

        
        drillTransform = affordance.actor.GetUserTransform()
        reach = transformUtils.copyFrame(affordanceReach.actor.GetUserTransform())
        drillTransformCopy = transformUtils.copyFrame(affordance.actor.GetUserTransform())
        drillToReach = vtkTransform()
        drillToReach.Identity()
        drillToReach.PostMultiply()
        drillToReach.Concatenate(drillTransformCopy)
        drillToReach.Concatenate(reach.GetLinearInverse())
        
        handfaceToWorld = self.ikPlanner.getLinkFrameAtPose(linkName, self.getPlanningStartPose())
        
        # find a transform that move forward wrt hand palm
        delta = transformUtils.frameFromPositionAndRPY(Pos, RPY)
        drillTransform.Identity()
        drillTransform.PostMultiply()
        drillTransform.Concatenate(drillToReach)
        #drillTransform.Concatenate(delta)
        drillTransform.Concatenate(handfaceToWorld)
        
        #drillTransform.Concatenate(drillTransformCopy)
        
    def stop(self):
        self.manipPlanner.sendPlanPause()
        self.contactDetector.onContactCallback = None
        self.contactDetector = None
        
    
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
        self.log('in guardedMoveForward')
        max_dist = float(self.ui.disttomoveEdit.text)
        for forwardDist in np.linspace(0.5, 0.1, num=5):
            plan = self.planDeltaMove('Y', 'Local', forwardDist)
            if plan is not None:
                self.log('in guardedMoveForward: forward %f' % forwardDist)
                break
        
        if plan is None:
            self.log('in guardedMoveForward: Bad move')
            return
            
        self.contactDetector = FTContactSensor(self.onContactCallback)
        if self.autoMode:
            self.manipPlanner.commitManipPlan(plan)
                        
    def TLDReceived(self, data):
        #print 'TLD received', data
        self.TLDResultMsg = deepcopy(data)
        
    def TargetReceived(self, data):
        self.log( 'Target received (%.3f,%.3f,%.3f), (%.3f,%.3f,%.3f,%.3f)' % \
                  (data.trans[0], data.trans[1], data.trans[2], \
                   data.quat[0], data.quat[1], data.quat[2], data.quat[3]) )
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
        
        self.log( "dist %.3f" % dist )
        threshold = float(self.ui.criterionEdit.text)
        if(dist < threshold):
            #easygui.msgbox("The correction movement is less than 0.015, you can go grasp it", title="Done")
            self.log("The correction movement is %.3f less than %.3f, you can go grasp it" % (dist, threshold))
            
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

    def drawFrameInCamera(self, t, frameName='new frame',visible=True):

        imageView = self.cameraView.views['CAMERALHAND']
        v = imageView.view
        q = self.cameraView.imageManager.queue
        localToCameraT = vtk.vtkTransform()
        q.getTransform('local', 'CAMERALHAND', localToCameraT)

        res = vis.showFrame( vtk.vtkTransform() , 'temp', view=v, visible=True, scale = 0.2)
        om.removeFromObjectModel(res)
        pd = res.polyData
        pd = filterUtils.transformPolyData(pd, t)
        pd = filterUtils.transformPolyData(pd, localToCameraT)
        q.projectPoints('CAMERALHAND', pd )
        vis.showPolyData(pd, ('overlay ' + frameName), view=v, colorByName='Axes',parent='camera overlay',visible=visible)

    def drawObjectInCamera(self,objectName,visible=True):
        
        imageView = self.cameraView.views['CAMERALHAND']
        v = imageView.view
        q = self.cameraView.imageManager.queue
        localToCameraT = vtk.vtkTransform()
        q.getTransform('local', 'CAMERALHAND', localToCameraT)

        obj = om.findObjectByName(objectName)
        if obj is None:
            return
        objToLocalT = transformUtils.copyFrame(obj.actor.GetUserTransform() or vtk.vtkTransform())
        objPolyDataOriginal = obj.polyData
        pd = objPolyDataOriginal
        pd = filterUtils.transformPolyData(pd, objToLocalT)
        pd = filterUtils.transformPolyData(pd, localToCameraT)
        q.projectPoints('CAMERALHAND', pd)
        vis.showPolyData(pd, ('overlay ' + objectName), view=v, color=[0,1,0],parent='camera overlay',visible=visible)         

    def drawDrill(self):
        q = om.findObjectByName('camera overlay')
        om.removeFromObjectModel(q)

        imageView = self.cameraView.views['CAMERALHAND']
        imageView.imageActor.SetOpacity(.5)
        
        self.drawObjectInCamera('drill',visible=True)
        
        obj = om.findObjectByName('reach frame')
        if obj is None:
            return
        
        objToLocalT = transformUtils.copyFrame(obj.actor.GetUserTransform())
        self.drawFrameInCamera(objToLocalT, 'reach frame',visible=False)

        v = imageView.view
        v.render()
    