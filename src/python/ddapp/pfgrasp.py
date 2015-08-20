from copy import deepcopy, copy
from math import sqrt
import math
import os
import subprocess
import time
import vtk
from vtk import vtkTransform
from ddapp import objectmodel as om
from ddapp import ikplanner

from ddapp import ik
import bot_frames

from PythonQt import QtCore, QtGui
from ddapp import filterUtils
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
    threshold = 0.05
    
    onContactCallback = None
    
    lcmsub = None
    graspingHand = None 
    
    def __init__(self, onContactCallback, graspingHand):
        lcmUtils.addSubscriber('WRIST_STRAIN_GAUGES', lcmdrc.atlas_strain_gauges_t, self.handleFTSignal)
        self.onContactCallback = onContactCallback
        self.graspingHand = graspingHand
    
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
            if self.graspingHand == 'left': index = 2
            else : index = 8
            
            if filtered[index] < -0.05:  # this is the threshold for detecting hitting a drill on a table, [2] means the z axis which is parallel to hand direction
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
        self.imageViewName = 'CAMERALHAND'
        self.setGraspingHand(self.defaultGraspingHand)
        
        self.TLDResultMsg = lcmdrc.image_roi_t()
        self.tldsub = lcmUtils.addSubscriber('TLD_OBJECT_ROI_RESULT', lcmdrc.image_roi_t, self.TLDReceived)
        self.targetsub = lcmUtils.addSubscriber('REACH_TARGET_POSE', bot_frames.update_t, self.TargetReceived)
        self.autoMode = False
        
        self.drill = Drill()

    def setGraspingHand(self, graspingHand):
        self.graspingHand = graspingHand
	self.drillDemo.setGraspingHand(graspingHand)
        if graspingHand == 'left':
            self.imageViewName = 'CAMERALHAND'
        else:
            self.imageViewName = 'CAMERARHAND'
        
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
        p,q = self.ikPlanner.createPositionOrientationGraspConstraints(self.graspingHand,handfaceToWorld)
        q.tspan=[0.5,np.inf]
        
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
    
    def onModifiedDrillFrame(self, frame):
        self.drawDrill()
    
    def spawnDrillAffordance(self):
        if om.findObjectByName('drill') is None: 
            self.drillDemo.spawnDrillAffordance()
       
        if self.graspingHand == 'left':
            self.moveDrill()
        else:
            self.moveDrill(RPY=[0,180,0])

        om.findObjectByName('drill frame').connectFrameModified(self.onModifiedDrillFrame)
        
    def apply3DFit(self):
        if om.findObjectByName('drill') is None: 
            self.log('No 3D fit of drill. Click Spawn Drill button to provide a fit.')
        
        msg = lcmdrc.pfgrasp_command_t()
        msg.command = lcmdrc.pfgrasp_command_t.RUN_ONE_ITER_W_3D_PRIOR
        affordanceReach = om.findObjectByName('grasp frame')
        affordanceReach.actor.GetUserTransform().GetPosition(msg.pos)
        lcmUtils.publish('PFGRASP_CMD', msg)
    
    def moveDrill(self,Pos=[0,0,0],RPY=[0,0,0],Style='Local'):
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
        drillTransform.Concatenate(delta)
        drillTransform.Concatenate(handfaceToWorld)
        
        
    def stop(self):
        self.manipPlanner.sendPlanPause()
        if self.contactDetector is not None:
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
        for forwardDist in np.linspace(max_dist, 0.01, num=5):
            plan = self.planDeltaMove('Y', 'Local', forwardDist)
            if plan is not None:
                self.log('in guardedMoveForward: forward %f' % forwardDist)
                break
        
        if plan is None:
            self.log('in guardedMoveForward: Bad move')
            return
            
        self.contactDetector = FTContactSensor(self.onContactCallback, self.graspingHand)
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
        
        targetToWorld = transformUtils.transformFromPose(self.TargetMsg.trans, self.TargetMsg.quat)
    
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

    def turnPointwiseOffSlow(self):
        ikplanner.getIkOptions().setProperty('Use pointwise', False)
        ikplanner.getIkOptions().setProperty('Quasistatic shrink factor', 0.1)
        ikplanner.getIkOptions().setProperty('Max joint degrees/s',15)
    
    def planGraspLineMotion(self):
        self.turnPointwiseOffSlow()
        startPose = self.getPlanningStartPose()

        graspFrame = vtk.vtkTransform()
        graspFrame.Identity()
        graspFrame.PostMultiply()
        if self.graspingHand == 'right':
            graspFrame.Concatenate(transformUtils.frameFromPositionAndRPY([0,0,0], [0,180,0]))
        graspFrame.Concatenate(transformUtils.copyFrame(om.findObjectByName('grasp frame').actor.GetUserTransform()))

        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, graspFrame, \
                                                            lockBase=False, lockBack=True)
        # constraint orientation
        p,q = self.ikPlanner.createPositionOrientationGraspConstraints(self.graspingHand, graspFrame)
        q.tspan=[0.5,1]
        
        constraintSet.constraints.append(q)
        
        # constraint line axis 
        positionConstraint, orientationConstraint, axisConstraint = self.ikPlanner.createMoveOnLineConstraints(startPose, graspFrame)
        
        ## broken robot arm has a new joint limit
        if self.graspingHand == 'left':
            constraintSet.constraints.append(self.createBrokenArmConstraint())
        
        constraintSet.constraints.append(axisConstraint)
        constraintSet.constraints[-1].tspan = [0.5,np.inf]
        endPose, info = constraintSet.runIk()
        #print endPose
        if info > 10:
            self.log("in Target received: Bad movement")
            return
        graspPlan = constraintSet.runIkTraj()

    def createBrokenArmConstraint(self):
        p = ik.PostureConstraint()
        p.joints = ['l_arm_elx']
        p.jointsLowerBound = [0.673677]
        p.jointsUpperBound = [np.inf]
        p.tspan = [1, 1]
        return p
        
    def planReach(self):
        startPose = self.getPlanningStartPose()
	
        reachFrame = vtk.vtkTransform()
        reachFrame.Identity()
        reachFrame.PostMultiply()
        if self.graspingHand == 'right':
            reachFrame.Concatenate(transformUtils.frameFromPositionAndRPY([0,0,0], [0,180,0]))    
        reachFrame.Concatenate(transformUtils.copyFrame(om.findObjectByName('reach frame').actor.GetUserTransform()))
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, reachFrame, lockBase=False, lockBack=True)
        
        ## broken robot arm has a new joint limit
        if self.graspingHand == 'left':
            constraintSet.constraints.append(self.createBrokenArmConstraint())
            
        endPose, info = constraintSet.runIk()
        #print endPose
        
        if info > 10:
            self.log("in Target received: Bad movement")
            return
        reachPlan = constraintSet.runIkTraj()
        #print reachPlan

    def drawFrameInCamera(self, t, frameName='new frame',visible=True):

        imageView = self.cameraView.views[self.imageViewName]
        v = imageView.view
        q = self.cameraView.imageManager.queue
        localToCameraT = vtk.vtkTransform()
        q.getTransform('local', self.imageViewName, localToCameraT)

        res = vis.showFrame( vtk.vtkTransform() , 'temp', view=v, visible=True, scale = 0.2)
        om.removeFromObjectModel(res)
        pd = res.polyData
        pd = filterUtils.transformPolyData(pd, t)
        pd = filterUtils.transformPolyData(pd, localToCameraT)
        q.projectPoints(self.imageViewName, pd )
        vis.showPolyData(pd, ('overlay ' + frameName), view=v, colorByName='Axes',parent='camera overlay',visible=visible)

    def drawObjectInCamera(self,objectName,visible=True):
        
        imageView = self.cameraView.views[self.imageViewName]
        v = imageView.view
        q = self.cameraView.imageManager.queue
        localToCameraT = vtk.vtkTransform()
        q.getTransform('local', self.imageViewName, localToCameraT)

        obj = om.findObjectByName(objectName)
        if obj is None:
            return
        
        objToLocalT = transformUtils.copyFrame(obj.actor.GetUserTransform() or vtk.vtkTransform())
        objPolyDataOriginal = obj.polyData
        pd = objPolyDataOriginal
        pd = filterUtils.transformPolyData(pd, objToLocalT)
        pd = filterUtils.transformPolyData(pd, localToCameraT)
        q.projectPoints(self.imageViewName, pd)
        vis.showPolyData(pd, ('overlay ' + objectName), view=v, color=[0,1,0],parent='camera overlay',visible=visible)         

    def drawDrill(self, mustVisible = False):
        # on creation
        visible = True
        visibleframe = False
        
        if mustVisible:    
            visible = True
        else:
            # know previous preference to be visible or not
            overlayobj = om.findObjectByName('overlay ' + 'drill')
            if overlayobj is not None:
                visible = overlayobj.getProperty('Visible')
            q = om.findObjectByName('camera overlay')
        
        # get preference on visibility
        overlayobj = om.findObjectByName('overlay ' + 'grasp frame')
        if overlayobj is not None:
            visibleframe = overlayobj.getProperty('Visible')
        ###
        
        q = om.findObjectByName('camera overlay')
        if q is not None: om.removeFromObjectModel(q)

        imageView = self.cameraView.views[self.imageViewName]
        imageView.imageActor.SetOpacity(.5)
        
        self.drawObjectInCamera('drill',visible=visible)
        
        obj = om.findObjectByName('grasp frame')
        if obj is None:
            return
        
        objToLocalT = transformUtils.copyFrame(obj.actor.GetUserTransform())

        self.drawFrameInCamera(objToLocalT, 'grasp frame',visible=visibleframe)

        q = om.findObjectByName('camera overlay')

        v = imageView.view
        v.render()
    
