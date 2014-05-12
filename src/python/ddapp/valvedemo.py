import os
import sys
import vtkAll as vtk
from ddapp import botpy
import math
import time
import types
import functools
import numpy as np

from ddapp import transformUtils
from ddapp import lcmUtils
from ddapp.timercallback import TimerCallback
from ddapp.asynctaskqueue import AsyncTaskQueue
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp import applogic as app
from ddapp.debugVis import DebugData
from ddapp import ikplanner
from ddapp import ioUtils
from ddapp.simpletimer import SimpleTimer
from ddapp.utime import getUtime
from ddapp import robotstate
from ddapp import robotplanlistener
from ddapp import segmentation

import drc as lcmdrc

from PythonQt import QtCore, QtGui


class ValvePlannerDemo(object):

    def __init__(self, robotModel, footstepPlanner, manipPlanner, ikPlanner, handDriver, atlasDriver, multisenseDriver, affordanceFitFunction, sensorJointController, planPlaybackFunction, showPoseFunction):
        self.robotModel = robotModel
        self.footstepPlanner = footstepPlanner
        self.manipPlanner = manipPlanner
        self.ikPlanner = ikPlanner
        self.handDriver = handDriver
        self.atlasDriver = atlasDriver
        self.multisenseDriver = multisenseDriver
        self.affordanceFitFunction = affordanceFitFunction
        self.sensorJointController = sensorJointController
        self.planPlaybackFunction = planPlaybackFunction
        self.showPoseFunction = showPoseFunction
        self.graspingHand = 'left'
        self.planFromCurrentRobotState = False
        self.userPromptEnabled = True
        self.walkingPlan = None
        self.preGraspPlan = None
        self.graspPlan = None
        
        self.constraintSet = None

        self.plans = []
        
        
        
        self.valveRadius = 0.154 #foam valves
        self.scribeRadius = 0.1 # radius to arc the pointer around
        self.scribeDirection = -1 # 1 = clockwise | -1 = anticlockwise
        
        self.startAngle = 20 # 
        self.nextScribeAngle = self.startAngle
        
    def addPlan(self, plan):
        self.plans.append(plan)        


    def computeGroundFrame(self, robotModel):
        '''
        Given a robol model, returns a vtkTransform at a position between
        the feet, on the ground, with z-axis up and x-axis aligned with the
        robot pelvis x-axis.
        '''
        t1 = robotModel.getLinkFrame('l_foot')
        t2 = robotModel.getLinkFrame('r_foot')
        pelvisT = robotModel.getLinkFrame('pelvis')

        xaxis = [1.0, 0.0, 0.0]
        pelvisT.TransformVector(xaxis, xaxis)
        xaxis = np.array(xaxis)
        zaxis = np.array([0.0, 0.0, 1.0])
        yaxis = np.cross(zaxis, xaxis)
        yaxis /= np.linalg.norm(yaxis)
        xaxis = np.cross(yaxis, zaxis)

        stancePosition = (np.array(t2.GetPosition()) + np.array(t1.GetPosition())) / 2.0

        footHeight = 0.0811

        t = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
        t.PostMultiply()
        t.Translate(stancePosition)
        t.Translate([0.0, 0.0, -footHeight])

        return t


    def computeValveFrame(self, robotModel):


        position = [0.85, 0.4, 1.2] #0.65 backwards # 0.85 optimal # 1.25 forward
        rpy = [180, -90, 0]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.computeGroundFrame(robotModel))
        return t


    def computeGraspFrame(self):

        assert self.valveAffordance

        # for left_base_link
        #position = [-0.12, 0.0, 0.025]
        #rpy = [0, 90, 0]

        # reach to center and back - for palm point
        position = [0.0, 0.0, -0.1]
        rpy = [90, 0, 180]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.valveFrame.transform)

        self.graspFrame = vis.updateFrame(t, 'valve grasp frame', parent=self.valveAffordance, visible=False, scale=0.2)

        self.frameSync = vis.FrameSync()
        self.frameSync.addFrame(self.graspFrame)
        self.frameSync.addFrame(self.valveFrame)
        
        
    def computePointerTipFrame(self, engagedTip):
        if engagedTip:
            tipDepth = 0.02 # + is inside the wheel
        else:
            tipDepth = -0.1 # - is outside the wheel

        assert self.valveAffordance

        position = [ self.scribeRadius*math.cos( math.radians( self.nextScribeAngle )) ,  self.scribeRadius*math.sin( math.radians( self.nextScribeAngle ))  , tipDepth]
        rpy = [90, 0, 180]
        #rpy = [0.0,0.0,0.0]
        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.valveFrame.transform)

        self.pointerTipFrameDesired = vis.showFrame(t, 'pointer tip frame desired', parent=self.valveAffordance, visible=True, scale=0.2)

        #self.frameSync.addFrame(self.pointerTipFrameDesired, ignoreIncoming=True)

        #print "mfallon"
        #pointerTipLinkName = self.getEndEffectorLinkName() + '_pointer_tip'
        #print pointerTipLinkName
        #pointerTipFrame = self.robotModel.getLinkFrame(  pointerTipLinkName )
        #print pointerTipFrame
        #vis.updateFrame(pointerTipFrame, 'pointer tip frame', parent=self.valveAffordance, visible=True, scale=0.2)
        
        

    def computeStanceFrame(self):

        graspFrame = self.graspFrame.transform

        groundFrame = self.computeGroundFrame(self.robotModel)
        groundHeight = groundFrame.GetPosition()[2]

        graspPosition = np.array(graspFrame.GetPosition())
        graspYAxis = [0.0, 1.0, 0.0]
        graspZAxis = [0.0, 0.0, 1.0]
        graspFrame.TransformVector(graspYAxis, graspYAxis)
        graspFrame.TransformVector(graspZAxis, graspZAxis)

        xaxis = graspYAxis
        #xaxis = graspZAxis
        zaxis = [0, 0, 1]
        yaxis = np.cross(zaxis, xaxis)
        yaxis /= np.linalg.norm(yaxis)
        xaxis = np.cross(yaxis, zaxis)

        graspGroundFrame = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
        graspGroundFrame.PostMultiply()
        graspGroundFrame.Translate(graspPosition[0], graspPosition[1], groundHeight)

        position = [-0.67, -0.4, 0.0]
        rpy = [0, 0, 0]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(graspGroundFrame)

        self.graspStanceFrame = vis.updateFrame(t, 'valve grasp stance', parent=self.valveAffordance, visible=False, scale=0.2)

        self.frameSync.addFrame(self.graspStanceFrame)


    def computeFootstepPlan(self):

        #startPose = self.getEstimatedRobotStatePose()
        startPose = self.getPlanningStartPose()
        goalFrame = self.graspStanceFrame.transform

        request = self.footstepPlanner.constructFootstepPlanRequest(startPose, goalFrame)
        self.footstepPlan = self.footstepPlanner.sendFootstepPlanRequest(request, waitForResponse=True)


    def computeWalkingPlan(self):

        #startPose = self.getEstimatedRobotStatePose()
        startPose = self.getPlanningStartPose()
        self.walkingPlan = self.footstepPlanner.sendWalkingPlanRequest(self.footstepPlan, startPose, waitForResponse=True)
        self.addPlan(self.walkingPlan)

    def computeEndPose(self):
        graspLinks = {
            'l_hand' : 'left_base_link',
            'r_hand' : 'right_base_link',
           }
        linkName = graspLinks[self.getEndEffectorLinkName()]
        #startPose = self.getEstimatedRobotStatePose()
        startPose = self.getPlanningStartPose()
        self.endPosePlan = self.manipPlanner.sendEndPoseGoal(startPose, linkName, self.graspFrame.transform, waitForResponse=True)
        self.showEndPose()

    def showEndPose(self):
        endPose = robotstate.convertStateMessageToDrakePose(self.endPosePlan)
        self.showPoseFunction(endPose)


    def computePreGraspPose(self):

        #if self.planFromCurrentRobotState:
        #    startPose = self.getEstimatedRobotStatePose()
        #else:
        #    planState = self.walkingPlan.plan[-1]
        #    startPose = robotstate.convertStateMessageToDrakePose(planState)
        startPose = self.getPlanningStartPose()

        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.graspFrame)
        endPose, info = constraintSet.runIk()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(endPose, 'General', 'arm up pregrasp', side=self.graspingHand)

        self.preGraspPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(self.preGraspPlan)


    def planPostureGoal(self, groupName, poseName, side=None):

        #startPose = self.getEstimatedRobotStatePose()
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, groupName, poseName, side=side)
        self.posturePlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.planPlaybackFunction([self.posturePlan])


    def getEndEffectorLinkName(self):
        linkMap = {
                      'left' : 'l_hand',
                      'right': 'r_hand'
                  }
        return linkMap[self.graspingHand]

        



    def computeGraspPlan(self):

        startPose = self.getPlanningStartPose()

        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.graspFrame, lockTorso=True)
        endPose, info = constraintSet.runIk()
        self.graspPlan = constraintSet.runIkTraj()

        self.addPlan(self.graspPlan)


    def initGazeConstraintSet(self):
        startPose = self.getPlanningStartPose()
        startPoseName = 'gaze_plan_start'
        endPoseName = 'gaze_plan_end'
        self.ikPlanner.addPose(startPose, startPoseName)
        self.ikPlanner.addPose(startPose, endPoseName)
        constraints = self.ikPlanner.createMovingBodyConstraints(startPoseName, lockBack=False, lockBase=False, lockLeftArm=self.graspingHand=='right', lockRightArm=self.graspingHand=='left')
        self.constraintSet = ikplanner.ConstraintSet(self.ikPlanner, constraints, startPoseName, endPoseName)
        self.constraintSet.endPose = startPose


    def appendGazeConstraintsForTargetFrame(self, goalFrame, t):

        if self.constraintSet is None:
            self.initGazeConstraintSet()

        graspToHandLinkFrame = self.ikPlanner.newGraspToHandFrame(self.graspingHand)

        positionConstraint, _ = self.ikPlanner.createPositionOrientationGraspConstraints(self.graspingHand, goalFrame, graspToHandLinkFrame)
        gazeConstraint = self.ikPlanner.createGazeGraspConstraint(self.graspingHand, goalFrame, graspToHandLinkFrame)

        positionConstraint.tspan = [t, t]
        gazeConstraint.tspan = [t, t]

        self.constraintSet.constraints.extend([positionConstraint, gazeConstraint])


    def planGazeTrajectory(self):

        self.ikPlanner.ikServer.usePointwise = False
        plan = self.constraintSet.runIkTraj()
        self.constraintSet = None

        self.addPlan(plan)


    def commitFootstepPlan(self):
        self.footstepPlanner.commitFootstepPlan(self.footstepPlan)

    def commitPreGraspPlan(self):
        self.manipPlanner.commitManipPlan(self.preGraspPlan)

    def commitGraspPlan(self):
        self.manipPlanner.commitManipPlan(self.graspPlan)

    def commitStandPlan(self):
        self.manipPlanner.commitManipPlan(self.standPlan)

    def sendPelvisCrouch(self):
        self.atlasDriver.sendPelvisHeightCommand(0.7)

    def sendPelvisStand(self):
        self.atlasDriver.sendPelvisHeightCommand(0.8)

    def computeStandPlan(self):
        startPose = self.getPlanningStartPose()
        self.standPlan = self.ikPlanner.computeStandPlan(startPose)

    def sendOpenHand(self):
        self.handDriver.sendOpen()

    def sendCloseHand(self):
        self.handDriver.sendClose(60)

    def sendNeckPitchLookDown(self):
        self.multisenseDriver.setNeckPitch(40)

    def sendNeckPitchLookForward(self):
        self.multisenseDriver.setNeckPitch(15)


    def waitForAtlasBehaviorAsync(self, behaviorName):
        assert behaviorName in self.atlasDriver.getBehaviorMap().values()
        while self.atlasDriver.getCurrentBehaviorName() != behaviorName:
            yield


    def printAsync(self, s):
        yield
        print s


    def userPrompt(self, message):

        if not self.userPromptEnabled:
            return

        yield
        result = raw_input(message)
        if result != 'y':
            raise Exception('user abort.')


    def delay(self, delayTimeInSeconds):
        yield
        t = SimpleTimer()
        while t.elapsed() < delayTimeInSeconds:
            yield


    def waitForCleanLidarSweepAsync(self):
        currentRevolution = self.multisenseDriver.displayedRevolution
        desiredRevolution = currentRevolution + 2
        while self.multisenseDriver.displayedRevolution < desiredRevolution:
            yield


    def spawnValveAffordance(self):

        valveFrame = self.computeValveFrame(self.robotModel)

        folder = om.getOrCreateContainer('affordances')
        z = DebugData()
        z.addLine ( np.array([0, 0, -0.0254]) , np.array([0, 0, 0.0254]), radius= self.valveRadius) #foam valves
        valveMesh = z.getPolyData()

        self.valveAffordance = vis.showPolyData(valveMesh, 'valve', color=[0.0, 1.0, 0.0], cls=vis.AffordanceItem, parent=folder, alpha=0.3)
        self.valveAffordance.actor.SetUserTransform(valveFrame)
        self.valveFrame = vis.showFrame(valveFrame, 'valve frame', parent=self.valveAffordance, visible=False, scale=0.2)

        self.computeGraspFrame()
        self.computeStanceFrame()
        self.computePointerTipFrame(0)

    def findValveAffordance(self):
        self.valveAffordance = om.findObjectByName('valve')
        self.valveFrame = om.findObjectByName('valve frame')

    def getEstimatedRobotStatePose(self):
        return np.array(self.sensorJointController.getPose('EST_ROBOT_STATE'))


    def getPlanningStartPose(self):
        if self.planFromCurrentRobotState:
            return self.getEstimatedRobotStatePose()
        else:
            if self.plans:
                return robotstate.convertStateMessageToDrakePose(self.plans[-1].plan[-1])
            else:
                return self.getEstimatedRobotStatePose()
                
    #def getPlanningStartPose(self):
    #    if self.planFromCurrentRobotState:
    #        return self.getEstimatedRobotStatePose()
    #    else:
    #        assert False


    def cleanupFootstepPlans(self):
        om.removeFromObjectModel(om.findObjectByName('walking goal'))
        om.removeFromObjectModel(om.findObjectByName('footstep plan'))

    def playNominalPlan(self):
        #plans = [self.walkingPlan, self.preGraspPlan, self.graspPlan]
        #plans = [self.preGraspPlan, self.graspPlan]
        assert None not in self.plans
        self.planPlaybackFunction(self.plans)

    def playPreGraspPlan(self):
        self.planPlaybackFunction([self.preGraspPlan])

    def playGraspPlan(self):
        self.planPlaybackFunction([self.graspPlan])


    def playStandPlan(self):
        self.planPlaybackFunction([self.standPlan])

    def computeNominalPlan(self):

        # False if simulating
        self.planFromCurrentRobotState = False

        self.findValveAffordance()
        self.computeGraspFrame()
        self.computeStanceFrame()
        #self.computeFootstepPlan()
        #self.computeWalkingPlan()
        self.computePreGraspPose()
        self.computeGraspPlan()




        self.computePointerTipFrame(0)
        self.appendGazeConstraintsForTargetFrame(self.pointerTipFrameDesired, 1)
        self.computePointerTipFrame(1)
        self.appendGazeConstraintsForTargetFrame(self.pointerTipFrameDesired, 2)
        self.planGazeTrajectory()


        noScribeSamples =12
        noTurns=2
        for i in range(0,noScribeSamples*noTurns):
            self.computePointerTipFrame(1)
            self.appendGazeConstraintsForTargetFrame(self.pointerTipFrameDesired, i+1)
            self.nextScribeAngle = self.nextScribeAngle + self.scribeDirection*360.0/noScribeSamples
            print i , self.nextScribeAngle

        self.planGazeTrajectory()



        self.playNominalPlan()

    def sendPlanWithHeightMode(self):
        self.atlasDriver.sendPlanUsingBdiHeight(True)

    def autonomousExecute(self):

        self.planFromCurrentRobotState = True

        taskQueue = AsyncTaskQueue()

        # stand and open hand
        taskQueue.addTask(self.userPrompt('stand and open hand. continue? y/n: '))
        taskQueue.addTask(self.atlasDriver.sendStandCommand)
        taskQueue.addTask(self.sendOpenHand)
        taskQueue.addTask(self.sendPlanWithHeightMode)

        # user prompt
        taskQueue.addTask(self.userPrompt('sending neck pitch forward. continue? y/n: '))

        # set neck pitch
        taskQueue.addTask(self.printAsync('neck pitch forward'))
        taskQueue.addTask(self.sendNeckPitchLookForward)
        taskQueue.addTask(self.delay(1.0))

        # user prompt
        taskQueue.addTask(self.userPrompt('perception and fitting. continue? y/n: '))

        # perception & fitting
        taskQueue.addTask(self.printAsync('waiting for clean lidar sweep'))
        taskQueue.addTask(self.waitForCleanLidarSweepAsync)


        taskQueue.addTask(self.printAsync('fitting valve affordance'))
        taskQueue.addTask(self.affordanceFitFunction)
        taskQueue.addTask(self.findValveAffordance)

        # compute grasp & stance
        taskQueue.addTask(self.printAsync('computing grasp and stance frames'))
        taskQueue.addTask(self.computeGraspFrame)
        taskQueue.addTask(self.computeStanceFrame)

        # footstep plan
        taskQueue.addTask(self.printAsync('compute footstep plan'))
        taskQueue.addTask(self.computeFootstepPlan)

        # user prompt
        taskQueue.addTask(self.userPrompt('sending footstep plan. continue? y/n: '))

        # walk
        taskQueue.addTask(self.printAsync('walking'))
        taskQueue.addTask(self.commitFootstepPlan)
        taskQueue.addTask(self.waitForAtlasBehaviorAsync('step'))
        taskQueue.addTask(self.waitForAtlasBehaviorAsync('stand'))

        # user prompt
        taskQueue.addTask(self.userPrompt('sending neck pitch. continue? y/n: '))

        # set neck pitch
        taskQueue.addTask(self.printAsync('neck pitch down'))
        taskQueue.addTask(self.sendNeckPitchLookDown)
        taskQueue.addTask(self.delay(1.0))

        # user prompt
        #taskQueue.addTask(self.userPrompt('crouch. continue? y/n: '))

        # crouch
        #taskQueue.addTask(self.printAsync('send manip mode'))
        #taskQueue.addTask(self.atlasDriver.sendManipCommand)
        #taskQueue.addTask(self.delay(1.0))
        #taskQueue.addTask(self.printAsync('crouching'))
        #taskQueue.addTask(self.sendPelvisCrouch)
        #taskQueue.addTask(self.delay(3.0))


        # user prompt
        taskQueue.addTask(self.userPrompt('plan pre grasp. continue? y/n: '))


        # compute pre grasp plan
        taskQueue.addTask(self.printAsync('computing pre grasp plan'))
        taskQueue.addTask(self.computePreGraspPose)
        taskQueue.addTask(self.playPreGraspPlan)

        # user prompt
        taskQueue.addTask(self.userPrompt('commit manip plan. continue? y/n: '))

        # commit pre grasp plan
        taskQueue.addTask(self.atlasDriver.sendManipCommand)
        taskQueue.addTask(self.delay(1.0))
        taskQueue.addTask(self.printAsync('commit pre grasp plan'))
        taskQueue.addTask(self.commitPreGraspPlan)
        taskQueue.addTask(self.delay(10.0))


        # user prompt
        taskQueue.addTask(self.userPrompt('perception and fitting. continue? y/n: '))

        # perception & fitting
        taskQueue.addTask(self.printAsync('waiting for clean lidar sweep'))
        taskQueue.addTask(self.waitForCleanLidarSweepAsync)

        taskQueue.addTask(self.printAsync('fitting valve affordance'))
        taskQueue.addTask(self.affordanceFitFunction)
        taskQueue.addTask(self.findValveAffordance)

        # compute valve grasp frame
        taskQueue.addTask(self.printAsync('computing valve grasp frame'))
        taskQueue.addTask(self.computeGraspFrame)


        # compute grasp plan
        taskQueue.addTask(self.printAsync('computing grasp plan'))
        taskQueue.addTask(self.computeGraspPlan)
        taskQueue.addTask(self.playGraspPlan)

        # user prompt
        taskQueue.addTask(self.userPrompt('commit manip plan. continue? y/n: '))

        # commit grasp plan
        taskQueue.addTask(self.printAsync('commit grasp plan'))
        taskQueue.addTask(self.commitGraspPlan)
        taskQueue.addTask(self.delay(10.0))

        # recompute grasp plan
        taskQueue.addTask(self.printAsync('recompute grasp plan'))
        taskQueue.addTask(self.computeGraspPlan)
        taskQueue.addTask(self.playGraspPlan)

        # user prompt
        taskQueue.addTask(self.userPrompt('commit manip plan. continue? y/n: '))

        # commit grasp plan
        taskQueue.addTask(self.printAsync('commit grasp plan'))
        taskQueue.addTask(self.commitGraspPlan)
        taskQueue.addTask(self.delay(3.0))


        # user prompt
        taskQueue.addTask(self.userPrompt('closing hand. continue? y/n: '))

        # close hand
        taskQueue.addTask(self.printAsync('close hand'))
        taskQueue.addTask(self.sendCloseHand)
        taskQueue.addTask(self.delay(3.0))


        taskQueue.addTask(self.userPrompt('send stand command. continue? y/n: '))
        taskQueue.addTask(self.atlasDriver.sendStandCommand)
        taskQueue.addTask(self.delay(5.0))
        taskQueue.addTask(self.atlasDriver.sendManipCommand)
        taskQueue.addTask(self.delay(1.0))

        '''
        # user prompt
        taskQueue.addTask(self.userPrompt('compute stand plan. continue? y/n: '))

        # stand
        taskQueue.addTask(self.computeStandPlan)
        taskQueue.addTask(self.playStandPlan)

        taskQueue.addTask(self.userPrompt('commit stand. continue? y/n: '))

        # compute pre grasp plan
        taskQueue.addTask(self.commitStandPlan)
        taskQueue.addTask(self.delay(10.0))
        '''

        # user prompt
        #taskQueue.addTask(self.userPrompt('commit manip plan. continue? y/n: '))

        # commit pre grasp plan
        #taskQueue.addTask(self.printAsync('commit pre grasp plan'))
        #taskQueue.addTask(self.commitPreGraspPlan)
        #taskQueue.addTask(self.delay(10.0))

        taskQueue.addTask(self.printAsync('done!'))

        return taskQueue


