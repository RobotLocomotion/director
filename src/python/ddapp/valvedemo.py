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
from ddapp import planplayback

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

        self.planFromCurrentRobotState = True
        self.visOnly = True
        self.useFootstepPlanner = False


        self.userPromptEnabled = True
        self.walkingPlan = None
        self.preGraspPlan = None
        self.graspPlan = None
        self.constraintSet = None

        self.plans = []

        self.scribeInAir = False
        self.scribeDirection = 1 # 1 = clockwise | -1 = anticlockwise
        self.startAngle = -30 # suitable for both types of valve
        self.nextScribeAngle = self.startAngle

        self.valveRadius = 0.19558 # nominal initial value. 7.7in radius metal valve
        
        ### Testing Parameters:
        self.valveHeight = 1.2192 # 4ft


    def scribeRadius(self):
        if self.scribeInAir:
            return self.valveRadius - 0.08
        else:
            return self.valveRadius - 0.08

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

        position = [0.85, 0.4, self.valveHeight]
        rpy = [180, -90, 0]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.computeGroundFrame(robotModel))
        return t


    def computeGraspFrame(self):

        assert self.valveAffordance

        # reach to center and back - for palm point
        position = [0.0, 0.0, -0.1]
        rpy = [90, 0, 180]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.valveFrame.transform)

        self.graspFrame = vis.updateFrame(t, 'valve grasp frame', parent=self.valveAffordance, visible=False, scale=0.2)

        self.frameSync = vis.FrameSync()
        self.frameSync.addFrame(self.graspFrame)
        self.frameSync.addFrame(self.valveFrame)


    def removePointerTipFrames(self):
        for obj in om.getObjects():
            if obj.getProperty('Name') == 'pointer tip frame desired':
                om.removeFromObjectModel(obj)


    def computePointerTipFrame(self, engagedTip):
        if engagedTip:
            tipDepth = 0.0
        else:
            tipDepth = -0.1 # - is outside the wheel

        assert self.valveAffordance

        position = [ self.scribeRadius()*math.cos( math.radians( self.nextScribeAngle )) ,  self.scribeRadius()*math.sin( math.radians( self.nextScribeAngle ))  , tipDepth]
        rpy = [90, 0, 180]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.valveFrame.transform)

        self.pointerTipFrameDesired = vis.showFrame(t, 'pointer tip frame desired', parent=self.valveAffordance, visible=True, scale=0.2)


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


        if self.scribeInAir:
            position = [-0.6, -0.4, 0.0] # stand further away when scribing in air
        else:
            position = [-0.48, -0.4, 0.0]

        rpy = [0, 0, 16]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(graspGroundFrame)

        self.graspStanceFrame = vis.updateFrame(t, 'valve grasp stance', parent=self.valveAffordance, visible=False, scale=0.2)

        self.frameSync.addFrame(self.graspStanceFrame)


    def moveRobotToStanceFrame(self):
        frame = self.graspStanceFrame.transform

        self.sensorJointController.setPose('q_nom')
        stancePosition = frame.GetPosition()
        stanceOrientation = frame.GetOrientation()

        self.sensorJointController.q[:2] = [stancePosition[0], stancePosition[1]]
        self.sensorJointController.q[5] = math.radians(stanceOrientation[2])
        self.sensorJointController.push()


    def computeFootstepPlan(self):
        startPose = self.getPlanningStartPose()
        goalFrame = self.graspStanceFrame.transform
        request = self.footstepPlanner.constructFootstepPlanRequest(startPose, goalFrame)
        self.footstepPlan = self.footstepPlanner.sendFootstepPlanRequest(request, waitForResponse=True)


    def computeWalkingPlan(self):
        startPose = self.getPlanningStartPose()
        self.walkingPlan = self.footstepPlanner.sendWalkingPlanRequest(self.footstepPlan, startPose, waitForResponse=True)
        self.addPlan(self.walkingPlan)


    def computePreGraspPlan(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'arm up pregrasp', side=self.graspingHand)
        self.preGraspPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(self.preGraspPlan)


    def computeGraspPlan(self):

        startPose = self.getPlanningStartPose()

        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.graspFrame, lockTorso=True)
        endPose, info = constraintSet.runIk()
        self.graspPlan = constraintSet.runIkTraj()

        self.addPlan(self.graspPlan)


    def initGazeConstraintSet(self, goalFrame):

        # create constraint set
        startPose = self.getPlanningStartPose()
        startPoseName = 'gaze_plan_start'
        endPoseName = 'gaze_plan_end'
        self.ikPlanner.addPose(startPose, startPoseName)
        self.ikPlanner.addPose(startPose, endPoseName)
        self.constraintSet = ikplanner.ConstraintSet(self.ikPlanner, [], startPoseName, endPoseName)
        self.constraintSet.endPose = startPose

        # add body constraints
        bodyConstraints = self.ikPlanner.createMovingBodyConstraints(startPoseName, lockBase=True, lockBack=False, lockLeftArm=self.graspingHand=='right', lockRightArm=self.graspingHand=='left')
        self.constraintSet.constraints.extend(bodyConstraints)

        # add gaze constraint
        self.graspToHandLinkFrame = self.ikPlanner.newGraspToHandFrame(self.graspingHand)
        gazeConstraint = self.ikPlanner.createGazeGraspConstraint(self.graspingHand, goalFrame, self.graspToHandLinkFrame)
        self.constraintSet.constraints.insert(0, gazeConstraint)


    def appendDistanceConstraint(self):

        # add point to point distance constraint
        c = ikplanner.ik.PointToPointDistanceConstraint()
        c.bodyNameA = self.ikPlanner.getHandLink(self.graspingHand)
        c.bodyNameB = 'world'
        c.pointInBodyA = self.graspToHandLinkFrame
        c.pointInBodyB = self.valveFrame.transform
        c.lowerBound = [self.scribeRadius()]
        c.upperBound = [self.scribeRadius()]
        self.constraintSet.constraints.insert(0, c)


    def appendGazeConstraintForTargetFrame(self, goalFrame, t):

        gazeConstraint = self.ikPlanner.createGazeGraspConstraint(self.graspingHand, goalFrame, self.graspToHandLinkFrame)
        gazeConstraint.tspan = [t, t]
        self.constraintSet.constraints.append(gazeConstraint)


    def appendPositionConstraintForTargetFrame(self, goalFrame, t):
        positionConstraint, _ = self.ikPlanner.createPositionOrientationGraspConstraints(self.graspingHand, goalFrame, self.graspToHandLinkFrame)
        positionConstraint.tspan = [t, t]
        self.constraintSet.constraints.append(positionConstraint)


    def planGazeTrajectory(self):

        self.ikPlanner.ikServer.usePointwise = False

        plan = self.constraintSet.runIkTraj()
        self.addPlan(plan)


    def commitFootstepPlan(self):
        self.footstepPlanner.commitFootstepPlan(self.footstepPlan)


    def commitManipPlan(self):
            self.manipPlanner.commitManipPlan(self.plans[-1])

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
        z.addLine ( np.array([0, 0, -0.0254]) , np.array([0, 0, 0.0254]), radius= self.valveRadius)
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

        self.computeGraspFrame()
        self.computeStanceFrame()
        self.computePointerTipFrame(0)


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


    def removeFootstepPlan(self):
        om.removeFromObjectModel(om.findObjectByName('footstep plan'))
        self.footstepPlan = None


    def playNominalPlan(self):
        assert None not in self.plans
        self.planPlaybackFunction(self.plans)


    def computePreGraspPlanGaze(self):

        self.computePointerTipFrame(0)
        self.initGazeConstraintSet(self.pointerTipFrameDesired)
        self.appendPositionConstraintForTargetFrame(self.pointerTipFrameDesired, 1)
        self.planGazeTrajectory()


    def computeInsertPlan(self):
        self.computePointerTipFrame(1)
        self.initGazeConstraintSet(self.pointerTipFrameDesired)
        self.appendPositionConstraintForTargetFrame(self.pointerTipFrameDesired, 1)
        self.planGazeTrajectory()


    def computeTurnPlan(self, turnDegrees=360, numberOfSamples=12):

        degreeStep = float(turnDegrees) / numberOfSamples
        tipMode = 0 if self.scribeInAir else 1

        self.computePointerTipFrame(tipMode)
        self.initGazeConstraintSet(self.pointerTipFrameDesired)
        #self.appendDistanceConstraint()

        for i in xrange(numberOfSamples):
            self.nextScribeAngle += self.scribeDirection*degreeStep
            self.computePointerTipFrame(tipMode)
            self.appendPositionConstraintForTargetFrame(self.pointerTipFrameDesired, i+1)

        gazeConstraint = self.constraintSet.constraints[0]
        assert isinstance(gazeConstraint, ikplanner.ik.WorldGazeDirConstraint)
        gazeConstraint.tspan = [1.0, numberOfSamples]

        self.planGazeTrajectory()


    def computeStandPlan(self):
        startPose = self.getPlanningStartPose()
        self.standPlan = self.ikPlanner.computeNominalPlan(startPose)
        self.addPlan(self.standPlan)


    def computeNominalPlan(self):

        self.plans = []

        self.removeFootstepPlan()
        self.removePointerTipFrames()

#        self.findValveAffordance()
#        self.computeGraspFrame()
#        self.computeStanceFrame()

#        if self.useFootstepPlanner:
#            self.computeFootstepPlan()
#            self.computeWalkingPlan()
#        else:
#            self.moveRobotToStanceFrame()

        self.computePreGraspPlan()
        self.computePreGraspPlanGaze()

        if not self.scribeInAir:
            self.computeInsertPlan()

        self.computeTurnPlan()
        self.computePreGraspPlanGaze()
        self.computePreGraspPlan()
        self.computeStandPlan()


        self.playNominalPlan()


    def waitForPlanExecution(self, plan):
        planElapsedTime = planplayback.PlanPlayback.getPlanElapsedTime(plan)
        print 'waiting for plan execution:', planElapsedTime

        return self.delay(planElapsedTime + 1.0)


    def animateLastPlan(self):
        plan = self.plans[-1]

        if not self.visOnly:
            self.commitManipPlan()

        return self.waitForPlanExecution(plan)


    def addWalkingTasksToQueue(self, taskQueue, planFunc, walkFunc):

        if self.useFootstepPlanner:
            taskQueue.addTask(planFunc)

            if self.visOnly:
                taskQueue.addTask(self.computeWalkingPlan)
                taskQueue.addTask(self.animateLastPlan)
            else:

                taskQueue.addTask(self.userPrompt('send stand command. continue? y/n: '))
                taskQueue.addTask(self.atlasDriver.sendStandCommand)
                taskQueue.addTask(self.waitForAtlasBehaviorAsync('stand'))

                taskQueue.addTask(self.userPrompt('commit footsteps. continue? y/n: '))
                taskQueue.addTask(self.commitFootstepPlan)
                taskQueue.addTask(self.waitForAtlasBehaviorAsync('step'))
                taskQueue.addTask(self.waitForAtlasBehaviorAsync('stand'))

            taskQueue.addTask(self.removeFootstepPlan)
        else:
            taskQueue.addTask(walkFunc)



    def autonomousExecute(self):


        taskQueue = AsyncTaskQueue()


        taskQueue.addTask(self.printAsync('computing grasp and stance frames'))
        taskQueue.addTask(self.removePointerTipFrames)
        taskQueue.addTask(self.findValveAffordance)
        taskQueue.addTask(self.computeGraspFrame)
        taskQueue.addTask(self.computeStanceFrame)


        self.addWalkingTasksToQueue(taskQueue, self.computeFootstepPlan, self.moveRobotToStanceFrame)
        self.addWalkingTasksToQueue(taskQueue, self.computeFootstepPlan, self.moveRobotToStanceFrame)

        taskQueue.addTask(self.atlasDriver.sendManipCommand)
        taskQueue.addTask(self.waitForAtlasBehaviorAsync('manip'))


        planningFunctions = [
                    self.computePreGraspPlan,
                    self.computePreGraspPlanGaze,
                    self.computeInsertPlan,
                    self.computeTurnPlan,
                    self.computePreGraspPlanGaze,
                    self.computePreGraspPlan,
                    self.computeStandPlan,
                    ]


        for planFunc in planningFunctions:
            taskQueue.addTask(planFunc)
            taskQueue.addTask(self.userPrompt('continue? y/n: '))
            taskQueue.addTask(self.animateLastPlan)


        taskQueue.addTask(self.printAsync('done!'))

        return taskQueue


