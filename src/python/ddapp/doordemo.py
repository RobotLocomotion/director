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
from ddapp import affordanceitems
from ddapp.simpletimer import SimpleTimer
from ddapp.utime import getUtime
from ddapp import robotstate
from ddapp import robotplanlistener
from ddapp import segmentation
from ddapp import planplayback

import drc as lcmdrc

from PythonQt import QtCore, QtGui


class DoorDemo(object):

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
        self.graspingHand = 'right'

        self.endPose = None

        self.planFromCurrentRobotState = True
        self.visOnly = True
        self.useFootstepPlanner = False
        self.userPromptEnabled = True

        self.constraintSet = None
        self.plans = []

        self.pinchDistance = 0.1
        self.doorHandleFrame = None
        self.doorHandleGraspFrame = None
        self.doorHingeFrame = None


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


    def computeDoorHandleFrame(self, robotModel):

        position = [0.77, -0.4, 3*12*0.0254]
        rpy = [0, 0, 20]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.computeGroundFrame(robotModel))
        return t



    def computeDoorHandleGraspFrame(self):

        pitch = 20
        roll = 0
        yaw = 0

        position = [0.0, -0.08, 0.0]
        rpy = [0.0 + pitch, 180 + roll, -(90 + yaw)]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(transformUtils.copyFrame(self.doorHandleFrame.transform))

        self.doorHandleFrame.frameSync = vis.FrameSync()
        self.doorHandleGraspFrame = vis.updateFrame(t, 'door handle grasp frame', parent=self.doorHandleAffordance, visible=True, scale=0.2)

        self.doorHandleFrame.frameSync.addFrame(self.doorHandleFrame)
        self.doorHandleFrame.frameSync.addFrame(self.doorHandleGraspFrame, ignoreIncoming=True)


    def computeDoorHingeFrame(self):

        position = [0.0, -35 * 0.0254, 0.0]
        rpy = [0, 0, 0]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(transformUtils.copyFrame(self.doorHandleFrame.transform))

        if self.doorHingeFrame:
            self.doorHingeFrame.frameSync = None

        self.doorHingeFrame = vis.updateFrame(t, 'door hinge frame', parent=self.doorHandleAffordance, visible=True, scale=0.2)

        self.doorHingeFrame.frameSync = vis.FrameSync()
        self.doorHingeFrame.frameSync.addFrame(self.doorHingeFrame)
        self.doorHingeFrame.frameSync.addFrame(self.doorHandleFrame, ignoreIncoming=True)


    def computeDoorHandleStanceFrame(self):

        graspFrame = self.doorHandleFrame.transform

        groundFrame = self.computeGroundFrame(self.robotModel)
        groundHeight = groundFrame.GetPosition()[2]

        graspPosition = np.array(graspFrame.GetPosition())
        xaxis = [1.0, 0.0, 0.0]
        yaxis = [0.0, 1.0, 0.0]
        zaxis = [0, 0, 1]
        graspFrame.TransformVector(xaxis, xaxis)
        graspFrame.TransformVector(yaxis, yaxis)

        yaxis = np.cross(zaxis, xaxis)
        yaxis /= np.linalg.norm(yaxis)
        xaxis = np.cross(yaxis, zaxis)

        graspGroundFrame = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
        graspGroundFrame.PostMultiply()
        graspGroundFrame.Translate(graspPosition[0], graspPosition[1], groundHeight)


        #position = [-0.75, 0.3, 0.0]
        #rpy = [0, 0, -30]

        position = [-0.77, 0.4, 0.0]
        rpy = [0, 0, -20]


        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(graspGroundFrame)

        self.doorHandleStanceFrame = vis.updateFrame(t, 'door handle grasp stance', parent=self.doorHandleAffordance, visible=True, scale=0.2)
        #self.frameSync.addFrame(self.doorHandleStanceFrame)


    def printRobotDistanceToDoorHandle(self):
        pelvisFrame = self.robotModel.getLinkFrame('pelvis')
        pelvisFrame.PostMultiply()
        pelvisFrame.Concatenate(self.doorHandleFrame.transform.GetLinearInverse())
        print 'translation: %.3f, %.3f, %.3f' % pelvisFrame.GetPosition()
        print 'orientation: %.3f, %.3f, %.3f' % pelvisFrame.GetOrientation()


    def moveRobotToStanceFrame(self):
        frame = self.doorHandleStanceFrame.transform

        self.sensorJointController.setPose('q_nom')
        stancePosition = frame.GetPosition()
        stanceOrientation = frame.GetOrientation()

        self.sensorJointController.q[:2] = [stancePosition[0], stancePosition[1]]
        self.sensorJointController.q[5] = math.radians(stanceOrientation[2])
        self.sensorJointController.push()


    def computeFootstepPlan(self):
        startPose = self.getPlanningStartPose()
        goalFrame = self.doorHandleStanceFrame.transform
        request = self.footstepPlanner.constructFootstepPlanRequest(startPose, goalFrame)
        self.footstepPlan = self.footstepPlanner.sendFootstepPlanRequest(request, waitForResponse=True)


    def computeWalkingPlan(self):
        startPose = self.getPlanningStartPose()
        self.walkingPlan = self.footstepPlanner.sendWalkingPlanRequest(self.footstepPlan, startPose, waitForResponse=True)
        self.addPlan(self.walkingPlan)


    def computePreGraspPlan(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'arm up pregrasp', side=self.graspingHand)
        plan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(plan)


    def computeGraspPlan(self):

        startPose = self.getPlanningStartPose()

        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.doorHandleGraspFrame)
        endPose, info = constraintSet.runIk()
        plan = constraintSet.runIkTraj()

        self.addPlan(plan)


    def initConstraintSet(self, pinchDistance=None):

        if pinchDistance is None:
            pinchDistance = self.pinchDistance

        # create constraint set
        startPose = self.getPlanningStartPose()
        endPose = self.endPose if self.endPose is not None else startPose
        startPoseName = 'gaze_plan_start'
        endPoseName = 'gaze_plan_end'
        self.ikPlanner.addPose(startPose, startPoseName)
        self.ikPlanner.addPose(endPose, endPoseName)
        self.constraintSet = ikplanner.ConstraintSet(self.ikPlanner, [], endPoseName, startPoseName)
        self.constraintSet.endPose = endPose

        if self.endPose is not None:
            self.constraintSet.nominalPoseName = 'gaze_plan_end'


        # add body constraints
        bodyConstraints = self.ikPlanner.createMovingBodyConstraints(startPoseName, lockBase=False, lockBack=False, lockLeftArm=self.graspingHand=='right', lockRightArm=self.graspingHand=='left')
        self.constraintSet.constraints.extend(bodyConstraints)

        self.graspToHandLinkFrame = self.ikPlanner.newPalmOffsetGraspToHandFrame(self.graspingHand, distance=pinchDistance)


    def appendBaseConstraint(self):

        p = self.ikPlanner.createKneePostureConstraint([1.745, 1.745])
        p.tspan = [1.0, np.inf]
        self.constraintSet.constraints.append(p)


    def appendBackConstraint(self):

        p = ikplanner.ik.PostureConstraint()
        p.joints = ['back_bkz']
        p.jointsLowerBound = [0.1]
        p.jointsUpperBound = [0.1]
        p.tspan = [1.0, np.inf]
        self.constraintSet.constraints.append(p)


    def appendGraspConstraintForTargetFrame(self, goalFrame, t):

        positionConstraint, orientationConstraint = self.ikPlanner.createPositionOrientationGraspConstraints(self.graspingHand, goalFrame, self.graspToHandLinkFrame)

        positionConstraint.tspan = [t, t]
        orientationConstraint.tspan = [t, t]
        self.constraintSet.constraints.append(positionConstraint)
        self.constraintSet.constraints.append(orientationConstraint)

        vis.showFrame(goalFrame, 'turn frame %d' % t, scale=0.1, visible=True, parent='debug')


    def planGraspTrajectory(self):

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


    def spawnDoorAffordance(self):

        groundFrame = self.computeGroundFrame(self.robotModel)
        groundHeight = groundFrame.GetPosition()[2]

        doorOffsetX = 0.9
        doorOffsetY = 0.0

        doorGroundFrame = transformUtils.frameFromPositionAndRPY([doorOffsetX, doorOffsetY, 0.0], [0.0, 0.0, 0.0])
        doorGroundFrame.PreMultiply()
        doorGroundFrame.Concatenate(groundFrame)

        doorGroundFrame = vis.showFrame(doorGroundFrame, 'door ground frame')

        doorWidth = 36 * 0.0254
        doorHeight = 81 * 0.0254
        doorDepth = 1.5 * 0.0254

        doorSide = 1 # handle on left
        doorSide = -1 # handle on right
        handleHeightFromGround = 37 * 0.0254
        handleDistanceFromEdge = 2 * 0.0254
        handleDistanceFromDoor = 3.0 * 0.0254
        handleLength = 5 * 0.0254

        doorJamWidth = 0.5
        doorJamDepth = 4.5 * 0.0254


        handleFrame = transformUtils.frameFromPositionAndRPY([-handleDistanceFromDoor, doorSide*(doorWidth/2.0 - handleDistanceFromEdge - handleLength/2.0), handleHeightFromGround], [0.0, 0.0, 0.0])
        handleFrame.PreMultiply()
        handleFrame.Concatenate(doorGroundFrame.transform)


        doorFrame = transformUtils.frameFromPositionAndRPY([0.0, 0.0, doorHeight/2.0], [0.0, 0.0, 0.0])
        doorFrame.PreMultiply()
        doorFrame.Concatenate(doorGroundFrame.transform)


        leftDoorJamFrame = transformUtils.frameFromPositionAndRPY([0.0, (doorWidth/2.0 + doorJamWidth/2.0), doorHeight/2.0], [0.0, 0.0, 0.0])
        leftDoorJamFrame.PreMultiply()
        leftDoorJamFrame.Concatenate(doorGroundFrame.transform)

        rightDoorJamFrame = transformUtils.frameFromPositionAndRPY([0.0, -(doorWidth/2.0 + doorJamWidth/2.0), doorHeight/2.0], [0.0, 0.0, 0.0])
        rightDoorJamFrame.PreMultiply()
        rightDoorJamFrame.Concatenate(doorGroundFrame.transform)



        desc = dict(classname='BoxAffordanceItem', Name='door handle',
           pose=transformUtils.poseFromTransform(handleFrame), Dimensions=[0.02, handleLength, 0.02], Color=[0.0, 1.0, 0.0])
        handleAffordance = segmentation.affordanceManager.newAffordanceFromDescription(desc)

        desc = dict(classname='BoxAffordanceItem', Name='door',
           pose=transformUtils.poseFromTransform(doorFrame), Dimensions=[doorDepth, doorWidth, doorHeight], Color=[0.5, 0.5, 0.5])
        doorAffordance = segmentation.affordanceManager.newAffordanceFromDescription(desc)

        desc = dict(classname='BoxAffordanceItem', Name='left door jam',
           pose=transformUtils.poseFromTransform(leftDoorJamFrame), Dimensions=[doorJamDepth, doorJamWidth, doorHeight], Color=[0.7, 0.0, 0.0])
        leftDoorJamAffordance = segmentation.affordanceManager.newAffordanceFromDescription(desc)

        desc = dict(classname='BoxAffordanceItem', Name='right door jam',
           pose=transformUtils.poseFromTransform(rightDoorJamFrame), Dimensions=[doorJamDepth, doorJamWidth, doorHeight], Color=[0.7, 0.0, 0.0])
        rightDoorJamAffordance = segmentation.affordanceManager.newAffordanceFromDescription(desc)

        doorFrame = doorAffordance.getChildFrame()
        handleFrame = handleAffordance.getChildFrame()

        leftDoorJamFrame = leftDoorJamAffordance.getChildFrame()
        rightDoorJamFrame = rightDoorJamAffordance.getChildFrame()

        self.doorFrameSync = vis.FrameSync()
        self.doorFrameSync.addFrame(doorGroundFrame)
        self.doorFrameSync.addFrame(doorFrame, ignoreIncoming=True)
        self.doorFrameSync.addFrame(leftDoorJamFrame, ignoreIncoming=True)
        self.doorFrameSync.addFrame(rightDoorJamFrame, ignoreIncoming=True)

        self.doorHandleFrameSync = vis.FrameSync()
        self.doorHandleFrameSync.addFrame(doorFrame)
        self.doorHandleFrameSync.addFrame(handleFrame, ignoreIncoming=True)



    def spawnDoorHandleAffordance(self):

        handleFrame = self.computeDoorHandleFrame(self.robotModel)

        folder = om.getOrCreateContainer('affordances')
        d = DebugData()
        d.addLine ([0.0, 0.0, 0.0], [0.0, -0.15, 0.0], radius=0.015)
        mesh = d.getPolyData()

        self.doorHandleAffordance = vis.showPolyData(mesh, 'door handle', color=[0.0, 1.0, 0.0], cls=affordanceitems.AffordanceItem, parent=folder, alpha=1.0)
        self.doorHandleAffordance.actor.SetUserTransform(handleFrame)
        self.doorHandleFrame = vis.showFrame(handleFrame, 'door handle frame', parent=self.doorHandleAffordance, visible=True, scale=0.2)

        self.computeDoorHandleGraspFrame()
        self.computeDoorHingeFrame()
        self.computeDoorHandleStanceFrame()


    def findDoorHandleAffordance(self):

        self.doorHandleAffordance = om.findObjectByName('door handle')
        self.doorHandleFrame = self.doorHandleAffordance.findChild('door handle frame')

        self.computeDoorHandleGraspFrame()
        self.computeDoorHingeFrame()
        self.computeDoorHandleStanceFrame()


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


    def computePreGraspPlanReach(self, pinchDistance=0.14):

        self.initConstraintSet(pinchDistance=pinchDistance)
        self.appendGraspConstraintForTargetFrame(self.doorHandleGraspFrame.transform, 1)

        self.appendBaseConstraint()
        #self.appendBackConstraint()

        self.planGraspTrajectory()


    def computePreGraspPlanTouch(self):

        self.initConstraintSet()
        self.appendGraspConstraintForTargetFrame(self.doorHandleGraspFrame.transform, 1)

        self.appendBaseConstraint()
        #self.appendBackConstraint()

        self.planGraspTrajectory()


    def turnDoorHandle(self, degrees):
        self.doorHandleFrame.transform.PreMultiply()
        self.doorHandleFrame.transform.RotateX(degrees)
        self.doorHandleFrame.transform.Modified()
        self.doorHandleFrame.transform.PostMultiply()


    def turnDoorHinge(self, degrees):
        self.doorHingeFrame.transform.PreMultiply()
        self.doorHingeFrame.transform.RotateZ(degrees)
        self.doorHingeFrame.transform.Modified()
        self.doorHingeFrame.transform.PostMultiply()


    def pitchGrasp(self, degrees):
        self.doorHandleGraspFrame.transform.PreMultiply()
        self.doorHandleGraspFrame.transform.RotateX(degrees)
        self.doorHandleGraspFrame.transform.Modified()
        self.doorHandleGraspFrame.transform.PostMultiply()


    def yawGrasp(self, degrees):
        self.doorHandleGraspFrame.transform.PreMultiply()
        self.doorHandleGraspFrame.transform.RotateZ(degrees)
        self.doorHandleGraspFrame.transform.Modified()
        self.doorHandleGraspFrame.transform.PostMultiply()


    def computeTurnEndPose(self, turnDegrees=60, graspYawDegrees=10):

        self.initConstraintSet()
        self.appendBaseConstraint()
        self.turnDoorHandle(turnDegrees)
        self.yawGrasp(graspYawDegrees)
        self.appendGraspConstraintForTargetFrame(transformUtils.copyFrame(self.doorHandleGraspFrame.transform), 1.0)
        self.turnDoorHandle(-turnDegrees)
        self.yawGrasp(-graspYawDegrees)

        self.endPose, info = self.constraintSet.runIk()

        self.ikPlanner.addPose(self.endPose, 'door_turn_end_pose')

        self.showPoseFunction(self.endPose)


    def computeTurnPlan(self, turnDegrees=60, graspYawDegrees=10, numberOfSamples=15):

        degreeStep = float(turnDegrees) / numberOfSamples
        yawDegreeStep = float(graspYawDegrees) / numberOfSamples

        self.initConstraintSet()
        self.appendBaseConstraint()

        for i in xrange(numberOfSamples):
            self.turnDoorHandle(degreeStep)
            self.yawGrasp(yawDegreeStep)
            self.appendGraspConstraintForTargetFrame(transformUtils.copyFrame(self.doorHandleGraspFrame.transform), i+1)

        self.turnDoorHandle(-turnDegrees)
        self.yawGrasp(-graspYawDegrees)

        self.planGraspTrajectory()


    def computeOpenPlan(self, turnDegrees=30, numberOfSamples=15, handleDegrees=60, graspYawDegrees=10, straightenDegrees=35):

        degreeStep = float(turnDegrees) / numberOfSamples
        degreeStepStraighten = float(straightenDegrees) / numberOfSamples

        self.initConstraintSet()
        self.appendBaseConstraint()

        self.turnDoorHandle(handleDegrees)
        self.yawGrasp(graspYawDegrees)

        for i in xrange(numberOfSamples):

            self.turnDoorHinge(degreeStep)
            self.pitchGrasp(-degreeStepStraighten)

            self.appendGraspConstraintForTargetFrame(transformUtils.copyFrame(self.doorHandleGraspFrame.transform), i+1)

        self.turnDoorHinge(-turnDegrees)
        self.turnDoorHandle(-handleDegrees)
        self.yawGrasp(-graspYawDegrees)
        self.pitchGrasp(straightenDegrees)


        self.planGraspTrajectory()


    def computeStandPlan(self):
        startPose = self.getPlanningStartPose()
        self.standPlan = self.ikPlanner.computeNominalPlan(startPose)
        self.addPlan(self.standPlan)


    def computeNominalPlan(self):

        self.plans = []

        self.removeFootstepPlan()
        self.removePointerTipFrames()
        self.removePointerTipPath()

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
        taskQueue.addTask(self.removePointerTipPath)
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


