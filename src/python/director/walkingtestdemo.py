import os
import sys
import vtkAll as vtk
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
from ddapp import affordanceitems
from ddapp import robotstate
from ddapp import robotplanlistener
from ddapp import segmentation
from ddapp import planplayback

import drc as lcmdrc
import copy

from PythonQt import QtCore, QtGui



class walkingTestDemo(object):

    def __init__(self, robotModel, playbackRobotModel, teleopRobotModel, footstepPlanner, manipPlanner, ikPlanner,
                 lhandDriver, rhandDriver, atlasDriver, multisenseDriver, sensorJointController,
                 planPlaybackFunction, showPoseFunction):
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
        self.sensorJointController = sensorJointController
        self.planPlaybackFunction = planPlaybackFunction
        self.showPoseFunction = showPoseFunction

        self.goalTransform1 = transformUtils.frameFromPositionAndRPY([1,0,0],[0,0,0])
        self.goalTransform2 = transformUtils.frameFromPositionAndRPY([2,0,0],[0,0,10])
        #self.goalTransform2 = transformUtils.frameFromPositionAndRPY([1,1,0],[0,0,90])

        self.visOnly = False # True for development, False for operation
        self.planFromCurrentRobotState = True # False for development, True for operation
        useDevelopment = False
        if (useDevelopment):
            self.visOnly = True # True for development, False for operation
            self.planFromCurrentRobotState = False # False for development, True for operation

        self.optionalUserPromptEnabled = False
        self.requiredUserPromptEnabled = True

        self.constraintSet = None

        self.plans = []

        self._setupSubscriptions()


    def _setupSubscriptions(self):
        sub0 = lcmUtils.addSubscriber('AUTONOMOUS_TEST_WALKING', lcmdrc.utime_t, self.autonomousTest)

    def addPlan(self, plan):
        self.plans.append(plan)


    ### Planning Functions ###############################################################
    def planNominal(self):
        startPose = self.getPlanningStartPose()
        self.standPlan = self.ikPlanner.computeNominalPlan(startPose)
        self.addPlan(self.standPlan)

    def planFootsteps(self, goalFrame):
        startPose = self.getPlanningStartPose()
        request = self.footstepPlanner.constructFootstepPlanRequest(startPose, goalFrame)
        self.footstepPlan = self.footstepPlanner.sendFootstepPlanRequest(request, waitForResponse=True)

    def planWalking(self):
        startPose = self.getPlanningStartPose()
        walkingPlan = self.footstepPlanner.sendWalkingPlanRequest(self.footstepPlan, startPose, waitForResponse=True)
        self.addPlan(walkingPlan)


    ########## Glue Functions ####################################
    def moveRobotToStanceFrame(self, frame):
        self.sensorJointController.setPose('q_nom')
        stancePosition = frame.GetPosition()
        stanceOrientation = frame.GetOrientation()

        q = self.sensorJointController.q.copy()
        q[:2] = [stancePosition[0], stancePosition[1]]
        q[5] = math.radians(stanceOrientation[2])
        self.sensorJointController.setPose('EST_ROBOT_STATE', q)


    def sendNeckPitchLookDown(self):
        self.multisenseDriver.setNeckPitch(40)

    def sendNeckPitchLookForward(self):
        self.multisenseDriver.setNeckPitch(15)

    def printAsync(self, s):
        yield
        print s

    def optionalUserPrompt(self, message):
        if not self.optionalUserPromptEnabled:
            return

        yield
        result = raw_input(message)
        if result != 'y':
            raise Exception('user abort.')

    def requiredUserPrompt(self, message):
        if not self.requiredUserPromptEnabled:
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


    def getEstimatedRobotStatePose(self):
        return self.sensorJointController.getPose('EST_ROBOT_STATE')

    def getPlanningStartPose(self):
        if self.planFromCurrentRobotState:
            return self.getEstimatedRobotStatePose()
        else:
            if self.plans:
                return robotstate.convertStateMessageToDrakePose(self.plans[-1].plan[-1])
            else:
                return self.getEstimatedRobotStatePose()

    def cleanupFootstepPlans(self):
        om.removeFromObjectModel(om.findObjectByName('walking goal'))
        om.removeFromObjectModel(om.findObjectByName('footstep plan'))
        self.footstepPlan = None

    def playSequenceNominal(self):
        assert None not in self.plans
        self.planPlaybackFunction(self.plans)

    def commitManipPlan(self):
        self.manipPlanner.commitManipPlan(self.plans[-1])

    def commitFootstepPlan(self):
        self.footstepPlanner.commitFootstepPlan(self.footstepPlan)

    def waitForPlanExecution(self):
        while self.atlasDriver.getControllerStatus() != 'manipulating':
            yield
        while self.atlasDriver.getControllerStatus() == 'manipulating':
            yield

    def waitForWalkExecution(self):
        while self.atlasDriver.getControllerStatus() != 'walking':
            yield
        while self.atlasDriver.getControllerStatus() == 'walking':
            yield

    def waitForPlanAnimation(self, plan):
        planElapsedTime = planplayback.PlanPlayback.getPlanElapsedTime(plan)
        print 'waiting for plan animation:', planElapsedTime
        return self.delay(planElapsedTime)

    def animateLastPlan(self):
        plan = self.plans[-1]
        if self.visOnly:
            return self.waitForPlanAnimation(plan)
        else:
            self.commitManipPlan()
            return self.waitForPlanExecution()


    ######### Nominal Plans and Execution  #################################################################
    def planSequence(self, playbackNominal=True):

        self.planFromCurrentRobotState = False
        self.plans = []

        #self.moveRobotToStanceFrame(self.goalTransform1)

        self.planFootsteps(self.goalTransform1)
        self.planWalking()

        self.planFootsteps(self.goalTransform2)
        self.planWalking()

        self.planNominal()

        if (playbackNominal is True):
            self.playSequenceNominal()


    def autonomousTest(self, msg):
        print "Got the autonomousTest message, executing walking test sequence"
        q = self.autonomousExecute()
        q.start()


    def sendAutonmousTestDone(self):
        msg = lcmdrc.utime_t()
        msg.utime = getUtime()
        lcmUtils.publish('AUTONOMOUS_TEST_WALKING_DONE', msg)


    def autonomousExecute(self):

        self.planFromCurrentRobotState = True
        self.visOnly = False

        taskQueue = AsyncTaskQueue()

        taskQueue.addTask(self.printAsync('Plan and execute walking'))
        taskQueue.addTask( functools.partial(self.planFootsteps, self.goalTransform1) )
        taskQueue.addTask(self.optionalUserPrompt('Send footstep plan. continue? y/n: '))
        taskQueue.addTask(self.commitFootstepPlan)
        taskQueue.addTask(self.waitForWalkExecution)

        taskQueue.addTask(self.printAsync('Plan and execute walking'))
        taskQueue.addTask( functools.partial(self.planFootsteps, self.goalTransform2) )
        taskQueue.addTask(self.optionalUserPrompt('Send footstep plan. continue? y/n: '))
        taskQueue.addTask(self.commitFootstepPlan)
        taskQueue.addTask(self.waitForWalkExecution)

        taskQueue.addTask(self.sendAutonmousTestDone)

        return taskQueue
