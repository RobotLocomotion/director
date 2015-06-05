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
from ddapp.ikparameters import IkParameters
from ddapp import ioUtils
from ddapp.simpletimer import SimpleTimer
from ddapp.utime import getUtime
from ddapp import affordanceitems
from ddapp import robotstate
from ddapp import robotplanlistener
from ddapp import segmentation
from ddapp import planplayback
from ddapp import affordanceupdater
from ddapp import segmentationpanel
from ddapp import vtkNumpy as vnp

from ddapp.tasks.taskuserpanel import TaskUserPanel
from ddapp.tasks.taskuserpanel import ImageBasedAffordanceFit

import ddapp.tasks.robottasks as rt
import ddapp.tasks.taskmanagerwidget as tmw

import drc as lcmdrc
import copy

from PythonQt import QtCore, QtGui


class SwitchPlanner(object):
    def __init__(self, robotSystem):
        self.robotSystem = robotSystem
        self.assignFrames()
        self.plans = []

    def assignFrames(self):

        # foot to box
        self.footToBox = transformUtils.transformFromPose(np.array([-0.6436723 ,  0.18848073, -1.13987699]),
                                             np.array([ 0.99576385,  0.        ,  0.        , -0.09194753]))

        self.palmToBox = transformUtils.transformFromPose(np.array([-0.13628039, -0.12582009,  0.33638863]), np.array([-0.69866187,  0.07267815,  0.70683338,  0.08352274]))


    def spawnBoxAffordanceAtFrame(self, boxFrame):
        print 'spawning switch box affordance'
        dimensions = [0.08, 0.19, 0.25]
        depth = dimensions[0]
        boxFrame.PreMultiply()
        boxFrame.Translate(depth/2.0, 0.0, 0.0)
        pose = transformUtils.poseFromTransform(boxFrame)
        desc = dict(classname='BoxAffordanceItem', Name='Switch Box', Dimensions=dimensions, pose=pose, Color=[0,1,0])
        self.boxAffordance = segmentation.affordanceManager.newAffordanceFromDescription(desc)
        self.updateReachFrame()

    def updateReachFrame(self):
        graspFrame = transformUtils.copyFrame(self.palmToBox)
        boxFrame = om.findObjectByName('Switch Box').getChildFrame().transform

        graspFrame.PostMultiply()
        graspFrame.Concatenate(boxFrame)
        vis.updateFrame(graspFrame, 'reach frame', scale=0.2)


    def planArmsPrep1(self, startPose=None):
        ikPlanner = self.robotSystem.ikPlanner

        if startPose is None:
            startPose = self.getPlanningStartPose()

        startPoseName = 'q_arms_prep1_start'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)

        endPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'surprise:switch', 'arm_balance', side='left')
        endPose = ikPlanner.getMergedPostureFromDatabase(endPose, 'surprise:switch', 'reach_up_2', side='right')


        ikParameters = IkParameters(maxDegreesPerSecond=30)
        plan = ikPlanner.computePostureGoal(startPose, endPose, feetOnGround=False, ikParameters=ikParameters)
        self.addPlan(plan)

    def planArmsPrep2(self, startPose=None):
        ikPlanner = self.robotSystem.ikPlanner

        if startPose is None:
            startPose = self.getPlanningStartPose()

        startPoseName = 'q_arms_prep2_start'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)

        endPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'surprise:switch', 'reach_up_1', side='right')

        ikParameters = IkParameters(maxDegreesPerSecond=30)
        plan = ikPlanner.computePostureGoal(startPose, endPose, feetOnGround=False, ikParameters=ikParameters)
        self.addPlan(plan)

    def planReach(self):
        ikPlanner = self.robotSystem.ikPlanner
        startPose = self.getPlanningStartPose()
        startPoseName = 'q_reach_start'
        endPoseName = 'q_reach_end'
        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)

        side = 'right'
        movingReachConstraint = ikPlanner.createMovingReachConstraints(startPoseName, lockBase=True, lockBack=True, lockArm=True, side=side)

        palmToHand = ikPlanner.getPalmToHandLink(side=side)
        targetFrame = om.findObjectByName('reach frame').transform
        poseConstraints = ikPlanner.createPositionOrientationGraspConstraints(side, targetFrame, graspToHandLinkFrame=palmToHand, angleToleranceInDegrees=5.0)

        constraints = []
        constraints.extend(movingReachConstraint)
        constraints.extend(poseConstraints)
        constraintSet = ikplanner.ConstraintSet(ikPlanner, constraints, endPoseName, startPoseName)

        constraintSet.ikParameters = IkParameters(maxDegreesPerSecond=30)

        seedPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'surprise:switch', 'above_switch', side='right')
        seedPoseName = 'q_above_switch'
        self.robotSystem.ikPlanner.addPose(seedPose, seedPoseName)

        constraintSet.seedPoseName = seedPoseName
        constraintSet.nominalPoseName = seedPoseName

        endPose, info = constraintSet.runIk()
        plan = constraintSet.planEndPoseGoal()
        self.addPlan(plan)


    def getPlanningStartPose(self):
        return self.robotSystem.robotStateJointController.q

    def addPlan(self, plan):
        self.plans.append(plan)

    def handToPinch(self):
        pass


    def planWalking(self):
        startPose = self.getPlanningStartPose()
        walkingPlan = self.footstepPlanner.sendWalkingPlanRequest(self.footstepPlan, startPose, waitForResponse=True)
        self.addPlan(walkingPlan)

    def getStanceFrame(self):
        return self.robotSystem.footstepsDriver.getFeetMidPoint(self.robotSystem.robotStateModel, useWorldZ=False)

    def spawnBoxAffordance(self):
        stanceFrame = self.getStanceFrame()
        boxFrame = transformUtils.copyFrame(stanceFrame)
        boxFrame.PreMultiply()
        boxFrame.Concatenate(self.footToBox.GetLinearInverse())
        self.spawnBoxAffordanceAtFrame(boxFrame)


    def spawnFootstepFrame(self):
        # should snap this to
        boxFrame = om.findObjectByName('Switch Box').getChildFrame().transform

        goalFrame = transformUtils.copyFrame(self.footToBox)
        goalFrame.PostMultiply()
        goalFrame.Concatenate(boxFrame)


        # translate goal frame to match current robot height
        stanceFrame = self.getStanceFrame()
        stanceHeight = stanceFrame.GetPosition()[2]
        goalHeight = goalFrame.GetPosition()[2]

        goalFrame.PreMultiply()
        goalFrame.Translate(0.0, 0.0, stanceHeight - goalHeight)
        vis.updateFrame(goalFrame, 'switch box stance frame', scale=0.2)

    def planNominal(self):
        ikPlanner = self.robotSystem.ikPlanner
        startPose = self.getPlanningStartPose()
        endPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'safe nominal')
        endPose, info = ikPlanner.computeStandPose(endPose)
        newPlan = ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def commitManipPlan(self):
        self.robotSystem.manipPlanner.commitManipPlan(self.plans[-1])