import os
import sys
from . import vtkAll as vtk
import math
import time
import types
import functools
import numpy as np

from director import transformUtils
from director import lcmUtils
from director.timercallback import TimerCallback
from director.asynctaskqueue import AsyncTaskQueue
from director import objectmodel as om
from director import visualization as vis
from director import applogic as app
from director.debugVis import DebugData
from director import ikplanner
from director.ikparameters import IkParameters
from director import ioUtils
from director.simpletimer import SimpleTimer
from director.utime import getUtime
from director import affordanceitems
from director import robotstate
from director import robotplanlistener
from director import segmentation
from director import planplayback
from director import affordanceupdater
from director import segmentationpanel
from director import vtkNumpy as vnp

from director.tasks.taskuserpanel import TaskUserPanel
from director.tasks.taskuserpanel import ImageBasedAffordanceFit

import director.tasks.robottasks as rt
import director.tasks.taskmanagerwidget as tmw

import drc as lcmdrc
import copy

from PythonQt import QtCore, QtGui


class SwitchPlanner(object):
    def __init__(self, robotSystem):
        self.robotSystem = robotSystem


        self.robotModel = robotSystem.robotStateModel
        self.ikPlanner = robotSystem.ikPlanner
        self.lockBackForManip = True
        self.lockBaseForManip = True
        self.graspingHand = 'right'

        self.assignFrames()
        self.plans = []

    def assignFrames(self):

        # foot to box
        self.footToBox = transformUtils.transformFromPose(np.array([-0.6436723 ,  0.18848073, -1.13987699]),
                                             np.array([ 0.99576385,  0.        ,  0.        , -0.09194753]))

        # self.palmToBox = transformUtils.transformFromPose(np.array([-0.13628039, -0.12582009,  0.33638863]), np.array([-0.69866187,  0.07267815,  0.70683338,  0.08352274]))

        self.palmToBox = transformUtils.transformFromPose(np.array([-0.13516451, -0.12463758,  0.25173153]), np.array([-0.69867721,  0.07265162,  0.70682793,  0.08346358]))

        pinchToBox = self.getPinchToPalmFrame()
        pinchToBox.PostMultiply()
        pinchToBox.Concatenate(self.palmToBox)

        self.pinchToBox = pinchToBox

        # self.pinchToBox =

    def spawnBoxAffordanceAtFrame(self, boxFrame):
        print('spawning switch box affordance')
        dimensions = [0.08, 0.19, 0.25]
        depth = dimensions[0]
        boxFrame.PreMultiply()
        boxFrame.Translate(depth/2.0, 0.0, 0.0)
        pose = transformUtils.poseFromTransform(boxFrame)
        desc = dict(classname='BoxAffordanceItem', Name='Switch Box', Dimensions=dimensions, pose=pose, Color=[0,1,0])
        self.boxAffordance = segmentation.affordanceManager.newAffordanceFromDescription(desc)
        self.updateReachFrame()

    def updateReachFrame(self):
        graspFrame = transformUtils.copyFrame(self.pinchToBox)
        boxFrame = om.findObjectByName('Switch Box').getChildFrame().transform

        graspFrame.PostMultiply()
        graspFrame.Concatenate(boxFrame)
        vis.updateFrame(graspFrame, 'pinch reach frame', scale=0.2)


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

    def planPinchReach(self, maxDegreesPerSecond=None):
        if maxDegreesPerSecond is None:
            maxDegreesPerSecond=10

        ikPlanner = self.ikPlanner

        targetFrame = om.findObjectByName('pinch reach frame').transform
        pinchToHand = self.getPinchToHandFrame()
        startPose = self.getPlanningStartPose()
        constraintSet = self.computeGraspPose(startPose, targetFrame, graspToHand=pinchToHand)

        constraintSet.ikParameters = IkParameters(maxDegreesPerSecond=maxDegreesPerSecond)
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

    def getPinchToHandFrame(self):
        pinchToHand = transformUtils.transformFromPose(np.array([ -1.22270636e-07,  -3.11575498e-01,   0.00000000e+00]), np.array([  3.26794897e-07,  -2.42861455e-17,  -1.85832253e-16,
         1.00000000e+00]))

        return pinchToHand

    def getPinchToPalmFrame(self):
        pinchToPalm = transformUtils.copyFrame(self.getPinchToHandFrame())
        palmToHand = self.ikPlanner.getPalmToHandLink(side='right')
        pinchToPalm.PostMultiply()
        pinchToPalm.Concatenate(palmToHand.GetLinearInverse())

        return pinchToPalm

    def getThumbToPalmFrame(self):
        return vtk.vtkTransform()

    def getGraspToHandFrame(self):

        mode = 'palm'

        graspToPalm = {'palm':vtk.vtkTransform,
                       'pinch':self.getPinchToPalmFrame,
                       'thumb':self.getThumbToPalmFrame}[mode]()

        return self.ikPlanner.newGraspToHandFrame(self.graspingHand, graspToPalmFrame=graspToPalm)


    def computeGraspPose(self, startPose, targetFrame, graspToHand=None):

        side = self.graspingHand
        if graspToHand is None:
            graspToHand = self.ikPlanner.getPalmToHandLink(side=self.graspingHand)


        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, side, targetFrame, lockBase=self.lockBaseForManip, lockBack=self.lockBackForManip, graspToHandLinkFrame=graspToHand)

        return constraintSet

    def computeGraspPlan(self, targetFrame, graspToHandFrame, inLine=False, ikParameters=None):

        startPose = self.getPlanningStartPose()
        endPose, constraintSet = self.computeGraspPose(startPose, targetFrame)
        if ikParameters:
            constraintSet.ikParameters = ikParameters

        constraintSet.ikParameters.usePointwise = False

        if inLine:

            handLinkName = self.ikPlanner.getHandLink(self.graspingHand)
            graspToHand = graspToHandFrame

            handToWorld1 = self.ikPlanner.getLinkFrameAtPose(handLinkName, startPose)
            handToWorld2 = self.ikPlanner.getLinkFrameAtPose(handLinkName, endPose)

            handToWorld1 = transformUtils.concatenateTransforms([graspToHand, handToWorld1])
            handToWorld2 = transformUtils.concatenateTransforms([graspToHand, handToWorld2])

            motionVector = np.array(handToWorld2.GetPosition()) - np.array(handToWorld1.GetPosition())
            motionTargetFrame = transformUtils.getTransformFromOriginAndNormal(np.array(handToWorld2.GetPosition()), motionVector)


            #vis.updateFrame(motionTargetFrame, 'motion target frame', scale=0.1)
            #d = DebugData()
            #d.addLine(np.array(handToWorld2.GetPosition()), np.array(handToWorld2.GetPosition()) - motionVector)
            #vis.updatePolyData(d.getPolyData(), 'motion vector', visible=False)

            p = self.ikPlanner.createLinePositionConstraint(handLinkName, graspToHand, motionTargetFrame,
                  lineAxis=2, bounds=[-np.linalg.norm(motionVector), 0.001], positionTolerance=0.001)
            p.tspan = np.linspace(0, 1, 5)

            constraintSet.constraints.append(p)
            newPlan = constraintSet.runIkTraj()

        else:

            newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)

        return newPlan
