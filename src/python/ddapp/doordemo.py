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
from ddapp.fieldcontainer import FieldContainer
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp import applogic as app
from ddapp.debugVis import DebugData
from ddapp import ik
from ddapp import ikplanner
from ddapp import ioUtils
from ddapp import affordanceitems
from ddapp.simpletimer import SimpleTimer
from ddapp.utime import getUtime
from ddapp import robotstate
from ddapp import robotplanlistener
from ddapp import segmentation
from ddapp import planplayback
from ddapp.footstepsdriver import FootstepRequestGenerator
from ddapp.tasks.taskuserpanel import TaskUserPanel
from ddapp.tasks.taskuserpanel import ImageBasedAffordanceFit


import ddapp.tasks.robottasks as rt
import ddapp.tasks.taskmanagerwidget as tmw

import drc as lcmdrc

from PythonQt import QtCore, QtGui


class DoorDemo(object):

    def __init__(self, robotModel, footstepPlanner, manipPlanner, ikPlanner, lhandDriver, rhandDriver, atlasDriver, multisenseDriver, affordanceFitFunction, sensorJointController, planPlaybackFunction, showPoseFunction):
        self.robotModel = robotModel
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
        self.graspingHand = 'right'

        self.endPose = None

        self.planFromCurrentRobotState = True
        self.visOnly = False
        self.useFootstepPlanner = False
        self.userPromptEnabled = True

        self.constraintSet = None
        self.plans = []

        self.pinchDistance = 0.1
        self.doorHandleFrame = None
        self.doorHandleGraspFrame = None
        self.doorHingeFrame = None

        self.handleTouchHeight = 0.0
        self.handleTouchDepth = -0.06
        self.handleTouchWidth = 0.05

        self.handleReachAngle = 20

        self.handleTurnHeight = -0.08
        self.handleTurnWidth = 0.01
        self.handleTurnAngle = 60

        self.handleLiftHeight = 0.12
        self.handlePushDepth = 0.0
        self.handlePushAngle = 5
        self.handleOpenDepth = 0.1
        self.handleOpenWidth = 0.4

        self.speedHigh = 60
        self.speedLow = 20

        self.setFootstepThroughDoorParameters()


    def setIkParameters(self, usePointwise=None, maxDegreesPerSecond=None):
        usePointwiseOrig = self.ikPlanner.ikServer.usePointwise
        maxDegreesPerSecondOrig = self.ikPlanner.ikServer.maxDegreesPerSecond
        if usePointwise is not None:
            self.ikPlanner.ikServer.usePointwise = usePointwise
        if maxDegreesPerSecond is not None:
            self.ikPlanner.ikServer.maxDegreesPerSecond = maxDegreesPerSecond
        return (usePointwiseOrig, maxDegreesPerSecondOrig)

    def addPlan(self, plan):
        self.plans.append(plan)


    def computeGraspOrientation(self):
        return [180 + self.handleReachAngle, 0, 90]

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



    def computeDoorHandleGraspFrame(self):
        graspOrientation = self.computeGraspOrientation()
        self.doorHandleAxisFrame = self.computeDoorHandleAxisFrame()

        def makeFrame(name, offset, turnAngle=0):
            t = transformUtils.frameFromPositionAndRPY(offset, graspOrientation)
            t.PostMultiply()
            t.Concatenate(transformUtils.frameFromPositionAndRPY([0.0, 0.0, 0.0], [-turnAngle, 0, 0]))
            t.Concatenate(transformUtils.copyFrame(self.doorHandleAxisFrame.transform))
            return vis.updateFrame(t, name, parent=self.doorHandleAffordance, visible=False, scale=0.2)

        def makeFrameNew(name, transforms):
            t = transformUtils.concatenateTransforms(transforms)
            return vis.updateFrame(t, name, parent=self.doorHandleAffordance, visible=False, scale=0.2)

        graspToAxisTransform = transformUtils.frameFromPositionAndRPY([0.0, 0.0, 0.0],
                                                                      graspOrientation)
        self.doorHandleGraspFrame = makeFrameNew('door handle grasp frame',
                                                 [graspToAxisTransform,
                                                  self.doorHandleAxisFrame.transform])

        reachToGraspTransform = transformUtils.frameFromPositionAndRPY([self.handleTouchWidth,
                                                                        self.handleTouchDepth,
                                                                        -self.handleTouchHeight],
                                                                       [0.0, 0.0, 0.0])
        self.doorHandleReachFrame = makeFrameNew('door handle reach frame', [reachToGraspTransform,
                                                                          self.doorHandleGraspFrame.transform])

        handleTurnTransform = transformUtils.frameFromPositionAndRPY([0.0, 0.0, 0.0], [-self.handleTurnAngle, 0, 0])
        self.doorHandleTurnFrame = makeFrameNew('door handle turn frame', [reachToGraspTransform,
                                                                           graspToAxisTransform,
                                                                           handleTurnTransform,
                                                                           self.doorHandleAxisFrame.transform])

        handlePushTransform = transformUtils.frameFromPositionAndRPY([0.0, 0.0, 0.0],
                                                                     [0, 0, self.handlePushAngle])
        self.doorHandlePushFrame = makeFrameNew('door handle push frame',
                                                [self.doorHandleTurnFrame.transform,
                                                 self.doorHingeFrame.transform.GetInverse(),
                                                 handlePushTransform,
                                                 self.doorHingeFrame.transform])

        self.doorHandlePushLiftFrame = makeFrameNew('door handle push lift frame',
                                                [self.doorHandleReachFrame.transform,
                                                 self.doorHingeFrame.transform.GetInverse(),
                                                 handlePushTransform,
                                                 self.doorHingeFrame.transform])

        self.doorHandlePushLiftAxisFrame = makeFrameNew('door handle push lift axis frame',
                                                [self.doorHandleAxisFrame.transform,
                                                 self.doorHingeFrame.transform.GetInverse(),
                                                 handlePushTransform,
                                                 self.doorHingeFrame.transform])


        self.doorHandlePushOpenFrame = makeFrame('door handle push open frame', [self.handleOpenDepth, self.handleOpenWidth, self.handleLiftHeight])

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.RotateX(25)
        t.Concatenate(self.doorHandlePushOpenFrame.transform)
        self.doorHandlePushOpenFrame.copyFrame(t)

        self.doorHandleFrame.frameSync = vis.FrameSync()
        self.doorHandleFrame.frameSync.addFrame(self.doorHandleFrame)
        self.doorHandleFrame.frameSync.addFrame(self.doorHandleAxisFrame)
        self.doorHandleFrame.frameSync.addFrame(self.doorHandleGraspFrame, ignoreIncoming=True)
        self.doorHandleFrame.frameSync.addFrame(self.doorHandleReachFrame, ignoreIncoming=True)
        self.doorHandleFrame.frameSync.addFrame(self.doorHandleTurnFrame, ignoreIncoming=True)
        self.doorHandleFrame.frameSync.addFrame(self.doorHandlePushFrame, ignoreIncoming=True)
        self.doorHandleFrame.frameSync.addFrame(self.doorHandlePushLiftFrame, ignoreIncoming=True)
        self.doorHandleFrame.frameSync.addFrame(self.doorHandlePushLiftAxisFrame, ignoreIncoming=True)
        self.doorHandleFrame.frameSync.addFrame(self.doorHandlePushOpenFrame, ignoreIncoming=True)

    def computeDoorHandleAxisFrame(self):
        handleLength = self.doorHandleAffordance.getProperty('Dimensions')[1]
        t = transformUtils.frameFromPositionAndRPY([0.0, -handleLength/2.0, 0.0], [0, 0, 0])
        t.PostMultiply()
        t.Concatenate(transformUtils.copyFrame(self.doorHandleFrame.transform))
        return vis.updateFrame(t, 'door handle axis frame', parent=self.doorHandleAffordance,
                               visible=False, scale=0.2)


    def computeDoorHingeFrame(self):
        doorAffordance = om.findObjectByName('door')
        doorDimensions = doorAffordance.getProperty('Dimensions')
        doorDepth = doorDimensions[0]
        doorWidth = doorDimensions[1]
        t = transformUtils.frameFromPositionAndRPY([doorDepth/2, doorWidth/2.0, 0.0], [0, 0, 0])
        t.PostMultiply()
        t.Concatenate(transformUtils.copyFrame(doorAffordance.getChildFrame().transform))
        self.doorHingeFrame = vis.updateFrame(t, 'door hinge frame', parent=doorAffordance,
                                              visible=False, scale=0.2)
        return self.doorHingeFrame


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


    def planNominal(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'safe nominal')
        endPose, info = self.ikPlanner.computeStandPose(endPose)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)


    def planPreReach(self):

        usePointwiseOrig, maxDegreesPerSecondOrig = self.setIkParameters(usePointwise=False,
                                                                         maxDegreesPerSecond=self.speedHigh)

        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'door', 'pre-reach both')
        endPose, info = self.ikPlanner.computeStandPose(endPose)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)
        self.setIkParameters(usePointwiseOrig, maxDegreesPerSecondOrig)


    def planTuckArms(self):


        usePointwiseOrig, maxDegreesPerSecondOrig = self.setIkParameters(usePointwise=False,
                                                                         maxDegreesPerSecond=self.speedHigh)
        self.ikPlanner.ikServer.numberOfAddedKnots = 0

        otherSide = 'left' if self.graspingHand == 'right' else 'right'

        startPose = self.getPlanningStartPose()

        standPose, info = self.ikPlanner.computeStandPose(startPose)

        q2 = self.ikPlanner.getMergedPostureFromDatabase(standPose, 'door', 'hand up tuck', side=otherSide)
        a = 0.25
        q2 = (1.0 - a)*np.array(standPose) + a*q2
        q2 = self.ikPlanner.getMergedPostureFromDatabase(q2, 'door', 'hand up tuck', side=self.graspingHand)
        a = 0.75
        q2 = (1.0 - a)*np.array(standPose) + a*q2

        endPose = self.ikPlanner.getMergedPostureFromDatabase(standPose, 'door', 'hand up tuck', side=self.graspingHand)
        endPose = self.ikPlanner.getMergedPostureFromDatabase(endPose, 'door', 'hand up tuck', side=otherSide)

        newPlan = self.ikPlanner.computeMultiPostureGoal([startPose, q2, endPose])
        self.addPlan(newPlan)

        self.setIkParameters(usePointwiseOrig, maxDegreesPerSecondOrig)

    def planReach(self):

        usePointwiseOrig, maxDegreesPerSecondOrig = self.setIkParameters(usePointwise=False,
                                                                         maxDegreesPerSecond=self.speedLow)

        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.doorHandleReachFrame)
        endPose, info = constraintSet.runIk()
        plan = constraintSet.runIkTraj()


        self.addPlan(plan)

        self.setIkParameters(usePointwiseOrig, maxDegreesPerSecondOrig)

    def createHingeConstraint(self, referenceFrame, axis, linkName, startPose, tspan=[0, 1]):

        constraints = []
        linkFrame = self.ikPlanner.getLinkFrameAtPose(linkName, startPose)

        #turnTransform = transformUtils.frameFromPositionAndRPY([0.0, 0.0, 0.0], [-turnAngle, 0, 0])
        #turnTransform = vtkTransform()
        #turnTransform.RotateWXYZ(turnAngle, axis)
        #linkToReferenceTransfrom = tranformUtils.concatenateTransforms([linkFrame,
                                                                        #referenceFrame.transform.GetInverse()])
        #finalLinkFrame = transformUtils.concatenateTransforms([linkToReferenceTransfrom,
                                                               #turnTransform,
                                                               #referenceFrame.transform])
        def addPivotPoint(constraints, pivotPoint):
            constraints.append(ik.PositionConstraint())
            constraints[-1].linkName = linkName
            constraints[-1].referenceFrame = referenceFrame.transform
            constraints[-1].lowerBound = np.array(pivotPoint)
            constraints[-1].upperBound = np.array(pivotPoint)
            pivotPointInWorld = referenceFrame.transform.TransformDoublePoint(pivotPoint)
            constraints[-1].pointInLink = linkFrame.GetInverse().TransformDoublePoint(pivotPointInWorld)
            constraints[-1].tspan = tspan

        addPivotPoint(constraints, [0.0, 0.0, 0.0])
        addPivotPoint(constraints, axis)
        return constraints


    def planHandleTurn(self):

        startPose = self.getPlanningStartPose()
        graspOrientation = self.computeGraspOrientation()
        linkFrame = self.ikPlanner.getLinkFrameAtPose(self.ikPlanner.getHandLink(), startPose)

        finalGraspToReferenceTransfrom = transformUtils.concatenateTransforms(
            [self.ikPlanner.getPalmToHandLink(self.graspingHand), linkFrame,
             self.doorHandleAxisFrame.transform.GetInverse()])

        handleTurnTransform = transformUtils.frameFromPositionAndRPY([0.0, 0.0, 0.0],
                                                                     [-self.handleTurnAngle, 0, 0])
        doorHandleTurnFrame = transformUtils.concatenateTransforms([finalGraspToReferenceTransfrom,
                                                                    handleTurnTransform,
                                                                    self.doorHandleAxisFrame.transform])

        vis.updateFrame(doorHandleTurnFrame, 'debug turn', parent=self.doorHandleAffordance, visible=False, scale=0.2)
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand,
                                                           doorHandleTurnFrame)
        endPose, info = constraintSet.runIk()
        constraints = constraintSet.constraints

        constraints.extend(self.createHingeConstraint(self.doorHandleAxisFrame, [1.0, 0.0, 0.0],
                                                      self.ikPlanner.getHandLink(),
                                                      constraintSet.startPoseName))
        constraints.append(self.ikPlanner.createLockedBasePostureConstraint(constraintSet.startPoseName))
        constraints.append(self.ikPlanner.createLockedBackPostureConstraint(constraintSet.startPoseName))
        constraints.extend(self.ikPlanner.createFixedFootConstraints(constraintSet.startPoseName))
        constraints.append(self.ikPlanner.createLockedArmPostureConstraint(constraintSet.startPoseName))

        plan = constraintSet.runIkTraj()

        self.addPlan(plan)

    def planDoorPushOpen(self):

        usePointwiseOrig, maxDegreesPerSecondOrig = self.setIkParameters(usePointwise=False,
                                                                         maxDegreesPerSecond=self.speedLow)

        nonGraspingHand = 'right' if self.graspingHand == 'left' else 'left'

        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'door', 'smash', side=nonGraspingHand)

        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

        self.setIkParameters(usePointwiseOrig, maxDegreesPerSecondOrig)


    def planHandlePush(self):

        startPose = self.getPlanningStartPose()
        linkFrame = self.ikPlanner.getLinkFrameAtPose(self.ikPlanner.getHandLink(), startPose)

        finalGraspToReferenceTransfrom = transformUtils.concatenateTransforms(
            [self.ikPlanner.getPalmToHandLink(self.graspingHand), linkFrame,
             self.doorHingeFrame.transform.GetInverse()])

        handlePushTransform = transformUtils.frameFromPositionAndRPY([0.0, 0.0, 0.0],
                                                                     [0, 0, self.handlePushAngle])
        doorHandlePushFrame = transformUtils.concatenateTransforms([finalGraspToReferenceTransfrom,
                                                                    handlePushTransform,
                                                                    self.doorHingeFrame.transform])

        vis.updateFrame(doorHandlePushFrame, 'debug push', parent=self.doorHandleAffordance, visible=False, scale=0.2)
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand,
                                                           doorHandlePushFrame)
        endPose, info = constraintSet.runIk()
        constraints = constraintSet.constraints

        constraints.extend(self.createHingeConstraint(self.doorHingeFrame, [0.0, 0.0, 1.0],
                                                      self.ikPlanner.getHandLink(side=self.graspingHand),
                                                      constraintSet.startPoseName))
        constraints.append(self.ikPlanner.createLockedBasePostureConstraint(constraintSet.startPoseName))
        #constraints.append(self.ikPlanner.createLockedBackPostureConstraint(constraintSet.startPoseName))
        constraints.extend(self.ikPlanner.createFixedFootConstraints(constraintSet.startPoseName))
        constraints.append(self.ikPlanner.createLockedArmPostureConstraint(constraintSet.startPoseName))

        plan = constraintSet.runIkTraj()

        self.addPlan(plan)

    def planHandlePushLift(self):

        startPose = self.getPlanningStartPose()
        graspOrientation = self.computeGraspOrientation()
        linkFrame = self.ikPlanner.getLinkFrameAtPose(self.ikPlanner.getHandLink(), startPose)

        finalGraspToReferenceTransfrom = transformUtils.concatenateTransforms(
            [self.ikPlanner.getPalmToHandLink(self.graspingHand), linkFrame,
             self.doorHandlePushLiftAxisFrame.transform.GetInverse()])

        handleTurnTransform = transformUtils.frameFromPositionAndRPY([0.0, 0.0, 0.0],
                                                                     [self.handleTurnAngle, 0, 0])
        doorHandlePushLiftFrame = transformUtils.concatenateTransforms([finalGraspToReferenceTransfrom,
                                                                    handleTurnTransform,
                                                                    self.doorHandlePushLiftAxisFrame.transform])

        vis.updateFrame(doorHandlePushLiftFrame, 'debug push lift', parent=self.doorHandleAffordance, visible=False, scale=0.2)
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand,
                                                           doorHandlePushLiftFrame)
        endPose, info = constraintSet.runIk()
        constraints = constraintSet.constraints

        constraints.extend(self.createHingeConstraint(self.doorHandlePushLiftAxisFrame, [1.0, 0.0, 0.0],
                                                      self.ikPlanner.getHandLink(),
                                                      constraintSet.startPoseName))
        constraints.append(self.ikPlanner.createLockedBasePostureConstraint(constraintSet.startPoseName))
        constraints.append(self.ikPlanner.createLockedBackPostureConstraint(constraintSet.startPoseName))
        constraints.extend(self.ikPlanner.createFixedFootConstraints(constraintSet.startPoseName))
        constraints.append(self.ikPlanner.createLockedArmPostureConstraint(constraintSet.startPoseName))

        plan = constraintSet.runIkTraj()

        self.addPlan(plan)


    #def planHandlePushLift(self):

        #self.ikPlanner.ikServer.usePointwise = False
        #self.ikPlanner.ikServer.maxDegreesPerSecond = 10
        #self.ikPlanner.maxBaseMetersPerSecond = 0.01

        #startPose = self.getPlanningStartPose()
        #constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.doorHandlePushLiftFrame)
        #endPose, info = constraintSet.runIk()
        #plan = constraintSet.runIkTraj()

        #self.ikPlanner.ikServer.maxDegreesPerSecond = 30
        #self.ikPlanner.maxBaseMetersPerSecond = 0.05

        #self.addPlan(plan)


    def planDoorTouch(self):

        usePointwiseOrig, maxDegreesPerSecondOrig = self.setIkParameters(usePointwise=False,
                                                                         maxDegreesPerSecond=self.speedHigh)
        nonGraspingHand = 'right' if self.graspingHand == 'left' else 'left'

        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'door', 'pre-smash', side=nonGraspingHand)

        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

        self.setIkParameters(usePointwiseOrig, maxDegreesPerSecondOrig)

    def planHandlePushOpen(self):

        usePointwiseOrig, maxDegreesPerSecondOrig = self.setIkParameters(usePointwise=False,
                                                                         maxDegreesPerSecond=self.speedLow)
        self.ikPlanner.ikServer.usePointwise = False
        self.ikPlanner.ikServer.maxDegreesPerSecond = 20

        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.doorHandlePushOpenFrame)
        endPose, info = constraintSet.runIk()
        plan = constraintSet.runIkTraj()

        self.addPlan(plan)

        self.setIkParameters(usePointwiseOrig, maxDegreesPerSecondOrig)


    def planFootstepsToDoor(self):
        startPose = self.getPlanningStartPose()
        goalFrame = self.doorHandleStanceFrame.transform
        request = self.footstepPlanner.constructFootstepPlanRequest(startPose, goalFrame)
        self.footstepPlan = self.footstepPlanner.sendFootstepPlanRequest(request, waitForResponse=True)


    def planFootstepsThroughDoor(self):
        startPose = self.getPlanningStartPose()
        goalFrame = self.doorWalkFrame.transform
        request = self.footstepPlanner.constructFootstepPlanRequest(startPose, goalFrame)
        request.params.nom_step_width = 0.21
        self.footstepPlan = self.footstepPlanner.sendFootstepPlanRequest(request, waitForResponse=True)
        rt._addPlanItem(self.footstepPlan, 'door walk frame footstep plan', rt.FootstepPlanItem)


    def setFootstepThroughDoorParameters(self):

        bias = -0.02
        self.doorFootstepParams = FieldContainer(
            leadingFoot = 'right',
            preEntryFootWidth = -0.12 +bias,
            preEntryFootDistance = -0.6,
            entryFootWidth = 0.07 +bias,
            entryFootDistance = -0.26,
            exitFootWidth = -0.08 +bias,
            exitFootDistance = 0.12,
            exitStepDistance = 0.3,
            endStanceWidth = 0.26,
            numberOfExitSteps = 1,
            centerStepDistance = 0.26,
            centerStanceWidth = 0.20,
            centerLeadingFoot = 'right'
        )

    def setTestingFootstepThroughDoorParameters(self):
        self.doorFootstepParams = FieldContainer(
            endStanceWidth       = 0.26,
            entryFootDistance    = -0.26,
            entryFootWidth       = 0.12,
            exitFootDistance     = 0.12,
            exitFootWidth        = -0.12,
            exitStepDistance     = 0.3,
            leadingFoot          = 'right',
            numberOfExitSteps    = 1,
            preEntryFootDistance = -0.55,
            preEntryFootWidth    = -0.12
        )

    def getRelativeFootstepsThroughDoorWithSway(self):

        p = self.doorFootstepParams

        stepFrames = [

            [p.preEntryFootDistance, p.preEntryFootWidth, 0.0],
            [p.entryFootDistance, p.entryFootWidth, 0.0],
            [p.exitFootDistance, p.exitFootWidth, 0.0],

        ]

        for i in xrange(p.numberOfExitSteps):

            sign = -1 if (i%2) else 1
            stepFrames.append([p.exitFootDistance + (i+1)*p.exitStepDistance, sign*p.endStanceWidth/2.0, 0.0])

        lastStep = list(stepFrames[-1])
        lastStep[1] *= -1
        stepFrames.append(lastStep)

        #print '------------'
        #print 'step deltas:'
        #for a,b in zip(stepFrames, stepFrames[1:]):
        #    print b[0] - a[0], b[1] - a[1]

        return FootstepRequestGenerator.makeStepFrames(stepFrames, relativeFrame=self.doorGroundFrame.transform, showFrames=False), p.leadingFoot


    def getRelativeFootstepsThroughDoorCentered(self):

        p = self.doorFootstepParams

        stepDistance = p.centerStepDistance
        stanceWidth = p.centerStanceWidth
        leadingFoot = p.centerLeadingFoot


        stepFrames = []
        for i in xrange(30):

            sign = -1 if leadingFoot is 'right' else 1
            if i % 2:
                sign = -sign

            stepX = (i+1)*stepDistance
            if stepX > 1.5:
                stepX = 1.5
            stepFrames.append([stepX, sign*stanceWidth/2.0, 0.0])

            if stepX == 1.5:
                break

        lastStep = list(stepFrames[-1])
        lastStep[1] *= -1
        stepFrames.append(lastStep)

        stepFrames[-1][1] = np.sign(stepFrames[-1][1])*(p.endStanceWidth/2.0)
        stepFrames[-2][1] = np.sign(stepFrames[-2][1])*(p.endStanceWidth/2.0)

        return FootstepRequestGenerator.makeStepFrames(stepFrames, relativeFrame=self.doorHandleStanceFrame.transform, showFrames=False), leadingFoot


    def planManualFootstepsTest(self, stepDistance=0.26, stanceWidth=0.26, numberOfSteps=4, leadingFoot='right'):

        stepFrames = []
        for i in xrange(numberOfSteps):

            sign = -1 if leadingFoot is 'right' else 1
            if i % 2:
                sign = -sign

            stepFrames.append([(i+1)*stepDistance, sign*stanceWidth/2.0, 0.0])

        lastStep = list(stepFrames[-1])
        lastStep[1] *= -1
        stepFrames.append(lastStep)

        stanceFrame = FootstepRequestGenerator.getRobotStanceFrame(self.robotModel)
        stepFrames = FootstepRequestGenerator.makeStepFrames(stepFrames, relativeFrame=stanceFrame)
        startPose = self.getPlanningStartPose()

        helper = FootstepRequestGenerator(self.footstepPlanner)
        request = helper.makeFootstepRequest(startPose, stepFrames, leadingFoot)

        self.footstepPlanner.sendFootstepPlanRequest(request, waitForResponse=True)


    def planFootstepsThroughDoorManual(self):

        startPose = self.getPlanningStartPose()
        #stepFrames, leadingFoot = self.getRelativeFootstepsThroughDoorWithSway()
        stepFrames, leadingFoot = self.getRelativeFootstepsThroughDoorCentered()

        helper = FootstepRequestGenerator(self.footstepPlanner)
        request = helper.makeFootstepRequest(startPose, stepFrames, leadingFoot, numberOfFillSteps=2)

        self.footstepPlan = self.footstepPlanner.sendFootstepPlanRequest(request, waitForResponse=True)
        rt._addPlanItem(self.footstepPlan, 'door walk frame footstep plan', rt.FootstepPlanItem)


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


    def fitDoor(self, doorGroundFrame):
        om.removeFromObjectModel(om.findObjectByName('affordances'))
        self.spawnDoorAffordance()
        affordanceFrame = om.findObjectByName('door ground frame')
        assert affordanceFrame is not None
        affordanceFrame.copyFrame(doorGroundFrame)

        om.findObjectByName('door').setProperty('Visible', False)


    def spawnDoorAffordance(self):

        groundFrame = self.computeGroundFrame(self.robotModel)

        doorOffsetX = 0.7
        doorOffsetY = 0.0

        doorGroundFrame = transformUtils.frameFromPositionAndRPY([doorOffsetX, 0.0, 0.0], [0.0, 0.0, 0.0])
        doorGroundFrame.PostMultiply()
        doorGroundFrame.Concatenate(groundFrame)

        stanceFrame = transformUtils.frameFromPositionAndRPY([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        stanceFrame.PostMultiply()
        stanceFrame.Concatenate(groundFrame)

        doorWalkFrame = transformUtils.frameFromPositionAndRPY([doorOffsetX + 0.6, 0.0, 0.0], [0.0, 0.0, 0.0])
        doorWalkFrame.PostMultiply()
        doorWalkFrame.Concatenate(groundFrame)


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
        handleFrame.PostMultiply()
        handleFrame.Concatenate(doorGroundFrame)


        doorFrame = transformUtils.frameFromPositionAndRPY([0.0, 0.0, doorHeight/2.0], [0.0, 0.0, 0.0])
        doorFrame.PostMultiply()
        doorFrame.Concatenate(doorGroundFrame)


        leftDoorJamFrame = transformUtils.frameFromPositionAndRPY([0.0, (doorWidth/2.0 + doorJamWidth/2.0), doorHeight/2.0], [0.0, 0.0, 0.0])
        leftDoorJamFrame.PostMultiply()
        leftDoorJamFrame.Concatenate(doorGroundFrame)

        rightDoorJamFrame = transformUtils.frameFromPositionAndRPY([0.0, -(doorWidth/2.0 + doorJamWidth/2.0), doorHeight/2.0], [0.0, 0.0, 0.0])
        rightDoorJamFrame.PostMultiply()
        rightDoorJamFrame.Concatenate(doorGroundFrame)



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


        doorGroundFrame = vis.showFrame(doorGroundFrame, 'door ground frame', parent=doorAffordance)
        stanceFrame = vis.showFrame(stanceFrame, 'door stance frame', parent=doorAffordance)

        doorWalkFrame = vis.showFrame(doorWalkFrame, 'door walk frame', parent=doorAffordance)


        doorFrame = doorAffordance.getChildFrame()
        handleFrame = handleAffordance.getChildFrame()

        leftDoorJamFrame = leftDoorJamAffordance.getChildFrame()
        rightDoorJamFrame = rightDoorJamAffordance.getChildFrame()

        self.doorFrameSync = vis.FrameSync()
        self.doorFrameSync.addFrame(doorGroundFrame)
        self.doorFrameSync.addFrame(stanceFrame, ignoreIncoming=True)
        self.doorFrameSync.addFrame(doorWalkFrame, ignoreIncoming=True)
        self.doorFrameSync.addFrame(doorFrame, ignoreIncoming=True)
        self.doorFrameSync.addFrame(leftDoorJamFrame, ignoreIncoming=True)
        self.doorFrameSync.addFrame(rightDoorJamFrame, ignoreIncoming=True)

        self.doorHandleFrameSync = vis.FrameSync()
        self.doorHandleFrameSync.addFrame(doorFrame)
        self.doorHandleFrameSync.addFrame(handleFrame, ignoreIncoming=True)


        self.findDoorHandleAffordance()
        self.doorGroundFrame = doorGroundFrame
        self.doorHandleStanceFrame = stanceFrame
        self.doorWalkFrame = doorWalkFrame


    def findDoorHandleAffordance(self):

        self.doorHandleAffordance = om.findObjectByName('door handle')
        self.doorHandleFrame = self.doorHandleAffordance.getChildFrame()

        self.computeDoorHingeFrame()
        self.computeDoorHandleGraspFrame()
        #self.computeDoorHandleStanceFrame()


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


class DoorImageFitter(ImageBasedAffordanceFit):

    def __init__(self, doorDemo):
        ImageBasedAffordanceFit.__init__(self, numberOfPoints=1)
        self.doorDemo = doorDemo

    def fit(self, polyData, points):
        doorGroundFrame = segmentation.segmentDoorPlane(polyData, points[0])
        self.doorDemo.fitDoor(doorGroundFrame)


class DoorTaskPanel(TaskUserPanel):

    def __init__(self, doorDemo):

        TaskUserPanel.__init__(self, windowTitle='Door Task')

        self.doorDemo = doorDemo

        self.fitter = DoorImageFitter(self.doorDemo)
        self.initImageView(self.fitter.imageView)

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()

    def addButtons(self):

        self.addManualButton('Spawn door', self.doorDemo.spawnDoorAffordance)
        self.addManualSpacer()
        self.addManualButton('Footsteps to door', self.doorDemo.planFootstepsToDoor)
        self.addManualSpacer()
        self.addManualButton('Raise arm', self.doorDemo.planPreReach)
        self.addManualButton('Finger pinch', self.fingerPinch)
        self.addManualSpacer()
        self.addManualButton('Reach', self.doorDemo.planReach)
        self.addManualButton('Turn', self.doorDemo.planHandleTurn)
        self.addManualButton('Push', self.doorDemo.planHandlePush)
        self.addManualButton('Lift', self.doorDemo.planHandlePushLift)
        self.addManualButton('Touch door', self.doorDemo.planDoorTouch)
        self.addManualButton('Push Open', self.doorDemo.planDoorPushOpen)
        self.addManualButton('Tuck Arms', self.doorDemo.planTuckArms)
        self.addManualSpacer()
        self.addManualButton('Footsteps through door', self.doorDemo.planFootstepsThroughDoor)
        self.addManualButton('Show walking plan', self.doorDemo.computeWalkingPlan)
        self.addManualSpacer()
        self.addManualButton('Nominal', self.doorDemo.planNominal)
        self.addManualSpacer()
        self.addManualButton('Commit Manip', self.doorDemo.commitManipPlan)


    def getSide(self):
        return self.params.getPropertyEnumValue('Hand').lower()

    def fingerPinch(self):
        rt.CloseHand(side=self.getSide().capitalize(), mode='Pinch', amount=100).run()


    def addDefaultProperties(self):
        self.params.addProperty('Hand', 1, attributes=om.PropertyAttributes(enumNames=['Left', 'Right']))
        self._syncProperties()

    def onPropertyChanged(self, propertySet, propertyName):
        self._syncProperties()

    def _syncProperties(self):
        self.doorDemo.graspingHand = self.params.getPropertyEnumValue('Hand').lower()
        self.doorDemo.ikPlanner.reachingSide = self.doorDemo.graspingHand

    def addTasks(self):

        # some helpers
        self.folder = None
        def addTask(task, parent=None):
            parent = parent or self.folder
            self.taskTree.onAddTask(task, copy=False, parent=parent)
        def addFunc(func, name, parent=None):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)
        def addFolder(name, parent=None):
            self.folder = self.taskTree.addGroup(name, parent=parent)
            return self.folder


        d = self.doorDemo

        self.taskTree.removeAllTasks()
        side = self.params.getPropertyEnumValue('Hand')

        ###############
        # add the tasks

        # prep
        folder = addFolder('Prep')
        addTask(rt.CloseHand(name='close left hand', side='Left'))
        addTask(rt.CloseHand(name='close right hand', side='Right'))
        addTask(rt.SetNeckPitch(name='set neck position', angle=20))
        #addTask(rt.PlanPostureGoal(name='plan walk posture', postureGroup='General', postureName='safe nominal', side='Default'))
        #addTask(rt.CheckPlanInfo(name='check manip plan info'))
        #addTask(rt.CommitManipulationPlan(name='execute manip plan', planName='safe nominal posture plan'))
        #addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'))


        # fit
        addTask(rt.WaitForMultisenseLidar(name='wait for lidar sweep'))
        addTask(rt.UserPromptTask(name='fit door', message='Please fit and approve door affordance.'))
        addTask(rt.FindAffordance(name='check door affordance', affordanceName='door'))

        # walk
        folder = addFolder('Walk and refit')
        addTask(rt.RequestFootstepPlan(name='plan walk to door', stanceFrameName='door stance frame'))
        addTask(rt.UserPromptTask(name='approve footsteps', message='Please approve footstep plan.'))
        addTask(rt.CommitFootstepPlan(name='walk to door', planName='door stance frame footstep plan'))
        addTask(rt.WaitForWalkExecution(name='wait for walking'))

        # refit
        addTask(rt.SetNeckPitch(name='set neck position', angle=35))
        addTask(rt.WaitForMultisenseLidar(name='wait for lidar sweep'))
        addTask(rt.UserPromptTask(name='fit door', message='Please fit and approve door handle affordance.'))

        # set fingers
        addTask(rt.CloseHand(name='set finger pinch', side=side, mode='Pinch', amount=100))


        def addManipTask(name, planFunc, userPrompt=False):

            folder = addFolder(name)
            addFunc(planFunc, name='plan')
            if not userPrompt:
                addTask(rt.CheckPlanInfo(name='check manip plan info'))
            else:
                addTask(rt.UserPromptTask(name='approve manip plan', message='Please approve manipulation plan.'))
            addFunc(d.commitManipPlan, name='execute manip plan')
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'))


        addManipTask('Raise arms', d.planPreReach, userPrompt=False)
        addManipTask('Reach', d.planReach, userPrompt=False)
        addTask(rt.UserPromptTask(name='Approve hand position',
                                  message='Please verify that the hand is ready to grasp the handle'))
        addFunc(self.fingerPinch, name='Pinch handle')
        addManipTask('Turn', d.planHandleTurn, userPrompt=True)
        addTask(rt.UserPromptTask(name='Approve handle turn',
                                  message='Please verify that the handle has turned'))
        addManipTask('Push ajar', d.planHandlePush, userPrompt=True)
        addTask(rt.UserPromptTask(name='Approve door position',
                                  message='Please verify that the door is ajar'))
        addManipTask('Lift', d.planHandlePushLift, userPrompt=False)
        addTask(rt.CloseHand(name='Open hand', side=side, mode='Pinch', amount=0))
        addManipTask('Raise pushing hand', d.planDoorTouch, userPrompt=False)
        addTask(rt.CloseHand(name='Open hand', side=side, mode='Pinch', amount=0))
        addManipTask('Push open', d.planDoorPushOpen, userPrompt=False)
        addTask(rt.UserPromptTask(name='Approve door position',
                                  message='Please verify that the door is open'))
        addManipTask('Tuck Arms', d.planTuckArms, userPrompt=False)

        # walk
        folder = addFolder('Walk through door')
        #addTask(rt.RequestFootstepPlan(name='plan walk through door', stanceFrameName='door walk frame'))
        addFunc(d.planFootstepsThroughDoorManual, name='plan walk through door')
        addTask(rt.UserPromptTask(name='approve footsteps', message='Please approve footstep plan.'))
        addTask(rt.CommitFootstepPlan(name='walk to door', planName='door walk frame footstep plan'))
        addTask(rt.WaitForWalkExecution(name='wait for walking'))
