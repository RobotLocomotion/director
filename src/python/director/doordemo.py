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
from ddapp.fieldcontainer import FieldContainer
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp import applogic as app
from ddapp.debugVis import DebugData
from ddapp import ik
from ddapp.ikparameters import IkParameters
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
        self.graspingHand = 'left'

        self.endPose = None

        self.planFromCurrentRobotState = True
        self.visOnly = False
        self.useFootstepPlanner = False
        self.userPromptEnabled = True

        self.constraintSet = None
        self.plans = []

        self.usePinchGrasp = False
        self.pinchDistance = 0.1
        self.doorHandleFrame = None
        self.doorHandleGraspFrame = None
        self.doorHingeFrame = None

        self.handleTouchHeight = 0.0
        self.handleTouchDepth = -0.08
        self.handleTouchWidth = 0.06

        self.handleReachAngle = 20

        self.handleTurnHeight = -0.08
        self.handleTurnWidth = 0.01
        self.handleTurnAngle = 60

        self.handleLiftHeight = 0.12
        self.handlePushDepth = 0.0
        self.handlePushAngle = 2
        self.handleOpenDepth = 0.1
        self.handleOpenWidth = 0.4

        self.speedHigh = 60
        self.speedLow = 15

        self.setFootstepThroughDoorParameters()

        self.setChopParametersToDefaults()


    def setChopParametersToDefaults(self):

        self.preChopDepth = -0.06
        self.preChopWidth = -0.08
        self.preChopHeight = 0.10
        self.chopDistance = -0.15
        self.chopSidewaysDistance = 0.03

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
        doorSide = 1 if self.graspingHand == 'left' else -1
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

        if self.usePinchGrasp:
            reachToGraspTransform = transformUtils.frameFromPositionAndRPY([-doorSide*self.handleTouchWidth,
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
        else:
            reachToAxisTransform = transformUtils.frameFromPositionAndRPY([self.preChopDepth,
                                                                        doorSide*self.preChopWidth,
                                                                        self.preChopHeight],
                                                                        [0, 90, -90])

            obj = om.findObjectByName('door handle reach frame')
            self.doorHandleReachFrame = makeFrameNew('door handle reach frame',
                                                     [reachToAxisTransform, self.doorHandleAxisFrame.transform])

            # the first time the frame is created, set some display properties
            if not obj:
                obj = self.doorHandleReachFrame
                obj.setProperty('Edit', True)
                obj.setProperty('Visible', True)
                rep = obj.widget.GetRepresentation()
                rep.SetRotateAxisEnabled(0, False)
                rep.SetRotateAxisEnabled(1, False)
                rep.SetRotateAxisEnabled(2, False)
                obj.widget.HandleRotationEnabledOff()
                obj.setProperty('Edit', False)


            preChopToReachTransform = transformUtils.frameFromPositionAndRPY([0.0,
                                                                       -0.15,
                                                                        0.0],
                                                                        [0, 0, 0])
            self.doorHandlePreChopFrame = makeFrameNew('door handle pre-chop frame',
                                                     [preChopToReachTransform, self.doorHandleReachFrame.transform])


        self.doorHandleFrame.frameSync = vis.FrameSync()
        self.doorHandleFrame.frameSync.addFrame(self.doorHandleFrame)
        self.doorHandleFrame.frameSync.addFrame(self.doorHandleAxisFrame)
        self.doorHandleFrame.frameSync.addFrame(self.doorHandleGraspFrame, ignoreIncoming=True)
        self.doorHandleFrame.frameSync.addFrame(self.doorHandleReachFrame, ignoreIncoming=True)
        if self.usePinchGrasp:
            self.doorHandleFrame.frameSync.addFrame(self.doorHandleTurnFrame, ignoreIncoming=True)
            self.doorHandleFrame.frameSync.addFrame(self.doorHandlePushFrame, ignoreIncoming=True)
            self.doorHandleFrame.frameSync.addFrame(self.doorHandlePushLiftFrame, ignoreIncoming=True)
            self.doorHandleFrame.frameSync.addFrame(self.doorHandlePushLiftAxisFrame, ignoreIncoming=True)
            self.doorHandleFrame.frameSync.addFrame(self.doorHandlePushOpenFrame, ignoreIncoming=True)
        else:
            self.doorHandleFrame.frameSync.addFrame(self.doorHandlePreChopFrame, ignoreIncoming=True)


    def computeDoorHandleAxisFrame(self):
        handleLength = self.doorHandleAffordance.getProperty('Dimensions')[1]
        doorSide = 1 if self.graspingHand == 'left' else -1
        t = transformUtils.frameFromPositionAndRPY([0.0, doorSide*handleLength/2.0, 0.0], [0, 0, 0])
        t.PostMultiply()
        t.Concatenate(transformUtils.copyFrame(self.doorHandleFrame.transform))
        return vis.updateFrame(t, 'door handle axis frame', parent=self.doorHandleAffordance,
                               visible=False, scale=0.2)


    def computeDoorHingeFrame(self):
        doorSide = 1 if self.graspingHand == 'left' else -1
        doorAffordance = om.findObjectByName('door')
        doorDimensions = doorAffordance.getProperty('Dimensions')
        doorDepth = doorDimensions[0]
        doorWidth = doorDimensions[1]
        t = transformUtils.frameFromPositionAndRPY([doorDepth/2, -doorSide*doorWidth/2.0, 0.0], [0, 0, 0])
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

        ikParameters = IkParameters(usePointwise=False, maxDegreesPerSecond=self.speedHigh)
        nonGraspingHand = 'right' if self.graspingHand == 'left' else 'left'

        startPose = self.getPlanningStartPose()

        if self.usePinchGrasp:
            endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'door',
                                                                'pre-reach grasping',
                                                                side=self.graspingHand)
        else:
            endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'door',
                                                                'pre-reach chop',
                                                                side=self.graspingHand)

        endPose = self.ikPlanner.getMergedPostureFromDatabase(endPose, 'door',
                                                              'pre-reach non-grasping',
                                                              side=nonGraspingHand)
        endPose, info = self.ikPlanner.computeStandPose(endPose, ikParameters=ikParameters)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose, ikParameters=ikParameters)
        self.addPlan(newPlan)


    def planUnReach(self):

        ikParameters = IkParameters(usePointwise=False, maxDegreesPerSecond=self.speedLow)

        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'door',
                                                              'pre-reach grasping',
                                                              side=self.graspingHand)
        endPose, info = self.ikPlanner.computeStandPose(endPose, ikParameters=ikParameters)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose, ikParameters=ikParameters)
        self.addPlan(newPlan)


    def planTuckArms(self):


        ikParameters = IkParameters(usePointwise=False, maxDegreesPerSecond=self.speedHigh)

        otherSide = 'left' if self.graspingHand == 'right' else 'right'

        startPose = self.getPlanningStartPose()

        standPose, info = self.ikPlanner.computeStandPose(startPose, ikParameters=ikParameters)

        q2 = self.ikPlanner.getMergedPostureFromDatabase(standPose, 'door', 'hand up tuck', side=otherSide)
        a = 0.25
        q2 = (1.0 - a)*np.array(standPose) + a*q2
        q2 = self.ikPlanner.getMergedPostureFromDatabase(q2, 'door', 'hand up tuck', side=self.graspingHand)
        a = 0.75
        q2 = (1.0 - a)*np.array(standPose) + a*q2

        endPose = self.ikPlanner.getMergedPostureFromDatabase(standPose, 'door', 'hand up tuck', side=self.graspingHand)
        endPose = self.ikPlanner.getMergedPostureFromDatabase(endPose, 'door', 'hand up tuck', side=otherSide)

        newPlan = self.ikPlanner.computeMultiPostureGoal([startPose, q2, endPose], ikParameters=ikParameters)
        self.addPlan(newPlan)

    def planTuckArmsPrePush(self):


        ikParameters = IkParameters(usePointwise=False, maxDegreesPerSecond=self.speedHigh)

        otherSide = 'left' if self.graspingHand == 'right' else 'right'

        startPose = self.getPlanningStartPose()

        standPose, info = self.ikPlanner.computeStandPose(startPose, ikParameters=ikParameters)

        q2 = self.ikPlanner.getMergedPostureFromDatabase(standPose, 'door', 'hand up tuck', side=self.graspingHand)
        a = 0.25
        q2 = (1.0 - a)*np.array(standPose) + a*q2
        q2 = self.ikPlanner.getMergedPostureFromDatabase(q2, 'door', 'hand up tuck', side=otherSide)
        a = 0.75
        q2 = (1.0 - a)*np.array(standPose) + a*q2

        endPose = self.ikPlanner.getMergedPostureFromDatabase(standPose, 'door', 'hand up tuck', side=self.graspingHand)
        endPose = self.ikPlanner.getMergedPostureFromDatabase(endPose, 'door', 'hand up tuck', side=otherSide)

        newPlan = self.ikPlanner.computeMultiPostureGoal([startPose, q2, endPose], ikParameters=ikParameters)
        self.addPlan(newPlan)


    def planChop(self, deltaZ=None, deltaY=None, deltaX=None):

        startPose = self.getPlanningStartPose()

        if deltaZ is None:
            deltaZ = self.chopDistance
        if deltaY is None:
            deltaY = self.chopSidewaysDistance
        if deltaX is None:
            deltaX = 0.0

        linkOffsetFrame = self.ikPlanner.getPalmToHandLink(self.graspingHand)
        handLinkName = self.ikPlanner.getHandLink(self.graspingHand)
        startFrame = self.ikPlanner.getLinkFrameAtPose(handLinkName, startPose)
        endToStartTransform = transformUtils.frameFromPositionAndRPY([deltaZ, -deltaX, -deltaY],
                                                                     [0, 0, 0])
        endFrame = transformUtils.concatenateTransforms([endToStartTransform, startFrame]);
        vis.updateFrame(endFrame, 'debug chop', parent=self.doorHandleAffordance, visible=False, scale=0.2)
        palmToWorld1 = transformUtils.concatenateTransforms([linkOffsetFrame, startFrame])
        palmToWorld2 = transformUtils.concatenateTransforms([linkOffsetFrame, endFrame])

        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, palmToWorld2)
        constraintSet.nominalPoseName = 'q_start'
        constraintSet.ikParameters =  IkParameters(usePointwise=False,
                                                   maxDegreesPerSecond=self.speedLow,
                                                   numberOfAddedKnots=2)

        endPose, info = constraintSet.runIk()


        motionVector = np.array(palmToWorld2.GetPosition()) - np.array(palmToWorld1.GetPosition())
        motionTargetFrame = transformUtils.getTransformFromOriginAndNormal(np.array(palmToWorld2.GetPosition()), motionVector)

        p = self.ikPlanner.createLinePositionConstraint(handLinkName, linkOffsetFrame, motionTargetFrame, lineAxis=2, bounds=[-np.linalg.norm(motionVector), 0.0], positionTolerance=0.001)
        constraintSet.constraints.append(p)

        plan = constraintSet.runIkTraj()
        self.addPlan(plan)

    def stopPushing(self):
        startPose = self.getPlanningStartPose
        plan = self.robotSystem.ikPlanner.computePostureGoal(startPose, startPose)
        self.addPlan(plan)
        self.commitManipPlan()

    def planReach(self, reachTargetFrame=None, jointSpeedLimit=None):

        if reachTargetFrame is None:
            reachTargetFrame = self.doorHandleReachFrame

        if jointSpeedLimit is None:
            jointSpeedLimit = self.speedLow

        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, reachTargetFrame)
        constraintSet.nominalPoseName = 'q_start'
        constraintSet.ikParameters =  IkParameters(usePointwise=False,
                                                   maxDegreesPerSecond=self.speedLow,
                                                   numberOfAddedKnots=2)

        endPose, info = constraintSet.runIk()

        linkOffsetFrame = self.ikPlanner.getPalmToHandLink(self.graspingHand)
        handLinkName = self.ikPlanner.getHandLink(self.graspingHand)
        handToWorld1 = self.ikPlanner.getLinkFrameAtPose(handLinkName, startPose)
        handToWorld2 = self.ikPlanner.getLinkFrameAtPose(handLinkName, endPose)
        palmToWorld1 = transformUtils.concatenateTransforms([linkOffsetFrame, handToWorld1])
        palmToWorld2 = transformUtils.concatenateTransforms([linkOffsetFrame, handToWorld2])

        motionVector = np.array(palmToWorld2.GetPosition()) - np.array(palmToWorld1.GetPosition())
        motionTargetFrame = transformUtils.getTransformFromOriginAndNormal(np.array(palmToWorld2.GetPosition()), motionVector)

        p = self.ikPlanner.createLinePositionConstraint(handLinkName, linkOffsetFrame, motionTargetFrame, lineAxis=2, bounds=[-np.linalg.norm(motionVector), 0.0], positionTolerance=0.001)
        constraintSet.constraints.append(p)

        plan = constraintSet.runIkTraj()
        self.addPlan(plan)

    def planPreChop(self):
        self.planReach(self.doorHandlePreChopFrame, self.speedHigh)


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


    def planHandleTurn(self, turnAngle=None):

        doorSide = 1 if self.graspingHand == 'left' else -1
        if turnAngle is None:
            turnAngle = self.handleTurnAngle

        startPose = self.getPlanningStartPose()
        linkFrame = self.ikPlanner.getLinkFrameAtPose(self.ikPlanner.getHandLink(), startPose)

        finalGraspToReferenceTransfrom = transformUtils.concatenateTransforms(
            [self.ikPlanner.getPalmToHandLink(self.graspingHand), linkFrame,
             self.doorHandleAxisFrame.transform.GetInverse()])

        handleTurnTransform = transformUtils.frameFromPositionAndRPY([0.0, 0.0, 0.0],
                                                                     [doorSide*turnAngle, 0, 0])
        doorHandleTurnFrame = transformUtils.concatenateTransforms([finalGraspToReferenceTransfrom,
                                                                    handleTurnTransform,
                                                                    self.doorHandleAxisFrame.transform])

        vis.updateFrame(doorHandleTurnFrame, 'debug turn', parent=self.doorHandleAffordance, visible=False, scale=0.2)
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand,
                                                           doorHandleTurnFrame)

        constraintSet.ikParameters = IkParameters(usePointwise=False,
                                                  maxDegreesPerSecond=self.speedLow,
                                                  numberOfAddedKnots=2)
        constraintSet.nominalPoseName = 'q_start'
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

        ikParameters = IkParameters(usePointwise=False, maxDegreesPerSecond=15)

        nonGraspingHand = 'right' if self.graspingHand == 'left' else 'left'

        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'door', 'smash', side=nonGraspingHand)

        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose, ikParameters=ikParameters)
        self.addPlan(newPlan)

    def planDoorPushOpenTwist(self):

        ikParameters = IkParameters(usePointwise=False, maxDegreesPerSecond=60)

        nonGraspingHand = 'right' if self.graspingHand == 'left' else 'left'

        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'door', 'smash 2', side=nonGraspingHand)

        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose, ikParameters=ikParameters)
        self.addPlan(newPlan)


    def planHandlePush(self):

        doorSide = 1 if self.graspingHand == 'left' else -1

        startPose = self.getPlanningStartPose()
        linkFrame = self.ikPlanner.getLinkFrameAtPose(self.ikPlanner.getHandLink(), startPose)

        finalGraspToReferenceTransfrom = transformUtils.concatenateTransforms(
            [self.ikPlanner.getPalmToHandLink(self.graspingHand), linkFrame,
             self.doorHingeFrame.transform.GetInverse()])

        handlePushTransform = transformUtils.frameFromPositionAndRPY([0.0, 0.0, 0.0],
                                                                     [0, 0, -doorSide*self.handlePushAngle])
        doorHandlePushFrame = transformUtils.concatenateTransforms([finalGraspToReferenceTransfrom,
                                                                    handlePushTransform,
                                                                    self.doorHingeFrame.transform])

        vis.updateFrame(doorHandlePushFrame, 'debug push', parent=self.doorHandleAffordance, visible=False, scale=0.2)
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand,
                                                           doorHandlePushFrame)

        constraintSet.ikParameters = IkParameters(usePointwise=False, maxDegreesPerSecond=self.speedLow)
        constraintSet.nominalPoseName = 'q_start'
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

        self.planHandleTurn(-self.handleTurnAngle)


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

        ikParameters = IkParameters(usePointwise=False, maxDegreesPerSecond=self.speedHigh)
        nonGraspingHand = 'right' if self.graspingHand == 'left' else 'left'

        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'door', 'pre-smash', side=nonGraspingHand)

        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose, ikParameters=ikParameters)
        self.addPlan(newPlan)

    def planHandlePushOpen(self):

        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.doorHandlePushOpenFrame)
        constraintSet.ikParameters = IkParameters(usePointwise=False,
                                                  maxDegreesPerSecond=self.speedLow,
                                                  numberOfAddedKnots=2)

        endPose, info = constraintSet.runIk()
        plan = constraintSet.runIkTraj()

        self.addPlan(plan)


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


    def commitManipPlan(self):
            self.manipPlanner.commitManipPlan(self.plans[-1])

    def fitDoor(self, doorGroundFrame):
        om.removeFromObjectModel(om.findObjectByName('affordances'))
        self.spawnDoorAffordance()
        affordanceFrame = om.findObjectByName('door ground frame')
        assert affordanceFrame is not None
        affordanceFrame.copyFrame(doorGroundFrame)

        om.findObjectByName('door').setProperty('Visible', False)


    def showDoorHandlePoints(self, polyData):

        doorHandle = om.findObjectByName('door handle')
        door = om.findObjectByName('door')

        doorWidth = door.getProperty('Dimensions')[1]
        doorAxes = transformUtils.getAxesFromTransform(door.getChildFrame().transform)
        doorOrigin = np.array(door.getChildFrame().transform.GetPosition())

        handleAxes = transformUtils.getAxesFromTransform(doorHandle.getChildFrame().transform)
        handleOrigin = np.array(doorHandle.getChildFrame().transform.GetPosition())

        doorSide = 1 if self.graspingHand == 'left' else -1

        polyData = segmentation.cropToLineSegment(polyData, doorOrigin - doorAxes[0]*0.02, doorOrigin - doorAxes[0]*0.1)
        polyData = segmentation.cropToLineSegment(polyData, doorOrigin, doorOrigin + doorAxes[1]*(doorWidth*0.5-0.01)*doorSide)
        polyData = segmentation.cropToLineSegment(polyData, handleOrigin - handleAxes[2]*0.1, handleOrigin + handleAxes[2]*0.1)

        pointsName = 'door handle points'
        existed = om.findObjectByName(pointsName) is not None
        obj = vis.updatePolyData(polyData, pointsName, parent=doorHandle, color=[1,0,0])
        if not existed:
            obj.setProperty('Point Size', 10)


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
        doorDepth = 0.5 * 0.0254

        doorSide = 1 if self.graspingHand == 'left' else -1
        handleHeightFromGround = 35 * 0.0254
        handleDistanceFromEdge = 1.625 * 0.0254
        handleDistanceFromDoor = 1.75 * 0.0254
        handleLength = 4.125 * 0.0254
        handleDepth = 0.25 * 0.0254

        doorJamWidth = 0.5
        doorJamDepth = 4.5 * 0.0254


        handleFrame = transformUtils.frameFromPositionAndRPY([-handleDistanceFromDoor - doorDepth/2.0 - handleDepth/2, doorSide*(doorWidth/2.0 - handleDistanceFromEdge - handleLength/2.0), handleHeightFromGround], [0.0, 0.0, 0.0])
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
           pose=transformUtils.poseFromTransform(handleFrame), Dimensions=[handleDepth, handleLength, 0.02], Color=[0.0, 1.0, 0.0])
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

        doorWalkFrame = vis.showFrame(doorWalkFrame, 'door walk frame', visible=False, parent=doorAffordance)


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




class DoorImageFitter(ImageBasedAffordanceFit):

    def __init__(self, doorDemo):
        ImageBasedAffordanceFit.__init__(self, numberOfPoints=1)
        self.doorDemo = doorDemo

    def fit(self, polyData, points):
        stanceFrame = FootstepRequestGenerator.getRobotStanceFrame(self.doorDemo.robotModel)

        doorGroundFrame = segmentation.segmentDoorPlane(polyData, points[0], stanceFrame)
        self.doorDemo.fitDoor(doorGroundFrame)
        self.doorDemo.showDoorHandlePoints(polyData)

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
        self.addManualButton('Footsteps through door', self.doorDemo.planFootstepsThroughDoor)
        self.addManualSpacer()
        self.addManualButton('Raise arms', self.doorDemo.planPreReach)
        self.addManualButton('Tuck Arms (pre-push)', self.doorDemo.planTuckArmsPrePush)
        self.addManualButton('Tuck Arms', self.doorDemo.planTuckArms)
        self.addManualSpacer()
        self.addManualButton('Open pinch', self.openPinch)
        self.addManualButton('Close pinch', self.closePinch)
        self.addManualSpacer()
        self.addManualButton('Reach', self.doorDemo.planReach)
        self.addManualButton('Un-reach', self.doorDemo.planUnReach)
        self.addManualSpacer()
        self.addManualButton('Pre-chop out', self.doorDemo.planPreChop)
        self.addManualButton('Chop', self.doorDemo.planChop)
        self.addManualButton('Un-chop', functools.partial(self.doorDemo.planChop, deltaX=-0.1, deltaY=-0.1, deltaZ=0.1))
        self.addManualSpacer()
        self.addManualButton('Turn more', functools.partial(self.doorDemo.planHandleTurn, 10))
        self.addManualButton('Turn less', functools.partial(self.doorDemo.planHandleTurn, -10))
        self.addManualButton('Twist arm', self.doorDemo.planDoorPushOpenTwist)
        self.addManualSpacer()
        self.addManualButton('Commit Manip', self.doorDemo.commitManipPlan)
        self.addManualButton('Stop pushing', self.doorDemo.stopPushing)


    def getSide(self):
        return self.params.getPropertyEnumValue('Hand').lower()

    def openPinch(self):
        rt.OpenHand(side=self.getSide().capitalize(), mode='Pinch').run()

    def closePinch(self):
        rt.CloseHand(side=self.getSide().capitalize(), mode='Pinch').run()

    def addDefaultProperties(self):
        self.params.addProperty('Hand', 0, attributes=om.PropertyAttributes(enumNames=['Left', 'Right']))
        self.params.addProperty('Pre-chop width', self.doorDemo.preChopWidth, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Pre-chop depth', self.doorDemo.preChopDepth, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Pre-chop height', self.doorDemo.preChopHeight, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Chop distance', self.doorDemo.chopDistance, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Chop sideways distance', self.doorDemo.chopSidewaysDistance, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self._syncProperties()

    def onPropertyChanged(self, propertySet, propertyName):
        if propertyName == 'Hand':
            self.taskTree.removeAllTasks()
            self.addTasks()
        self.doorDemo.findDoorHandleAffordance()
        self._syncProperties()

    def _syncProperties(self):
        self.doorDemo.graspingHand = self.params.getPropertyEnumValue('Hand').lower()
        self.doorDemo.ikPlanner.reachingSide = self.doorDemo.graspingHand
        if hasattr(self.doorDemo, 'doorHandleAffordance'):
            self.doorDemo.computeDoorHandleGraspFrame()
        self.doorDemo.chopDistance = self.params.getProperty('Chop distance')
        self.doorDemo.chopSidewaysDistance = self.params.getProperty('Chop sideways distance')
        self.doorDemo.preChopWidth = self.params.getProperty('Pre-chop width')
        self.doorDemo.preChopDepth = self.params.getProperty('Pre-chop depth')
        self.doorDemo.preChopHeight = self.params.getProperty('Pre-chop height')

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
        #addTask(rt.PlanPostureGoal(name='plan walk posture', postureGroup='General', postureName='safe nominal', side='Default'))
        #addTask(rt.CheckPlanInfo(name='check manip plan info'))
        #addTask(rt.CommitManipulationPlan(name='execute manip plan', planName='safe nominal posture plan'))
        #addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'))


        # fit
        #addTask(rt.WaitForMultisenseLidar(name='wait for lidar sweep'))
        addTask(rt.UserPromptTask(name='fit door', message='Please fit and approve door affordance.'))
        addTask(rt.FindAffordance(name='check door affordance', affordanceName='door'))
        addTask(rt.SetNeckPitch(name='set neck position', angle=35))

        # walk
        folder = addFolder('Walk and refit')
        addTask(rt.RequestFootstepPlan(name='plan walk to door', stanceFrameName='door stance frame'))
        addTask(rt.UserPromptTask(name='approve footsteps', message='Please approve footstep plan.'))
        addTask(rt.CommitFootstepPlan(name='walk to door', planName='door stance frame footstep plan'))
        addTask(rt.WaitForWalkExecution(name='wait for walking'))

        # refit
        #addTask(rt.WaitForMultisenseLidar(name='wait for lidar sweep'))
        addTask(rt.UserPromptTask(name='fit door', message='Please fit and approve door handle affordance.'))

        # set fingers
        addTask(rt.OpenHand(name='open hand', side=side, mode='Pinch'))


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
        addManipTask('Raise pushing hand', d.planDoorTouch, userPrompt=False)
        if d.usePinchGrasp:
            addManipTask('Reach', d.planReach, userPrompt=True)
            addFunc(self.closePinch, name='Pinch handle')
            addTask(rt.UserPromptTask(name='Approve grasp',
                                    message='Please verify the pinch grasp'))
            addManipTask('Turn', d.planHandleTurn, userPrompt=False)
            addTask(rt.UserPromptTask(name='Approve handle turn',
                                    message='Please verify that the handle has turned'))
        else:
            addFunc(self.doorDemo.setChopParametersToDefaults, name='re-set chop parameters')
            addFunc(self.closePinch, name='Close hand')
            addManipTask('Reach', d.planReach, userPrompt=True)
            addManipTask('Chop', d.planChop, userPrompt=True)

        addManipTask('Push ajar', d.planHandlePush, userPrompt=False)
        addTask(rt.UserPromptTask(name='Approve door position',
                                  message='Please verify that the door is ajar'))
        addManipTask('Push ajar again', d.planHandlePush, userPrompt=False)

        if d.usePinchGrasp:
            addManipTask('Lift', d.planHandlePushLift, userPrompt=False)
            addTask(rt.CloseHand(name='Open hand', side=side, mode='Pinch', amount=0))

        addManipTask('Push open', d.planDoorPushOpen, userPrompt=False)
        addTask(rt.UserPromptTask(name='Approve door position',
                                  message='Please verify that the door is open'))
        addTask(rt.CloseHand(name='Close hand', side=side))
        addManipTask('Tuck Arms', d.planTuckArms, userPrompt=False)
        addTask(rt.CloseHand(name='Close fist', side=side))

        # walk
        folder = addFolder('Walk through door')
        #addTask(rt.RequestFootstepPlan(name='plan walk through door', stanceFrameName='door walk frame'))
        addFunc(d.planFootstepsThroughDoorManual, name='plan walk through door')
        addTask(rt.UserPromptTask(name='approve footsteps', message='Please approve footstep plan.'))
        addTask(rt.CommitFootstepPlan(name='walk to door', planName='door walk frame footstep plan'))
        addTask(rt.WaitForWalkExecution(name='wait for walking'))


        folder = addFolder('Prep for walking')
        addTask(rt.CloseHand(name='close left hand', side='Left'))
        addTask(rt.CloseHand(name='close right hand', side='Right'))
        addTask(rt.PlanPostureGoal(name='plan walk posture', postureGroup='General', postureName='safe nominal', side='Default'))
        addTask(rt.UserPromptTask(name='approve manip plan', message='Please approve manip plan.'))
        addTask(rt.CommitManipulationPlan(name='execute manip plan', planName='safe nominal posture plan'))
        addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'))

