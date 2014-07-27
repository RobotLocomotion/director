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


def printTF(tf, msg):
    print msg
    print tf.GetPosition()
    print tf.GetOrientation()


class Drill(object):
    def __init__(self):
        self.affordance = None
        self.frame = None

        # relative to drill frame:
        self.stanceFrame = None
        self.buttonFrame = None
        self.tipFrame = None # not used for pointing the drill - out of date
        self.guardFrame = None

        self.drillToGuardTransform = None # used for pointing the drill
        self.palmToDrillTransform = None

        self.frameSync = None

class Wall(object):
    def __init__(self):
        self.affordance = None

        # at startup location (no walking)
        #self.init_XYZ = [0.5, 0.3, 1.1]
        #self.init_RPY = [0,0,240]
        # away from robot - requires walking
        self.initXYZ = [1.65, -0.40, 1.1]
        self.initRPY = [0,0,-180]

        self.transform = None

        self.stanceFrame = None
        self.cutPointFrames = []

        self.frameSync = None


class DrillPlannerDemo(object):

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

        self.planFromCurrentRobotState = True # False for operation, True for development

        # For testing:
        self.visOnly = True
        self.useFootstepPlanner = True # for testing used False
        self.flushNominalPlanSequence = False
        # For autonomousExecute
        #self.visOnly = False
        #self.useFootstepPlanner = True

        self.userPromptEnabled = True
        self.constraintSet = None

        self.plans = []

        self.drill = Drill()
        self.wall = Wall()

        # params:
        self.retractDepthNominal = 0.055 # depth to move drill away from wall


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


    ### Drill Focused Functions ######################################################################
    def computeDrillFrame(self, robotModel):

        # drill close to origin (default)
        #position = [0.65, 0.4, 0.9]
        #rpy = [1, 1, 1]

        # drill further away - requires walking
        position = [1.5, 0.6, 1.2]
        rpy = [1, 1, 1]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.computeGroundFrame(robotModel))
        return t


    def computeDrillGraspFrame(self):
        self.computeDrillGraspFrameBarrel()


    def computeDrillGraspFrameRotary(self):

        assert self.drill.affordance

        position = [0.0,-0.18,0.0]
        rpy = [-90,90,0]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.drill.frame.transform)

        self.drill.graspFrame = vis.updateFrame(t, 'grasp frame', parent=self.drill.affordance, visible=False, scale=0.3)


    def computeDrillGraspFrameBarrel(self):

        assert self.drill.affordance

        # for left_base_link
        #position = [-0.12, 0.0, 0.025]
        #rpy = [0, 90, 0]

        # for palm point
        position = [-0.04, 0.0, 0.01]
        rpy = [0, 90, -90]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.drill.frame.transform)

        self.drill.graspFrame = vis.updateFrame(t, 'grasp frame', parent=self.drill.affordance, visible=True, scale=0.2)
        self.drill.sampleGraspFrame = vis.updateFrame(transformUtils.copyFrame(t), 'sample grasp frame 0', parent=self.drill.affordance, visible=False, scale=0.2)

        self.drill.frameSync = vis.FrameSync()
        self.drill.frameSync.addFrame(self.drill.graspFrame)
        self.drill.frameSync.addFrame(self.drill.sampleGraspFrame)
        self.drill.frameSync.addFrame(self.drill.frame)


    def computeDrillTipFrame(self):

        assert self.drill.affordance

        position = [0.18, 0.0, 0.13]
        rpy = [0, 0, 0]
        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.drill.frame.transform)

        self.drill.tipFrame = vis.updateFrame(t, 'drill tip frame', parent=self.drill.affordance, visible=False, scale=0.2)

        self.drill.frameSync.addFrame(self.drill.frame)
        self.drill.frameSync.addFrame(self.drill.tipFrame)


    def computeDrillStanceFrame(self):

        graspFrame = self.drill.graspFrame.transform

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

        self.drill.stanceFrame = vis.updateFrame(t, 'drill stance', parent=self.drill.affordance, visible=False, scale=0.2)

        self.drill.frameSync.addFrame(self.drill.frame)
        self.drill.frameSync.addFrame(self.drill.stanceFrame)


    def computeDrillButtonFrame(self):
        buttonXYZ = [ self.drill.affordance.params['button_x'], self.drill.affordance.params['button_y'], self.drill.affordance.params['button_z'] ]
        buttonNormal = np.array([ self.drill.affordance.params['button_nx'], self.drill.affordance.params['button_ny'], self.drill.affordance.params['button_nz'] ])

        yaxis = -buttonNormal
        xaxis = [0, 0, 1]
        zaxis = np.cross(yaxis, xaxis)
        xaxis = np.cross(zaxis, yaxis)
        xaxis /= np.linalg.norm(xaxis)
        zaxis /= np.linalg.norm(zaxis)
        t = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
        t.PostMultiply()
        t.Translate(buttonXYZ)

        #t = transformUtils.frameFromPositionAndRPY(buttonXYZ, buttonRPY)
        t.Concatenate( self.drill.frame.transform)
        self.drill.buttonFrame = vis.showFrame(t, 'drill button', parent=self.drill.affordance, visible=True, scale=0.2)


    def computeDrillGuardFrame(self):
        guardXYZ = [ self.drill.affordance.params['guard_x'], self.drill.affordance.params['guard_y'], self.drill.affordance.params['guard_z'] ]
        guardNormal = np.array([ self.drill.affordance.params['guard_nx'], self.drill.affordance.params['guard_ny'], self.drill.affordance.params['guard_nz'] ])

        # this is kind of hacked up to put the normal in the direction of the palm frame:
        xaxis = -guardNormal
        zaxis = [0, 0, 1]
        yaxis = np.cross(xaxis, zaxis)
        zaxis = np.cross(yaxis, xaxis)
        #zaxis /= np.linalg.norm(zaxis)
        #yaxis /= np.linalg.norm(yaxis)

        t = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
        t.PostMultiply()
        t.Translate(guardXYZ)
        self.drill.drillToGuardTransform = transformUtils.copyFrame(t)
        t.Concatenate( self.drill.frame.transform)
        self.drill.guardFrame = vis.updateFrame(t, 'drill guard', parent=self.drill.affordance, visible=True, scale=0.2)


    def spawnDrillAffordance(self, drillType="Button"):

        drillFrame = self.computeDrillFrame(self.robotModel)

        folder = om.getOrCreateContainer('affordances')
        if (drillType=="Button"):
            drillMesh = segmentation.getDrillMesh()
            params = segmentation.getDrillAffordanceParams(np.array(drillFrame.GetPosition()), [1,0,0], [0,1,0], [0,0,1])
        else:
            drillMesh = segmentation.getDrillBarrelMesh()
            params =[]

        self.drill.affordance = vis.showPolyData(drillMesh, 'drill', color=[0.0, 1.0, 0.0], cls=vis.FrameAffordanceItem, parent=folder)
        self.drill.affordance.actor.SetUserTransform(drillFrame)
        self.drill.frame = vis.showFrame(drillFrame, 'drill frame', parent=self.drill.affordance, visible=False, scale=0.2)
        self.drill.affordance.setAffordanceParams(params)

        self.computeDrillGraspFrame()
        self.computeDrillStanceFrame()
        self.computeDrillTipFrame()
        self.computeDrillButtonFrame()
        self.computeDrillGuardFrame()


    def moveDrillToHand(self):
        hand = 'left'
        rotation = 90
        offset = .0
        drillFlip = False
        self.drill.palmToDrillTransform = segmentation.getDrillInHandOffset(rotation, offset, drillFlip)

        # create a drill and put it in the hand:
        segmentation.moveDrillToHand(self.drill.palmToDrillTransform, hand)

        self.drill.affordance = om.findObjectByName('drill')
        self.drill.frame = om.findObjectByName('drill frame')

        self.computeDrillButtonFrame()
        self.computeDrillGuardFrame()


    def getHandToGuard(self):
        gazeToHandLinkFrame = self.ikPlanner.newGraspToHandFrame(self.graspingHand)

        handToPalmTransform = vtk.vtkTransform()
        handToPalmTransform.PostMultiply()
        handToPalmTransform.Concatenate( self.robotModel.getLinkFrame('left_palm') )
        handToPalmTransform.Concatenate( self.robotModel.getLinkFrame('l_hand').GetLinearInverse() )

        # planning is done in drill tip frame - which is relative to the hand link:
        # drill-to-hand = tip-to-drill * drill-to-palm * palm-to-hand
        handToGuard = vtk.vtkTransform()
        handToGuard.PostMultiply()
        handToGuard.Concatenate( self.drill.drillToGuardTransform )
        handToGuard.Concatenate( self.drill.palmToDrillTransform )
        handToGuard.Concatenate( handToPalmTransform )
        return handToGuard


    def getWorldToGuard(self):
        handToGuard = self.getHandToGuard()
        worldToGuard = transformUtils.copyFrame( handToGuard)
        worldToGuard.Concatenate( self.robotModel.getLinkFrame('l_hand')  )
        return worldToGuard


    def findDrillAffordance(self):
        self.drill.affordance = om.findObjectByName('drill')
        self.drill.frame = om.findObjectByName('drill frame')

    def spawnWallAffordance(self):
        rightAngleLocation = "bottom left"
        self.wall.transform = transformUtils.frameFromPositionAndRPY(self.wall.initXYZ, self.wall.initRPY)
        segmentation.createDrillWall(rightAngleLocation, self.wall.transform)
        self.wall.affordance = om.getOrCreateContainer('wall')

        graspFrame = self.wall.transform

        groundFrame = self.computeGroundFrame(self.robotModel)
        groundHeight = groundFrame.GetPosition()[2]

        graspPosition = np.array(  graspFrame.GetPosition())
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

        position = [-0.28, -0.53, 0.0]
        rpy = [0, 0, 30]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(graspGroundFrame)

        self.wall.stanceFrame = vis.updateFrame(t, 'wall stance', parent=self.wall.affordance, visible=True, scale=0.2)

        self.wall.frameSync= vis.FrameSync()
        self.wall.frameSync.addFrame(self.wall.affordance.getChildFrame())
        self.wall.frameSync.addFrame(self.wall.stanceFrame)

    ### End Drill Focused Functions ###############################################################
    ### Planning Functions ###############################################################

    def planStand(self):
        startPose = self.getPlanningStartPose()
        newPlan = self.ikPlanner.computeNominalPlan(startPose)
        self.addPlan(newPlan)


    def planFootsteps(self, goalFrame):
        startPose = self.getPlanningStartPose()
        # goalFrame = self.drill.stanceFrame.transform
        request = self.footstepPlanner.constructFootstepPlanRequest(startPose, goalFrame)
        self.footstepPlan = self.footstepPlanner.sendFootstepPlanRequest(request, waitForResponse=True)


    def planWalking(self):
        startPose = self.getPlanningStartPose()
        walkingPlan = self.footstepPlanner.sendWalkingPlanRequest(self.footstepPlan, startPose, waitForResponse=True)
        self.addPlan(walkingPlan)


    def planEndPose(self):
        graspLinks = {
            'l_hand' : 'left_base_link',
            'r_hand' : 'right_base_link',
           }
        linkName = graspLinks[self.getEndEffectorLinkName()]
        startPose = self.getEstimatedRobotStatePose()
        self.endPosePlan = self.manipPlanner.sendEndPoseGoal(startPose, linkName, self.drill.graspFrame.transform, waitForResponse=True)
        self.showEndPose()


    def planGrasp(self):
        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.drill.graspFrame, lockTorso=True)
        endPose, info = constraintSet.runIk()
        graspPlan = constraintSet.runIkTraj()
        self.addPlan(graspPlan)


    def planDrillRaisePowerOn(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'drill', 'drill in camera - 2014', side=self.graspingHand)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def planPointerRaisePowerOn(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'drill', 'drill in camera - 2014 pointer', side=self.pointerHand() )
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def planDrillRaiseForCutting(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'drill', 'drill near target - 2014', side=self.graspingHand )
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def planPreGrasp(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'arm up pregrasp', side=self.graspingHand)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)


    def initGazeConstraintSet(self, goalFrame, gazeHand, gazeToHandLinkFrame, gazeAxis=[-1.0, 0.0, 0.0] ):

        # create constraint set
        startPose = self.getPlanningStartPose()
        startPoseName = 'gaze_plan_start'
        endPoseName = 'gaze_plan_end'
        self.ikPlanner.addPose(startPose, startPoseName)
        self.ikPlanner.addPose(startPose, endPoseName)
        self.constraintSet = ikplanner.ConstraintSet(self.ikPlanner, [], startPoseName, endPoseName)
        self.constraintSet.endPose = startPose

        # add body constraints
        bodyConstraints = self.ikPlanner.createMovingBodyConstraints(startPoseName, lockBase=True, lockBack=False, lockLeftArm=gazeHand=='right', lockRightArm=gazeHand=='left')
        self.constraintSet.constraints.extend(bodyConstraints)

        gazeConstraint = self.ikPlanner.createGazeGraspConstraint(gazeHand, goalFrame, gazeToHandLinkFrame, coneThresholdDegrees= 0.0 , gazeAxis=[-1,0,0])
        self.constraintSet.constraints.insert(0, gazeConstraint)


    def appendPositionConstraintForTargetFrame(self, goalFrame, t, gazeHand, gazeToHandLinkFrame):
        positionConstraint, _ = self.ikPlanner.createPositionOrientationGraspConstraints(gazeHand, goalFrame, gazeToHandLinkFrame)
        positionConstraint.tspan = [t, t]
        self.constraintSet.constraints.append(positionConstraint)


    def planGazeTrajectory(self):
        self.ikPlanner.ikServer.usePointwise = False
        plan = self.constraintSet.runIkTraj()
        self.addPlan(plan)


    def planPointerPressGaze(self):
        gazeHand = self.pointerHand()
        self.moveDrillToHand()

        # add gaze constraint
        gazeToHandLinkFrame = self.ikPlanner.newGraspToHandFrame(gazeHand)
        self.initGazeConstraintSet(self.drill.buttonFrame, gazeHand, gazeToHandLinkFrame, gazeAxis=[0.0, 1.0, 0.0])
        self.appendPositionConstraintForTargetFrame(self.drill.buttonFrame, 1, gazeHand, gazeToHandLinkFrame)
        #self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedSlow
        self.planGazeTrajectory()
        #self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedHigh


    def planNextCut(self, getNextAction , argument1=None ):
        if (argument1 is None):
            self.moveDrillToHand
            success = getNextAction()
            print success
        else:
            success = getNextAction(argument1)

        if (success is False):
            return success

        self.planDrillingGaze()
        return success


    def planFirstCutDesired(self, engagedTip=True):

        t0 = transformUtils.copyFrame( self.wall.transform )

        t1 = transformUtils.frameFromPositionAndRPY([0, self.wall.affordance.params['p2y'], self.wall.affordance.params['p2z'] ], [0,0,0])
        t1.Concatenate(  self.wall.transform )

        t2 = transformUtils.frameFromPositionAndRPY([0, self.wall.affordance.params['p3y'], self.wall.affordance.params['p3z'] ], [0,0,0])
        t2.Concatenate(  self.wall.transform )

        self.wall.cutPointFrames.append( vis.updateFrame(t0, 'cut point 0', parent=self.wall.affordance, visible=False, scale=0.2) )
        self.wall.cutPointFrames.append( vis.updateFrame(t1, 'cut point 1', parent=self.wall.affordance, visible=False, scale=0.2) )
        self.wall.cutPointFrames.append( vis.updateFrame(t2, 'cut point 2', parent=self.wall.affordance, visible=False, scale=0.2) )
        self.wall.frameSync.addFrame( self.wall.cutPointFrames[0] )
        self.wall.frameSync.addFrame( self.wall.cutPointFrames[1] )
        self.wall.frameSync.addFrame( self.wall.cutPointFrames[2] )

        self.cutOrientation = self.wall.transform.GetOrientation()
        self.cutPoints = [t0.GetPosition() , t1.GetPosition(), t2.GetPosition(), t0.GetPosition() ]
        self.currentCutGoal = 0

        nextCutPoseGoal = transformUtils.frameFromPositionAndRPY( self.cutPoints[ self.currentCutGoal ] , self.cutOrientation )
        self.nextCutGoalFrame = vis.updateFrame(nextCutPoseGoal, 'next cut goal', visible=True, scale=0.2, parent=om.getOrCreateContainer('affordances'))

        if (engagedTip == False):
            nextCutPose = transformUtils.frameFromPositionAndRPY( [0.055, 0, 0] , [0,0,0] )
            nextCutPose.Concatenate( nextCutPoseGoal )
        else:
            nextCutPose = transformUtils.copyFrame( nextCutPoseGoal )

        self.nextCutFrame = vis.updateFrame(nextCutPose, 'next cut', visible=True, scale=0.2, parent=om.getOrCreateContainer('affordances'))


    def computeRetractionDesired(self, retractGuard=True):
        self.worldToGuard = self.getWorldToGuard()

        # relative position of g
        goalToGuard = transformUtils.copyFrame( self.worldToGuard )
        goalToGuard.Concatenate( self.nextCutGoalFrame.transform.GetLinearInverse() )

        if (retractGuard == True):
            retractDepth = self.retractDepthNominal
        else: # this can be used to insert where we are located:
            retractDepth = 0.0

        self.goalToNextCut = transformUtils.frameFromPositionAndRPY( [retractDepth, goalToGuard.GetPosition()[1], goalToGuard.GetPosition()[2] ] , [0,0,0] )
        self.worldToNextCut = transformUtils.copyFrame( self.goalToNextCut )
        self.worldToNextCut.Concatenate( self.nextCutGoalFrame.transform )
        self.nextCutFrame = vis.updateFrame(self.worldToNextCut, 'next cut', visible=True, scale=0.2)


    def computeNextCutDesired(self):
        # determine the next cutting position (self.nextCutFrame)
        #
        # Overview:
        # if close to the current goal, switch to the next goal
        #    if there are no more goals, you are finished
        # move in the direction of the goal
        #   find a point thats x cm closer to the goal in the plane of the target

        self.worldToGuard = self.getWorldToGuard()

        # relative position of guard
        goalToGuard = transformUtils.copyFrame( self.worldToGuard )
        goalToGuard.Concatenate( self.nextCutGoalFrame.transform.GetLinearInverse() )

        distToGoal2D = np.linalg.norm( goalToGuard.GetPosition()[1:3] )
        print "distToGoal2D", distToGoal2D , "from" , self.currentCutGoal

        if (distToGoal2D < 0.05):
            if ( self.currentCutGoal == len(self.cutPoints) -1):
                print distToGoal2D , " - within threshold of last goal", self.currentCutGoal
                return False

            self.currentCutGoal=self.currentCutGoal+1
            nextCutPose = transformUtils.frameFromPositionAndRPY( self.cutPoints[ self.currentCutGoal ] , self.cutOrientation )
            self.nextCutGoalFrame = vis.updateFrame(nextCutPose, 'next cut goal', visible=True, scale=0.2)
            #print distToGoal2D , " - within threshold. Moving to", self.currentCutGoal

        # TODO: change this to a metric movement towards a goal:
        self.goalToNextCut = transformUtils.frameFromPositionAndRPY( [0, goalToGuard.GetPosition()[1] *0.5 , goalToGuard.GetPosition()[2] *0.5] , [0,0,0] )
        #printTF(self.goalToNextCut, "goalToNextCut")
        self.worldToNextCut = transformUtils.copyFrame( self.goalToNextCut )
        self.worldToNextCut.Concatenate( self.nextCutGoalFrame.transform )
        #printTF(self.worldToNextCut, "worldToNextCut")
        self.nextCutFrame = vis.updateFrame(self.worldToNextCut, 'next cut', visible=True, scale=0.2)

        return True


    def planDrillingGaze(self):
        # sets the designer's position of the robot to the last sample in the plan
        # this is required when playing planNominalCut() so that the hand-to-drill transform can be correctly determined. REMOVE!
        self.sensorJointController.setPose("EST_ROBOT_STATE",  robotstate.convertStateMessageToDrakePose( self.plans[-1].plan[-1] ) )
        # required to move drill to hand
        self.moveDrillToHand() # should not required, remove when not needed

        handToGuard = self.getHandToGuard()

        if (1==0):
            vis.updateFrame(self.robotModel.getLinkFrame('l_hand'), 'l_hand mfallon', visible=True, scale=0.2)
            worldToGuard = self.getWorldToGuard()
            vis.updateFrame(worldToGuard, 'world to drill guard', visible=True, scale=0.2)

            print "handToGuard"
            print handToGuard.GetPosition()
            print handToGuard.GetOrientation()

        self.initGazeConstraintSet(self.nextCutFrame, self.graspingHand, handToGuard, gazeAxis=[-1.0, 0.0, 0.0])
        self.appendPositionConstraintForTargetFrame(self.nextCutFrame, 1, self.graspingHand, handToGuard)
        #self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedSlow
        self.planGazeTrajectory()
        #self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedHigh


    ########## Glue Functions ####################################
    def moveRobotToStanceFrame(self, frame):
        #frame = self.drill.stanceFrame.transform
        self.sensorJointController.setPose('q_nom')
        stancePosition = frame.GetPosition()
        stanceOrientation = frame.GetOrientation()

        q = self.sensorJointController.q.copy()
        q[:2] = [stancePosition[0], stancePosition[1]]
        q[5] = math.radians(stanceOrientation[2])
        self.sensorJointController.setPose('EST_ROBOT_STATE', q)


    def showEndPose(self):
        endPose = robotstate.convertStateMessageToDrakePose(self.endPosePlan)
        self.showPoseFunction(endPose)


    def planPostureGoal(self, groupName, poseName, side=None):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, groupName, poseName, side=side)
        self.posturePlan = self.ikPlanner.computePostureGoal(startPose, endPose)


    def getEndEffectorLinkName(self):
        linkMap = {
                      'left' : 'l_hand',
                      'right': 'r_hand'
                  }
        return linkMap[self.graspingHand]


    def sendPelvisCrouch(self):
        self.atlasDriver.sendPelvisHeightCommand(0.7)

    def sendPelvisStand(self):
        self.atlasDriver.sendPelvisHeightCommand(0.8)


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


    def pointerHand(self):
        if (self.graspingHand == 'left'):
            return 'right'
        else:
            return 'left'


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

    def playNominalPlan(self):
        assert None not in self.plans
        self.planPlaybackFunction(self.plans)

    def sendPlanWithHeightMode(self):
        self.atlasDriver.sendPlanUsingBdiHeight(True)


    ######### Nominal Plans and Execution  #################################################################
    def planNominalPickUp(self, playbackNominal=True):

        self.planFromCurrentRobotState = False
        self.plans = []

        # self.findDrillAffordance()
        if (om.findObjectByName('drill') is None):
            #self.moveDrillToHand() # should not required, remove when not needed
            self.spawnDrillAffordance()

        if (om.findObjectByName('wall') is None):
            self.spawnWallAffordance()


        self.computeDrillGraspFrame()
        self.computeDrillStanceFrame()

        if self.useFootstepPlanner:
            self.planFootsteps( self.drill.stanceFrame.transform )
            self.planWalking()
        else:
            self.moveRobotToStanceFrame(self.drill.stanceFrame.transform)

        self.planPreGrasp()
        self.planGrasp()

        self.planPreGrasp()
        self.planStand()

        if (playbackNominal is True):
            self.playNominalPlan()


    def planNominalTurnOn(self, playbackNominal=True):

        self.planFromCurrentRobotState = False

        if (self.flushNominalPlanSequence):
            self.plans = []

        if (om.findObjectByName('wall') is None):
            self.spawnWallAffordance()

        # sets the designer's position of the robot to the last sample in the plan
        # this is required when playing planNominalCut() so that the hand-to-drill transform can be correctly determined. REMOVE!
        self.sensorJointController.setPose("EST_ROBOT_STATE",  robotstate.convertStateMessageToDrakePose( self.plans[-1].plan[-1] ) )

        if self.useFootstepPlanner:
            self.planFootsteps( self.wall.stanceFrame.transform )
            self.planWalking()
        else:
            self.moveRobotToStanceFrame( self.wall.stanceFrame.transform )

        self.planDrillRaisePowerOn()
        self.planPointerRaisePowerOn()

        # sets the designer's position of the robot to the last sample in the plan
        # this is required when playing planNominalCut() so that the hand-to-drill transform can be correctly determined. REMOVE!
        self.sensorJointController.setPose("EST_ROBOT_STATE",  robotstate.convertStateMessageToDrakePose( self.plans[-1].plan[-1] ) )
        # required to move drill to hand
        self.moveDrillToHand() # should not required, remove when not needed
        self.planPointerPressGaze()
        self.planPointerRaisePowerOn()

        # lower again
        #self.planStand()

        # required to move drill to hand
        #self.moveDrillToHand() # should not required, remove when not needed

        if (playbackNominal is True):
            self.playNominalPlan()


    def planNominalCut(self, playbackNominal=True):

        # sets the designer's position of the robot to the last sample in the plan
        # this is required when playing planNominalCut() so that the hand-to-drill transform can be correctly determined. REMOVE!
        self.sensorJointController.setPose("EST_ROBOT_STATE",  robotstate.convertStateMessageToDrakePose( self.plans[-1].plan[-1] ) )

        self.planFromCurrentRobotState = False

        if (self.flushNominalPlanSequence):
            self.plans = []

        #if (om.findObjectByName('drill') is None):
        #    #self.moveDrillToHand() # should not required, remove when not needed
        #    self.spawnDrillAffordance()

        #if (om.findObjectByName('wall') is None):
        #    self.spawnWallAffordance()

        #self.spawnWallAffordance()

        self.planDrillRaiseForCutting()

        self.moveDrillToHand()

        self.planNextCut( self.planFirstCutDesired, False )
        self.planNextCut( self.planFirstCutDesired, True )

        success = True
        while (success is True):
            success = self.planNextCut( self.computeNextCutDesired )
            print " "

        self.planNextCut( self.computeRetractionDesired, True )
        self.planDrillRaiseForCutting()
        self.planStand()

        if (playbackNominal is True):
            self.playNominalPlan()


    def planNominal(self, playbackNominal=True):
        self.planNominalPickUp(playbackNominal=False)
        self.planNominalTurnOn(playbackNominal=False)
        self.planNominalCut(playbackNominal=False)
        self.playNominalPlan()

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

        taskQueue.addTask(self.printAsync('fitting drill affordance'))
        taskQueue.addTask(self.affordanceFitFunction)
        taskQueue.addTask(self.findDrillAffordance)

        # compute grasp & stance
        taskQueue.addTask(self.printAsync('computing grasp and stance frames'))
        taskQueue.addTask(self.computeDrillGraspFrame)
        taskQueue.addTask(self.computeDrillStanceFrame)

        # footstep plan
        taskQueue.addTask(self.printAsync('compute footstep plan'))
        taskQueue.addTask(self.planFootsteps)

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
        taskQueue.addTask(self.planPreGrasp)

        # user prompt
        taskQueue.addTask(self.userPrompt('commit manip plan. continue? y/n: '))

        # commit pre grasp plan
        taskQueue.addTask(self.atlasDriver.sendManipCommand)
        taskQueue.addTask(self.delay(1.0))
        taskQueue.addTask(self.printAsync('commit pre grasp plan'))
        taskQueue.addTask(self.delay(10.0))


        # user prompt
        taskQueue.addTask(self.userPrompt('perception and fitting. continue? y/n: '))

        # perception & fitting
        taskQueue.addTask(self.printAsync('waiting for clean lidar sweep'))
        taskQueue.addTask(self.waitForCleanLidarSweepAsync)

        taskQueue.addTask(self.printAsync('fitting drill affordance'))
        taskQueue.addTask(self.affordanceFitFunction)
        taskQueue.addTask(self.findDrillAffordance)

        # compute grasp frame
        taskQueue.addTask(self.printAsync('computing grasp frame'))
        taskQueue.addTask(self.computeDrillGraspFrame)


        # compute grasp plan
        taskQueue.addTask(self.printAsync('computing grasp plan'))
        taskQueue.addTask(self.planGrasp)

        # user prompt
        taskQueue.addTask(self.userPrompt('commit manip plan. continue? y/n: '))

        # commit grasp plan
        taskQueue.addTask(self.printAsync('commit grasp plan'))
        taskQueue.addTask(self.commitGraspPlan)
        taskQueue.addTask(self.delay(10.0))

        # recompute grasp plan
        taskQueue.addTask(self.printAsync('recompute grasp plan'))
        taskQueue.addTask(self.planGrasp)
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
        taskQueue.addTask(self.planStand)

        taskQueue.addTask(self.userPrompt('commit stand. continue? y/n: '))

        # compute pre grasp plan
        taskQueue.addTask(self.delay(10.0))
        '''

        # user prompt
        #taskQueue.addTask(self.userPrompt('commit manip plan. continue? y/n: '))

        # commit pre grasp plan
        #taskQueue.addTask(self.printAsync('commit pre grasp plan'))
        #taskQueue.addTask(self.delay(10.0))

        taskQueue.addTask(self.printAsync('done!'))

        return taskQueue