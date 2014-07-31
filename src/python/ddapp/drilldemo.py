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
from ddapp import affordancegraspupdater

import drc as lcmdrc

from PythonQt import QtCore, QtGui


class Drill(object):
    def __init__(self):
        self.affordance = None
        self.frame = None

        # relative to drill frame:
        self.stanceFrame = None
        self.buttonFrame = None
        self.bitFrame = None
        self.graspFrame = None
        self.reachFrame = None

        self.drillToButtonTransform = None # used for turning on the drill
        self.drillToBitTransform = None # used for cutting with the drill
        self.faceToDrillTransform = None
        self.frameSync = None

        # params:
        self.model="dewalt_button"
        self.faceToDrillRotation = 90
        self.faceToDrillOffset = 0
        self.faceToDrillFlip = False

        # initial drill position (requires walking)
        self.initXYZ = [1.5, 0.6, 0.9] # height requires crouching
        self.initRPY = [1,1,1]

        self.graspFrameXYZ = [-0.04, 0.0, 0.01]
        self.graspFrameRPY = [0, 90, -90]

        # where to stand relative to the drill on a table:
        self.relativeStanceXYZ = [-0.67, -0.4, 0.0]
        self.relativeStanceRPY = [0, 0, 0]

class Wall(object):
    def __init__(self):
        self.affordance = None

        self.transform = None
        self.stanceFrame = None
        self.cutPointFrames = []

        self.frameSync = None

        # params:
        self.rightAngleLocation = "bottom left"
        # at startup location (no walking)
        #self.initXYZ = [0.5, 0.3, 1.1]
        #self.initRPY = [0,0,240]
        # away from robot - requires walking
        self.initXYZ = [1.65, -0.40, 1.1]
        self.initRPY = [0,0,0]

        # where to stand relative to the wall target:
        self.relativeStanceXYZ = [0.28, 0.45, 0.0]
        self.relativeStanceRPY = [0, 0, 210]
        # where to stand relative to the wall target - but distant - for button pressing
        self.relativeStanceFarXYZ = [0.28, 1.0, 0.0]
        self.relativeStanceFarRPY = [0, 0, 210]


class DrillPlannerDemo(object):

    def __init__(self, robotModel, playbackRobotModel, teleopRobotModel, footstepPlanner, manipPlanner, ikPlanner, handDriver, atlasDriver, multisenseDriver, affordanceFitFunction, sensorJointController, planPlaybackFunction, showPoseFunction):
        self.robotModel = robotModel
        self.playbackRobotModel = playbackRobotModel # not used inside the demo
        self.teleopRobotModel = teleopRobotModel # not used inside the demo
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

        defaultGraspingHand = "left"
        self.setGraspingHand(defaultGraspingHand)

        self.planFromCurrentRobotState = False # False for development, True for operation

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
        self.reachDepth = 0.12 # depth to reach to before going for grasp
        self.cutLength = 0.05 # length to cut each time
        self.retractDepthNominal = -0.055 # depth to move drill away from wall
        self.goalThreshold = 0.05 # how close we need to get to the cut goal (the triangle corners

        extraModels = [self.robotModel, self.playbackRobotModel, self.teleopRobotModel]
        self.affordanceUpdater  = affordancegraspupdater.AffordanceGraspUpdater(self.playbackRobotModel, extraModels)


        # These changes are all that are required to run with different combinations
        if ( self.drill.model == 'dewalt_barrel' ):
            print "Using a Dewalt Barrel Drill"
            self.wall.relativeStanceXYZ = [-0.3, 0.75, 0]
            self.wall.relativeStanceRPY = [0, 0, -90]
            self.wall.relativeStanceFarXYZ = [-0.3, 1.25, 0]
            self.wall.relativeStanceFarRPY = [0, 0, -90]
            if ( self.graspingHand == 'right' ):
                self.drill.faceToDrillRotation = -90

        if ( self.graspingHand == 'right' ):
            print "Planning with drill in the right hand"
            self.drill.relativeStanceXYZ[1] = -self.drill.relativeStanceXYZ[1]
            self.drill.graspFrameRPY = [0,-90,-90]
            self.drill.initXYZ[1] = -self.drill.initXYZ[1]
            self.drill.faceToDrillFlip = True

            self.wall.rightAngleLocation = "bottom right"
            self.wall.initXYZ[1] = -self.wall.initXYZ[1]
            self.wall.relativeStanceXYZ[0] = -self.wall.relativeStanceXYZ[0]
            self.wall.relativeStanceRPY[2] = -self.wall.relativeStanceRPY[2] + 180

            self.wall.relativeStanceFarXYZ[0] = -self.wall.relativeStanceFarXYZ[0]
            self.wall.relativeStanceFarRPY[2] = -self.wall.relativeStanceFarRPY[2] + 180


    def setGraspingHand(self, graspingHand="left"):
        self.graspingHand = graspingHand
        self.graspingFaceLink = '%s_hand_face' % self.graspingHand[0]
        self.graspingHandLink = '%s_hand' % self.graspingHand[0]
        if (self.graspingHand == 'left'):
            self.pointerHand='right'
        else:
            self.pointerHand='left'

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


    def computeRobotStanceFrame(self, objectTransform, relativeStanceTransform ):
        '''
        Given a robot model, determine the height of the ground
        using an XY and Yaw standoff, combined to determine the relative 6DOF standoff
        For a grasp or approach stance
        '''

        groundFrame = self.computeGroundFrame(self.robotModel)
        groundHeight = groundFrame.GetPosition()[2]

        graspPosition = np.array(objectTransform.GetPosition())
        graspYAxis = [0.0, 1.0, 0.0]
        graspZAxis = [0.0, 0.0, 1.0]
        objectTransform.TransformVector(graspYAxis, graspYAxis)
        objectTransform.TransformVector(graspZAxis, graspZAxis)

        xaxis = graspYAxis
        #xaxis = graspZAxis
        zaxis = [0, 0, 1]
        yaxis = np.cross(zaxis, xaxis)
        yaxis /= np.linalg.norm(yaxis)
        xaxis = np.cross(yaxis, zaxis)

        graspGroundTransform = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
        graspGroundTransform.PostMultiply()
        graspGroundTransform.Translate(graspPosition[0], graspPosition[1], groundHeight)

        #transformUtils.frameFromPositionAndRPY( self.wall.stanceFrameXYZ , self.wall.stanceFrameRPY )
        robotStance = transformUtils.copyFrame( relativeStanceTransform )
        robotStance.Concatenate(graspGroundTransform)

        return robotStance

    ### Drill Focused Functions ######################################################################
    def computeConvenientDrillFrame(self, robotModel):
        # makes a drill in front of the robot
        # drill further away - requires walking
        # position = [1.5, 0.6, 1.2]
        position = [1.5, 0.6, 0.9]
        rpy = [1, 1, 1]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.computeGroundFrame(robotModel))
        return t


    def computeDrillGraspFrame(self):
        t = transformUtils.frameFromPositionAndRPY( self.drill.graspFrameXYZ , self.drill.graspFrameRPY )
        t.Concatenate(self.drill.frame.transform)
        self.drill.graspFrame = vis.updateFrame(t, 'grasp frame', parent=self.drill.affordance, visible=False, scale=0.2)

    def computeDrillReachFrame(self):
        ''' Reach ~10cm short of the grasp frame '''
        reachXYZ = self.drill.graspFrameXYZ
        reachXYZ[0] = reachXYZ[0] - self.reachDepth
        t = transformUtils.frameFromPositionAndRPY( reachXYZ , self.drill.graspFrameRPY )
        t.Concatenate(self.drill.frame.transform)
        self.drill.reachFrame = vis.updateFrame(t, 'reach frame', parent=self.drill.affordance, visible=False, scale=0.2)

    def computeDrillStanceFrame(self):
        objectTransform = self.drill.graspFrame.transform
        self.drill.relativeStanceTransform = transformUtils.frameFromPositionAndRPY( self.drill.relativeStanceXYZ , self.drill.relativeStanceRPY )
        robotStance = self.computeRobotStanceFrame( objectTransform, self.drill.relativeStanceTransform )
        self.drill.stanceFrame = vis.updateFrame(robotStance, 'drill stance', parent=self.drill.affordance, visible=False, scale=0.2)


    def computeDrillButtonFrame(self):
        position = [ self.drill.affordance.params['button_x'], self.drill.affordance.params['button_y'], self.drill.affordance.params['button_z'] ]
        rpy = np.array([ self.drill.affordance.params['button_roll'], self.drill.affordance.params['button_pitch'], self.drill.affordance.params['button_yaw'] ])
        t = transformUtils.frameFromPositionAndRPY(position, rpy)

        self.drill.drillToButtonTransform = transformUtils.copyFrame( t )
        t.Concatenate( transformUtils.copyFrame(self.drill.frame.transform))
        self.drill.buttonFrame = vis.updateFrame(t, 'drill button', parent=self.drill.affordance, visible=False, scale=0.2)


    def computeDrillBitFrame(self):
        position = [ self.drill.affordance.params['bit_x'], self.drill.affordance.params['bit_y'], self.drill.affordance.params['bit_z'] ]
        rpy = np.array([ self.drill.affordance.params['bit_roll'], self.drill.affordance.params['bit_pitch'], self.drill.affordance.params['bit_yaw'] ])
        t = transformUtils.frameFromPositionAndRPY(position, rpy)

        self.drill.drillToBitTransform = transformUtils.copyFrame(t)
        t.Concatenate(transformUtils.copyFrame( self.drill.frame.transform))
        self.drill.bitFrame = vis.updateFrame(t, 'drill bit', parent=self.drill.affordance, visible=True, scale=0.05)


    def computeFaceToDrillTransform(self):
        # if the drill is held in the hand
        # what will be its position, relative to the l_hand_face (or other)
        self.drill.faceToDrillTransform = segmentation.getDrillInHandOffset(self.drill.faceToDrillRotation, self.drill.faceToDrillOffset, self.drill.faceToDrillFlip)


    def spawnDrillAffordance(self):

        drillTransform = transformUtils.frameFromPositionAndRPY(self.drill.initXYZ, self.drill.initRPY)
        #drillTransform = self.computeConvenientDrillFrame(self.robotModel)

        folder = om.getOrCreateContainer('affordances')
        if (self.drill.model == "dewalt_button"):
            drillMesh = segmentation.getDrillMesh()
            params = segmentation.getDrillAffordanceParams(np.array(drillTransform.GetPosition()), [1,0,0], [0,1,0], [0,0,1])
        else:
            drillMesh = segmentation.getDrillBarrelMesh()
            params = segmentation.getDrillAffordanceParams(np.array(drillTransform.GetPosition()), [1,0,0], [0,1,0], [0,0,1], 'dewalt_barrel')

        self.drill.affordance = vis.showPolyData(drillMesh, 'drill', color=[0.0, 1.0, 0.0], cls=vis.FrameAffordanceItem, parent=folder)
        self.drill.affordance.actor.SetUserTransform(drillTransform)
        self.drill.frame = vis.showFrame(drillTransform, 'drill frame', parent=self.drill.affordance, visible=False, scale=0.2)
        self.drill.affordance.setAffordanceParams(params)

        self.computeDrillGraspFrame()
        self.computeDrillReachFrame()
        self.computeDrillStanceFrame()
        self.computeDrillButtonFrame()
        self.computeDrillBitFrame()
        self.computeFaceToDrillTransform()

        self.drill.frameSync = vis.FrameSync()
        self.drill.frameSync.addFrame(self.drill.frame)
        self.drill.frameSync.addFrame(self.drill.graspFrame)
        self.drill.frameSync.addFrame(self.drill.reachFrame)
        self.drill.frameSync.addFrame(self.drill.stanceFrame)
        self.drill.frameSync.addFrame(self.drill.buttonFrame)
        self.drill.frameSync.addFrame(self.drill.bitFrame)


    def moveDrillToHand(self):
        # This function moves the drill to the hand using the pose used for planning
        # It is similar to segmentation.moveDrillToHand() other wise
        # TODO: deprecate segmentation.moveDrillToHand()

        self.drill.affordance = om.findObjectByName('drill')
        self.drill.frame = om.findObjectByName('drill frame')

        drillTransform = self.drill.affordance.actor.GetUserTransform()
        drillTransform.PostMultiply()
        drillTransform.Identity()
        drillTransform.Concatenate( self.drill.faceToDrillTransform )
        startPose = self.getPlanningStartPose()
        drillTransform.Concatenate( self.ikPlanner.getLinkFrameAtPose( self.graspingFaceLink , startPose) )


    def getHandToBit(self, startPose):
        gazeToHandLinkFrame = self.ikPlanner.newGraspToHandFrame(self.graspingHand)

        handToFaceTransform = vtk.vtkTransform()
        handToFaceTransform.PostMultiply()
        handToFaceTransform.Concatenate( self.ikPlanner.getLinkFrameAtPose( self.graspingFaceLink , startPose) )
        handToFaceTransform.Concatenate( self.ikPlanner.getLinkFrameAtPose( self.graspingHandLink , startPose).GetLinearInverse() )
        self.handToFaceTransform = handToFaceTransform

        # planning is done in drill tip frame - which is relative to the hand link:
        # drill-to-hand = tip-to-drill * drill-to-face * face-to-hand
        handToBit = vtk.vtkTransform()
        handToBit.PostMultiply()
        handToBit.Concatenate( self.drill.drillToBitTransform )
        handToBit.Concatenate( self.drill.faceToDrillTransform )
        handToBit.Concatenate( handToFaceTransform )
        return handToBit


    def getWorldToBit(self, startPose):
        handToBit = self.getHandToBit(startPose)
        worldToBit = transformUtils.copyFrame( handToBit)
        worldToBit.Concatenate( self.ikPlanner.getLinkFrameAtPose( self.graspingHandLink , startPose)  )
        return worldToBit


    def getHandToButton(self, startPose):
        gazeToHandLinkFrame = self.ikPlanner.newGraspToHandFrame(self.graspingHand)

        handToFaceTransform = vtk.vtkTransform()
        handToFaceTransform.PostMultiply()
        handToFaceTransform.Concatenate( self.ikPlanner.getLinkFrameAtPose( self.graspingFaceLink , startPose) )
        handToFaceTransform.Concatenate( self.ikPlanner.getLinkFrameAtPose( self.graspingHandLink , startPose).GetLinearInverse() )
        self.handToFaceTransform = handToFaceTransform

        # button-to-hand = button-to-drill * drill-to-face * face-to-hand
        handToButton = vtk.vtkTransform()
        handToButton.PostMultiply()
        handToButton.Concatenate( self.drill.drillToButtonTransform )
        handToButton.Concatenate( self.drill.faceToDrillTransform )
        handToButton.Concatenate( handToFaceTransform )
        return handToButton


    def getWorldToButton(self, startPose):
        handToButton = self.getHandToButton(startPose)
        worldToButton = transformUtils.copyFrame( handToButton)
        worldToButton.Concatenate( self.ikPlanner.getLinkFrameAtPose( self.graspingHandLink , startPose)  )
        return worldToButton


    def findDrillAffordance(self):
        self.drill.affordance = om.findObjectByName('drill')
        self.drill.frame = om.findObjectByName('drill frame')


    def spawnWallAffordance(self):
        self.wall.transform = transformUtils.frameFromPositionAndRPY(self.wall.initXYZ, self.wall.initRPY)
        segmentation.createDrillWall(self.wall.rightAngleLocation, self.wall.transform)
        self.wall.affordance = om.getOrCreateContainer('wall')

        objectTransform = self.wall.transform
        self.wall.relativeStanceTransform = transformUtils.frameFromPositionAndRPY( self.wall.relativeStanceXYZ , self.wall.relativeStanceRPY )
        robotStance = self.computeRobotStanceFrame( objectTransform, self.wall.relativeStanceTransform )
        self.wall.stanceFrame = vis.updateFrame(robotStance, 'wall stance', parent=self.wall.affordance, visible=True, scale=0.2)
        self.wall.relativeStanceFarTransform = transformUtils.frameFromPositionAndRPY( self.wall.relativeStanceFarXYZ , self.wall.relativeStanceFarRPY )
        robotStanceFar = self.computeRobotStanceFrame( objectTransform, self.wall.relativeStanceFarTransform )
        self.wall.stanceFarFrame = vis.updateFrame(robotStanceFar, 'wall stance far', parent=self.wall.affordance, visible=True, scale=0.2)

        self.wall.frameSync= vis.FrameSync()
        self.wall.frameSync.addFrame(self.wall.affordance.getChildFrame())
        self.wall.frameSync.addFrame(self.wall.stanceFrame)
        self.wall.frameSync.addFrame(self.wall.stanceFarFrame)


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


    def planReach(self):
        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.drill.reachFrame, lockTorso=False)
        endPose, info = constraintSet.runIk()
        graspPlan = constraintSet.runIkTraj()
        self.addPlan(graspPlan)

    def planGrasp(self):
        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.drill.graspFrame, lockTorso=False)
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
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'drill', 'drill in camera - 2014 pointer', side=self.pointerHand )
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def planPointerLowerPowerOn(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'drill', 'one hand down - 2014', side=self.pointerHand )
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)


    def planDrillLowerSafe(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'drill', 'one hand down with drill - 2014', side=self.graspingHand )
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)


    def planDrillRaiseForCutting(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'drill', 'drill near target - 2014', side=self.graspingHand )

        if (self.drill.model == 'dewalt_barrel'):
           endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'arm up pregrasp', side=self.graspingHand)

        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def planPreGrasp(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'arm up pregrasp', side=self.graspingHand)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)


    def initGazeConstraintSet(self, goalFrame, gazeHand, gazeToHandLinkFrame, gazeAxis=[-1.0, 0.0, 0.0], lockBase=False, lockBack=False):

        # create constraint set
        startPose = self.getPlanningStartPose()
        startPoseName = 'gaze_plan_start'
        endPoseName = 'gaze_plan_end'
        self.ikPlanner.addPose(startPose, startPoseName)
        self.ikPlanner.addPose(startPose, endPoseName)
        self.constraintSet = ikplanner.ConstraintSet(self.ikPlanner, [], startPoseName, endPoseName)
        self.constraintSet.endPose = startPose

        # add body constraints
        bodyConstraints = self.ikPlanner.createMovingBodyConstraints(startPoseName, lockBase=lockBase, lockBack=lockBack, lockLeftArm=gazeHand=='right', lockRightArm=gazeHand=='left')
        self.constraintSet.constraints.extend(bodyConstraints)

        coneThresholdDegrees = 0.0
        gazeConstraint = self.ikPlanner.createGazeGraspConstraint(gazeHand, goalFrame, gazeToHandLinkFrame, coneThresholdDegrees , gazeAxis)
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
        gazeHand = self.pointerHand

        # add gaze constraint for pointer along the
        gazeToHandLinkFrame = self.ikPlanner.newGraspToHandFrame(gazeHand)

        worldToButton = self.getWorldToButton( self.getPlanningStartPose() )
        worldToButtonFrame = vis.updateFrame(worldToButton, 'test button', visible=False, scale=0.2, parent=om.getOrCreateContainer('affordances'))

        # was [0,1,0]
        # all_axes = transformUtils.getAxesFromTransform(  gazeToHandLinkFrame )
        # gazeAxis = all_axes[0]
        self.initGazeConstraintSet(worldToButtonFrame, gazeHand, gazeToHandLinkFrame, gazeAxis=[-1.0, 0.0, 0.0], lockBase=True, lockBack=True)
        self.appendPositionConstraintForTargetFrame(worldToButtonFrame, 1, gazeHand, gazeToHandLinkFrame)
        #self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedSlow
        self.planGazeTrajectory()
        #self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedHigh


    def planNextCut(self, getNextAction , argument1=None ):
        if (argument1 is None):
            success = getNextAction()
            #print success
        else:
            success = getNextAction(argument1)

        if (success is False):
            return success

        self.planDrillingGaze()
        return success


    def updateNextCutPoseGoal(self):
        nextCutPoseGoal = transformUtils.frameFromPositionAndRPY( self.cutPoints[ self.currentCutGoal ] , self.cutOrientation )
        self.nextCutGoalFrame = vis.updateFrame(nextCutPoseGoal, 'next cut goal', visible=True, scale=0.2, parent=self.wall.affordance)
        d_dot = DebugData()
        d_dot.addSphere( self.nextCutGoalFrame.transform.GetPosition() , radius=0.02)
        aff = vis.updatePolyData(d_dot.getPolyData(), 'next cut goal dot', color=[1,0.5,0], visible=True, parent=self.wall.affordance)
        return nextCutPoseGoal


    def updateNextCutPose(self, nextCutPose):
        self.nextCutFrame = vis.updateFrame(nextCutPose, 'next cut', visible=True, scale=0.2, parent=self.wall.affordance)
        d_dot2 = DebugData()
        d_dot2.addSphere( self.nextCutFrame.transform.GetPosition() , radius=0.015)
        aff2 = vis.updatePolyData(d_dot2.getPolyData(), 'next cut dot', color=[1,0.5,0], visible=True, parent=self.wall.affordance)


    def computeFirstCutDesired(self, engagedTip=True):

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
        nextCutPoseGoal = self.updateNextCutPoseGoal()

        if (engagedTip == False):
            nextCutPose = transformUtils.frameFromPositionAndRPY( [ self.retractDepthNominal , 0, 0] , [0,0,0] )
            nextCutPose.Concatenate( nextCutPoseGoal )
        else:
            nextCutPose = transformUtils.copyFrame( nextCutPoseGoal )

        self.updateNextCutPose(nextCutPose)


    def computeRetractionDesired(self, retractBit=True):
        self.worldToBit = self.getWorldToBit( self.getPlanningStartPose() )

        # relative position of g
        goalToBit = transformUtils.copyFrame( self.worldToBit )
        goalToBit.Concatenate( self.nextCutGoalFrame.transform.GetLinearInverse() )

        if (retractBit == True):
            retractDepth = self.retractDepthNominal
        else: # this can be used to insert where we are located:
            retractDepth = 0.0

        self.goalToNextCut = transformUtils.frameFromPositionAndRPY( [retractDepth, goalToBit.GetPosition()[1], goalToBit.GetPosition()[2] ] , [0,0,0] )
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

        self.worldToBit = self.getWorldToBit( self.getPlanningStartPose() )

        # relative position of bit
        goalToBit = transformUtils.copyFrame( self.worldToBit )
        goalToBit.Concatenate( self.nextCutGoalFrame.transform.GetLinearInverse() )

        distToGoal2D = np.linalg.norm( goalToBit.GetPosition()[1:3] )
        print "distToGoal2D", distToGoal2D , "from" , self.currentCutGoal

        if (distToGoal2D < self.goalThreshold ): # typically 5cm
            if ( self.currentCutGoal == len(self.cutPoints) -1):
                print distToGoal2D , " - within threshold of last goal", self.currentCutGoal
                return False

            self.currentCutGoal=self.currentCutGoal+1
            nextCutPoseGoal = self.updateNextCutPoseGoal()

            # switch to follow to the new goal:
            goalToBit = transformUtils.copyFrame( self.worldToBit )
            goalToBit.Concatenate( self.nextCutGoalFrame.transform.GetLinearInverse() )
            #print distToGoal2D , " - within threshold. Moving to", self.currentCutGoal

        # TODO: change this to a metric movement towards a goal:
        distToGoal2D = np.linalg.norm( goalToBit.GetPosition()[1:3])

        if (distToGoal2D < self.cutLength): # finish the cut to the goal
            self.goalToNextCut = transformUtils.frameFromPositionAndRPY( [0, goalToBit.GetPosition()[1], goalToBit.GetPosition()[2] ] , [0,0,0] )
        else:
            # this is kind of odd - but it works. the distance to the goal that the cut to finish at
            cutsLeft = distToGoal2D/self.cutLength
            goalToBit.GetPosition()[1]
            scale = (cutsLeft-1)/cutsLeft
            self.goalToNextCut = transformUtils.frameFromPositionAndRPY( [0,  scale*goalToBit.GetPosition()[1] ,  scale*goalToBit.GetPosition()[2]  ] , [0,0,0] )

        self.worldToNextCut = transformUtils.copyFrame( self.goalToNextCut )
        self.worldToNextCut.Concatenate( self.nextCutGoalFrame.transform )

        self.updateNextCutPose( self.worldToNextCut )
        return True


    def planDrillingGaze(self):

        startPose = self.getPlanningStartPose()
        handToBit = self.getHandToBit(startPose)

        if (1==0):
            vis.updateFrame( self.ikPlanner.getLinkFrameAtPose( self.graspingHandLink , startPose)  , 'hand testing', visible=True, scale=0.2)
            worldToBit = self.getWorldToBit(startPose)
            vis.updateFrame(worldToBit, 'world to drill bit', visible=True, scale=0.2)

            print "handToBit"
            print handToBit.GetPosition()
            print handToBit.GetOrientation()

        # button drill: [1,0,0] , barrel drill: [0,1,0] - axis is along bit xaxis (this was explictly defined
        all_axes = transformUtils.getAxesFromTransform( handToBit )

        self.initGazeConstraintSet(self.nextCutFrame, self.graspingHand, handToBit, gazeAxis=all_axes[0])
        self.appendPositionConstraintForTargetFrame(self.nextCutFrame, 1, self.graspingHand, handToBit)
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

        ikplanner.getIkOptions().setProperty('Use pointwise', False)
        ikplanner.getIkOptions().setProperty('Quasistatic shrink factor', 0.1)

        self.planFromCurrentRobotState = False
        self.plans = []

        # self.findDrillAffordance()
        if (om.findObjectByName('drill') is None):
            self.spawnDrillAffordance()

        if (om.findObjectByName('wall') is None):
            self.spawnWallAffordance()


        if self.useFootstepPlanner:
            self.planFootsteps( self.drill.stanceFrame.transform )
            self.planWalking()
        else:
            self.moveRobotToStanceFrame(self.drill.stanceFrame.transform)

        self.planPreGrasp()
        self.planReach()
        self.planGrasp()

        # self.affordanceUpdater.graspAffordance('drill', self.graspingHand)

        self.planPreGrasp()
        self.planDrillLowerSafe()

        if (playbackNominal is True):
            self.playNominalPlan()


    def planNominalTurnOn(self, playbackNominal=True):

        self.planFromCurrentRobotState = False
        if (self.flushNominalPlanSequence):
            self.plans = []
        if (om.findObjectByName('drill') is None):
            self.spawnDrillAffordance()
            #self.affordanceUpdater.graspAffordance('drill', self.graspingHand)
        if (om.findObjectByName('wall') is None):
            self.spawnWallAffordance()

        if self.useFootstepPlanner:
            self.planFootsteps( self.wall.stanceFarFrame.transform )
            self.planWalking()
        else:
            self.moveRobotToStanceFrame( self.wall.stanceFarFrame.transform )

        self.planDrillRaisePowerOn()
        self.planPointerRaisePowerOn()

        self.planPointerPressGaze()
        self.planPointerRaisePowerOn()

        self.planPointerLowerPowerOn()

        self.planDrillRaiseForCutting()

        if (playbackNominal is True):
            self.playNominalPlan()


    def planNominalCut(self, playbackNominal=True):

        self.planFromCurrentRobotState = False
        if (self.flushNominalPlanSequence):
            self.plans = []
        if (om.findObjectByName('drill') is None):
            self.spawnDrillAffordance()
            #self.affordanceUpdater.graspAffordance('drill', self.graspingHand)
        if (om.findObjectByName('wall') is None):
            self.spawnWallAffordance()

        # Walking:
        if self.useFootstepPlanner:
            self.planFootsteps( self.wall.stanceFrame.transform )
            self.planWalking()
        else:
            self.moveRobotToStanceFrame( self.wall.stanceFrame.transform )

        ikplanner.getIkOptions().setProperty('Use pointwise', True)
        ikplanner.getIkOptions().setProperty('Quasistatic shrink factor', 0.5)

        self.planNextCut( self.computeFirstCutDesired, False )
        self.planNextCut( self.computeFirstCutDesired, True )

        success = True
        while (success is True):
            success = self.planNextCut( self.computeNextCutDesired )
            print " "

        self.planNextCut( self.computeRetractionDesired, True )

        ikplanner.getIkOptions().setProperty('Use pointwise', False)
        ikplanner.getIkOptions().setProperty('Quasistatic shrink factor', 0.1)

        self.planDrillRaiseForCutting()

        if self.useFootstepPlanner:
            self.planFootsteps( self.wall.stanceFarFrame.transform )
            self.planWalking()
        else:
            self.moveRobotToStanceFrame( self.wall.stanceFarFrame.transform )

        self.planDrillLowerSafe()

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