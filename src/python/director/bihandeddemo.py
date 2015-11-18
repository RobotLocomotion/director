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
from ddapp import affordanceupdater
from ddapp import planplayback

from ddapp import segmentationpanel

import drc as lcmdrc
import copy

from PythonQt import QtCore, QtGui


class Board(object):
    def __init__(self, b): #b is the BihandedPlannerDemo instance. This is to get access to its functions in the Board class
        self.b = b
        self.affordance = None
        self.frame = None

        self.graspFrame = None
        self.graspLeftFrame = None
        self.graspRightFrame = None
        self.reachFrame = None
        self.reachLeftFrame = None
        self.reachRightFrame = None

        self.frameSync = None

        # initial board position
        self.initXYZ = [1.5, 0.6, 0.9] # height requires crouching
        self.initRPY = [1,1,1]
        
    def spawnBoardAffordance(self, randomize=False):
        self.boardLength = 1.5

        if randomize:

            position = [random.uniform(0.5, 0.8), random.uniform(-0.2, 0.2), random.uniform(0.5, 0.8)]
            rpy = [random.choice((random.uniform(-35, 35), random.uniform(70, 110))), random.uniform(-10, 10),  random.uniform(-5, 5)]
            zwidth = random.uniform(0.5, 1.0)

        else:
            if self.b.val:
                position = [0.4, 0.0, 1.]
            else:
                position = [0.6, 0.0, 1.]
                
            rpy = [90, 1, 0]
            zwidth = self.boardLength

        xwidth = 3.75 * .0254
        ywidth = 1.75 * .0254
        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.b.computeGroundFrame(self.b.robotModel))
        xaxis = [1,0,0]
        yaxis = [0,1,0]
        zaxis = [0,0,1]
        for axis in (xaxis, yaxis, zaxis):
            t.TransformVector(axis, axis)

        self.affordance = segmentation.createBlockAffordance(t.GetPosition(), xaxis, yaxis, zaxis, xwidth, ywidth, zwidth, 'board', parent='affordances')
        self.affordance.setProperty('Color', QtGui.QColor(200, 150, 100))
        t = self.affordance.actor.GetUserTransform()
        self.frame = vis.showFrame(t, 'board frame', parent=self.affordance, visible=False, scale=0.2)

    def findBoardAffordance(self):
        self.affordance = om.findObjectByName('board')
        self.frame = om.findObjectByName('board frame')

        self.boardLength = self.affordance.params.get('zwidth')

        # so as not to grasp at the tips:
        graspLength = self.boardLength/2 - 0.05

        if self.b.val:
            self.graspLeftHandFrameXYZ = [-0.045, 0.0, -graspLength]
            self.graspLeftHandFrameRPY = [0, -90, -90]

            self.graspRightHandFrameXYZ = [-0.045, 0.0, graspLength]
            self.graspRightHandFrameRPY = [0, -90, -90]
        else:
            self.graspLeftHandFrameXYZ = [-0.045, 0.0, -graspLength]
            self.graspLeftHandFrameRPY = [0, 90, -90]

            self.graspRightHandFrameXYZ = [-0.045, 0.0, graspLength]
            self.graspRightHandFrameRPY = [0, 90, -90]

        self.relativeStanceXYZ = [-0.6, -graspLength, 0.0]
        self.relativeAsymmetricStanceXYZ = [-0.6, -2*graspLength, 0.0]
        self.relativeStanceRPY = [0, 0, 0]

        self.reachDepth = 0.12 # depth to reach to before going for grasp

        self.computeBoardGraspFrames()
        self.computeBoardReachFrames()
        
        self.computeBoardStanceFrame()

        self.frameSync = vis.FrameSync()
        self.frameSync.addFrame(self.frame)
        self.frameSync.addFrame(self.graspLeftFrame)
        self.frameSync.addFrame(self.graspRightFrame)
        self.frameSync.addFrame(self.reachLeftFrame)
        self.frameSync.addFrame(self.reachRightFrame)
        self.frameSync.addFrame(self.stanceFrame)


    def computeBoardGraspFrames(self):
        t = transformUtils.frameFromPositionAndRPY( self.graspLeftHandFrameXYZ , self.graspLeftHandFrameRPY )
        t_copy = transformUtils.copyFrame(t)
        t_copy.Concatenate(self.frame.transform)
        self.graspLeftFrame = vis.updateFrame(t_copy, 'grasp left frame', parent=self.affordance, visible=False, scale=0.2)
        self.graspLeftFrame.addToView(app.getDRCView())
        
        t = transformUtils.frameFromPositionAndRPY( self.graspRightHandFrameXYZ , self.graspRightHandFrameRPY )
        t_copy = transformUtils.copyFrame(t)
        t_copy.Concatenate(self.frame.transform)
        self.graspRightFrame = vis.updateFrame(t_copy, 'grasp right frame', parent=self.affordance, visible=False, scale=0.2)
        self.graspRightFrame.addToView(app.getDRCView())


    def computeBoardReachFrames(self):
        ''' Reach ~10cm short of the grasp frame '''
        reachLeftXYZ = copy.deepcopy( self.graspLeftHandFrameXYZ )
        reachLeftXYZ[0] = reachLeftXYZ[0] - self.reachDepth
        reachLeftRPY = copy.deepcopy ( self.graspLeftHandFrameRPY )

        tl = transformUtils.frameFromPositionAndRPY( reachLeftXYZ , reachLeftRPY )
        tl.Concatenate(self.frame.transform)
        self.reachLeftFrame = vis.updateFrame(tl, 'reach left frame', parent=self.affordance, visible=False, scale=0.2)
        self.reachLeftFrame.addToView(app.getDRCView())
        
        reachRightXYZ = copy.deepcopy( self.graspRightHandFrameXYZ )
        reachRightXYZ[0] = reachRightXYZ[0] - self.reachDepth
        reachRightRPY = copy.deepcopy ( self.graspRightHandFrameRPY )

        tr = transformUtils.frameFromPositionAndRPY( reachRightXYZ , reachRightRPY )
        tr.Concatenate(self.frame.transform)
        self.reachRightFrame = vis.updateFrame(tr, 'reach right frame', parent=self.affordance, visible=False, scale=0.2)
        self.reachRightFrame.addToView(app.getDRCView())
        

    def computeBoardStanceFrame(self):
        objectTransform = transformUtils.copyFrame( self.graspLeftFrame.transform )
        self.relativeStanceTransform = transformUtils.copyFrame( transformUtils.frameFromPositionAndRPY( self.relativeStanceXYZ , self.relativeStanceRPY ) )
        robotStance = self.b.computeRobotStanceFrame( objectTransform, self.relativeStanceTransform )
        self.stanceFrame = vis.updateFrame(robotStance, 'board stance', parent=self.affordance, visible=False, scale=0.2)
        self.stanceFrame.addToView(app.getDRCView())

        objectTransform = transformUtils.copyFrame( self.graspLeftFrame.transform )
        self.relativeAsymmetricStanceTransform = transformUtils.copyFrame( transformUtils.frameFromPositionAndRPY( self.relativeAsymmetricStanceXYZ , self.relativeStanceRPY ) )
        robotAsymmetricStance = self.b.computeRobotStanceFrame( objectTransform, self.relativeAsymmetricStanceTransform )
        self.asymmetricStanceFrame = vis.updateFrame(robotAsymmetricStance, 'board Asymmetric stance', parent=self.affordance, visible=False, scale=0.2)
        self.asymmetricStanceFrame.addToView(app.getDRCView())



class BihandedPlannerDemo(object):

    def __init__(self, robotModel, playbackRobotModel, teleopRobotModel, footstepPlanner, manipPlanner, ikPlanner,
                 lhandDriver, rhandDriver, atlasDriver, multisenseDriver, affordanceFitFunction, sensorJointController,
                 planPlaybackFunction, showPoseFunction, cameraView, segmentationpanel):
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
        self.affordanceFitFunction = affordanceFitFunction
        self.sensorJointController = sensorJointController
        self.planPlaybackFunction = planPlaybackFunction
        self.showPoseFunction = showPoseFunction
        self.cameraView = cameraView
        self.segmentationpanel = segmentationpanel
        self.pointerTracker = None
        self.projectCallback = None
        self.segmentationpanel.init() # TODO: check with Pat. I added dependency on segmentationpanel, but am sure its appropriate

        board = om.findObjectByName('board')
        while board is not None:
            om.removeFromObjectModel(board)
            board = om.findObjectByName('board')
        
        if robotModel.model.filename().split('/')[-1] == 'V1_sim_mit.urdf':
            self.val = True
            self.v4 = False
        elif robotModel.model.filename().split('/')[-1] == 'model_LR_RR.urdf':
            self.v4 = True
            self.val = False

        # development/live flags
        self.useFootstepPlanner = True
        self.visOnly = False # True for development, False for operation
        self.planFromCurrentRobotState = True # False for development, True for operation
        self.usePointerPerceptionOffset = True # True for development, False for operation
        useDevelopment = True
        if (useDevelopment):
            self.visOnly = True # True for development, False for operation
            self.planFromCurrentRobotState = False # False for development, True for operation
            self.usePointerPerceptionOffset = False

        self.flushNominalPlanSequence = False

        self.userPromptEnabled = True
        self.constraintSet = None

        self.plans = []   

        self.board = Board(self)

        # params:

        #extraModels = [self.robotModel, self.playbackRobotModel, self.teleopRobotModel]
        #self.affordanceUpdater  = affordanceupdater.AffordanceGraspUpdater(self.playbackRobotModel, extraModels)

        extraModels = [self.robotModel]
        self.affordanceUpdater  = affordanceupdater.AffordanceGraspUpdater(self.robotModel, self.ikPlanner, extraModels)

        # top level switch between BDI (locked base) and MIT (moving base and back)
        self.lockBack = False
        self.lockBase = False

    def addPlan(self, plan):
        self.plans.append(plan)

    def computeGroundFrame(self, robotModel):
        '''
        Given a robol model, returns a vtkTransform at a position between
        the feet, on the ground, with z-axis up and x-axis aligned with the
        robot pelvis x-axis.
        '''
        t1 = robotModel.getLinkFrame( self.ikPlanner.leftFootLink )
        t2 = robotModel.getLinkFrame( self.ikPlanner.rightFootLink )
        pelvisT = robotModel.getLinkFrame( self.ikPlanner.pelvisLink )

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

        robotStance = transformUtils.copyFrame( relativeStanceTransform )
        robotStance.Concatenate(graspGroundTransform)

        return robotStance

    ### Planning Functions ###############################################################

    def planStand(self):
        # stand at a nominal ~85cm height with hands in current configuration
        startPose = self.getPlanningStartPose()
        self.standPlan = self.ikPlanner.computeStandPlan(startPose)
        self.addPlan(self.standPlan)

    # These are operational conveniences:
    def planFootstepsBoard(self):
        self.planFootsteps(self.board.stanceFrame.transform)

    def planAsymmetricFootstepsBoard(self):
        self.planFootsteps(self.board.asymmetricStanceFrame.transform)


    def planFootsteps(self, goalFrame):
        startPose = self.getPlanningStartPose()

        # 'typical'
        request = self.footstepPlanner.constructFootstepPlanRequest(startPose, goalFrame)
        # overcome footstep planner bug
        #print "using footstep planner hack - drill demo"
        #nominalPoseAtRobot = np.hstack([self.sensorJointController.q[:6], self.sensorJointController.getPose('q_nom')[6:]] )
        #request = self.footstepPlanner.constructFootstepPlanRequest(nominalPoseAtRobot, goalFrame )

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
        self.endPosePlan = self.manipPlanner.sendEndPoseGoal(startPose, linkName, self.board.graspFrame.transform, waitForResponse=True)
        self.showEndPose()

    def planBihandedReach(self):
        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planEndEffectorGoalLine(startPose, self.board.reachRightFrame, self.board.reachLeftFrame,
                                                               lockBase=self.lockBase, lockBack=self.lockBack, lockArm = False, angleToleranceInDegrees = 10)
        endPose, info = constraintSet.runIk()
        graspPlan = constraintSet.runIkTraj()
        self.addPlan(graspPlan)

    def planAsymmetricReach(self):
        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planAsymmetricGoal(startPose, self.board.reachRightFrame, self.board.reachLeftFrame,
                                                         lockBase=self.lockBase, lockBack=self.lockBack, lockArm = False)
        endPose, info = constraintSet.runIk()
        reachPlan = constraintSet.runIkTraj()
        self.addPlan(reachPlan)
        
    def planBihandedGrasp(self):
        startPose = self.getPlanningStartPose()
        self.constraintSet = self.ikPlanner.planEndEffectorGoalLine(startPose, self.board.graspRightFrame, self.board.graspLeftFrame,
                                                               lockBase=self.lockBase, lockBack=self.lockBack, lockArm = False)
        endPose, info = self.constraintSet.runIk()
        graspPlan = self.constraintSet.runIkTraj()
        self.addPlan(graspPlan)     
        
    def planAsymmetricGrasp(self):
        startPose = self.getPlanningStartPose()
        self.constraintSet = self.ikPlanner.planAsymmetricGoal(startPose, self.board.graspRightFrame, self.board.graspLeftFrame,
                                                               lockBase=self.lockBase, lockBack=self.lockBack, lockArm = False)
        endPose, info = self.constraintSet.runIk()
        graspPlan = self.constraintSet.runIkTraj()
        self.addPlan(graspPlan)   

    def planGraspLift(self):
        t3 = transformUtils.frameFromPositionAndRPY( [0.00,0.0, 0.04] , [0,0,0] )
        liftTransform = transformUtils.copyFrame(self.board.graspFrame.transform)
        liftTransform.Concatenate(t3)

        self.board.liftFrame = vis.updateFrame(liftTransform, 'lift frame', parent="affordances", visible=False, scale=0.2)
        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.board.liftFrame, lockBase=self.lockBase, lockBack=self.lockBack)
        endPose, info = constraintSet.runIk()
        liftPlan = constraintSet.runIkTraj()
        self.addPlan(liftPlan)

    def planPlace(self):
        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.placeFrame, lockBase=self.lockBase, lockBack=self.lockBack)
        endPose, info = constraintSet.runIk()
        placePlan = constraintSet.runIkTraj()
        self.addPlan(placePlan)

    def planDereach(self):
        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.dereachFrame, lockBase=self.lockBase, lockBack=self.lockBack)
        endPose, info = constraintSet.runIk()
        dereachPlan = constraintSet.runIkTraj()
        self.addPlan(dereachPlan)

    def planBihandedPreGrasp(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'bihanded', 'board pregrasp both')
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)


    def initGazeConstraintSet(self, goalFrame, gazeHand, gazeToHandLinkFrame, targetAxis=[-1.0, 0.0, 0.0], bodyAxis=[-1.0, 0.0, 0.0], lockBase=False, lockBack=False):

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
        gazeConstraint = self.ikPlanner.createGazeGraspConstraint(gazeHand, goalFrame, gazeToHandLinkFrame, coneThresholdDegrees , targetAxis, bodyAxis)
        self.constraintSet.constraints.insert(0, gazeConstraint)


    def appendPositionConstraintForTargetFrame(self, goalFrame, t, gazeHand, gazeToHandLinkFrame):
        positionConstraint, _ = self.ikPlanner.createPositionOrientationGraspConstraints(gazeHand, goalFrame, gazeToHandLinkFrame)
        positionConstraint.tspan = [t, t]
        self.constraintSet.constraints.append(positionConstraint)


    def planGazeTrajectory(self):
        self.ikPlanner.ikServer.usePointwise = False
        plan = self.constraintSet.runIkTraj()
        self.addPlan(plan)


    ########## Glue Functions ####################################
    def moveRobotToStanceFrame(self, frame):
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

    def getHandDriver(self, side):
        assert side in ('left', 'right')
        return self.lhandDriver if side == 'left' else self.rhandDriver

    def openHand(self,side):
        #self.handDriver(side).sendOpen()
        self.getHandDriver(side).sendCustom(0.0, 100.0, 100.0, 0)

    def closeHand(self, side):
        #self.handDriver(side).sendClose(60)
        self.getHandDriver(side).sendCustom(100.0, 100.0, 100.0, 0)

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

    def commitManipPlan(self):
        self.manipPlanner.commitManipPlan(self.plans[-1])

    def commitFootstepPlan(self):
        self.footstepPlanner.commitFootstepPlan(self.footstepPlan)

    def waitForPlanExecution(self, plan):
        planElapsedTime = planplayback.PlanPlayback.getPlanElapsedTime(plan)
        print 'waiting for plan execution:', planElapsedTime

        return self.delay(planElapsedTime + 1.0)

    def animateLastPlan(self):
        plan = self.plans[-1]

        if not self.visOnly:
            self.commitManipPlan()

        return self.waitForPlanExecution(plan)

    def turnPointwiseOff(self):
        ikplanner.getIkOptions().setProperty('Use pointwise', False)
        ikplanner.getIkOptions().setProperty('Quasistatic shrink factor', 0.1)
        ikplanner.getIkOptions().setProperty('Max joint degrees/s',30)

    def turnPointwiseOffSlow(self):
        ikplanner.getIkOptions().setProperty('Use pointwise', False)
        ikplanner.getIkOptions().setProperty('Quasistatic shrink factor', 0.1)
        ikplanner.getIkOptions().setProperty('Max joint degrees/s',15)

    ######### Nominal Plans and Execution  #################################################################

    def planBihandedGraspingSequence(self, playbackNominal = True):
        
        self.turnPointwiseOff()
        
        self.plans = []
        
        self.board.findBoardAffordance()
        
        if self.useFootstepPlanner:
            self.planFootstepsBoard()
            self.planWalking()
        else:
            self.moveRobotToStanceFrame(self.board.stanceFrame.transform)
        
        if not self.val:
            self.planBihandedPreGrasp()
        self.planBihandedReach()
        self.planBihandedGrasp()

        if (playbackNominal is True):
            self.playNominalPlan()

    def planBihandedAsymmetricGraspingSequence(self, playbackNominal = True):
        
        self.turnPointwiseOff()

        self.plans = []

        self.board.findBoardAffordance()

        if self.useFootstepPlanner:
            self.planAsymmetricFootstepsBoard()
            self.planWalking()
        else:
            self.moveRobotToStanceFrame(self.board.asymmetricStanceFrame.transform)

        if not self.val:
            self.planBihandedPreGrasp()
        self.planAsymmetricReach()
        self.planAsymmetricGrasp()

        if (playbackNominal is True):
            self.playNominalPlan()
        


    def planNominal(self, playbackNominal=True):
        self.planBihandedGraspingSequence()
