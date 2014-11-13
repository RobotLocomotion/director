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
from ddapp import affordanceitems
from ddapp import robotstate
from ddapp import robotplanlistener
from ddapp import segmentation
from ddapp import affordancegraspupdater
from ddapp import planplayback

from ddapp import segmentationpanel

import drc as lcmdrc
import copy

from PythonQt import QtCore, QtGui


class Board(object):
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

        # initial drill position (requires walking)
        self.initXYZ = [1.5, 0.6, 0.9] # height requires crouching
        self.initRPY = [1,1,1]

        self.graspLeftHandFrameXYZ = [-0.045, 0.0, -0.35] #-0.0275
        self.graspLeftHandFrameRPY = [0, -90, -90]

        # where to stand relative to the drill on a table:
        self.relativeStanceXYZ = [-0.69, -0.4, 0.0] # was -0.67, due to drift set to -0.69
        self.relativeStanceRPY = [0, 0, 0]



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
        self.drillYawSliderValue = 0.0
        self.segmentationpanel.init() # TODO: check with Pat. I added dependency on segmentationpanel, but am sure its appropriate

        defaultGraspingHand = "left"
        self.setGraspingHand(defaultGraspingHand)

        # development/live flags
        self.useFootstepPlanner = True
        self.visOnly = False # True for development, False for operation
        self.planFromCurrentRobotState = True # False for development, True for operation
        self.usePointerPerceptionOffset = True # True for development, False for operation
        useDevelopment = False
        if (useDevelopment):
            self.visOnly = True # True for development, False for operation
            self.planFromCurrentRobotState = False # False for development, True for operation
            self.usePointerPerceptionOffset = False

        self.flushNominalPlanSequence = False

        self.userPromptEnabled = True
        self.constraintSet = None

        self.plans = []

        self.board = Board()

        # params:
        self.reachDepth = 0.12 # depth to reach to before going for grasp

        #extraModels = [self.robotModel, self.playbackRobotModel, self.teleopRobotModel]
        #self.affordanceUpdater  = affordancegraspupdater.AffordanceGraspUpdater(self.playbackRobotModel, extraModels)

        extraModels = [self.robotModel]
        self.affordanceUpdater  = affordancegraspupdater.AffordanceGraspUpdater(self.robotModel, extraModels)


        if ( self.graspingHand == 'right' ):
            print "Planning with drill in the right hand"
            self.board.relativeStanceXYZ[1] = -self.board.relativeStanceXYZ[1]
            self.board.graspLeftHandFrameRPY = [0,-90,-90]
            self.board.initXYZ[1] = -self.board.initXYZ[1]
            #self.board.faceToDrillFlip = True


    def onImageViewDoubleClick(self, displayPoint, modifiers, imageView):

        if modifiers != QtCore.Qt.ControlModifier:
            return

        imagePixel = imageView.getImagePixel(displayPoint)
        cameraPos, ray = imageView.getWorldPositionAndRay(imagePixel)


        polyData = segmentation.getDisparityPointCloud(decimation=1, removeOutliers=False)
        pickPoint = segmentation.extractPointsAlongClickRay(cameraPos, ray, polyData)

        buttonT = vtk.vtkTransform()
        buttonT.Translate(pickPoint)

        d = DebugData()
        d.addSphere((0,0,0), radius=0.005)
        obj = vis.updatePolyData(d.getPolyData(), 'sensed drill button', parent='segmentation', color=[1,0,0])
        obj.actor.SetUserTransform(buttonT)


        om.findObjectByName('intersecting ray').setProperty('Visible', False)
        om.findObjectByName('ray points').setProperty('Visible', False)

        if self.pointerTracker:
            self.pointerTracker.updateFit(polyData)
            pointerTip = self.pointerTracker.getPointerTip()
            t = vtk.vtkTransform()
            t.Translate(pointerTip)
            d.addSphere((0,0,0), radius=0.005)
            obj = vis.updatePolyData(d.getPolyData(), 'sensed pointer tip', parent='segmentation', color=[1,0,0])
            obj.actor.SetUserTransform(t)

        self.updateDrillToHand()


    def updateDrillToHand(self):

        om.removeFromObjectModel(om.findObjectByName('drill'))
        self.spawnDrillAffordance()
        self.moveDrillToHand()
        self.moveDrillToSensedButton()
        if self.projectCallback:
            self.projectCallback()



    def setGraspingHand(self, graspingHand="left"):
        self.graspingHand = graspingHand
        self.graspingFaceLink = '%s_hand_face' % self.graspingHand[0]
        self.graspingHandLink = '%s_hand' % self.graspingHand[0]
        if (self.graspingHand == 'left'):
            self.pointerHand='right'
        else:
            self.pointerHand='left'

    def resetDrill(self):
        self.drill = Drill()

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


    def computeLeftHandBoardGraspFrame(self):
        t = transformUtils.frameFromPositionAndRPY( self.board.graspLeftHandFrameXYZ , self.board.graspLeftHandFrameRPY )
        t_copy = transformUtils.copyFrame(t)
        t_copy.Concatenate(self.board.frame.transform)
        self.board.graspFrame = vis.updateFrame(t_copy, 'grasp frame', parent=self.board.affordance, visible=False, scale=0.2)
        self.board.graspFrame.addToView(app.getDRCView())


    def computeDrillReachFrame(self):
        ''' Reach ~10cm short of the grasp frame '''
        reachXYZ = copy.deepcopy( self.board.graspLeftHandFrameXYZ )
        reachXYZ[0] = reachXYZ[0] - self.reachDepth
        reachRPY = copy.deepcopy ( self.board.graspLeftHandFrameRPY )

        t = transformUtils.frameFromPositionAndRPY( reachXYZ , reachRPY )
        t.Concatenate(self.board.frame.transform)
        self.board.reachFrame = vis.updateFrame(t, 'reach frame', parent=self.board.affordance, visible=False, scale=0.2)
        self.board.reachFrame.addToView(app.getDRCView())

    def computeDrillStanceFrame(self):
        objectTransform = transformUtils.copyFrame( self.board.graspFrame.transform )
        self.board.relativeStanceTransform = transformUtils.copyFrame( transformUtils.frameFromPositionAndRPY( self.board.relativeStanceXYZ , self.board.relativeStanceRPY ) )
        robotStance = self.computeRobotStanceFrame( objectTransform, self.board.relativeStanceTransform )
        self.board.stanceFrame = vis.updateFrame(robotStance, 'drill stance', parent=self.board.affordance, visible=False, scale=0.2)
        self.board.stanceFrame.addToView(app.getDRCView())


    def computeDrillButtonFrame(self):
        position = [ self.board.affordance.params['button_x'], self.board.affordance.params['button_y'], self.board.affordance.params['button_z'] ]
        rpy = np.array([ self.board.affordance.params['button_roll'], self.board.affordance.params['button_pitch'], self.board.affordance.params['button_yaw'] ])
        t = transformUtils.frameFromPositionAndRPY(position, rpy)

        self.board.drillToButtonTransform = transformUtils.copyFrame( t )
        t.Concatenate( transformUtils.copyFrame(self.board.frame.transform))
        self.board.buttonFrame = vis.updateFrame(t, 'drill button', parent=self.board.affordance, visible=False, scale=0.2)
        self.board.buttonFrame.addToView(app.getDRCView())


    def computeDrillBitFrame(self):
        position = [ self.board.affordance.params['bit_x'], self.board.affordance.params['bit_y'], self.board.affordance.params['bit_z'] ]
        rpy = np.array([ self.board.affordance.params['bit_roll'], self.board.affordance.params['bit_pitch'], self.board.affordance.params['bit_yaw'] ])
        t = transformUtils.frameFromPositionAndRPY(position, rpy)

        self.board.drillToBitTransform = transformUtils.copyFrame(t)
        t.Concatenate(transformUtils.copyFrame( self.board.frame.transform))
        self.board.bitFrame = vis.updateFrame(t, 'drill bit', parent=self.board.affordance, visible=False, scale=0.05)
        self.board.bitFrame.addToView(app.getDRCView())


    def computeFaceToDrillTransform(self):
        # if the drill is held in the hand
        # what will be its position, relative to the l_hand_face (or other)
        # Updated to read the settings from the gui panel.
        # TODO: should I avoid calling to faceToDrillTransform and only use computeFaceToDrillTransform to avoid inconsistance?
        rotation, offset, depthOffset, lateralOffset, flip = self.segmentationpanel._segmentationPanel.getDrillInHandParams()
        rotation += self.drillYawSliderValue
        self.board.faceToDrillTransform = segmentation.getDrillInHandOffset(rotation, offset, depthOffset, lateralOffset, flip)


    def spawnDrillAffordance(self):

        drillTransform = transformUtils.frameFromPositionAndRPY(self.board.initXYZ, self.board.initRPY)
        #drillTransform = self.computeConvenientDrillFrame(self.robotModel)

        folder = om.getOrCreateContainer('affordances')

        drillMesh = segmentation.getDrillMesh()
        params = segmentation.getDrillAffordanceParams(np.array(drillTransform.GetPosition()), [1,0,0], [0,1,0], [0,0,1])

        self.board.affordance = vis.showPolyData(drillMesh, 'drill', color=[0.0, 1.0, 0.0], cls=affordanceitems.FrameAffordanceItem, parent=folder)
        self.board.affordance.actor.SetUserTransform(drillTransform)
        self.board.frame = vis.showFrame(drillTransform, 'drill frame', parent=self.board.affordance, visible=False, scale=0.2)
        self.board.affordance.setAffordanceParams(params)

        self.findDrillAffordance()


    def spawnBoardAffordance(self, randomize=False):

        if randomize:

            position = [random.uniform(0.5, 0.8), random.uniform(-0.2, 0.2), random.uniform(0.5, 0.8)]
            rpy = [random.choice((random.uniform(-35, 35), random.uniform(70, 110))), random.uniform(-10, 10),  random.uniform(-5, 5)]
            zwidth = random.uniform(0.5, 1.0)

        else:

            position = [0.7, 0.0, 1.2]
            rpy = [90, 1, 0]
            zwidth = 1.00 # length of the board

        xwidth = 3.75 * .0254
        ywidth = 1.75 * .0254
        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.computeGroundFrame(self.robotModel))
        xaxis = [1,0,0]
        yaxis = [0,1,0]
        zaxis = [0,0,1]
        for axis in (xaxis, yaxis, zaxis):
            t.TransformVector(axis, axis)

        affordance = segmentation.createBlockAffordance(t.GetPosition(), xaxis, yaxis, zaxis, xwidth, ywidth, zwidth, 'board', parent='affordances')
        affordance.setProperty('Color', QtGui.QColor(200, 150, 100))
        t = affordance.actor.GetUserTransform()
        affordanceFrame = vis.showFrame(t, 'board frame', parent=affordance, visible=False, scale=0.2)

        self.findBoardAffordance()

    def findBoardAffordance(self):
        '''
        find an affordance from the view graph - with a populated frame and the param
        then populate the required frames
        '''
        self.board.affordance = om.findObjectByName('board')
        self.board.frame = om.findObjectByName('board frame')

        self.computeLeftHandBoardGraspFrame()
        self.computeDrillReachFrame()
        self.computeDrillStanceFrame()




    def findDrillAffordance(self):
        '''
        find an affordance from the view graph - with a populated frame and the param
        then populate the required frames
        '''
        self.board.affordance = om.findObjectByName('drill')
        self.board.frame = om.findObjectByName('drill frame')

        self.computeLeftHandBoardGraspFrame()
        self.computeDrillReachFrame()
        self.computeDrillStanceFrame()
        self.computeDrillButtonFrame()
        self.computeDrillBitFrame()
        self.computeFaceToDrillTransform()

        self.board.frameSync = vis.FrameSync()
        self.board.frameSync.addFrame(self.board.frame)
        self.board.frameSync.addFrame(self.board.graspFrame)
        self.board.frameSync.addFrame(self.board.reachFrame)
        self.board.frameSync.addFrame(self.board.stanceFrame)
        self.board.frameSync.addFrame(self.board.buttonFrame)
        self.board.frameSync.addFrame(self.board.bitFrame)

    def moveDrillToHand(self):
        # This function moves the drill to the hand using the pose used for planning
        # It is similar to segmentation.moveDrillToHand() other wise
        # TODO: deprecate segmentation.moveDrillToHand()

        self.board.affordance = om.findObjectByName('drill')
        self.board.frame = om.findObjectByName('drill frame')

        self.computeFaceToDrillTransform()
        drillTransform = self.board.affordance.actor.GetUserTransform()
        drillTransform.PostMultiply()
        drillTransform.Identity()
        drillTransform.Concatenate( self.board.faceToDrillTransform )
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
        self.computeFaceToDrillTransform()
        handToBit = vtk.vtkTransform()
        handToBit.PostMultiply()
        handToBit.Concatenate( self.board.drillToBitTransform )
        handToBit.Concatenate( self.board.faceToDrillTransform )
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
        self.computeFaceToDrillTransform()
        handToButton = vtk.vtkTransform()
        handToButton.PostMultiply()
        handToButton.Concatenate( self.board.drillToButtonTransform )
        handToButton.Concatenate( self.board.faceToDrillTransform )
        handToButton.Concatenate( handToFaceTransform )
        return handToButton


    def getWorldToButton(self, startPose):
        handToButton = self.getHandToButton(startPose)
        worldToButton = transformUtils.copyFrame( handToButton)
        worldToButton.Concatenate( self.ikPlanner.getLinkFrameAtPose( self.graspingHandLink , startPose)  )
        return worldToButton



    ### End Drill Focused Functions ###############################################################
    ### Planning Functions ###############################################################

    # is this used?
    #def planStand(self):
    #    startPose = self.getPlanningStartPose()
    #    newPlan = self.ikPlanner.computeNominalPlan(startPose)
    #    self.addPlan(newPlan)

    def planStand(self):
        # stand at a nominal ~85cm height with hands in current configuration
        startPose = self.getPlanningStartPose()
        self.standPlan = self.ikPlanner.computeStandPlan(startPose)
        self.addPlan(self.standPlan)

    # These are operational conveniences:
    def planFootstepsBoard(self):
        self.planFootsteps(self.board.stanceFrame.transform)


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


    def planReach(self):
        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.board.reachFrame, lockBase=False, lockBack=True)
        endPose, info = constraintSet.runIk()
        graspPlan = constraintSet.runIkTraj()
        self.addPlan(graspPlan)

    def planGrasp(self):
        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.board.graspFrame, lockBase=False, lockBack=True)
        endPose, info = constraintSet.runIk()
        graspPlan = constraintSet.runIkTraj()
        self.addPlan(graspPlan)

        t3 = transformUtils.frameFromPositionAndRPY( [0.00,0.0, 0.025] , [0,0,0] )
        placeTransform = transformUtils.copyFrame(self.board.graspFrame.transform)
        placeTransform.Concatenate(t3)
        dereachTransform = transformUtils.copyFrame( self.board.reachFrame.transform )
        dereachTransform.Concatenate(t3)
        self.placeFrame = vis.updateFrame(placeTransform, 'place frame', parent="affordances", visible=False, scale=0.2)
        self.dereachFrame = vis.updateFrame(dereachTransform, 'dereach frame', parent="affordances", visible=False, scale=0.2)



    def planGraspLift(self):

        t3 = transformUtils.frameFromPositionAndRPY( [0.00,0.0, 0.04] , [0,0,0] )
        liftTransform = transformUtils.copyFrame(self.board.graspFrame.transform)
        liftTransform.Concatenate(t3)

        self.board.liftFrame = vis.updateFrame(liftTransform, 'lift frame', parent="affordances", visible=False, scale=0.2)
        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.board.liftFrame, lockBase=False, lockBack=True)
        endPose, info = constraintSet.runIk()
        liftPlan = constraintSet.runIkTraj()
        self.addPlan(liftPlan)



    def planPlace(self):
        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.placeFrame, lockBase=False, lockBack=True)
        endPose, info = constraintSet.runIk()
        placePlan = constraintSet.runIkTraj()
        self.addPlan(placePlan)

    def planDereach(self):
        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.dereachFrame, lockBase=False, lockBack=True)
        endPose, info = constraintSet.runIk()
        dereachPlan = constraintSet.runIkTraj()
        self.addPlan(dereachPlan)

    def planBothRaisePowerOn(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'drill', 'both raise power on')
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def planDrillRaisePowerOn(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'drill', 'drill raise power on', side=self.graspingHand)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def planPointerRaisePowerOn(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'drill', 'pointer raise power on', side=self.pointerHand )
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def planPointerLowerPowerOn(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'drill', 'pointer down', side=self.pointerHand )
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)


    def planDrillLowerSafe(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'drill', 'drill down', side=self.graspingHand )
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)


    def planDrillRaiseForCutting(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'drill', 'drill raise for cutting', side=self.graspingHand )

        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def planPreGrasp(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'arm up pregrasp', side=self.graspingHand)
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


    def getPointerToHandFrame(self):
        '''
        Get the Transfrom from the pointer to the hand link
        Specifically the frame at the very end of the pointer tip
        The result is what mfallon has previously referred to as hand-to-pointer
        '''

        # previous method used "right_pointer_tip" internally:
        #pointerToHandLinkFrame = self.ikPlanner.newGraspToHandFrame(gazeHand)

        # NB: uses the end of the pointer:
        startPose = self.getPlanningStartPose()

        handFrameName = '%s_hand' % self.pointerHand[0]
        handTransform= self.ikPlanner.getLinkFrameAtPose( handFrameName, startPose)
        ftFrameName = '%s_hand_force_torque' % self.pointerHand[0]
        ftTransform= self.ikPlanner.getLinkFrameAtPose( ftFrameName, startPose)

        if (self.usePointerPerceptionOffset is False):
            # actually use FK to move the pointer to the mark - with an approximation of the thumb pointer Oct 2014
            ftToPointerTransform = transformUtils.frameFromPositionAndRPY( [0,0.22,-0.06] ,  [0,0,0] )
            pointerSensedTransform = vtk.vtkTransform()
            pointerSensedTransform.PostMultiply()
            pointerSensedTransform.Concatenate( ftToPointerTransform )
            pointerSensedTransform.Concatenate( ftTransform )

        else:
            # move a point near to but facing in the same axis as the FT axis to the mark
            pointerSensedTransformXYZ = transformUtils.copyFrame(om.findObjectByName("sensed pointer tip").actor.GetUserTransform())
            pointerSensedTransform = transformUtils.frameFromPositionAndRPY(pointerSensedTransformXYZ.GetPosition() ,  np.array(transformUtils.rollPitchYawFromTransform(ftTransform))*180/np.pi )

        vis.updateFrame(handTransform, "handTransform", visible=False)
        vis.updateFrame(ftTransform, "ftTransform", visible=False)
        vis.updateFrame(pointerSensedTransform, "pointerSensedTransform", visible=False)
        pointerTransform = transformUtils.copyFrame(pointerSensedTransform)

        pointerToHandLinkFrame = vtk.vtkTransform()
        pointerToHandLinkFrame.PostMultiply()
        pointerToHandLinkFrame.Concatenate( pointerTransform )
        pointerToHandLinkFrame.Concatenate( handTransform.GetLinearInverse() )


        return pointerToHandLinkFrame


    def planPointerPressGaze(self, pointerDepth=0.00):
        # move to a point along the button axis
        # pointerDepth is negative is away from the drill and positive is inside drill

        # change the nominal pose to the start pose ... q_nom was unreliable when used repeated

        nominalPoseOld = ikplanner.getIkOptions().getProperty('Nominal pose')
        jointSpeedOld = ikplanner.getIkOptions().getProperty('Max joint degrees/s')
        ikplanner.getIkOptions().setProperty('Nominal pose', 'q_start')
        ikplanner.getIkOptions().setProperty('Max joint degrees/s', 3)

        # 1. determine the goal position
        #worldToButton = self.getWorldToButton( self.getPlanningStartPose() )
        #worldToPress = transformUtils.copyFrame(worldToButton)
        worldToPress = transformUtils.copyFrame(self.board.buttonFrame.transform)
        worldToPress.PreMultiply()
        t3 = transformUtils.frameFromPositionAndRPY( [0,0.0, pointerDepth] , [0,0,0] )
        worldToPress.Concatenate(t3)

        worldToPressFrame = vis.updateFrame(worldToPress, 'button press goal', visible=False, scale=0.2, parent=om.getOrCreateContainer('affordances'))

        # 2. add gaze constraint along the pointer tip hand
        pointerToHandLinkFrame = self.getPointerToHandFrame()

        # this mirrors the y-axis pointer for left and right:
        bodyAxisTip = self.ikPlanner.getPalmToHandLink(self.pointerHand).TransformVector([0,1,0])

        self.initGazeConstraintSet(worldToPressFrame, self.pointerHand, pointerToHandLinkFrame, targetAxis=[0.0, 0.0, 1.0], bodyAxis=bodyAxisTip, lockBase=True, lockBack=True)
        # self.initGazeConstraintSet(worldToPressFrame, self.pointerHand, pointerToHandLinkFrame, gazeAxis=[-1.0, 0.0, 0.0], lockBase=True, lockBack=True)
        self.appendPositionConstraintForTargetFrame(worldToPressFrame, 1, self.pointerHand, pointerToHandLinkFrame)
        #self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedSlow
        self.planGazeTrajectory()
        #self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedHigh

        ikplanner.getIkOptions().setProperty('Nominal pose', nominalPoseOld)
        ikplanner.getIkOptions().setProperty('Max joint degrees/s', jointSpeedOld)


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





    def computeRetractionDesired(self, retractBit=True):
        self.worldToBit = self.getWorldToBit( self.getPlanningStartPose() )

        # relative position of g
        goalToBit = transformUtils.copyFrame( self.worldToBit )
        goalToBit.Concatenate( self.nextCutGoalFrame.transform.GetLinearInverse() )

        if (retractBit == True):
            retractDepth = self.retractBitDepthNominal
        else: # this can be used to insert where we are located:
            retractDepth = 0.0

        self.goalToNextCut = transformUtils.frameFromPositionAndRPY( [retractDepth, goalToBit.GetPosition()[1], goalToBit.GetPosition()[2] ] , [0,0,0] )
        self.worldToNextCut = transformUtils.copyFrame( self.goalToNextCut )
        self.worldToNextCut.Concatenate( self.nextCutGoalFrame.transform )
        self.nextCutFrame = vis.updateFrame(self.worldToNextCut, 'next cut', visible=True, scale=0.2)


    def moveDrillToSensedButton(self):
        ''' Take the position of the sensed button and the current orientation of the drill aff
            Move the drill the the position inferred by the sensed button
        '''
        buttonSensedTransform = transformUtils.copyFrame(om.findObjectByName("sensed drill button").actor.GetUserTransform())
        drillTransformOriginal = self.board.affordance.actor.GetUserTransform()
        buttonTransformOriginal = transformUtils.copyFrame( drillTransformOriginal )
        buttonTransformOriginal.PreMultiply()
        buttonTransformOriginal.Concatenate(self.board.drillToButtonTransform)

        buttonTransformNew = transformUtils.frameFromPositionAndRPY(buttonSensedTransform.GetPosition() ,  np.array(transformUtils.rollPitchYawFromTransform( buttonTransformOriginal ))*180/np.pi )
        drillTransformNew = transformUtils.copyFrame(buttonTransformNew)
        drillTransformNew.PreMultiply()
        t3 = transformUtils.copyFrame ( self.board.drillToButtonTransform.GetLinearInverse() )
        drillTransformNew.Concatenate(t3)

        self.board.affordance.actor.SetUserTransform(drillTransformNew)
        self.board.frame = vis.updateFrame(drillTransformNew, 'drill frame', parent=self.board.affordance, visible=False, scale=0.2)



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

        self.initGazeConstraintSet(self.nextCutFrame, self.graspingHand, handToBit, targetAxis=all_axes[0], bodyAxis=all_axes[0])
        self.appendPositionConstraintForTargetFrame(self.nextCutFrame, 1, self.graspingHand, handToBit)
        #self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedSlow
        self.planGazeTrajectory()
        #self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedHigh


    ########## Glue Functions ####################################
    def moveRobotToStanceFrame(self, frame):
        #frame = self.board.stanceFrame.transform
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
    def planNominalPickUp(self, playbackNominal=True):

        self.turnPointwiseOff()

        self.planFromCurrentRobotState = False
        self.plans = []

        if (om.findObjectByName('board') is None):
            self.spawnBoardAffordance()



        if self.useFootstepPlanner:
            #self.planFootsteps( self.board.stanceFrame.transform )
            self.planFootstepsBoard()
            self.planWalking()
        else:
            self.moveRobotToStanceFrame(self.board.stanceFrame.transform)

        self.planPreGrasp()
        self.planReach()
        self.planGrasp()

        # self.affordanceUpdater.graspAffordance('drill', self.graspingHand)

        #self.planStand()
        #self.planPreGrasp()
        #self.planDrillLowerSafe()

        if (playbackNominal is True):
            self.playNominalPlan()



    def planNominal(self, playbackNominal=True):
        self.planNominalPickUp(playbackNominal=False)
        self.planNominalTurnOn(playbackNominal=False)
        self.planNominalCut(playbackNominal=False)
        self.playNominalPlan()
