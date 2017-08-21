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
from director import ikconstraints
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
from director import footstepsdriverpanel
from director import vtkNumpy as vnp

from director.tasks.taskuserpanel import TaskUserPanel
from director.tasks.taskuserpanel import ImageBasedAffordanceFit

import director.tasks.robottasks as rt
import director.tasks.taskmanagerwidget as tmw

import drc as lcmdrc
import copy

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

        # initial drill position (requires walking)
        self.initXYZ = [1.5, 0.6, 0.9] # height requires crouching
        self.initRPY = [1,1,1]

        self.graspFrameXYZ = [-0.045, 0.0, 0.0275]
        self.graspFrameRPY = [0, 90, -90]

        # where to stand relative to the drill on a table:
        self.relativeStanceXYZ = [-0.69, -0.4, 0.0] # was -0.67, due to drift set to -0.69
        self.relativeStanceRPY = [0, 0, 0]

class Wall(object):
    def __init__(self):
        self.affordance = None

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


        self.lockBackForDrilling = False
        self.lockBaseForDrilling = True

        self.useFingerGrasp = True

        self.drillTrajectoryMetersPerSecondSlow = 0.04
        self.drillTrajectoryMetersPerSecondFast = 0.20
        self.drillTrajectoryMaxDegreesPerSecond = 15
        self.drillGraspYaw = 20
        self.thumbPressToHandFrame = None

        self.drillCutTailLength = 0.02


        self.segmentationpanel.init() # TODO: check with Pat. I added dependency on segmentationpanel, but am sure its appropriate

        defaultGraspingHand = 'right'
        self.setGraspingHand(defaultGraspingHand)

        # live operation flags
        self.useFootstepPlanner = False
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

        self.drill = Drill()
        self.wall = Wall()

        # params:
        self.reachDepth = 0.12 # depth to reach to before going for grasp
        self.cutLength = 0.05 # length to cut each time
        self.retractBitDepthNominal = -0.08 # depth to move drill away from wall
        self.goalThreshold = 0.05 # how close we need to get to the cut goal (the triangle corners

        #extraModels = [self.robotModel, self.playbackRobotModel, self.teleopRobotModel]
        #self.affordanceUpdater  = affordanceupdater.AffordanceGraspUpdater(self.playbackRobotModel, extraModels)

        extraModels = [self.playbackRobotModel, self.teleopRobotModel]
        self.affordanceUpdater  = affordanceupdater.AffordanceGraspUpdater(self.robotModel, self.ikPlanner, extraModels)

        # These changes are all that are required to run with different combinations
        if ( self.drill.model == 'dewalt_barrel' ):
            print("Using a Dewalt Barrel Drill")
            self.wall.relativeStanceXYZ = [-0.3, 0.75, 0]
            self.wall.relativeStanceRPY = [0, 0, -90]
            self.wall.relativeStanceFarXYZ = [-0.3, 1.25, 0]
            self.wall.relativeStanceFarRPY = [0, 0, -90]
            #if ( self.graspingHand == 'right' ):
            #    self.drill.faceToDrillRotation = -90

        if ( self.graspingHand == 'right' ):
            self.drill.relativeStanceXYZ[1] = -self.drill.relativeStanceXYZ[1]
            self.drill.graspFrameRPY = [0,-90,-90]
            self.drill.initXYZ[1] = -self.drill.initXYZ[1]
            #self.drill.faceToDrillFlip = True

            self.wall.rightAngleLocation = "bottom right"
            self.wall.initXYZ[1] = -self.wall.initXYZ[1]
            self.wall.relativeStanceXYZ[0] = -self.wall.relativeStanceXYZ[0]
            self.wall.relativeStanceRPY[2] = -self.wall.relativeStanceRPY[2] + 180

            self.wall.relativeStanceFarXYZ[0] = -self.wall.relativeStanceFarXYZ[0]
            self.wall.relativeStanceFarRPY[2] = -self.wall.relativeStanceFarRPY[2] + 180

        self._setupSubscriptions()


    def _setupSubscriptions(self):
        sub0 = lcmUtils.addSubscriber('TAG_DETECTION', lcmdrc.tag_detection_t, self.onTagDetection)
        sub0.setSpeedLimit(5)

    def onTagDetection(self, msg):
        # Store a tag detection for later use
        if (msg.id != 2):
            print("Received detection not from Drill Wall, ignoring")
            return

        pickedPoint = msg.cxy
        self.tagImageUtime = msg.utime
        self.tagPosition, self.tagRay = self.cameraView.views['CAMERA_LEFT'].getWorldPositionAndRay(pickedPoint, self.tagImageUtime)

        # don't want to automatically update - in case of a misdetection:
        updateAutomatically = True
        if (updateAutomatically):
            self.refitDrillWallFromTag()


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


    def refitDrillWallFromTag(self):
        '''
        Use the tag detection (previously received via LCM)
        to re-position the drill wall affordance
        '''

        # TODO: add stale image te
        #detectionAge = getUtime() - self.tagImageUtime
        #if ( detectionAge > 1E6):
        #  print detectionAge

        successfulFit = segmentation.segmentDrillWallFromTag( self.tagPosition, self.tagRay )
        if (successfulFit):
            self.findWallAffordance()

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

    def resetWall(self):
        self.wall = Wall()

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

    def computeRobotStanceFrame(self, objectTransform, relativeStanceTransform):
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


    def computeDrillGraspFrame(self):
        t = transformUtils.frameFromPositionAndRPY( self.drill.graspFrameXYZ , self.drill.graspFrameRPY )
        t_copy = transformUtils.copyFrame(t)
        t_copy.Concatenate(self.drill.frame.transform)
        self.drill.graspFrame = vis.updateFrame(t_copy, 'grasp frame', parent=self.drill.affordance, visible=False, scale=0.2)
        self.drill.graspFrame.addToView(app.getDRCView())


    def computeDrillReachFrame(self):
        ''' Reach ~10cm short of the grasp frame '''
        reachXYZ = copy.deepcopy( self.drill.graspFrameXYZ )
        reachXYZ[0] = reachXYZ[0] - self.reachDepth
        reachRPY = copy.deepcopy ( self.drill.graspFrameRPY )

        t = transformUtils.frameFromPositionAndRPY( reachXYZ , reachRPY )
        t.Concatenate(self.drill.frame.transform)
        self.drill.reachFrame = vis.updateFrame(t, 'reach frame', parent=self.drill.affordance, visible=False, scale=0.2)
        self.drill.reachFrame.addToView(app.getDRCView())

    def computeDrillStanceFrame(self):
        objectTransform = transformUtils.copyFrame( self.drill.graspFrame.transform )
        self.drill.relativeStanceTransform = transformUtils.copyFrame( transformUtils.frameFromPositionAndRPY( self.drill.relativeStanceXYZ , self.drill.relativeStanceRPY ) )
        robotStance = self.computeRobotStanceFrame( objectTransform, self.drill.relativeStanceTransform )
        self.drill.stanceFrame = vis.updateFrame(robotStance, 'drill stance', parent=self.drill.affordance, visible=False, scale=0.2)
        self.drill.stanceFrame.addToView(app.getDRCView())


    def computeDrillButtonFrame(self):
        position = [ self.drill.affordance.params['button_x'], self.drill.affordance.params['button_y'], self.drill.affordance.params['button_z'] ]
        rpy = np.array([ self.drill.affordance.params['button_roll'], self.drill.affordance.params['button_pitch'], self.drill.affordance.params['button_yaw'] ])
        t = transformUtils.frameFromPositionAndRPY(position, rpy)

        self.drill.drillToButtonTransform = transformUtils.copyFrame( t )
        t.Concatenate( transformUtils.copyFrame(self.drill.frame.transform))
        self.drill.buttonFrame = vis.updateFrame(t, 'drill button', parent=self.drill.affordance, visible=False, scale=0.2)
        self.drill.buttonFrame.addToView(app.getDRCView())


    def computeDrillBitFrame(self):
        position = [ self.drill.affordance.params['bit_x'], self.drill.affordance.params['bit_y'], self.drill.affordance.params['bit_z'] ]
        rpy = np.array([ self.drill.affordance.params['bit_roll'], self.drill.affordance.params['bit_pitch'], self.drill.affordance.params['bit_yaw'] ])
        t = transformUtils.frameFromPositionAndRPY(position, rpy)

        self.drill.drillToBitTransform = transformUtils.copyFrame(t)
        t.Concatenate(transformUtils.copyFrame( self.drill.frame.transform))
        self.drill.bitFrame = vis.updateFrame(t, 'drill bit', parent=self.drill.affordance, visible=False, scale=0.05)
        self.drill.bitFrame.addToView(app.getDRCView())


    def computeFaceToDrillTransform(self):
        # if the drill is held in the hand
        # what will be its position, relative to the l_hand_face (or other)
        # Updated to read the settings from the gui panel.
        # TODO: should I avoid calling to faceToDrillTransform and only use computeFaceToDrillTransform to avoid inconsistance?
        rotation, offset, depthOffset, lateralOffset, flip = self.segmentationpanel._segmentationPanel.getDrillInHandParams()
        rotation += self.drillYawSliderValue
        self.drill.faceToDrillTransform = segmentation.getDrillInHandOffset(rotation, offset, depthOffset, lateralOffset, flip)


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

        self.drill.affordance = vis.showPolyData(drillMesh, 'drill', color=[0.0, 1.0, 0.0], cls=affordanceitems.FrameAffordanceItem, parent=folder)
        self.drill.affordance.actor.SetUserTransform(drillTransform)
        self.drill.frame = vis.showFrame(drillTransform, 'drill frame', parent=self.drill.affordance, visible=False, scale=0.2)
        self.drill.affordance.setAffordanceParams(params)

        self.findDrillAffordance()


    def findDrillAffordance(self):
        '''
        find an affordance from the view graph - with a populated frame and the param
        then populate the required frames
        '''
        self.drill.affordance = om.findObjectByName('drill')
        self.drill.frame = om.findObjectByName('drill frame')

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

        self.computeFaceToDrillTransform()
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
        self.computeFaceToDrillTransform()
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
        self.computeFaceToDrillTransform()
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


    def spawnWallAffordance(self):
        wallTransform = transformUtils.frameFromPositionAndRPY(self.wall.initXYZ, self.wall.initRPY)
        segmentation.createDrillWall(self.wall.rightAngleLocation, wallTransform)
        self.findWallAffordance()


    def findWallAffordance(self):
        self.wall.affordance = om.findObjectByName('wall')
        self.wall.frame = om.findObjectByName('wall frame')

        objectTransform = self.wall.frame.transform
        #objectTransform = self.wall.transform

        self.wall.relativeStanceTransform = transformUtils.frameFromPositionAndRPY( self.wall.relativeStanceXYZ , self.wall.relativeStanceRPY )
        robotStance = self.computeRobotStanceFrame( objectTransform, self.wall.relativeStanceTransform )
        self.wall.stanceFrame = vis.updateFrame(robotStance, 'wall stance', parent=self.wall.affordance, visible=True, scale=0.2)
        self.wall.stanceFrame.addToView(app.getDRCView())
        self.wall.relativeStanceFarTransform = transformUtils.frameFromPositionAndRPY( self.wall.relativeStanceFarXYZ , self.wall.relativeStanceFarRPY )
        robotStanceFar = self.computeRobotStanceFrame( objectTransform, self.wall.relativeStanceFarTransform )
        self.wall.stanceFarFrame = vis.updateFrame(robotStanceFar, 'wall stance far', parent=self.wall.affordance, visible=True, scale=0.2)
        self.wall.stanceFarFrame.addToView(app.getDRCView())

        self.wall.frameSync= vis.FrameSync()
        self.wall.frameSync.addFrame(self.wall.affordance.getChildFrame())
        self.wall.frameSync.addFrame(self.wall.stanceFrame)
        self.wall.frameSync.addFrame(self.wall.stanceFarFrame)


    ### End Drill Focused Functions ###############################################################
    ### Planning Functions ###############################################################
    def planNominal(self):
        startPose = self.getPlanningStartPose()
        self.standPlan = self.ikPlanner.computeNominalPlan(startPose)
        self.addPlan(self.standPlan)

    # These are operational conveniences:
    def planFootstepsDrill(self):
        self.planFootsteps(self.drill.stanceFrame.transform)
    def planFootstepsTurnOn(self):
        self.planFootsteps(self.wall.stanceFarFrame.transform)
    def planFootstepsCut(self):
        self.planFootsteps(self.wall.stanceFrame.transform)

    def planFootsteps(self, goalFrame):
        startPose = self.getPlanningStartPose()
        request = self.footstepPlanner.constructFootstepPlanRequest(startPose, goalFrame)
        self.footstepPlan = self.footstepPlanner.sendFootstepPlanRequest(request, waitForResponse=True)

    def planWalking(self):
        startPose = self.getPlanningStartPose()
        walkingPlan = self.footstepPlanner.sendWalkingPlanRequest(self.footstepPlan, startPose, waitForResponse=True)
        self.addPlan(walkingPlan)


    def planPostureGoal(self, side, group, name, ikParameters=None):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, group, name, side=side)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose, ikParameters=ikParameters)
        self.addPlan(newPlan)


    def planPreGrasp(self):

        side = self.graspingHand
        q1 = self.getPlanningStartPose()
        q2 = self.ikPlanner.getMergedPostureFromDatabase(q1, 'General', 'arm up pregrasp waypoint', side=side)
        q3 = self.ikPlanner.getMergedPostureFromDatabase(q1, 'General', 'arm up pregrasp', side=side)

        newPlan = self.ikPlanner.computeMultiPostureGoal([q1, q2, q3], [0.0, 0.7, 1.0],
                                                         ikParameters=IkParameters(maxDegreesPerSecond=60))
        self.addPlan(newPlan)

        #self.planPostureGoal(self.graspingHand, 'General', 'arm up pregrasp')


    def planDrillRaiseNew(self):
        self.planPostureGoal(self.graspingHand, 'drill', 'new: drill raise for press 2',
                             ikParameters=IkParameters(maxDegreesPerSecond=30))



    def computeDrillGraspTargetFrame(self, reachDistance, reachHeight, reachYaw):

        graspToDrillTransform = self.getGraspToDrillTransform()
        drillToWorld = transformUtils.copyFrame(om.findObjectByName('drill frame').transform)

        drillPositionOffset = transformUtils.frameFromPositionAndRPY([reachDistance, 0.0, reachHeight], [0.0, 0.0, 0.0])

        if self.graspingHand == 'right':
            drillYawOffset = transformUtils.frameFromPositionAndRPY([0.0, 0.0, 0.0], [0.0, 0.0, reachYaw + 180.0])
            graspThumbFlip = transformUtils.frameFromPositionAndRPY([0.0, 0.0, 0.0], [0.0, 180.0, 0.0])
        else:
            drillYawOffset = transformUtils.frameFromPositionAndRPY([0.0, 0.0, 0.0], [0.0, 0.0, -reachYaw])
            graspThumbFlip = vtk.vtkTransform()

        targetFrame = transformUtils.concatenateTransforms([graspThumbFlip, graspToDrillTransform, drillPositionOffset, drillYawOffset, drillToWorld])

        vis.updateFrame(targetFrame, 'drill grasp offset frame', scale=0.2, parent='drill grasp frame')

        return targetFrame


    def computeDrillGraspPose(self, startPose, reachDistance, reachHeight, reachYaw):

        side = self.graspingHand
        targetFrame = self.computeDrillGraspTargetFrame(reachDistance, reachHeight, reachYaw)
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, side, targetFrame, lockBase=False, lockBack=False)

        endPose, info = constraintSet.runIk()
        return endPose, constraintSet


    def computeDrillGraspStanceFrame(self):

        drillGraspFrame = self.computeDrillGraspTargetFrame(0.0, 0.0, 0.0)

        stanceForward = transformUtils.getAxesFromTransform(drillGraspFrame)[1]
        stanceUp = [0,0,1]
        yaxis = np.cross(stanceUp, stanceForward)
        yaxis /= np.linalg.norm(yaxis)
        stanceForward = np.cross(yaxis, stanceUp)
        stanceForward /= np.linalg.norm(stanceForward)

        stanceFrame = self.footstepPlanner.getFeetMidPoint(self.robotModel)
        drillGroundPosition = [drillGraspFrame.GetPosition()[0], drillGraspFrame.GetPosition()[1], stanceFrame.GetPosition()[2]]

        stanceFrame = transformUtils.getTransformFromAxesAndOrigin(stanceForward, yaxis, stanceUp, stanceFrame.GetPosition())

        stanceOffsetX = -0.80
        stanceOffsetY = 0.4

        if self.graspingHand == 'left':
            stanceOffsetY = -stanceOffsetY

        stanceFrame.PostMultiply()
        stanceFrame.Translate(drillGroundPosition - np.array(stanceFrame.GetPosition()))
        stanceFrame.PreMultiply()
        stanceFrame.Translate(stanceOffsetX, stanceOffsetY, 0.0)

        vis.updateFrame(stanceFrame, 'drill grasp stance frame', parent=om.findObjectByName('drill'), scale=0.2, visible=True)


    def planParameterizedGrasp(self, distance=0.0, height=0.0, yaw=0.0, inLine=False, ikParameters=None):
        startPose = self.getPlanningStartPose()
        endPose, constraintSet = self.computeDrillGraspPose(startPose, distance, height, yaw)
        if ikParameters:
            constraintSet.ikParameters = ikParameters

        constraintSet.ikParameters.usePointwise = False

        if inLine:
            '''
            p = None
            for c in constraintSet.constraints:
                if isinstance(c, ikconstraints.PositionConstraint) and c.linkName == self.ikPlanner.getHandLink(self.graspingHand):
                    print c
                    p = c
            assert p
            '''

            handLinkName = self.ikPlanner.getHandLink(self.graspingHand)
            handToWorld1 = self.ikPlanner.getLinkFrameAtPose(handLinkName, startPose)
            handToWorld2 = self.ikPlanner.getLinkFrameAtPose(handLinkName, endPose)

            palmToHand = self.ikPlanner.getPalmToHandLink(self.graspingHand)
            handToWorld1 = transformUtils.concatenateTransforms([palmToHand, handToWorld1])
            handToWorld2 = transformUtils.concatenateTransforms([palmToHand, handToWorld2])

            motionVector = np.array(handToWorld2.GetPosition()) - np.array(handToWorld1.GetPosition())
            motionTargetFrame = transformUtils.getTransformFromOriginAndNormal(np.array(handToWorld2.GetPosition()), motionVector)


            #vis.updateFrame(motionTargetFrame, 'motion target frame', scale=0.1)
            #d = DebugData()
            #d.addLine(np.array(handToWorld2.GetPosition()), np.array(handToWorld2.GetPosition()) - motionVector)
            #vis.updatePolyData(d.getPolyData(), 'motion vector', visible=False)


            p = self.ikPlanner.createLinePositionConstraint(handLinkName, palmToHand, motionTargetFrame, lineAxis=2, bounds=[-np.linalg.norm(motionVector), 0.0], positionTolerance=0.001)
            p.tspan = np.linspace(0, 1, 5)

            constraintSet.constraints.append(p)

            newPlan = constraintSet.runIkTraj()

        else:
            newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)

        self.addPlan(newPlan)


    def planReachNew(self):
        self.planParameterizedGrasp(-0.17, 0.0, self.drillGraspYaw,
                                    ikParameters=IkParameters(maxDegreesPerSecond=30))

    def planGraspNew(self):
        if self.useFingerGrasp:
            graspHeight = 0.03
        else:
            graspHeight = 0.0

        self.planParameterizedGrasp(0.0, graspHeight, self.drillGraspYaw, inLine=True,
                                    ikParameters=IkParameters(maxDegreesPerSecond=15))

    def planLiftNew(self):
        self.planParameterizedGrasp(-0.1, 0.1, self.drillGraspYaw, inLine=True,
                                    ikParameters=IkParameters(maxDegreesPerSecond=15))

    def computeDrillButtGraspPose(self, startPose, depth):

        drillButtToWorld = transformUtils.copyFrame(om.findObjectByName('drill butt frame').transform)
        offsetToButt = transformUtils.frameFromPositionAndRPY([0.0, depth, 0.0], [0,0,0])
        targetFrame = transformUtils.concatenateTransforms([offsetToButt, drillButtToWorld])

        #vis.updateFrame(targetFrame, 'drill butt offset grasp frame', scale=0.2, parent='drill butt frame')

        side = self.ikPlanner.flipSide(self.graspingHand)

        #seedPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'drill', 'new: hand raise for pregrasp', side=side)
        seedPose, info = self.computeThumbPressPrepPose(startPose)

        seedPoseName = 'drill_grasp_seed_pose'
        self.ikPlanner.addPose(seedPose, seedPoseName)

        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, side, targetFrame, lockBase=True, lockBack=True)

        constraintSet.nominalPoseName = seedPoseName
        constraintSet.seedPoseName = seedPoseName

        endPose, info = constraintSet.runIk()
        return endPose


    def planHandRaiseForDrillButtPreGrasp(self):
        startPose = self.getPlanningStartPose()
        endPose = self.computeDrillButtGraspPose(startPose, -0.15)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose,
                                                    ikParameters=IkParameters(maxDegreesPerSecond=30))
        self.addPlan(newPlan)

    def planHandRaiseForDrillButtGrasp(self):
        startPose = self.getPlanningStartPose()
        endPose = self.computeDrillButtGraspPose(startPose, -0.04)

        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose,
                                                    ikParameters=IkParameters(maxDegreesPerSecond=10))

        self.addPlan(newPlan)

    def computeThumbPressPrepPose(self, startPose):

        side = self.ikPlanner.flipSide(self.graspingHand)

        seedPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'drill', 'new: hand raise for grasp', side=side)
        seedPoseName = 'drill_grasp_seed_pose'
        self.ikPlanner.addPose(seedPose, seedPoseName)

        targetFrame = om.findObjectByName('drill press prep frame').transform
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, side, targetFrame, lockBase=True, lockBack=True)

        constraintSet.nominalPoseName = seedPoseName
        constraintSet.seedPoseName = seedPoseName

        return constraintSet.runIk()


    def planThumbPressPrep(self):

        startPose = self.getPlanningStartPose()
        endPose, info = self.computeThumbPressPrepPose(startPose)

        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose,
                                                    ikParameters=IkParameters(maxDegreesPerSecond=15, quasiStaticShrinkFactor=0.5))

        self.addPlan(newPlan)


    def planHandsDown(self):
        otherSide = 'right' if self.graspingHand == 'left' else 'left'
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'handdown', side=otherSide)
        endPose = self.ikPlanner.getMergedPostureFromDatabase(endPose, 'drill', 'new: walk with drill', side=self.graspingHand)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose, ikParameters=IkParameters(maxDegreesPerSecond=30))
        self.addPlan(newPlan)


    def planWalkWithDrillPosture(self):

        '''
        startPose = self.getPlanningStartPose()
        startPoseName = 'reach_start'
        self.ikPlanner.addPose(startPose, startPoseName)

        constraints = []
        constraints.extend(self.ikPlanner.createMovingReachConstraints(startPoseName, side=self.graspingHand))
        constraints.append(self.ikPlanner.createJointPostureConstraintFromDatabase('drill', 'drill tuck for walk', side=self.graspingHand))
        constraints.append(self.ikPlanner.createBackZeroPostureConstraint())
        constraints[-1].tspan = [1.0, 1.0]

        constraintSet = ikplanner.ConstraintSet(self.ikPlanner, constraints, 'reach_end', startPoseName)

        endPose, info = constraintSet.runIk()
        plan = constraintSet.runIkTraj()

        self.addPlan(newPlan)
        '''

        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'drill', 'new: walk with drill', side=self.graspingHand)
        plan = self.ikPlanner.computePostureGoal(startPose, endPose,
                                                 ikParameters=IkParameters(maxDegreesPerSecond=30))
        self.addPlan(plan)

    def planDrillIntoWallPrep(self):

        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'drill', 'drill wall prep', side=self.graspingHand)

        self.storeDrillNominalPosture(endPose)

        plan = self.ikPlanner.computePostureGoal(startPose, endPose,
                                                 ikParameters=IkParameters(maxDegreesPerSecond=30))

        self.addPlan(plan)


    def getBitToDrillTransform(self):
        return transformUtils.frameFromPositionAndRPY([0.0, 0.0, 0.13], [0,0,0])


    def getButtonToDrillTransform(self):
        return transformUtils.frameFromPositionAndRPY([0.0078, -0.025, -0.07], [0,0,0])


    def getButtToDrillTransform(self):
        return transformUtils.frameFromPositionAndRPY([0.0, 0.0, -0.16], [90,0,0])

    def getGraspToDrillTransform(self):
        return transformUtils.frameFromPositionAndRPY([-0.02, 0.0, 0.0], [90, 90, 0.0])

    def getPressPrepToDrillTransform(self):
        return transformUtils.frameFromPositionAndRPY([0.03, -0.05, -0.2], [45,0,0])

    def getThumbToPalmTransform(self):
        return transformUtils.frameFromPositionAndRPY([0.0, 0.11, 0.085], [0.0, 0.0, 0.0])

    def computeFrameToHand(self, frameToWorld, side=None):
        startPose = self.getPlanningStartPose()
        side = side or self.graspingHand
        return self.ikPlanner.computeFrameToHand(startPose, side, frameToWorld)


    def computeDrillBitToHand(self):
        '''
        Returns the relative bitToHand frame using the current world poses
        of the drill affordance and hand link.
        '''

        bitToDrill = self.getBitToDrillTransform()
        drillToWorld = transformUtils.copyFrame(om.findObjectByName('drill').getChildFrame().transform)
        bitToWorld = transformUtils.concatenateTransforms([bitToDrill, drillToWorld])

        return self.computeFrameToHand(bitToWorld)


    def updateDrillTargetFrame(self, depth=0.0, offsetY=0.0, offsetZ=0.0):

        drillCircle = om.findObjectByName('drill circle')

        targetFrame = vtk.vtkTransform()
        targetFrame.PostMultiply()
        targetFrame.Translate(depth, offsetY, offsetZ)
        targetFrame.Concatenate(drillCircle.getChildFrame().transform)

        obj = vis.updateFrame(targetFrame, 'drill bit target', scale=0.2, parent=drillCircle)

        obj.setProperty('Edit', True)
        rep = obj.widget.GetRepresentation()
        rep.SetRotateAxisEnabled(0, False)
        rep.SetRotateAxisEnabled(1, False)
        rep.SetRotateAxisEnabled(2, False)
        obj.widget.HandleRotationEnabledOff()
        obj.setProperty('Edit', False)

    def storeDrillNominalPosture(self, pose):
        self.drillNominalPose = pose
        self.ikPlanner.addPose(pose, 'drill_nominal_pose')


    def setDrillPressTargetFrame(self, depthOffset, horizOffset, vertOffset, attackAngle=45):

        buttonToWorld = transformUtils.copyFrame(om.findObjectByName('drill button frame').transform)
        offsetToButton = transformUtils.frameFromPositionAndRPY([horizOffset, depthOffset, vertOffset], [attackAngle, 0, 0])
        offsetToWorld = transformUtils.concatenateTransforms([offsetToButton, buttonToWorld])

        targetFrame = om.findObjectByName('thumb press target frame')
        #newTarget = transformUtils.copyFrame(targetFrame.transform)
        #newTarget.PostMultiply()
        #newTarget.Translate(np.array(offsetToWorld.GetPosition()) - newTarget.GetPosition())

        targetFrame.copyFrame(offsetToWorld)


    def addThumbTargetFramesFromModel(self):

        drill = om.findObjectByName('drill')
        startPose = self.getPlanningStartPose()
        thumbSide = self.ikPlanner.flipSide(self.graspingHand)
        palmToHand = transformUtils.copyFrame(self.ikPlanner.getPalmToHandLink(thumbSide))
        palmToWorld = self.ikPlanner.newGraspToWorldFrame(startPose, thumbSide, palmToHand)

        thumbPressToWorld = transformUtils.concatenateTransforms([self.getThumbToPalmTransform(), palmToWorld])
        vis.updateFrame(thumbPressToWorld, 'thumb press target frame', scale=0.1, parent=drill, visible=True)

        self.thumbPressToHandFrame = transformUtils.concatenateTransforms([self.getThumbToPalmTransform(), palmToHand])

        print('thumb press to hand frame:', self.thumbPressToHandFrame.GetPosition(), transformUtils.rollPitchYawFromTransform(self.thumbPressToHandFrame))


    def planDrillButtonPress(self, jointDegreesPerSecond, quasiStaticShrinkFactor=None):

        ikParameters = IkParameters(maxDegreesPerSecond=jointDegreesPerSecond)
        if quasiStaticShrinkFactor is not None:
            ikParameters.quasiStaticShrinkFactor = quasiStaticShrinkFactor
        ikPlanner = self.ikPlanner
        startPose = self.getPlanningStartPose()
        thumbSide = ikPlanner.flipSide(self.graspingHand)
        assert self.thumbPressToHandFrame
        targetFrame = om.findObjectByName('thumb press target frame').transform

        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, thumbSide, targetFrame, self.thumbPressToHandFrame, lockBase=True, lockBack=True)


        seedPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'drill', 'thumb raise for press 2', side=thumbSide)
        seedPoseName = 'thumb_press_seed'
        self.ikPlanner.addPose(seedPose, seedPoseName)
        constraintSet.seedPoseName = seedPoseName
        constraintSet.nominalPoseName = seedPoseName

        constraintSet.ikParameters = ikParameters
        endPose, info = constraintSet.runIk()
        graspPlan = constraintSet.runIkTraj()
        self.addPlan(graspPlan)


    def planDrill(self, inPlane, inLine, translationSpeed, jointSpeed):

        targetFrame = om.findObjectByName('drill bit target')
        assert targetFrame
        self.planDrillTrajectory([targetFrame.transform], inPlane, inLine, translationSpeed, jointSpeed)

    def getDrillTargetOffsetFromCircle(self):

        circleToWorld = transformUtils.copyFrame(om.findObjectByName('drill circle').getChildFrame().transform)
        targetToWorld = transformUtils.copyFrame(om.findObjectByName('drill bit target').transform)
        targetToCircle = transformUtils.concatenateTransforms([targetToWorld, circleToWorld.GetLinearInverse()])

        return np.array(targetToCircle.GetPosition())


    def getDrillBitOffsetFromCircle(self):
        circleToWorld = transformUtils.copyFrame(om.findObjectByName('drill circle').getChildFrame().transform)
        bitToWorld = transformUtils.copyFrame(om.findObjectByName('drill bit frame').transform)
        bitToCircle = transformUtils.concatenateTransforms([bitToWorld, circleToWorld.GetLinearInverse()])
        return np.array(bitToCircle.GetPosition())

    def planDrillCircle(self):

        drillCircle = om.findObjectByName('drill circle')
        circleFrame = drillCircle.getChildFrame().transform
        radius = drillCircle.getProperty('Radius')
        numberOfTargets = drillCircle.getProperty('Segments')

        depth = self.getDrillTargetOffsetFromCircle()[0]

        targetFrames = []

        #om.removeFromObjectModel(om.findObjectByName('drill target frames'))
        #folder = om.getOrCreateContainer('drill target frames', parentObj=segmentation.getDebugFolder())
        d = DebugData()
        lastPoint = [None]

        def addTarget(x, y):
            t = transformUtils.copyFrame(circleFrame)
            t.PreMultiply()
            t.Translate(depth, x, y)
            targetFrames.append(t)
            #vis.showFrame(t, 'target %d' % i, scale=0.1, visible=False, parent=folder)

            p = t.GetPosition()
            d.addSphere(p, radius=0.002)
            if lastPoint[0]:
                d.addLine(p, lastPoint[0])
            lastPoint[0] = p


        for i in range(numberOfTargets+1):
            theta = (float(i)/numberOfTargets)*(2*np.pi) + np.pi/2.0
            x, y = radius*np.cos(theta), radius*np.sin(theta)
            addTarget(x, y)

        addTarget(-self.drillCutTailLength, radius+self.drillCutTailLength)

        vis.updatePolyData(d.getPolyData(), 'drill target trajectory', color=[1,1,0])

        self.planDrillTrajectory(targetFrames, inPlane=True, inLine=False, translationSpeed=self.drillTrajectoryMetersPerSecondSlow, jointSpeed=self.drillTrajectoryMaxDegreesPerSecond)


    def computeDrillBitFrameAtPose(self, pose, bitToHand):
        f = self.ikPlanner.getLinkFrameAtPose(self.ikPlanner.getHandLink(self.graspingHand), pose)
        f.PreMultiply()
        f.Concatenate(bitToHand)
        return f

    def planDrillTrajectory(self, targetFrames, inPlane, inLine, translationSpeed, jointSpeed):


        startPose = self.getPlanningStartPose()
        startPoseName = 'reach_start'
        self.ikPlanner.addPose(startPose, startPoseName)

        constraints = []
        constraints.extend(self.ikPlanner.createMovingReachConstraints(startPoseName, lockBase=self.lockBaseForDrilling, lockBack=self.lockBackForDrilling, side=self.graspingHand))

        bitToHand = self.computeDrillBitToHand()
        handLinkName = self.ikPlanner.getHandLink(self.graspingHand)

        gazeTargetFrame = targetFrames[-1]

        bodyAxis = [1.0, 0.0, 0.0] if self.graspingHand == 'right' else [-1.0, 0.0, 0.0]
        gazeConstraint = self.ikPlanner.createGazeGraspConstraint(self.graspingHand, gazeTargetFrame, targetAxis=[1.0, 0.0, 0.0] , bodyAxis=bodyAxis)


        endTime = 1.0 * len(targetFrames)

        if inPlane or inLine:
            gazeConstraint.tspan = [0.5, endTime]
        else:
            gazeConstraint.tspan = [endTime, endTime]

        constraints.append(gazeConstraint)

        lastTargetFrame = self.computeDrillBitFrameAtPose(startPose, bitToHand)

        for i, targetFrame in enumerate(targetFrames):

            positionConstraint, _ = self.ikPlanner.createPositionOrientationGraspConstraints(self.graspingHand, targetFrame, bitToHand)
            activeTime = float(i+1)
            positionConstraint.tspan = [activeTime, activeTime]
            constraints.append(positionConstraint)

            motionVector = np.array(targetFrame.GetPosition()) - np.array(lastTargetFrame.GetPosition())
            motionTargetFrame = transformUtils.getTransformFromOriginAndNormal(np.array(targetFrame.GetPosition()), motionVector)
            #vis.updateFrame(motionTargetFrame, 'motion target frame %d' %i, scale=0.1)
            #d = DebugData()
            #d.addLine(np.array(targetFrame.GetPosition()), np.array(targetFrame.GetPosition()) - motionVector)
            #vis.updatePolyData(d.getPolyData(), 'motion vector %d' % i)

            p = self.ikPlanner.createLinePositionConstraint(handLinkName, bitToHand, motionTargetFrame, lineAxis=2, bounds=[-np.linalg.norm(motionVector), 0.001], positionTolerance=0.001)
            p.tspan = [activeTime-1.0, activeTime]
            constraints.append(p)

            lastTargetFrame = targetFrame


        #if inPlane:
        #    p = self.ikPlanner.createPlanePositionConstraint(handLinkName, bitToHand, gazeTargetFrame, planeNormalAxis=0, bounds=[-0.001, 0.001])
        #    p.tspan = [0.0, endTime]
        #    constraints.append(p)


        #if inLine:
        #    p = self.ikPlanner.createLinePositionConstraint(handLinkName, bitToHand, gazeTargetFrame, lineAxis=0, bounds=[-np.inf, 0.0], positionTolerance=0.001)
        #    p.tspan = [0.0, endTime]
        #    constraints.append(p)

        if (inPlane or inLine) and len(targetFrames) == 1:
            numberOfAddedKnots = 5 # todo, select number of knots per distance traveled?
        else:
            numberOfAddedKnots = 0


        constraintSet = ikplanner.ConstraintSet(self.ikPlanner, constraints, 'reach_end', startPoseName)

        #for c in constraintSet.constraints:
        #    print c

        #constraintSet.seedPoseName = 'q_start'
        #constraintSet.nominalPoseName = 'q_start'
        constraintSet.seedPoseName = 'drill_nominal_pose'
        constraintSet.nominalPoseName = 'drill_nominal_pose'

        def updateDrillIk():
            endPose, info = constraintSet.runIk()

        def updateDrillPlan():
            constraintSet.ikParameters.usePointwise = False
            constraintSet.ikParameters.maxBodyTranslationSpeed = translationSpeed
            constraintSet.ikParameters.rescaleBodyNames = [handLinkName]
            constraintSet.ikParameters.rescaleBodyPts = list(bitToHand.GetPosition())
            constraintSet.ikParameters.maxDegreesPerSecond = jointSpeed
            constraintSet.ikParameters.numberOfAddedKnots = numberOfAddedKnots

            plan = constraintSet.runIkTraj()
            self.addPlan(plan)

        self.constraintSet = constraintSet
        self.updateDrillIk = updateDrillIk
        self.updateDrillPlan = updateDrillPlan

        if len(targetFrames) == 1:
            updateDrillIk()
        else:
            constraintSet.endPose = startPose
        updateDrillPlan()


    def planDrillDrop(self):
        def saveOriginalTraj(name):
            commands = ['%s = qtraj_orig;' % name]
            self.ikPlanner.ikServer.comm.sendCommands(commands)

        def concatenateAndRescaleTrajectories(trajectoryNames, concatenatedTrajectoryName, junctionTimesName, ikParameters):
            commands = []
            commands.append('joint_v_max = repmat(%s*pi/180, r.getNumVelocities()-6, 1);' % ikParameters.maxDegreesPerSecond)
            commands.append('xyz_v_max = repmat(%s, 3, 1);' % ikParameters.maxBaseMetersPerSecond)
            commands.append('rpy_v_max = repmat(%s*pi/180, 3, 1);' % ikParameters.maxBaseRPYDegreesPerSecond)
            commands.append('v_max = [xyz_v_max; rpy_v_max; joint_v_max];')
            commands.append("max_body_translation_speed = %r;" % ikParameters.maxBodyTranslationSpeed)
            commands.append("max_body_rotation_speed = %r;" % ikParameters.maxBodyRotationSpeed)
            commands.append('rescale_body_ids = [%s];' % (','.join(['links.%s' % linkName for linkName in ikParameters.rescaleBodyNames])))
            commands.append('rescale_body_pts = reshape(%s, 3, []);' % ikconstraints.ConstraintBase.toColumnVectorString(ikParameters.rescaleBodyPts))
            commands.append("body_rescale_options = struct('body_id',rescale_body_ids,'pts',rescale_body_pts,'max_v',max_body_translation_speed,'max_theta',max_body_rotation_speed,'robot',r);")
            commands.append('trajectories = {};')
            for name in trajectoryNames:
                commands.append('trajectories{end+1} = %s;' % name)
            commands.append('[%s, %s] = concatAndRescaleTrajectories(trajectories, v_max, %s, %s, body_rescale_options);' % (concatenatedTrajectoryName, junctionTimesName, ikParameters.accelerationParam, ikParameters.accelerationFraction))
            commands.append('t_new = linspace({0}.tspan(1),{0}.tspan(end),20);'.format(concatenatedTrajectoryName))
            commands.append('{0} = PPTrajectory(pchip(t_new, {0}.eval(t_new)));'.format(concatenatedTrajectoryName))
            commands.append('s.publishTraj(%s, 1);' % concatenatedTrajectoryName)
            self.ikPlanner.ikServer.comm.sendCommands(commands)
            return self.ikPlanner.ikServer.comm.getFloatArray(junctionTimesName)

        planNames = []

        planNames.append('qtraj_out_to_prep')
        self.planDrillIntoWallPrep()
        saveOriginalTraj(planNames[-1])

        self.planFromCurrentRobotState = False
        try:
            planNames.append('qtraj_prep_to_walk');
            self.planWalkWithDrillPosture()
            saveOriginalTraj(planNames[-1])

            planNames.append('qtraj_walk_to_raise');
            q1 = self.getPlanningStartPose()
            q2 = self.ikPlanner.getMergedPostureFromDatabase(q1, 'General', 'arm up pregrasp', side=self.graspingHand)
            newPlan = self.ikPlanner.computePostureGoal(q1, q2)
            self.addPlan(newPlan)
            saveOriginalTraj(planNames[-1])

            planNames.append('qtraj_walk_to_drop');
            q1 = self.getPlanningStartPose()
            q2 = self.ikPlanner.getMergedPostureFromDatabase(q1, 'drill', 'drop-drill', side=self.graspingHand)
            newPlan = self.ikPlanner.computePostureGoal(q1, q2)
            self.addPlan(newPlan)
            saveOriginalTraj(planNames[-1])

        finally:
            self.planFromCurrentRobotState = True;

        handLinkName = self.ikPlanner.getHandLink(self.graspingHand)
        ikParameters = IkParameters(usePointwise=True,
                                    rescaleBodyNames=[handLinkName], rescaleBodyPts=[0.0, 0.0, 0.0],
                                    maxBodyTranslationSpeed=self.drillTrajectoryMetersPerSecondFast)

        ikParameters = self.ikPlanner.mergeWithDefaultIkParameters(ikParameters)
        listener = self.ikPlanner.getManipPlanListener()
        concatenateAndRescaleTrajectories(planNames, 'qtraj_drill_drop', 'ts', ikParameters)
        plan = listener.waitForResponse()
        listener.finish()
        self.addPlan(plan)


    def spawnWallAffordanceTest(self, x, y, yaw, targetHeight, targetRadius):

        wallWidth = 1.0
        wallHeight = 2.0

        stanceFrame = self.footstepPlanner.getFeetMidPoint(self.robotModel)
        stanceFrame.PreMultiply()
        stanceFrame.Translate(x, y, wallHeight/2.0)
        pos = np.array(stanceFrame.GetPosition())
        stanceFrame.PostMultiply()
        stanceFrame.Translate(-pos)
        stanceFrame.RotateZ(yaw)
        stanceFrame.Translate(pos)

        wall = self.spawnWallAffordanceNew(stanceFrame, wallWidth, wallHeight)
        wallFrame = transformUtils.copyFrame(wall.getChildFrame().transform)
        wallFrame.Translate(0.0, 0.0, targetHeight - wallHeight/2.0)

        self.spawnDrillCircle(wallFrame, targetRadius)


    def spawnWallAffordanceNew(self, t, width, height):

        dimensions = [0.01, width, height]

        name = 'drill wall'
        aff = om.findObjectByName(name)
        if not aff:
            pose = transformUtils.poseFromTransform(t)
            desc = dict(classname='BoxAffordanceItem', Name=name, Dimensions=dimensions, pose=pose)
            aff = segmentation.affordanceManager.newAffordanceFromDescription(desc)
            aff.setProperty('Alpha', 0.2)
            aff.setProperty('Visible', False)
        else:
            aff.setProperty('Dimensions', dimensions)
            aff.getChildFrame().copyFrame(t)

        return aff

    def computeWallStanceFrame(self, stanceOffsetX, stanceOffsetY, stanceYaw):

        footFrame = self.footstepPlanner.getFeetMidPoint(self.robotModel)
        groundHeight = footFrame.GetPosition()[2]

        wall = om.findObjectByName('drill circle')
        assert wall

        wallFrame = wall.getChildFrame().transform

        wallGroundFrame = transformUtils.copyFrame(wallFrame)
        wallGroundFrame.PreMultiply()
        wallGroundFrame.Translate(0.0, 0.0, groundHeight - wallGroundFrame.GetPosition()[2])

        vis.updateFrame(wallGroundFrame, 'wall ground frame', parent=wall, scale=0.2, visible=False)

        stanceFrame = transformUtils.copyFrame(wallGroundFrame)
        stanceFrame.PreMultiply()
        stanceFrame.Translate(stanceOffsetX, stanceOffsetY, 0.0)

        pos = np.array(stanceFrame.GetPosition())
        stanceFrame.PostMultiply()
        stanceFrame.Translate(-pos)
        stanceFrame.RotateZ(stanceYaw)
        stanceFrame.Translate(pos)

        vis.updateFrame(stanceFrame, 'wall stance frame', parent=wall, scale=0.2, visible=True)


    def spawnDrillAffordanceTest(self):

        sign = 1 if self.graspingHand == 'left' else -1
        stanceFrame = self.footstepPlanner.getFeetMidPoint(self.robotModel)
        stanceFrame.PreMultiply()
        stanceFrame.Translate(0.85, 0.4*sign, 1.2)
        self.spawnDrillAffordanceNew(stanceFrame)


    def updateDrillRelativeFrame(self, frameName, frameToDrill, fixed=True):
        aff = om.findObjectByName('drill')
        frame = om.findObjectByName(frameName)
        frameToWorld = transformUtils.concatenateTransforms([transformUtils.copyFrame(frameToDrill), transformUtils.copyFrame(aff.getChildFrame().transform)])
        if not frame:
            frame = vis.updateFrame(frameToWorld, frameName, scale=0.1, visible=False, parent=aff)
            aff.getChildFrame().getFrameSync().addFrame(frame, ignoreIncoming=(not fixed))
        else:
            frame.copyFrame(frameToWorld)
        return frame

    def spawnDrillAffordanceNew(self, t):

        name = 'drill'
        aff = om.findObjectByName(name)
        if not aff:
            pose = transformUtils.poseFromTransform(t)
            desc = dict(classname='MeshAffordanceItem', Name=name, Filename='software/models/otdf/dewalt_drill_primitive.vtp', Color=[0,1,0], pose=pose)
            aff = segmentation.affordanceManager.newAffordanceFromDescription(desc)
        else:
            aff.getChildFrame().copyFrame(t)

        self.updateDrillRelativeFrames()
        return aff


    def updateDrillRelativeFrames(self):

        self.updateDrillRelativeFrame('drill bit frame', self.getBitToDrillTransform())
        self.updateDrillRelativeFrame('drill butt frame', self.getButtToDrillTransform())
        self.updateDrillRelativeFrame('drill grasp frame', self.getGraspToDrillTransform())
        self.updateDrillRelativeFrame('drill press prep frame', self.getPressPrepToDrillTransform())
        self.updateDrillRelativeFrame('drill button frame', self.getButtonToDrillTransform(), fixed=False)


    def moveDrillToHandNew(self):

        yaw = self.drillGraspYaw + 90
        flip = self.graspingHand == 'right'
        zoffset = 0.0

        startPose = self.getPlanningStartPose()
        handToWorld = self.ikPlanner.getLinkFrameAtPose(self.ikPlanner.getHandLink(self.graspingHand), startPose)
        palmToHand = self.ikPlanner.getPalmToHandLink(self.graspingHand)
        drillToPalm = segmentation.getDrillInHandOffset(yaw, zoffset, xTranslation=0.025, flip=flip)

        drillToWorld = vtk.vtkTransform()
        drillToWorld.PostMultiply()
        drillToWorld.Concatenate(drillToPalm)
        drillToWorld.Concatenate(palmToHand)
        drillToWorld.Concatenate(handToWorld)

        drill = om.findObjectByName('drill')
        assert drill
        drill.getChildFrame().copyFrame(drillToWorld)



    def spawnDrillCircle(self, t, radius):

        name = 'drill circle'
        aff = om.findObjectByName(name)
        if not aff:
            pose = transformUtils.poseFromTransform(t)
            desc = dict(classname='CapsuleRingAffordanceItem', Name=name, Radius=radius, pose=pose)
            desc['Segments'] = 20
            desc['Tube Radius'] = 0.002
            aff = segmentation.affordanceManager.newAffordanceFromDescription(desc)
            self.updateDrillTargetFrame(0.0, 0.0, 0.0)

        else:
            aff.setProperty('Radius', radius)
            aff.getChildFrame().copyFrame(t)

        return aff


    def planReach(self):
        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.drill.reachFrame, lockBase=False, lockBack=True)
        endPose, info = constraintSet.runIk()
        graspPlan = constraintSet.runIkTraj()
        self.addPlan(graspPlan)

    def planGrasp(self):
        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.drill.graspFrame, lockBase=False, lockBack=True)
        endPose, info = constraintSet.runIk()
        graspPlan = constraintSet.runIkTraj()
        self.addPlan(graspPlan)

        t3 = transformUtils.frameFromPositionAndRPY( [0.00,0.0, 0.025] , [0,0,0] )
        placeTransform = transformUtils.copyFrame(self.drill.graspFrame.transform)
        placeTransform.Concatenate(t3)
        dereachTransform = transformUtils.copyFrame( self.drill.reachFrame.transform )
        dereachTransform.Concatenate(t3)
        self.placeFrame = vis.updateFrame(placeTransform, 'place frame', parent="affordances", visible=False, scale=0.2)
        self.dereachFrame = vis.updateFrame(dereachTransform, 'dereach frame', parent="affordances", visible=False, scale=0.2)

    def planGraspLift(self):
        t3 = transformUtils.frameFromPositionAndRPY( [0.00,0.0, 0.04] , [0,0,0] )
        liftTransform = transformUtils.copyFrame(self.drill.graspFrame.transform)
        liftTransform.Concatenate(t3)

        self.drill.liftFrame = vis.updateFrame(liftTransform, 'lift frame', parent="affordances", visible=False, scale=0.2)
        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.drill.liftFrame, lockBase=False, lockBack=True)
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

        if (self.drill.model == 'dewalt_barrel'):
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
        self.constraintSet.ikParameters.usePointwise = False
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
        worldToPress = transformUtils.copyFrame(self.drill.buttonFrame.transform)
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

        #t0 = transformUtils.copyFrame( self.wall.transform )
        t0 = transformUtils.copyFrame( self.wall.frame.transform )

        t1 = transformUtils.frameFromPositionAndRPY([0, self.wall.affordance.params['p2y'], self.wall.affordance.params['p2z'] ], [0,0,0])
        t1.Concatenate(  self.wall.frame.transform )

        t2 = transformUtils.frameFromPositionAndRPY([0, self.wall.affordance.params['p3y'], self.wall.affordance.params['p3z'] ], [0,0,0])
        t2.Concatenate(  self.wall.frame.transform )

        self.wall.cutPointFrames.append( vis.updateFrame(t0, 'cut point 0', parent=self.wall.affordance, visible=False, scale=0.2) )
        self.wall.cutPointFrames.append( vis.updateFrame(t1, 'cut point 1', parent=self.wall.affordance, visible=False, scale=0.2) )
        self.wall.cutPointFrames.append( vis.updateFrame(t2, 'cut point 2', parent=self.wall.affordance, visible=False, scale=0.2) )
        self.wall.frameSync.addFrame( self.wall.cutPointFrames[0] )
        self.wall.frameSync.addFrame( self.wall.cutPointFrames[1] )
        self.wall.frameSync.addFrame( self.wall.cutPointFrames[2] )

        self.cutOrientation = self.wall.frame.transform.GetOrientation()
        self.cutPoints = [t0.GetPosition() , t1.GetPosition(), t2.GetPosition(), t0.GetPosition() ]
        self.currentCutGoal = 0
        nextCutPoseGoal = self.updateNextCutPoseGoal()

        if (engagedTip == False):
            nextCutPose = transformUtils.frameFromPositionAndRPY( [ self.retractBitDepthNominal , 0, 0] , [0,0,0] )
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
        drillTransformOriginal = self.drill.affordance.actor.GetUserTransform()
        buttonTransformOriginal = transformUtils.copyFrame( drillTransformOriginal )
        buttonTransformOriginal.PreMultiply()
        buttonTransformOriginal.Concatenate(self.drill.drillToButtonTransform)

        buttonTransformNew = transformUtils.frameFromPositionAndRPY(buttonSensedTransform.GetPosition() ,  np.array(transformUtils.rollPitchYawFromTransform( buttonTransformOriginal ))*180/np.pi )
        drillTransformNew = transformUtils.copyFrame(buttonTransformNew)
        drillTransformNew.PreMultiply()
        t3 = transformUtils.copyFrame ( self.drill.drillToButtonTransform.GetLinearInverse() )
        drillTransformNew.Concatenate(t3)

        self.drill.affordance.actor.SetUserTransform(drillTransformNew)
        self.drill.frame = vis.updateFrame(drillTransformNew, 'drill frame', parent=self.drill.affordance, visible=False, scale=0.2)



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
        print("distToGoal2D", distToGoal2D , "from" , self.currentCutGoal)

        if (distToGoal2D < self.goalThreshold ): # typically 5cm
            if ( self.currentCutGoal == len(self.cutPoints) -1):
                print(distToGoal2D , " - within threshold of last goal", self.currentCutGoal)
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

            print("handToBit")
            print(handToBit.GetPosition())
            print(handToBit.GetOrientation())

        # button drill: [1,0,0] , barrel drill: [0,1,0] - axis is along bit xaxis (this was explictly defined
        all_axes = transformUtils.getAxesFromTransform( handToBit )

        self.initGazeConstraintSet(self.nextCutFrame, self.graspingHand, handToBit, targetAxis=all_axes[0], bodyAxis=all_axes[0])
        self.appendPositionConstraintForTargetFrame(self.nextCutFrame, 1, self.graspingHand, handToBit)
        #self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedSlow
        self.planGazeTrajectory()
        #self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedHigh


    def planNavigationGoalAwayFromShelf(self):

        xoffset = -0.35
        yoffset = -0.1
        yaw = -60

        stanceFrame = self.footstepPlanner.getFeetMidPoint(self.robotModel)
        t = transformUtils.frameFromPositionAndRPY([xoffset, yoffset,  0.0], [0, 0, yaw])

        goalFrame = transformUtils.concatenateTransforms([t, stanceFrame])
        footstepsdriverpanel.panel.onNewWalkingGoal(goalFrame)


    ########## Glue Functions ####################################
    def moveRobotToStanceFrame(self, frame):
        self.sensorJointController.setPose('q_nom')
        stancePosition = frame.GetPosition()
        stanceOrientation = frame.GetOrientation()

        q = self.sensorJointController.q.copy()
        q[:2] = [stancePosition[0], stancePosition[1]]
        q[5] = math.radians(stanceOrientation[2])
        self.sensorJointController.setPose('EST_ROBOT_STATE', q)

    def getHandDriver(self, side):
        assert side in ('left', 'right')
        return self.lhandDriver if side == 'left' else self.rhandDriver

    def openHand(self,side):
        #self.handDriver(side).sendOpen()
        self.getHandDriver(side).sendCustom(0.0, 100.0, 100.0, 0)

    def closeHand(self, side):
        #self.handDriver(side).sendClose(60)
        self.getHandDriver(side).sendCustom(100.0, 100.0, 100.0, 0)

    def closeTopFingers(self, side):
        self.getHandDriver(side).sendFingerControl(254, 0, 254, 100, 100, 200, 0)

    def closeAllFingers(self, side):
        self.getHandDriver(side).sendFingerControl(254, 254, 254, 100, 100, 200, 0)

    def fingerGrasp(self):
        self.closeTopFingers('right')
        time.sleep(1.5)
        self.closeAllFingers('right')
        time.sleep(1.5)
        self.closeAllFingers('right')

    def fingerGraspTighten(self):
        self.closeAllFingers('right')

    def sendNeckPitchLookDown(self):
        self.multisenseDriver.setNeckPitch(40)

    def sendNeckPitchLookForward(self):
        self.multisenseDriver.setNeckPitch(15)

    def waitForAtlasBehaviorAsync(self, behaviorName):
        assert behaviorName in list(self.atlasDriver.getBehaviorMap().values())
        while self.atlasDriver.getCurrentBehaviorName() != behaviorName:
            yield

    def printAsync(self, s):
        yield
        print(s)

    def userPrompt(self, message):
        if not self.userPromptEnabled:
            return

        yield
        result = input(message)
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

    def waitForPlanAnimation(self, plan):
        planElapsedTime = planplayback.PlanPlayback.getPlanElapsedTime(plan)
        print('waiting for plan animation:', planElapsedTime)
        return self.delay(planElapsedTime)

    def animateLastPlan(self):
        plan = self.plans[-1]
        if self.visOnly:
            return self.waitForPlanAnimation(plan)
        else:
            self.commitManipPlan()
            return self.waitForPlanExecution()

    def turnPointwiseOff(self):
        ikplanner.getIkOptions().setProperty('Use pointwise', False)
        ikplanner.getIkOptions().setProperty('Quasistatic shrink factor', 0.1)
        ikplanner.getIkOptions().setProperty('Max joint degrees/s',30)

    def turnPointwiseOffSlow(self):
        ikplanner.getIkOptions().setProperty('Use pointwise', False)
        ikplanner.getIkOptions().setProperty('Quasistatic shrink factor', 0.1)
        ikplanner.getIkOptions().setProperty('Max joint degrees/s',15)

    ######### Nominal Plans and Execution  #################################################################
    def planSequencePickUp(self, playbackNominal=True):

        self.turnPointwiseOff()

        self.planFromCurrentRobotState = False
        self.plans = []

        if (om.findObjectByName('drill') is None):
            self.spawnDrillAffordance()

        if (om.findObjectByName('wall') is None):
            self.spawnWallAffordance()


        if self.useFootstepPlanner:
            #self.planFootsteps( self.drill.stanceFrame.transform )
            self.planFootstepsDrill()
            self.planWalking()
        else:
            self.moveRobotToStanceFrame(self.drill.stanceFrame.transform)

        self.planPreGrasp()
        self.planReach()
        self.planGrasp()

        # self.affordanceUpdater.graspAffordance('drill', self.graspingHand)

        self.planNominal()
        self.planPreGrasp()
        self.planDrillLowerSafe()

        if (playbackNominal is True):
            self.playSequenceNominal()


    def planSequenceTurnOn(self, playbackNominal=True):

        self.planFromCurrentRobotState = False
        if (self.flushNominalPlanSequence):
            self.plans = []
        if (om.findObjectByName('drill') is None):
            self.spawnDrillAffordance()
            #self.affordanceUpdater.graspAffordance('drill', self.graspingHand)
        if (om.findObjectByName('wall') is None):
            self.spawnWallAffordance()

        if self.useFootstepPlanner:
            self.planFootstepsTurnOn()
            #self.planFootsteps( self.wall.stanceFarFrame.transform )
            self.planWalking()
        else:
            self.moveRobotToStanceFrame( self.wall.stanceFarFrame.transform )

        self.planDrillRaisePowerOn()
        self.planPointerRaisePowerOn()

        self.planPointerPressGaze(0)
        self.planPointerPressGaze(-0.02)
        self.planPointerPressGaze(0)
        self.planPointerPressGaze(-0.02)
        self.planPointerPressGaze(0)
        self.planPointerPressGaze(-0.02)
        self.planPointerRaisePowerOn()

        self.planPointerLowerPowerOn()

        self.planDrillRaiseForCutting()

        if (playbackNominal is True):
            self.playSequenceNominal()


    def planSequenceCut(self, playbackNominal=True):

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
            self.planFootstepsCut()
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
            print(" ")

        self.planNextCut( self.computeRetractionDesired, True )

        ikplanner.getIkOptions().setProperty('Use pointwise', False)
        ikplanner.getIkOptions().setProperty('Quasistatic shrink factor', 0.1)

        self.planDrillRaiseForCutting()

        if self.useFootstepPlanner:
            self.planFootstepsTurnOn()
            #self.planFootsteps( self.wall.stanceFarFrame.transform )
            self.planWalking()
        else:
            self.moveRobotToStanceFrame( self.wall.stanceFarFrame.transform )

        self.planDrillLowerSafe()

        if (playbackNominal is True):
            self.playSequenceNominal()


    def planSequence(self, playbackNominal=True):
        self.planSequencePickUp(playbackNominal=False)
        self.planSequenceTurnOn(playbackNominal=False)
        self.planSequenceCut(playbackNominal=False)
        self.playSequenceNominal()



    def autonomousExecutePickup(self):

        self.planFromCurrentRobotState = True
        self.visOnly = False

        taskQueue = AsyncTaskQueue()
        taskQueue.addTask(self.turnPointwiseOff)

        # walk up
        taskQueue.addTask(self.printAsync('neck pitch down'))
        taskQueue.addTask(self.sendNeckPitchLookDown)
        taskQueue.addTask(self.userPrompt('please fit drill. continue? y/n: '))
        taskQueue.addTask(self.findDrillAffordance)

        graspPlanFunctions = [self.planPreGrasp, self.planReach, self.planGrasp]
        for planFunc in graspPlanFunctions:
            taskQueue.addTask(planFunc)
            taskQueue.addTask(self.userPrompt('grasp plan continue? y/n: '))
            taskQueue.addTask(self.animateLastPlan)
        taskQueue.addTask(functools.partial(self.closeHand, 'left'))

        taskQueue.addTask(self.planNominal)
        taskQueue.addTask(self.userPrompt('lift off table? y/n: '))
        taskQueue.addTask(self.animateLastPlan)


        taskQueue.addTask(self.planBothRaisePowerOn)
        taskQueue.addTask(self.userPrompt('raise for pressing? y/n: '))
        taskQueue.addTask(self.animateLastPlan)

        return taskQueue


    def autonomousExecute(self):

        self.planFromCurrentRobotState = True
        self.visOnly = False

        taskQueue = AsyncTaskQueue()
        taskQueue.addTask(self.turnPointwiseOff)

        # walk up
        taskQueue.addTask(self.printAsync('neck pitch down'))
        taskQueue.addTask(self.sendNeckPitchLookDown)
        taskQueue.addTask(self.userPrompt('please fit drill. continue? y/n: '))
        taskQueue.addTask(self.findDrillAffordance)
        taskQueue.addTask(self.planFootstepsDrill)
        taskQueue.addTask(self.userPrompt('send footstep plan. continue? y/n: '))
        taskQueue.addTask(self.commitFootstepPlan)

        # grasp drill:
        taskQueue.addTask(self.userPrompt('please fit drill. continue? y/n: '))
        taskQueue.addTask(self.findDrillAffordance)

        taskQueue.addTask(self.planPreGrasp)
        #taskQueue.addTask(self.userPrompt('lower pointer? y/n: '))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(self.planReach)
        #taskQueue.addTask(self.userPrompt('lower pointer? y/n: '))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(self.turnPointwiseOffSlow)

        taskQueue.addTask(self.planGrasp)
        #taskQueue.addTask(self.userPrompt('lower pointer? y/n: '))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(functools.partial(self.closeHand, 'left'))

        #taskQueue.addTask(self.planNominal)
        taskQueue.addTask(self.planGraspLift)
        #taskQueue.addTask(self.userPrompt('lift off table? y/n: '))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(self.planBothRaisePowerOn)
        #taskQueue.addTask(self.userPrompt('raise for pressing? y/n: '))
        taskQueue.addTask(self.animateLastPlan)

        return taskQueue


    def autonomousExecutePlaceDown(self):

        self.planFromCurrentRobotState = True
        self.visOnly = False

        taskQueue = AsyncTaskQueue()
        taskQueue.addTask(self.turnPointwiseOff)

        taskQueue.addTask(self.planPointerLowerPowerOn)
        #taskQueue.addTask(self.userPrompt('lower pointer? y/n: '))
        taskQueue.addTask(self.animateLastPlan)

        #taskQueue.addTask(self.planPreGrasp)
        ##taskQueue.addTask(self.userPrompt('pre grasp? y/n: '))
        #taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(self.planPlace)
        #taskQueue.addTask(self.userPrompt('place on table? y/n: '))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(functools.partial(self.openHand, 'left'))

        taskQueue.addTask(self.planDereach)
        #taskQueue.addTask(self.userPrompt('dereach hand? y/n: '))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(self.planPreGrasp)
        #taskQueue.addTask(self.userPrompt('pre grasp? y/n: '))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(self.planDrillLowerSafe)
        #taskQueue.addTask(self.userPrompt('lower? y/n: '))
        taskQueue.addTask(self.animateLastPlan)
        return taskQueue


    def autonomousExecuteOld(self):

        # stand and open hand
        taskQueue.addTask(self.userPrompt('stand and open hand. continue? y/n: '))
        taskQueue.addTask(self.atlasDriver.sendStandCommand)
        taskQueue.addTask(self.sendOpenHand)

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
        taskQueue.addTask(self.closeHand)
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



class DrillImageFitter(ImageBasedAffordanceFit):

    def __init__(self, drillDemo):
        ImageBasedAffordanceFit.__init__(self, numberOfPoints=1)
        self.drillDemo = drillDemo
        self.fitFunc = None

        self.pickLineRadius = 0.05
        self.pickNearestToCamera = False

        self.useLocalPlaneFit = True
        self.useVoxelGrid = True


    def fit(self, polyData, points):
        if self.fitFunc:
            self.fitFunc(polyData, points)

    def fitShelf(self, polyData, points):

        planePoints, normal = segmentation.applyLocalPlaneFit(polyData, points[0], searchRadius=0.1, searchRadiusEnd=0.2)
        origin = segmentation.computeCentroid(planePoints)

        vis.updatePolyData(planePoints, 'shelf plane points', parent=segmentation.getDebugFolder(), color=[0,1,0], visible=True)

        points = [segmentation.projectPointToPlane(p, origin, normal) for p in points]

        up = np.array([0,0,1])
        perpAxis = points[1] - points[0]
        edgeAxis = np.cross(up, perpAxis)
        perpAxis /= np.linalg.norm(perpAxis)
        edgeAxis /= np.linalg.norm(edgeAxis)

        edgePoints = segmentation.computeEdge(planePoints, edgeAxis, perpAxis)
        edgePoints = vnp.getVtkPolyDataFromNumpyPoints(edgePoints)
        vis.updatePolyData(edgePoints, 'edge points', parent=segmentation.getDebugFolder(), visible=True)


        linePoint, lineDirection, fitPoints = segmentation.applyLineFit(edgePoints)

        vis.updatePolyData(fitPoints, 'line fit points', parent=segmentation.getDebugFolder(), colorByName='ransac_labels', visible=False)

        linePoints = segmentation.thresholdPoints(fitPoints, 'ransac_labels', [1.0, 1.0])
        dists = np.dot(vnp.getNumpyFromVtk(linePoints, 'Points')-linePoint, lineDirection)

        p1 = linePoint + lineDirection*np.min(dists)
        p2 = linePoint + lineDirection*np.max(dists)

        d = DebugData()
        d.addSphere(p1, radius=0.015)
        d.addSphere(p2, radius=0.015)
        d.addLine(p1, p2, radius=0.007)

        vis.updatePolyData(d.getPolyData(), 'table edge', color=[0,1,1])


    def fitDrill(self, polyData, points):

        drillPoint = points[0]

        searchRegion = segmentation.cropToSphere(polyData, drillPoint, 0.2)

        viewDirection = segmentation.SegmentationContext.getGlobalInstance().getViewDirection()

        xaxis = viewDirection
        zaxis = [0,0,1]
        yaxis = np.cross(zaxis, xaxis)
        yaxis /= np.linalg.norm(yaxis)
        xaxis = np.cross(yaxis, zaxis)
        xaxis /= np.linalg.norm(xaxis)

        t = transformUtils.getTransformFromAxesAndOrigin(xaxis, yaxis, zaxis, drillPoint)

        polyData = segmentation.cropToBounds(polyData, t, [[-0.07, 0.07], [-0.07, 0.07], [-0.2, 0.2]])

        obj = vis.updatePolyData(polyData, 'cropped drill points', color=[1,0,0], visible=False)
        obj.setProperty('Point Size', 3)

        centroid = segmentation.computeCentroid(polyData)
        maxZ = np.max(segmentation.vnp.getNumpyFromVtk(polyData, 'Points')[:,2])

        origin = np.array([drillPoint[0], drillPoint[1], maxZ])

        drillGuardToOrigin = 0.128
        drillHandleRadius = 0.0287

        t = transformUtils.getTransformFromAxesAndOrigin(xaxis, yaxis, zaxis, origin)
        t.PreMultiply()
        t.Translate(drillHandleRadius, 0.0, -drillGuardToOrigin)

        if self.drillDemo.graspingHand == 'right':
            t.PreMultiply()
            t.RotateZ(180)

        drill = self.drillDemo.spawnDrillAffordanceNew(t)


    def fitDrillOnTable(self, polyData, points):

        self.fitDrill(polyData, points)

        drillPoint = points[0]
        tablePoint = points[1]

        viewDirection = segmentation.SegmentationContext.getGlobalInstance().getViewDirection()

        xaxis = viewDirection
        zaxis = [0,0,1]
        yaxis = np.cross(zaxis, xaxis)
        yaxis /= np.linalg.norm(yaxis)
        xaxis = np.cross(yaxis, zaxis)
        xaxis /= np.linalg.norm(xaxis)

        origin = np.array([drillPoint[0], drillPoint[1], tablePoint[2]])

        drillOriginToTable = 0.139
        drillHandleRadius = 0.0287

        t = transformUtils.getTransformFromAxesAndOrigin(xaxis, yaxis, zaxis, origin)
        t.PreMultiply()
        t.Translate(drillHandleRadius, 0.0, drillOriginToTable)

        if self.drillDemo.graspingHand == 'right':
            t.PreMultiply()
            t.RotateZ(180)

        drill = self.drillDemo.spawnDrillAffordanceNew(t)


    def fitDrillButtonPress(self, polyData, points):
        drill = om.findObjectByName('drill')
        drillFrame = om.findObjectByName('drill').getChildFrame().transform

        buttonFrame = om.findObjectByName('drill button frame')
        t = transformUtils.copyFrame(buttonFrame.transform)
        t.PostMultiply()
        t.Translate(np.array(points[0]) - np.array(t.GetPosition()))
        buttonFrame.copyFrame(t)
        buttonFrame.setProperty('Visible', True)

        ikPlanner = self.drillDemo.ikPlanner
        startPose = self.drillDemo.getPlanningStartPose()




        thumbSide = ikPlanner.flipSide(self.drillDemo.graspingHand)
        palmToHand = transformUtils.copyFrame(ikPlanner.getPalmToHandLink(thumbSide))
        palmToWorld = ikPlanner.newGraspToWorldFrame(startPose, thumbSide, palmToHand)
        handToWorld = ikPlanner.getLinkFrameAtPose(ikPlanner.getHandLink(thumbSide), startPose)

        thumbFitToWorld = transformUtils.copyFrame(palmToWorld)
        thumbFitToWorld.PostMultiply()
        thumbFitToWorld.Translate(np.array(points[1]) - np.array(thumbFitToWorld.GetPosition()))

        vis.updateFrame(thumbFitToWorld, 'thumb fit frame', scale=0.1, parent=segmentation.getDebugFolder(), visible=False)

        if thumbSide == 'left':
            pressToThumbFit = transformUtils.frameFromPositionAndRPY([-0.01, 0.0, -0.01], [0.0, 0.0, 0.0])
        else:
            pressToThumbFit = transformUtils.frameFromPositionAndRPY([0.01, 0.0, -0.01], [0.0, 0.0, 0.0])

        thumbPressToWorld = transformUtils.concatenateTransforms([pressToThumbFit, transformUtils.copyFrame(thumbFitToWorld)])
        vis.updateFrame(transformUtils.copyFrame(thumbPressToWorld), 'thumb press target frame', scale=0.1, parent=drill, visible=True)

        self.drillDemo.thumbPressToHandFrame = transformUtils.concatenateTransforms([thumbPressToWorld, handToWorld.GetLinearInverse()])

        print('thumb press fit to hand frame:', self.drillDemo.thumbPressToHandFrame.GetPosition(), transformUtils.rollPitchYawFromTransform(self.drillDemo.thumbPressToHandFrame))


    def fitDrillWall(self, polyData, points):



        if self.useLocalPlaneFit:

            planePoints, normal = segmentation.applyLocalPlaneFit(polyData, points[0], searchRadius=np.linalg.norm(points[1] - points[0]), searchRadiusEnd=1.0)

            obj = vis.updatePolyData(planePoints, 'wall plane points', color=[0,1,0], visible=False)
            obj.setProperty('Point Size', 7)

            viewDirection = segmentation.SegmentationContext.getGlobalInstance().getViewDirection()
            if np.dot(normal, viewDirection) < 0:
                normal = -normal

            origin = segmentation.computeCentroid(planePoints)

        else:

            viewDirection = segmentation.SegmentationContext.getGlobalInstance().getViewDirection()

            if self.useVoxelGrid:
                polyData = segmentation.applyVoxelGrid(polyData, leafSize=0.01)
                vis.updatePolyData(polyData, 'voxel points')

            polyData = segmentation.cropToSphere(polyData, points[0], np.linalg.norm(points[1] - points[0]))

            vis.updatePolyData(polyData, 'crop points')
            polyData, normal = segmentation.applyPlaneFit(polyData, 0.005, expectedNormal=viewDirection)

            vis.updatePolyData(polyData, 'fit points')

            polyData = segmentation.thresholdPoints(polyData, 'dist_to_plane', [-0.005, 0.005])
            vis.updatePolyData(polyData, 'wall fit points')

            origin = points[0]


        zaxis = [0,0,1]
        xaxis = normal
        yaxis = np.cross(zaxis, xaxis)
        yaxis /= np.linalg.norm(yaxis)
        zaxis = np.cross(xaxis, yaxis)
        zaxis /= np.linalg.norm(zaxis)

        t = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
        t.PostMultiply()
        t.Translate(origin)

        if self.useLocalPlaneFit:
            planePoints = segmentation.labelPointDistanceAlongAxis(planePoints, zaxis, origin=origin, resultArrayName='dist_along_z')
            planePoints = segmentation.labelPointDistanceAlongAxis(planePoints, yaxis, origin=origin, resultArrayName='dist_along_y')
            zdist = vnp.getNumpyFromVtk(planePoints, 'dist_along_z')
            ydist = vnp.getNumpyFromVtk(planePoints, 'dist_along_y')
            height = zdist.max() - zdist.min()
            width = ydist.max() - ydist.min()
        else:
            height = 1.0
            width = 1.0


        wall = self.drillDemo.spawnWallAffordanceNew(t, width, height)

        pickPointOnPlane = segmentation.projectPointToPlane(points[0], origin, normal)

        t = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
        t.PostMultiply()
        t.Translate(pickPointOnPlane)
        vis.updateFrame(t, 'drill wall pick point', parent=segmentation.getDebugFolder(), scale=0.2, visible=False)


        pickPoint2OnPlane = segmentation.projectPointToPlane(points[1], origin, normal)
        radius = np.linalg.norm(pickPoint2OnPlane - pickPointOnPlane)

        self.drillDemo.spawnDrillCircle(t, radius)

        pose = transformUtils.poseFromTransform(t)
        self.appendMessage('pick point: %r' % list(pickPointOnPlane))
        self.appendMessage('')
        self.appendMessage('drill wall:')
        self.appendMessage('origin: %r' % list(pose[0]))
        self.appendMessage('quat: %r' % list(pose[1]))
        self.appendMessage('rpy: %r' % list(transformUtils.quaternionToRollPitchYaw(pose[1])))


class DrillTaskPanel(TaskUserPanel):

    def __init__(self, drillDemo):

        TaskUserPanel.__init__(self, windowTitle='Drill Task')

        self.drillDemo = drillDemo

        self.fitter = DrillImageFitter(self.drillDemo)
        self.fitter.appendMessage = self.appendMessage
        self.initImageView(self.fitter.imageView)

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()

        self.drillFrame = None
        self.drillFrameCallback = None


    def fitDrill(self):
        self.fitter.imagePicker.numberOfPoints = 1
        self.fitter.pointCloudSource = 'lidar'
        self.fitter.fitFunc = self.fitter.fitDrill

    def fitDrillOnTable(self):
        self.fitter.imagePicker.numberOfPoints = 2
        self.fitter.pointCloudSource = 'lidar'
        self.fitter.fitFunc = self.fitter.fitDrillOnTable

    def fitDrillWall(self):
        self.fitter.imagePicker.numberOfPoints = 2
        self.fitter.pointCloudSource = 'lidar'
        self.fitter.fitFunc = self.fitter.fitDrillWall

    def fitShelf(self):
        self.fitter.imagePicker.numberOfPoints = 2
        self.fitter.pointCloudSource = 'lidar'
        self.fitter.fitFunc = self.fitter.fitShelf

    def fitDrillButtonPress(self):
        self.fitter.imagePicker.numberOfPoints = 2
        self.fitter.pointCloudSource = 'stereo'
        self.fitter.fitFunc = self.fitter.fitDrillButtonPress

    def spawnWall(self):

        self.drillDemo.spawnWallAffordanceTest(self.params.getProperty('wall offset x'), self.params.getProperty('wall offset y'),
          self.params.getProperty('wall yaw'), self.params.getProperty('target height'), self.params.getProperty('target radius'))

        self.setDrillTargetToCircleStart()

    def spawnDrill(self):
        self.drillDemo.spawnDrillAffordanceTest()

    def updateDrillInHand(self):
        if not self.drillDemo.affordanceUpdater.hasAffordance('drill'):
            return

        self.ungraspDrill()
        self.graspDrill()

    def graspDrill(self):

        if not om.findObjectByName('drill'):
            self.spawnDrill()

        self.ungraspDrill()

        self.drillDemo.moveDrillToHandNew()
        self.drillDemo.affordanceUpdater.graspAffordance('drill', self.drillDemo.graspingHand)


        drillFrame = om.findObjectByName('drill').getChildFrame()
        if drillFrame != self.drillFrame:
            self.drillFrameCallback = drillFrame.connectFrameModified(self.onDrillFrameModified)
            self.drillFrame = drillFrame

    def onDrillFrameModified(self, frame):
        pass

    def computeWallStanceFrame(self, planFootsteps=False):
        stanceX = -self.params.getProperty('wall offset x')
        stanceY = -self.params.getProperty('wall offset y')
        stanceYaw = -self.params.getProperty('wall yaw')
        self.drillDemo.computeWallStanceFrame(stanceX, stanceY, stanceYaw)

        if planFootsteps:
            self.drillDemo.planFootsteps( om.findObjectByName('wall stance frame').transform )

    def ungraspDrill(self):
        self.drillDemo.affordanceUpdater.ungraspAffordance('drill')

        drillFrame = om.findObjectByName('drill').getChildFrame()
        if drillFrame and self.drillFrameCallback:
            drillFrame.disconnectFrameModified(self.drillFrameCallback)

    def setDrillTargetToCircleStart(self):
        drillCircle = om.findObjectByName('drill circle')
        assert drillCircle
        radius = drillCircle.getProperty('Radius')

        self.params.setProperty('drilling depth', self.drillDemo.retractBitDepthNominal)
        self.params.setProperty('drilling horiz offset', self.drillDemo.drillCutTailLength)
        self.params.setProperty('drilling vert offset', radius + self.drillDemo.drillCutTailLength)

    def setDrillTargetToKnockOut(self):
        self.params.setProperty('drilling depth', self.drillDemo.retractBitDepthNominal)
        self.params.setProperty('drilling horiz offset', 0.0)
        self.params.setProperty('drilling vert offset', 0.0)

    def setDrillTargetToDrillBit(self):

        depth, horiz, vert = self.drillDemo.getDrillBitOffsetFromCircle()
        self.params.setProperty('drilling depth', depth)
        self.params.setProperty('drilling horiz offset', horiz)
        self.params.setProperty('drilling vert offset', vert)

    def updateDrillTargetPosition(self):
        self.drillDemo.updateDrillTargetFrame(self.params.getProperty('drilling depth'), self.params.getProperty('drilling horiz offset'), self.params.getProperty('drilling vert offset'))

    def updateDrillTargetDepth(self):
        depth, horiz, vert = self.drillDemo.getDrillTargetOffsetFromCircle()
        self.drillDemo.updateDrillTargetFrame(self.params.getProperty('drilling depth'), horiz, vert)

    def planDrillAlign(self):
        self.drillDemo.planDrill(inPlane=False, inLine=False, translationSpeed=self.drillDemo.drillTrajectoryMetersPerSecondFast, jointSpeed=30)

    def planDrillIn(self):
        self.drillDemo.planDrill(inPlane=False, inLine=True, translationSpeed=self.drillDemo.drillTrajectoryMetersPerSecondSlow, jointSpeed=self.drillDemo.drillTrajectoryMaxDegreesPerSecond)

    def planDrillOut(self):
        self.params.setProperty('drilling depth', self.drillDemo.retractBitDepthNominal)
        self.drillDemo.planDrill(inPlane=False, inLine=True, translationSpeed=self.drillDemo.drillTrajectoryMetersPerSecondSlow, jointSpeed=self.drillDemo.drillTrajectoryMaxDegreesPerSecond)

    def setDefaultDrillInDepth(self):
        self.params.setProperty('drilling depth', 0.01)

    def setDefaultKnockOutDepth(self):
        self.params.setProperty('drilling depth', 0.03)

    def planDrillMove(self):
        self.drillDemo.planDrill(inPlane=False, inLine=True, translationSpeed=self.drillDemo.drillTrajectoryMetersPerSecondSlow, jointSpeed=self.drillDemo.drillTrajectoryMaxDegreesPerSecond)

    def planThumbPressPrep(self):

        self.drillDemo.addThumbTargetFramesFromModel()
        self.drillDemo.setDrillPressTargetFrame(depthOffset=-0.07, horizOffset=0.0, vertOffset=0.0)
        self.drillDemo.planDrillButtonPress(30, quasiStaticShrinkFactor=0.5)

    def planThumbPressPrepClose(self):
        self.drillDemo.setDrillPressTargetFrame(depthOffset=-0.025, horizOffset=0.0, vertOffset=-0.005)
        self.drillDemo.planDrillButtonPress(3)

    def planThumbPressButton(self):
        self.drillDemo.setDrillPressTargetFrame(depthOffset=0.02, horizOffset=0.0, vertOffset=0.0)
        self.drillDemo.planDrillButtonPress(5)

    def planThumbPressExit(self):
        self.drillDemo.setDrillPressTargetFrame(depthOffset=-0.07, horizOffset=0.0, vertOffset=0.0)
        self.drillDemo.planDrillButtonPress(15)

    def planThumbPressMove(self):
        self.drillDemo.planDrillButtonPress(5)

    def addButtons(self):

        self.addManualButton('Fit drill', self.fitDrill)
        self.addManualButton('Fit drill on table', self.fitDrillOnTable)
        self.addManualButton('Fit drill button press', self.fitDrillButtonPress)
        self.addManualButton('Fit drill wall', self.fitDrillWall)
        self.addManualSpacer()
        self.addManualButton('Spawn wall', self.spawnWall)
        self.addManualButton('Spawn drill', self.spawnDrill)
        self.addManualSpacer()
        self.addManualButton('Reach', self.drillDemo.planReachNew)
        self.addManualButton('Grasp', self.drillDemo.planGraspNew)
        self.addManualSpacer()
        self.addManualButton('Lock in hand', self.graspDrill)
        self.addManualButton('Unlock in hand', self.ungraspDrill)
        self.addManualSpacer()
        self.addManualButton('Walk with drill posture', self.drillDemo.planWalkWithDrillPosture)
        self.addManualButton('Drill into wall prep', self.drillDemo.planDrillIntoWallPrep)
        self.addManualSpacer()
        self.addManualButton('Set drill target to start', self.setDrillTargetToCircleStart)
        self.addManualButton('Set drill target to bit', self.setDrillTargetToDrillBit)
        self.addManualSpacer()
        self.addManualButton('Plan thumb move', self.planThumbPressMove)
        self.addManualButton('Plan thumb exit', self.planThumbPressExit)
        self.addManualSpacer()
        self.addManualButton('Plan drill align', self.planDrillAlign)
        self.addManualButton('Plan drill in', self.planDrillIn)
        self.addManualButton('Plan drill move', self.planDrillMove)
        self.addManualButton('Plan drill out', self.planDrillOut)
        self.addManualButton('Plan drill circle', self.drillDemo.planDrillCircle)
        self.addManualSpacer()
        self.addManualButton('Commit Manip', self.drillDemo.commitManipPlan)
        self.addManualSpacer()
        self.addManualButton('Finger grasp', self.drillDemo.fingerGrasp)
        self.addManualButton('Finger tighten', self.drillDemo.fingerGraspTighten)

    def getSide(self):
        return self.params.getPropertyEnumValue('Drill Hand').lower()

    def addDefaultProperties(self):

        self.params.addProperty('Drill Hand', 1, attributes=om.PropertyAttributes(enumNames=['Left', 'Right']))
        #todo: use this as default
        self.drillDemo.graspingHand = self.getSide()

        self.params.addProperty('drill grasp yaw', self.drillDemo.drillGraspYaw, attributes=om.PropertyAttributes(minimum=-360, maximum=360))
        self.params.addProperty('wall yaw', 0, attributes=om.PropertyAttributes(minimum=-90, maximum=90, hidden=True))
        self.params.addProperty('wall offset x', 0.72, attributes=om.PropertyAttributes(minimum=0.0, maximum=2.0, decimals=2, singleStep=0.01, hidden=False))
        self.params.addProperty('wall offset y', 0.22, attributes=om.PropertyAttributes(minimum=-2.0, maximum=2.0, decimals=2, singleStep=0.01, hidden=False))
        self.params.addProperty('target height', 1.0, attributes=om.PropertyAttributes(minimum=0, maximum=2.0, decimals=2, singleStep=0.05, hidden=True))
        self.params.addProperty('target radius', 0.16, attributes=om.PropertyAttributes(minimum=0, maximum=1.0, decimals=2, singleStep=0.01, hidden=True))

        self.params.addProperty('drilling depth', self.drillDemo.retractBitDepthNominal, attributes=om.PropertyAttributes(minimum=-2.0, maximum=2.0, decimals=4, singleStep=0.0025))
        self.params.addProperty('drilling horiz offset', 0.0, attributes=om.PropertyAttributes(minimum=-2.0, maximum=2.0, decimals=4, singleStep=0.0025, hidden=True))
        self.params.addProperty('drilling vert offset', 0.16, attributes=om.PropertyAttributes(minimum=-2.0, maximum=2.0, decimals=4, singleStep=0.0025, hidden=True))


    def onPropertyChanged(self, propertySet, propertyName):

        propertyName = str(propertyName)

        if propertyName in ('drilling horiz offset', 'drilling vert offset'):
            self.updateDrillTargetPosition()

        elif propertyName == 'drilling depth':
            self.updateDrillTargetDepth()

        elif propertyName.startswith('drill hand '):
            self.updateDrillInHand()

        elif propertyName == 'Drill Hand':
            self.drillDemo.graspingHand = self.getSide()
            self.taskTree.removeAllTasks()
            self.addTasks()

        elif propertyName == 'drill grasp yaw':
            self.drillDemo.drillGraspYaw = self.params.getProperty('drill grasp yaw')


    def addTasks(self):

        # some helpers
        self.folder = None
        def addTask(task, parent=None):
            parent = parent or self.folder
            self.taskTree.onAddTask(task, copy=False, parent=parent)
        def addFunc(name, func, parent=None):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)
        def addFolder(name, parent=None):
            self.folder = self.taskTree.addGroup(name, parent=parent)
            return self.folder

        def addManipTask(name, planFunc, userPrompt=False):

            prevFolder = self.folder
            addFolder(name, prevFolder)
            addFunc('plan', planFunc)
            if not userPrompt:
                addTask(rt.CheckPlanInfo(name='check manip plan info'))
            else:
                addTask(rt.UserPromptTask(name='approve manip plan', message='Please approve manipulation plan.'))
            addFunc('execute manip plan', self.drillDemo.commitManipPlan)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'))
            self.folder = prevFolder


        self.taskTree.removeAllTasks()
        side = self.getSide()
        pressSide = self.drillDemo.ikPlanner.flipSide(side)

        useFingerGrasp = self.drillDemo.useFingerGrasp

        ###############
        # add the tasks

        # prep
        addFolder('Prep')
        addTask(rt.CloseHand(name='close left hand', side='Left'))
        addTask(rt.CloseHand(name='close right hand', side='Right'))

        # fit drill
        addFolder('Fit drill')
        addFunc('fit drill', self.fitDrill)
        addTask(rt.UserPromptTask(name='adjust drill', message='Please fit and approve drill affordance.'))
        addFunc('compute walk target', self.drillDemo.computeDrillGraspStanceFrame)


        # walk to drill
        addFolder('Walk')
        addTask(rt.RequestFootstepPlan(name='plan walk to drill', stanceFrameName='drill grasp stance frame'))
        addTask(rt.UserPromptTask(name='approve footsteps', message='Please approve footstep plan.'))
        addTask(rt.SetNeckPitch(name='set neck position', angle=35))
        addTask(rt.CommitFootstepPlan(name='walk to drill', planName='drill shelf stance frame footstep plan'))
        addTask(rt.WaitForWalkExecution(name='wait for walking'))

        # refit drill
        addFolder('Re-fit drill')
        addTask(rt.UserPromptTask(name='fit drill', message='Please fit and approve drill affordance.'))

        # pick up drill
        addFolder('Pick up drill')
        addManipTask('raise arm', self.drillDemo.planPreGrasp, userPrompt=True)
        addManipTask('reach to drill', self.drillDemo.planReachNew, userPrompt=True)
        addTask(rt.OpenHand(name='open hand', side=side.capitalize()))
        addManipTask('grasp drill', self.drillDemo.planGraspNew, userPrompt=True)
        addTask(rt.UserPromptTask(name='Confirm hand position', message='Please verify hand position for grasping'), parent=None)

        if useFingerGrasp:
            addFunc('finger grasp', self.drillDemo.fingerGrasp)
        else:
            addTask(rt.CloseHand(name='close hand', side=side.capitalize()))
            addTask(rt.DelayTask(name='wait to regrasp', delayTime=2.0))
            addTask(rt.CloseHand(name='re-close grip hand', side=side.capitalize()))

        #addManipTask('lift drill', self.drillDemo.planLiftNew, userPrompt=True)
        addManipTask('raise drill', self.drillDemo.planDrillRaiseNew, userPrompt=True)

        if useFingerGrasp:
            addFunc('finger grasp tighten', self.drillDemo.fingerGraspTighten)
        else:
            addTask(rt.CloseHand(name='re-close grip hand', side=side.capitalize()))
            addTask(rt.UserPromptTask(name='adjust drill in hand', message='Please adjust drill fit in hand'))

        # walk back
        addFolder('Walk back')
        addFunc('drop nav goal', self.drillDemo.planNavigationGoalAwayFromShelf)
        addTask(rt.UserPromptTask(name='Approve manual footstep plan', message='Please approve and manually execute footstep plan.'), parent=None)

        # turn on drill
        addFolder('Turn on drill')
        addTask(rt.OpenHand(name='open press hand', side=pressSide.capitalize()))
        #addManipTask('pregrasp drill butt', self.drillDemo.planHandRaiseForDrillButtPreGrasp, userPrompt=True)
        #addManipTask('grasp drill butt', self.drillDemo.planHandRaiseForDrillButtGrasp, userPrompt=True)
        addManipTask('thumb press prep', self.planThumbPressPrep, userPrompt=True)
        addFunc('fit drill button press', self.fitDrillButtonPress)
        addTask(rt.UserPromptTask(name='approve fit', message='Please fit and approve drill button press'), parent=None)
        #addManipTask('thumb prep closer', self.planThumbPressPrepClose, userPrompt=True)
        addManipTask('thumb press', self.planThumbPressButton, userPrompt=True)
        addManipTask('thumb press exit', self.planThumbPressExit, userPrompt=True)
        addTask(rt.UserPromptTask(name='verify drill is on', message='Please verify that drill is on'), parent=None)
        #addManipTask('thumb press exit', self.drillDemo.planHandRaiseForDrillButtGrasp, userPrompt=True)
        #addManipTask('thumb press exit 2', self.drillDemo.planHandRaiseForDrillButtPreGrasp, userPrompt=True)

        if useFingerGrasp:
            addFunc('finger grasp tighten', self.drillDemo.fingerGraspTighten)
        else:
            addTask(rt.CloseHand(name='re-close grip hand', side=side.capitalize()))

        addManipTask('hands down', self.drillDemo.planHandsDown, userPrompt=True)
        addTask(rt.CloseHand(name='close press hand', side=pressSide.capitalize()))

        # walk toward wall
        addFolder('Walk toward wall')
        addTask(rt.UserPromptTask(name='Walk back', message='Please walk for wall in sight'), parent=None)

        # fit wall
        addFolder('Fit wall')
        addFunc('fit drill wall', self.fitDrillWall)
        addTask(rt.UserPromptTask(name='approve fit', message='Please fit and approve drill wall'))

        # walk to wall
        addFolder('Walk to wall')
        addFunc('compute wall stance frame', self.computeWallStanceFrame)
        addTask(rt.RequestFootstepPlan(name='plan walk to wall', stanceFrameName='wall stance frame'))
        addTask(rt.UserPromptTask(name='approve footsteps', message='Please approve footstep plan.'))
        addTask(rt.CommitFootstepPlan(name='walk to wall', planName='wall stance frame footstep plan'))
        addTask(rt.WaitForWalkExecution(name='wait for walking'))

        # refit wall
        addFolder('Re-fit wall')
        addFunc('fit drill wall', self.fitDrillWall)
        addTask(rt.UserPromptTask(name='approve fit', message='Please fit and approve drill wall'))

        # drill into wall
        addFolder('Drill wall prep')
        addFunc('reset drill target', self.setDrillTargetToCircleStart)
        addManipTask('drill prep posture', self.drillDemo.planDrillIntoWallPrep, userPrompt=True)
        addManipTask('move drill to wall', self.planDrillAlign, userPrompt=True)

        # find drill depth
        addFolder('Set drill depth')
        addFunc('set drill in depth', self.setDefaultDrillInDepth)
        addManipTask('drill in', self.planDrillIn, userPrompt=True)
        addTask(rt.UserPromptTask(name='verify drill depth', message='Please verify drill depth'))
        addTask(rt.UserPromptTask(name='verify drill in hand fit', message='Please verify drill in hand fit'))

        # drill circle
        addFolder('Drill circle')
        addManipTask('drill circle', self.drillDemo.planDrillCircle, userPrompt=True)
        addFunc('reset drill target', self.setDrillTargetToDrillBit)
        addTask(rt.UserPromptTask(name='verify cut', message='Please verify the cut.'))

        # knock out wall
        addFolder('Knock out')
        addFunc('reset drill target', self.setDrillTargetToDrillBit)
        addManipTask('drill out', self.planDrillOut, userPrompt=True)
        addManipTask('drill prep posture', self.drillDemo.planDrillIntoWallPrep, userPrompt=True)
        addFunc('set drill target to knock out', self.setDrillTargetToKnockOut)
        addManipTask('move drill to wall', self.planDrillAlign, userPrompt=True)
        addFunc('set knock out depth', self.setDefaultKnockOutDepth)
        addManipTask('drill in', self.planDrillIn, userPrompt=True)
        addTask(rt.UserPromptTask(name='confirm knock out', message='Please confirm knock out.'))

        # retract
        addFolder('Retract')
        addFunc('reset drill target', self.setDrillTargetToDrillBit)
        addManipTask('drill out', self.planDrillOut, userPrompt=True)
        addManipTask('prepare to drop drill', self.drillDemo.planDrillDrop, userPrompt=True)
        addTask(rt.UserPromptTask(name='approve drill drop', message='Please verify the drill is ready to be dropped'))
        addTask(rt.OpenHand(name='open hand', side=side.capitalize()))
        addTask(rt.DelayTask(name='wait to close hand', delayTime=3.0))
        addTask(rt.CloseHand(name='close left hand', side=side.capitalize()))
        addManipTask('return to nominal posture', self.drillDemo.planNominal, userPrompt=True)
        #addManipTask('drill prep posture', self.drillDemo.planDrillIntoWallPrep, userPrompt=True)
        #addManipTask('tuck for walking', self.drillDemo.planWalkWithDrillPosture, userPrompt=True)
