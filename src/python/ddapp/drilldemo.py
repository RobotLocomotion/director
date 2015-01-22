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
from ddapp import planplayback
from ddapp import affordancegraspupdater
from ddapp import segmentationpanel

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
        self.segmentationpanel.init() # TODO: check with Pat. I added dependency on segmentationpanel, but am sure its appropriate

        defaultGraspingHand = "left"
        self.setGraspingHand(defaultGraspingHand)

        # live operation flags
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

        self.drill = Drill()
        self.wall = Wall()

        # params:
        self.reachDepth = 0.12 # depth to reach to before going for grasp
        self.cutLength = 0.05 # length to cut each time
        self.retractBitDepthNominal = -0.055 # depth to move drill away from wall
        self.goalThreshold = 0.05 # how close we need to get to the cut goal (the triangle corners

        #extraModels = [self.robotModel, self.playbackRobotModel, self.teleopRobotModel]
        #self.affordanceUpdater  = affordancegraspupdater.AffordanceGraspUpdater(self.playbackRobotModel, extraModels)

        extraModels = [self.robotModel]
        self.affordanceUpdater  = affordancegraspupdater.AffordanceGraspUpdater(self.robotModel, extraModels)

        # These changes are all that are required to run with different combinations
        if ( self.drill.model == 'dewalt_barrel' ):
            print "Using a Dewalt Barrel Drill"
            self.wall.relativeStanceXYZ = [-0.3, 0.75, 0]
            self.wall.relativeStanceRPY = [0, 0, -90]
            self.wall.relativeStanceFarXYZ = [-0.3, 1.25, 0]
            self.wall.relativeStanceFarRPY = [0, 0, -90]
            #if ( self.graspingHand == 'right' ):
            #    self.drill.faceToDrillRotation = -90

        if ( self.graspingHand == 'right' ):
            print "Planning with drill in the right hand"
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
            print "Received detection not from Drill Wall, ignoring"
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

    def planPreGrasp(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'arm up pregrasp', side=self.graspingHand)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

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
        print 'waiting for plan animation:', planElapsedTime
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
            print " "

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
