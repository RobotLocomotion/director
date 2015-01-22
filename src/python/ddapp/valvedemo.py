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

import drc as lcmdrc

from PythonQt import QtCore, QtGui


class ValvePlannerDemo(object):

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
        self.graspingObject='valve'
        self.graspingHand='left'

        
        # live operation flags
        self.useFootstepPlanner = True
        self.visOnly = True
        self.planFromCurrentRobotState = True
        useDevelopment = True
        if (useDevelopment):
            self.visOnly = True
            self.planFromCurrentRobotState = False

        self.optionalUserPromptEnabled = False
        self.requiredUserPromptEnabled = True
        self.constraintSet = None

        self.plans = []

        self.faceTransformLocal = None
        self.facePath = []

        self.scribeInAir = False
        self.palmInAngle = 30 # how much should the palm face the axis - 0 not at all, 90 entirely
        self.scribeRadius = None
        self.useLidar = True # else use stereo depth

        # IK server speed:
        self.speedLow = 10
        self.speedHigh = 30

        if (useDevelopment): # for simulated dev
            self.speedLow = 60
            self.speedHigh = 60

        # reach to center and back - for palm point
        self.clenchFrameXYZ = [0.0, 0.0, -0.1]
        self.clenchFrameRPY = [90, 0, 180]
        self.reachDepth = -0.12 # distance away from valve for palm face on approach reach

        # top level switch between BDI (locked base) and MIT (moving base and back)
        self.lockBack = False
        self.lockBase = False

        self.setupStance()

    def setupStance(self):

        if (self.graspingObject == 'valve'):
            self.nextScribeAngleInitial = -60 # reach 60 degrees left of the valve spoke
            self.turnAngle=60
            if self.scribeInAir:
                self.relativeStanceXYZInitial = [-0.6, -0.2, 0.0] # stand further away when scribing in air
            else:
                self.relativeStanceXYZInitial = [-0.48, -0.2, 0.0]
            self.relativeStanceRPYInitial = [0, 0, 16]
        else:
            self.nextScribeAngleInitial = 0 # reach right into the valve axis
            self.turnAngle=90
            if self.scribeInAir:
                self.relativeStanceXYZInitial = [-0.6, -0.4, 0.0] # stand further away when scribing in air
            else:
                self.relativeStanceXYZInitial = [-0.48, -0.4, 0.0]
            self.relativeStanceRPYInitial = [0, 0, 16]

        if (self.graspingHand is 'left'): # -1 = anticlockwise (left, default) | 1 = clockwise
            self.scribeDirection = -1
        else:
            self.scribeDirection = 1



    def resetTurnPath(self):
        for obj in om.getObjects():
            if obj.getProperty('Name') == 'face frame desired':
                om.removeFromObjectModel(obj)
        for obj in om.getObjects():
            if obj.getProperty('Name') == 'face frame desired path':
                om.removeFromObjectModel(obj)

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



    ### Valve Focused Functions ######################################################################
    def segmentValveWallAuto(self, mode):
        om.removeFromObjectModel(om.findObjectByName('affordances'))

        if (self.useLidar is True):
            vis.updatePolyData(segmentation.getCurrentRevolutionData(), 'pointcloud snapshot', parent='segmentation')
        else:
            vis.updatePolyData(segmentation.getDisparityPointCloud(4), 'pointcloud snapshot', parent='segmentation')

        self.affordanceFitFunction(.195, mode=mode)

    def computeValveStanceFrame(self):
        objectTransform = transformUtils.copyFrame( self.clenchFrame.transform )
        self.relativeStanceTransform = transformUtils.copyFrame( transformUtils.frameFromPositionAndRPY( self.relativeStanceXYZ , self.relativeStanceRPY ) )
        robotStance = self.computeRobotStanceFrame(objectTransform, self.relativeStanceTransform)
        self.stanceFrame = vis.updateFrame(robotStance, 'valve grasp stance', parent=self.valveAffordance, visible=False, scale=0.2)
        self.stanceFrame.addToView(app.getDRCView())

    def spawnValveFrame(self, robotModel, height):

        position = [0.7, 0.22, height]
        rpy = [180, -90, 0]
        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.computeGroundFrame(robotModel))
        return t

    def spawnValveAffordance(self):
        self.graspingObject = 'valve'
        spawn_height = 1.2192 # 4ft
        radius = 0.19558 # nominal initial value. 7.7in radius metal valve
        zwidth = 0.02
        thickness = 0.0254 # i think zwidth and thickness are duplicates

        valveFrame = self.spawnValveFrame(self.robotModel, spawn_height)
        folder = om.getOrCreateContainer('affordances')
        z = DebugData()
        #z.addLine ( np.array([0, 0, -thickness]) , np.array([0, 0, thickness]), radius=radius)
        z.addTorus( radius, 0.127 )
        z.addLine(np.array([0,0,0]), np.array([radius-zwidth,0,0]), radius=zwidth) # main bar
        valveMesh = z.getPolyData()

        self.valveAffordance = vis.showPolyData(valveMesh, 'valve', color=[0.0, 1.0, 0.0], cls=affordanceitems.FrameAffordanceItem, parent=folder, alpha=0.3)
        self.valveAffordance.actor.SetUserTransform(valveFrame)
        self.valveFrame = vis.showFrame(valveFrame, 'valve frame', parent=self.valveAffordance, visible=False, scale=0.2)

        params = dict(radius=radius, length=zwidth, xwidth=radius, ywidth=radius, zwidth=zwidth,
                      otdf_type='steering_cyl', friendly_name='valve')
        self.valveAffordance.setAffordanceParams(params)
        self.valveAffordance.updateParamsFromActorTransform()

    def spawnValveLeverAffordance(self):
        self.graspingObject = 'lever'
        spawn_height = 1.06 # 3.5ft
        pipe_radius = 0.01
        lever_length = 0.33

        valveFrame = self.spawnValveFrame(self.robotModel, spawn_height)
        folder = om.getOrCreateContainer('affordances')
        z = DebugData()
        z.addLine([0,0,0], [ lever_length , 0, 0], radius=pipe_radius)
        valveMesh = z.getPolyData()

        self.valveAffordance = vis.showPolyData(valveMesh, 'lever', color=[0.0, 1.0, 0.0], cls=affordanceitems.FrameAffordanceItem, parent=folder, alpha=0.3)
        self.valveAffordance.actor.SetUserTransform(valveFrame)
        self.valveFrame = vis.showFrame(valveFrame, 'lever frame', parent=self.valveAffordance, visible=False, scale=0.2)

        otdfType = 'lever_valve'
        params = dict( radius=pipe_radius, length=lever_length, friendly_name=otdfType, otdf_type=otdfType)
        self.valveAffordance.setAffordanceParams(params)
        self.valveAffordance.updateParamsFromActorTransform()

    def findAffordance(self):
        self.setupAffordanceParams()
        if (self.graspingObject is 'valve'):
            self.findValveAffordance()
        else:
            self.findValveLeverAffordance()

    def setupAffordanceParams(self):
        self.setupStance()

        self.relativeStanceXYZ = self.relativeStanceXYZInitial
        self.relativeStanceRPY = self.relativeStanceRPYInitial
        self.nextScribeAngle = self.nextScribeAngleInitial

        # mirror stance and rotation direction for right hand:
        if (self.graspingHand is 'right'):
            self.relativeStanceXYZ[1] = -self.relativeStanceXYZ[1]
            self.relativeStanceRPY[2] = -self.relativeStanceRPY[2]
            self.nextScribeAngle = -self.nextScribeAngle


    def findValveAffordance(self):

        self.valveAffordance = om.findObjectByName('valve')
        self.valveFrame = om.findObjectByName('valve frame')

        self.scribeRadius = self.valveAffordance.params.get('radius')# for pointer this was (radius - 0.06)

        self.computeClenchFrame()
        self.computeValveStanceFrame()

        self.frameSync = vis.FrameSync()
        self.frameSync.addFrame(self.valveFrame)
        self.frameSync.addFrame(self.clenchFrame)
        self.frameSync.addFrame(self.stanceFrame)


    def findValveLeverAffordance(self):

        self.valveAffordance = om.findObjectByName('lever')
        self.valveFrame = om.findObjectByName('lever frame')

        # length of lever is equivalent to radius of valve
        self.scribeRadius = self.valveAffordance.params.get('length') - 0.10

        self.computeClenchFrame()
        self.computeValveStanceFrame()

        self.frameSync = vis.FrameSync()
        self.frameSync.addFrame(self.valveFrame)
        self.frameSync.addFrame(self.clenchFrame)
        self.frameSync.addFrame(self.stanceFrame)

    def computeClenchFrame(self):
        t = transformUtils.frameFromPositionAndRPY(self.clenchFrameXYZ, self.clenchFrameRPY)
        t_copy = transformUtils.copyFrame(t)
        t_copy.Concatenate(self.valveFrame.transform)
        self.clenchFrame = vis.updateFrame(t_copy, 'valve clench frame', parent=self.valveAffordance, visible=False, scale=0.2)
        self.clenchFrame.addToView(app.getDRCView())

    def computeTouchFrame(self, touchValve):
        if touchValve:
            faceDepth = 0.0
        else:
            faceDepth = self.reachDepth

        assert self.valveAffordance

        t = transformUtils.frameFromPositionAndRPY([0,faceDepth,0], [0,0,0])

        position = [ self.scribeRadius*math.cos( math.radians( self.nextScribeAngle )) ,  self.scribeRadius*math.sin( math.radians( self.nextScribeAngle ))  , 0]
        # roll angle governs how much the palm points along towards the rotation axis
        # yaw ensures thumb faces the axis
        if (self.graspingObject is 'valve'):
            # valve, left and right
            rpy = [90+self.palmInAngle, 0, (270+self.nextScribeAngle)]
        else:
            if (self.graspingHand is 'left'): # lever left
                rpy = [90, 0, (180+self.nextScribeAngle)]
            else:
                rpy = [90, 0, self.nextScribeAngle]

        t2 = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(t2)
        self.faceTransformLocal = transformUtils.copyFrame(t)

        t.Concatenate(self.valveFrame.transform)
        self.faceFrameDesired = vis.showFrame(t, 'face frame desired', parent=self.valveAffordance, visible=False, scale=0.2)

    def drawFacePath(self):
      
        path = DebugData()
        for i in range(1,len(self.facePath)):
          p0 = self.facePath[i-1].GetPosition()
          p1 = self.facePath[i].GetPosition()
          path.addLine ( np.array( p0 ) , np.array(  p1 ), radius= 0.005)
          
        pathMesh = path.getPolyData()
        self.pointerTipLinePath = vis.showPolyData(pathMesh, 'face frame desired path', color=[0.0, 0.3, 1.0], cls=affordanceitems.AffordanceItem, parent=self.valveAffordance, alpha=0.6)
        self.pointerTipLinePath.actor.SetUserTransform(self.valveFrame.transform)

        
    ### End Valve Focused Functions ###############################################################
    ### Planning Functions ###############################################################
    def planNominal(self):
        startPose = self.getPlanningStartPose()
        self.standPlan = self.ikPlanner.computeNominalPlan(startPose)
        self.addPlan(self.standPlan)

    # These are operational conveniences:
    def planFootstepsToStance(self):
        self.planFootsteps(self.stanceFrame.transform)

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
        self.computeTouchFrame(False) # 0 = not in contact
        self.computeTouchPlan()

    def planGrasp(self):
        self.computeTouchFrame(True)
        self.computeTouchPlan()

    def computeTouchPlan(self):
        # new full 6 dof constraint:
        startPose = self.getPlanningStartPose()
        self.constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.faceFrameDesired, lockBase=self.lockBase, lockBack=self.lockBack)
        endPose, info = self.constraintSet.runIk()

        self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedLow
        self.planTrajectory()
        self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedHigh

    def planValveTurn(self, turnDegrees=360):
        # 10deg per sample
        numberOfSamples = int(round(turnDegrees/10.0))

        self.facePath = []
        self.resetTurnPath()

        degreeStep = float(turnDegrees) / numberOfSamples
        tipMode = False if self.scribeInAir else True

        self.computeTouchFrame(tipMode)
        self.initConstraintSet(self.faceFrameDesired)
        self.facePath.append(self.faceTransformLocal)

        for i in xrange(numberOfSamples):
            self.nextScribeAngle += self.scribeDirection*degreeStep
            self.computeTouchFrame(tipMode)
            self.appendPositionOrientationConstraintForTargetFrame(self.faceFrameDesired, i+1)
            self.facePath.append(self.faceTransformLocal)

        self.drawFacePath()
        self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedLow
        self.planTrajectory()
        self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedHigh

    def initConstraintSet(self, goalFrame):

        # create constraint set
        startPose = self.getPlanningStartPose()
        startPoseName = 'gaze_plan_start'
        endPoseName = 'gaze_plan_end'
        self.ikPlanner.addPose(startPose, startPoseName)
        self.ikPlanner.addPose(startPose, endPoseName)
        self.constraintSet = ikplanner.ConstraintSet(self.ikPlanner, [], startPoseName, endPoseName)
        self.constraintSet.endPose = startPose

        # add body constraints
        bodyConstraints = self.ikPlanner.createMovingBodyConstraints(startPoseName, lockBase=self.lockBase, lockBack=self.lockBack, lockLeftArm=self.graspingHand=='right', lockRightArm=self.graspingHand=='left')
        self.constraintSet.constraints.extend(bodyConstraints)

        # add gaze constraint - TODO: this gaze constraint shouldn't be necessary, fix
        self.graspToHandLinkFrame = self.ikPlanner.newGraspToHandFrame(self.graspingHand)
        gazeConstraint = self.ikPlanner.createGazeGraspConstraint(self.graspingHand, goalFrame, self.graspToHandLinkFrame, coneThresholdDegrees=5.0)
        self.constraintSet.constraints.insert(0, gazeConstraint)

    def appendDistanceConstraint(self):

        # add point to point distance constraint
        c = ikplanner.ik.PointToPointDistanceConstraint()
        c.bodyNameA = self.ikPlanner.getHandLink(self.graspingHand)
        c.bodyNameB = 'world'
        c.pointInBodyA = self.graspToHandLinkFrame
        c.pointInBodyB = self.valveFrame.transform
        c.lowerBound = [self.scribeRadius]
        c.upperBound = [self.scribeRadius]
        self.constraintSet.constraints.insert(0, c)

    def appendPositionOrientationConstraintForTargetFrame(self, goalFrame, t):
        positionConstraint, orientationConstraint = self.ikPlanner.createPositionOrientationGraspConstraints(self.graspingHand, goalFrame, self.graspToHandLinkFrame)
        positionConstraint.tspan = [t, t]
        orientationConstraint.tspan = [t, t]
        self.constraintSet.constraints.append(positionConstraint)
        self.constraintSet.constraints.append(orientationConstraint)

    def planTrajectory(self):
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

    def optionalUserPrompt(self, message):
        if not self.optionalUserPromptEnabled:
            return

        yield
        result = raw_input(message)
        if result != 'y':
            raise Exception('user abort.')

    def requiredUserPrompt(self, message):
        if not self.requiredUserPromptEnabled:
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

    ######### Nominal Plans and Execution  #################################################################
    def planSequence(self):

        self.cleanupFootstepPlans()
        self.resetTurnPath()

        self.planFromCurrentRobotState = False
        self.findAffordance()

        self.plans = []

        # Approach valve:
        if self.useFootstepPlanner:
            self.planFootstepsToStance()
            self.planWalking()
        else:
            self.moveRobotToStanceFrame(self.stanceFrame.transform )

        # Reach and Turn:
        self.planPreGrasp()
        self.planReach()
        self.planGrasp()
        self.planValveTurn(self.turnAngle)

        # Dereach and Stand
        self.planReach()
        self.planPreGrasp()
        self.planNominal()
        self.playSequenceNominal()


    def autonomousExecute(self):

        self.planFromCurrentRobotState = True
        self.visOnly = False

        taskQueue = AsyncTaskQueue()
        taskQueue.addTask(self.resetTurnPath)
        
        # Approach valve:
        taskQueue.addTask( functools.partial(self.segmentValveWallAuto, self.graspingObject) )
        taskQueue.addTask(self.optionalUserPrompt('Accept valve fit, continue? y/n: '))
        taskQueue.addTask(self.findAffordance)

        taskQueue.addTask(self.planFootstepsToStance)
        taskQueue.addTask(self.optionalUserPrompt('Send footstep plan. continue? y/n: '))
        taskQueue.addTask(self.commitFootstepPlan)
        taskQueue.addTask(self.requiredUserPrompt('Wait to arrive: '))

        # Reach and Grasp Valve:
        taskQueue.addTask(self.waitForCleanLidarSweepAsync)
        taskQueue.addTask( functools.partial(self.segmentValveWallAuto, self.graspingObject) )
        taskQueue.addTask(self.optionalUserPrompt('Accept valve re-fit, continue? y/n: '))
        taskQueue.addTask(self.findAffordance)

        planningFunctions = [
                    self.planPreGrasp,
                    self.planReach,
                    self.planGrasp,
                    functools.partial( self.planValveTurn, self.turnAngle),
                    self.planReach,
                    self.planPreGrasp,
                    self.planNominal,
                    ]

        for planFunc in planningFunctions:
            taskQueue.addTask(planFunc)
            taskQueue.addTask(self.optionalUserPrompt('Continue? y/n: '))
            taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(self.printAsync('done!'))

        return taskQueue
