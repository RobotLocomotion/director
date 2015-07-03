import os
import copy
import math
import functools
import numpy as np

from ddapp import transformUtils

from ddapp.asynctaskqueue import AsyncTaskQueue
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp import robotstate
from ddapp import segmentation
from ddapp import planplayback
from ddapp.pointpicker import PointPicker
from ddapp import vtkAll as vtk
from ddapp.simpletimer import SimpleTimer
from ddapp import affordanceupdater

from ddapp.debugVis import DebugData
from ddapp import affordanceitems
from ddapp import ikplanner


import ioUtils

class MappingDemo(object):

    def __init__(self, robotStateModel, playbackRobotModel, ikPlanner, manipPlanner, footstepPlanner, atlasDriver, lhandDriver, rhandDriver, multisenseDriver, view, sensorJointController, planPlaybackFunction):
        self.planPlaybackFunction = planPlaybackFunction
        self.robotStateModel = robotStateModel
        self.playbackRobotModel = playbackRobotModel
        self.ikPlanner = ikPlanner
        self.manipPlanner = manipPlanner
        self.footstepPlanner = footstepPlanner
        self.atlasDriver = atlasDriver
        self.lhandDriver = lhandDriver
        self.rhandDriver = rhandDriver
        self.multisenseDriver = multisenseDriver
        self.sensorJointController = sensorJointController
        self.view = view

        # live operation flags:
        self.visOnly = False
        self.planFromCurrentRobotState = True
        useDevelopment = False
        if (useDevelopment):
            self.visOnly = True
            self.planFromCurrentRobotState = False

        self.optionalUserPromptEnabled = True
        self.requiredUserPromptEnabled = True

        self.plans = []

        # top level switch between BDI or IHMC (locked base) and MIT (moving base and back)
        self.lockBack = True
        self.lockBase = True

        self.constraintSet = []

        self.targetSweepType = 'orientation' # gaze or orientation - but i've had problems with the gaze constraint
        self.coneThresholdDegrees = 5.0 # 0 is ok for reaching but often too tight for a trajectory
        self.boxLength = 0.3

    # Switch between simulation/visualisation and real robot operation
    def setMode(self, mode='visualization'):
        '''
        Switches between visualization and real robot operation.
        mode='visualization'
        mode='robot'
        '''

        if (mode == 'visualization'):
            print "Setting mode to VISUALIZATION"
            self.useDevelopment = True

            self.visOnly = True
            self.planFromCurrentRobotState = False
        else:
            print "Setting mode to ROBOT OPERATION"
            self.useDevelopment = False

            self.visOnly = False
            self.planFromCurrentRobotState = True

    def addPlan(self, plan):
        self.plans.append(plan)

    def planPostureFromDatabase(self, groupName, postureName, side='left'):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, groupName, postureName, side=side)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    ######### Target Focused Functions ##################################################################
    def spawnTargetAffordance(self):
        for obj in om.getObjects():
             if obj.getProperty('Name') == 'target':
                 om.removeFromObjectModel(obj)

        targetFrame = transformUtils.frameFromPositionAndRPY([0.6,0.2,0.6],[180,0,90])

        folder = om.getOrCreateContainer('affordances')
        z = DebugData()
        z.addLine(np.array([0,0,0]), np.array([-self.boxLength,0,0]), radius=0.02) # main bar
        z.addLine(np.array([-self.boxLength,0,0]), np.array([-self.boxLength,0,self.boxLength]), radius=0.02) # main bar
        z.addLine(np.array([-self.boxLength,0,self.boxLength]), np.array([0,0,self.boxLength]), radius=0.02) # main bar
        z.addLine(np.array([0,0,self.boxLength]), np.array([0,0,0]), radius=0.02) # main bar
        targetMesh = z.getPolyData()

        self.targetAffordance = vis.showPolyData(targetMesh, 'target', color=[0.0, 1.0, 0.0], cls=affordanceitems.FrameAffordanceItem, parent=folder, alpha=0.3)
        self.targetAffordance.actor.SetUserTransform(targetFrame)
        self.targetFrame = vis.showFrame(targetFrame, 'target frame', parent=self.targetAffordance, visible=False, scale=0.2)
        self.targetFrame = self.targetFrame.transform

        params = dict(length=self.boxLength, otdf_type='target', friendly_name='target')
        self.targetAffordance.setAffordanceParams(params)
        self.targetAffordance.updateParamsFromActorTransform()

    def drawTargetPath(self):
        path = DebugData()
        for i in range(1,len(self.targetPath)):
          p0 = self.targetPath[i-1].GetPosition()
          p1 = self.targetPath[i].GetPosition()
          path.addLine ( np.array( p0 ) , np.array(  p1 ), radius= 0.005)

        pathMesh = path.getPolyData()
        self.targetPathMesh = vis.showPolyData(pathMesh, 'target frame desired path', color=[0.0, 0.3, 1.0], parent=self.targetAffordance, alpha=0.6)
        self.targetPathMesh.actor.SetUserTransform(self.targetFrame)

    def resetTargetPath(self):
        for obj in om.getObjects():
            if obj.getProperty('Name') == 'target frame desired':
                om.removeFromObjectModel(obj)
        for obj in om.getObjects():
            if obj.getProperty('Name') == 'target frame desired path':
                om.removeFromObjectModel(obj)

    def computeNextTargetFrame(self):
        assert self.targetAffordance
        t = transformUtils.frameFromPositionAndRPY(self.nextPosition, [0, 0, 0])
        self.faceTransformLocal = transformUtils.copyFrame(t) # copy required
        t.Concatenate(self.targetFrame)
        self.faceFrameDesired = vis.showFrame(t, 'target frame desired', parent=self.targetAffordance, visible=False, scale=0.2)


    ######### Higher Level Planning Functions ##################################################################
    def computeNextRoomFrame(self):
        assert self.targetAffordance
        t = transformUtils.frameFromPositionAndRPY(self.nextPosition, [0, 0, 0])
        self.faceTransformLocal = transformUtils.copyFrame(t) # copy required
        t.Concatenate(self.targetFrame)
        self.faceFrameDesired = vis.showFrame(t, 'target frame desired', parent=self.targetAffordance, visible=False, scale=0.2)

    def planRoomReach(self):
        # A single one shot gaze-constrained reach: place xyz at goal and align y-axis of hand with x-axis of goal
        self.initConstraintSet()
        self.addConstraintForTargetFrame(self.startFrame, 1)
        self.planTrajectory()


    def getRoomSweepFrames(self, rotateHandFrame=False):
        topFrame = transformUtils.frameFromPositionAndRPY([0.65,0.0,0.8],[160,0,90])
        yawFrame = transformUtils.frameFromPositionAndRPY([0,0.0,0],[0,0,self.currentYawDegrees])
        if rotateHandFrame:
            fixHandFrame = transformUtils.frameFromPositionAndRPY([0,0.0,0],[0,-90,0])
            topFrame.PreMultiply()
            topFrame.Concatenate( fixHandFrame )
        topFrame.PostMultiply()
        topFrame.Concatenate( yawFrame )

        bottomFrame = transformUtils.frameFromPositionAndRPY([0.6,0.0,0.4],[210,0,90])
        yawFrame = transformUtils.frameFromPositionAndRPY([0,0.0,0],[0,0,self.currentYawDegrees])
        if rotateHandFrame:
            bottomFrame.PreMultiply()
            bottomFrame.Concatenate( fixHandFrame )
        bottomFrame.PostMultiply()
        bottomFrame.Concatenate( yawFrame )

        if (self.fromTop):
            self.startFrame = vis.showFrame(topFrame, 'frame start', visible=False, scale=0.1,parent=self.mapFolder)
            self.endFrame = vis.showFrame(bottomFrame, 'frame end', visible=False, scale=0.1,parent=self.mapFolder)
        else:
            self.startFrame = vis.showFrame(bottomFrame, 'frame start', visible=False, scale=0.1,parent=self.mapFolder)
            self.endFrame = vis.showFrame(topFrame, 'frame end', visible=False, scale=0.1,parent=self.mapFolder)


    def planRoomSweep(self):
        self.initConstraintSet()

        faceFrameDesired = transformUtils.frameInterpolate(self.startFrame.transform , self.endFrame.transform, 0)
        vis.showFrame(faceFrameDesired, 'frame 0', visible=True, scale=0.1,parent=self.mapFolder)
        self.addConstraintForTargetFrame(faceFrameDesired, 0)

        faceFrameDesired = transformUtils.frameInterpolate(self.startFrame.transform , self.endFrame.transform, 1.0/3.0)
        vis.showFrame(faceFrameDesired, 'frame 1', visible=True, scale=0.1,parent=self.mapFolder)
        self.addConstraintForTargetFrame(faceFrameDesired, 1)

        faceFrameDesired = transformUtils.frameInterpolate(self.startFrame.transform , self.endFrame.transform, 2.0/3.0)
        vis.showFrame(faceFrameDesired, 'frame 2', visible=True, scale=0.1,parent=self.mapFolder)
        self.addConstraintForTargetFrame(faceFrameDesired, 2)

        faceFrameDesired = transformUtils.frameInterpolate(self.startFrame.transform , self.endFrame.transform, 3.0/3.0)
        vis.showFrame(faceFrameDesired, 'frame 3', visible=True, scale=0.1,parent=self.mapFolder)
        self.addConstraintForTargetFrame(faceFrameDesired, 3)

        #self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedLow
        self.planTrajectory()
        #self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedHigh

    def moveRoomSweepOnwards(self):
        self.currentYawDegrees = self.currentYawDegrees - 20
        self.fromTop = not self.fromTop

    def planTargetReach(self):
        # A single one shot gaze-constrained reach: place xyz at goal and align y-axis of hand with x-axis of goal
        worldToTargetFrame = vis.updateFrame(self.targetFrame, 'gaze goal', visible=False, scale=0.2, parent=om.getOrCreateContainer('affordances'))

        self.initConstraintSet()
        self.addConstraintForTargetFrame(worldToTargetFrame, 1)
        self.planTrajectory()

    ######### Lower Level Planning Functions ##################################################################
    def planTrajectory(self):
        self.ikPlanner.ikServer.usePointwise = False
        plan = self.constraintSet.runIkTraj()
        self.addPlan(plan)

    def initConstraintSet(self):
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

    def addConstraintForTargetFrame(self,goalFrame, t):
        if (self.targetSweepType is 'orientation'):
            self.appendPositionOrientationConstraintForTargetFrame(goalFrame, t)
        elif (self.targetSweepType is 'gaze'):
            # align the palmGazeAxis axis (on the hand) with the vector 'targetAxis' from worldToTargetFrame?
            palmGazeAxis = self.ikPlanner.getPalmToHandLink(self.graspingHand).TransformVector([0,1,0])
            self.appendPositionGazeConstraintForTargetFrame(goalFrame, t, targetAxis=[0.0, 0.0, 1.0], bodyAxis=palmGazeAxis)

    def appendPositionGazeConstraintForTargetFrame(self, goalFrame, t, targetAxis=[-1.0, 0.0, 0.0], bodyAxis=[-1.0, 0.0, 0.0]):
        gazeConstraint = self.ikPlanner.createGazeGraspConstraint(self.graspingHand, goalFrame, self.graspToHandLinkFrame, self.coneThresholdDegrees , targetAxis, bodyAxis)
        gazeConstraint.tspan = [t, t]
        self.constraintSet.constraints.insert(0, gazeConstraint)

        positionConstraint, _ = self.ikPlanner.createPositionOrientationGraspConstraints(self.graspingHand, goalFrame, self.graspToHandLinkFrame)
        positionConstraint.tspan = [t, t]
        self.constraintSet.constraints.append(positionConstraint)

    def appendPositionOrientationConstraintForTargetFrame(self, goalFrame, t):
        positionConstraint, orientationConstraint = self.ikPlanner.createPositionOrientationGraspConstraints(self.graspingHand, goalFrame, self.graspToHandLinkFrame)
        positionConstraint.tspan = [t, t]
        orientationConstraint.tspan = [t, t]
        self.constraintSet.constraints.append(positionConstraint)
        self.constraintSet.constraints.append(orientationConstraint)


    ### End Planning Functions ####################################################################
    ########## Glue Functions #####################################################################
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

    def playSequenceNominal(self):
        assert None not in self.plans
        self.planPlaybackFunction(self.plans)

    def commitManipPlan(self):
            self.manipPlanner.commitManipPlan(self.plans[-1])

    def waitForPlanExecution(self, plan):
        planElapsedTime = planplayback.PlanPlayback.getPlanElapsedTime(plan)
        return self.delay(planElapsedTime + 1.0)

    def animateLastPlan(self):
        plan = self.plans[-1]
        if not self.visOnly:
            self.commitManipPlan()

        return self.waitForPlanExecution(plan)


    ######### Nominal Plans and Execution  #################################################################

    ####### Module for an arm to sweep out a gaze-constrained trajectory to map an area:
    # t.spawnTargetAffordance(), t.planTargetSweep()
    def planSequenceTargetSweep(self):
        self.graspingHand = 'left'
        self.planFromCurrentRobotState = False
        self.plans = []
        self.graspToHandLinkFrame = self.ikPlanner.newGraspToHandFrame(self.graspingHand)

        self.planTargetReach()

        self.nextPosition =[0,0,0]
        self.targetPath = []
        self.resetTargetPath()

        self.computeNextTargetFrame()
        self.initConstraintSet()
        self.targetPath.append(self.faceTransformLocal)

        pointsPerSide = 3
        deltaDistance = self.targetAffordance.params.get('length') / pointsPerSide # 5cm was good
        for i in xrange(pointsPerSide*0,pointsPerSide*1):
            self.nextPosition[0] += -deltaDistance
            self.computeNextTargetFrame()
            self.addConstraintForTargetFrame(self.faceFrameDesired, i+1)
            self.targetPath.append(self.faceTransformLocal)

        for i in xrange(pointsPerSide*1,pointsPerSide*2):
            self.nextPosition[2] += deltaDistance
            self.computeNextTargetFrame()
            self.addConstraintForTargetFrame(self.faceFrameDesired, i+1)
            self.targetPath.append(self.faceTransformLocal)

        for i in xrange(pointsPerSide*2,pointsPerSide*3):
            self.nextPosition[0] += deltaDistance
            self.computeNextTargetFrame()
            self.addConstraintForTargetFrame(self.faceFrameDesired, i+1)
            self.targetPath.append(self.faceTransformLocal)

        for i in xrange(pointsPerSide*3,pointsPerSide*4):
            self.nextPosition[2] += -deltaDistance
            self.computeNextTargetFrame()
            self.addConstraintForTargetFrame(self.faceFrameDesired, i+1)
            self.targetPath.append(self.faceTransformLocal)

        self.drawTargetPath()
        #self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedLow
        self.planTrajectory()
        #self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedHigh


    # Module to sweep the kuka arm around in a sphere - for map building
    def planSequenceRoomMap(self):
        self.graspingHand = 'left'
        self.targetSweepType = 'orientation'
        self.graspToHandLinkFrame = self.ikPlanner.newGraspToHandFrame(self.graspingHand)
        self.planFromCurrentRobotState = False
        self.plans = []
        self.currentYawDegrees = 60
        self.ikPlanner.ikServer.maxDegreesPerSecond = 10

        self.nextPosition =[0,0,0]
        self.targetPath = []
        self.resetTargetPath()
        self.fromTop = True

        self.mapFolder=om.getOrCreateContainer('room mapping')
        om.collapse(self.mapFolder)

        # taskQueue doesnt support a while loop:
        #while (self.currentYawDegrees >= -90):
        #    self.getRoomSweepFrames()
        #    self.planRoomReach()# move to next start point
        #    self.planRoomSweep()        # reach down/up
        #    self.currentYawDegrees = self.currentYawDegrees - 30
        #    self.fromTop = not self.fromTop

        self.getRoomSweepFrames()
        self.planRoomReach()# move to next start point
        self.planRoomSweep()        # reach down/up
        self.moveRoomSweepOnwards()

        self.getRoomSweepFrames()
        self.planRoomReach()# move to next start point
        self.planRoomSweep()        # reach down/up
        self.moveRoomSweepOnwards()

        self.getRoomSweepFrames()
        self.planRoomReach()# move to next start point
        self.planRoomSweep()        # reach down/up
        self.moveRoomSweepOnwards()

        self.getRoomSweepFrames()
        self.planRoomReach()# move to next start point
        self.planRoomSweep()        # reach down/up
        self.moveRoomSweepOnwards()

        self.getRoomSweepFrames()
        self.planRoomReach()# move to next start point
        self.planRoomSweep()        # reach down/up
        self.moveRoomSweepOnwards()

        self.getRoomSweepFrames()
        self.planRoomReach()# move to next start point
        self.planRoomSweep()        # reach down/up
        self.moveRoomSweepOnwards()

        self.getRoomSweepFrames()
        self.planRoomReach()# move to next start point
        self.planRoomSweep()        # reach down/up
        self.moveRoomSweepOnwards()

    def doneIndicator(self):
        print "We are done here."

    def setMaxDegreesPerSecond(self, maxDeg):
        self.ikPlanner.defaultIkParameters.maxDegreesPerSecond = maxDeg

    def autonomousRoomMapNew(self, side='left'):
        taskQueue = AsyncTaskQueue()
        lowSpeed = 5
        highSpeed = 30
        delayTime = 3 # TODO: for potential self.delay to wait for pointclouds to be registered

        taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'General', 'arm up pregrasp'))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, highSpeed))
        taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'roomMapping', 'p1_up'))
        taskQueue.addTask(self.animateLastPlan)
        taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, lowSpeed))
        taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'roomMapping', 'p1_down', side=side))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, highSpeed))
        taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'roomMapping', 'p2_down', side=side))
        taskQueue.addTask(self.animateLastPlan)
        taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, lowSpeed))
        taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'roomMapping', 'p2_up', side=side))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, highSpeed))
        taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'roomMapping', 'p3_up', side=side))
        taskQueue.addTask(self.animateLastPlan)
        taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, lowSpeed))
        taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'roomMapping', 'p3_down', side=side))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, highSpeed))
        taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'roomMapping', 'p4_down', side=side))
        taskQueue.addTask(self.animateLastPlan)
        taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, lowSpeed))
        taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'roomMapping', 'p4_up', side=side))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, highSpeed))
        taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'roomMapping', 'p5_up', side=side))
        taskQueue.addTask(self.animateLastPlan)
        taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, lowSpeed))
        taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'roomMapping', 'p5_down', side=side))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, highSpeed))
        taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'General', 'arm up pregrasp', side=side))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(self.doneIndicator)
        return taskQueue
    

    def autonomousExecuteRoomMap(self):
        self.graspingHand = 'left'
        self.targetSweepType = 'orientation'
        self.graspToHandLinkFrame = self.ikPlanner.newGraspToHandFrame(self.graspingHand)
        self.planFromCurrentRobotState = True
        self.visOnly = False
        self.ikPlanner.ikServer.maxDegreesPerSecond = 3#5
        self.currentYawDegrees = 60
        self.fromTop = True
        self.mapFolder=om.getOrCreateContainer('room mapping')

        taskQueue = AsyncTaskQueue()
        self.addTasksToQueueSweep(taskQueue)
        self.addTasksToQueueSweep(taskQueue)
        self.addTasksToQueueSweep(taskQueue)
        self.addTasksToQueueSweep(taskQueue)
        self.addTasksToQueueSweep(taskQueue)
        self.addTasksToQueueSweep(taskQueue)
        self.addTasksToQueueSweep(taskQueue)
        taskQueue.addTask(self.printAsync('done!'))
        taskQueue.addTask(self.doneIndicator)
        return taskQueue
        
    def addTasksToQueueSweep(self, taskQueue):        
        taskQueue.addTask(self.getRoomSweepFrames)
        taskQueue.addTask(self.planRoomReach)        
        taskQueue.addTask(self.optionalUserPrompt('execute reach? y/n: '))
        taskQueue.addTask(self.animateLastPlan)
        taskQueue.addTask(self.planRoomSweep)        
        taskQueue.addTask(self.optionalUserPrompt('execute sweep? y/n: '))
        taskQueue.addTask(self.animateLastPlan)
        taskQueue.addTask(self.moveRoomSweepOnwards)                
        
        return taskQueue
