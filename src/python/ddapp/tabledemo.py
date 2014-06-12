import ddapp.objectmodel as om
from ddapp.asynctaskqueue import AsyncTaskQueue

from ddapp import segmentation
from ddapp import visualization as vis
from ddapp import objectmodel as om
from ddapp.pointpicker import PointPicker
from ddapp import vtkAll as vtk
from ddapp import robotstate
from ddapp import drilldemo
from ddapp import transformUtils
from ddapp import planplayback
from ddapp.simpletimer import SimpleTimer

import copy
import math
import functools
import numpy as np

class TableDemo(object):

    def __init__(self, robotStateModel, playbackRobotModel, ikPlanner, manipPlanner, footstepPlanner, atlasDriver, lhandDriver, rhandDriver, multisenseDriver, view, sensorJointController):

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

        self.visOnly = False
        self.planFromCurrentRobotState = True
        self.userPromptEnabled = False
        self.useFootstepPlanner = True

        self.plans = []
        self.frameSyncs = {}

        self.useLeftArm = False
        self.useRightArm = True

        self.tableData = None
        self.binFrame = None


    def userFitTable(self):
        self.tableData = None
        self.picker = PointPicker(self.view, numberOfPoints=2, drawLines=True, callback=self.onSegmentTable)
        self.picker.start()

    def userFitBin(self):
        self.binFrame = None
        self.picker = PointPicker(self.view, numberOfPoints=2, drawLines=True, callback=self.onSegmentBin)
        self.picker.start()

    def getInputPointCloud(self):
        polyData = segmentation.getCurrentRevolutionData()
        if polyData is None:
            obj = om.findObjectByName('scene')
            if obj:
                polyData = obj.polyData

        return polyData


    def addPlan(self, plan):
        self.plans.append(plan)


    def onSegmentTable(self, p1, p2):
        self.picker.stop()
        om.removeFromObjectModel(self.picker.annotationObj)
        self.picker = None

        om.removeFromObjectModel(om.findObjectByName('table demo'))

        self.tableData = segmentation.segmentTableEdge(self.getInputPointCloud(), p1, p2)
        self.tableObj = vis.showPolyData(self.tableData.mesh, 'table', parent='table demo', color=[0,1,0])
        self.tableFrame = vis.showFrame(self.tableData.frame, 'table frame', parent=self.tableObj, scale=0.2)
        self.tableObj.actor.SetUserTransform(self.tableFrame.transform)


    def onSegmentBin(self, p1, p2):
        self.picker.stop()
        om.removeFromObjectModel(self.picker.annotationObj)
        self.picker = None

        om.removeFromObjectModel(om.findObjectByName('bin frame'))

        binEdge = p2 - p1
        zaxis = [0.0, 0.0, 1.0]
        xaxis = np.cross(binEdge, zaxis)
        xaxis /= np.linalg.norm(xaxis)
        yaxis = np.cross(zaxis, xaxis)

        t = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
        t.PostMultiply()
        t.Translate(p1)

        self.binFrame = vis.showFrame(t, 'bin frame', parent=None, scale=0.2)



    def sortClustersOnTable(self, clusters):
        '''
        returns list copy of clusters, sorted left to right using the
        table coordinate system.  (Table y axis points right to left)
        '''
        tableYAxis = self.tableData.axes[1]
        tableOrigin = np.array(self.tableData.frame.GetPosition())

        origins = [np.array(c.frame.GetPosition()) for c in clusters]
        dists = [np.dot(origin-tableOrigin, -tableYAxis) for origin in origins]

        return [clusters[i] for i in np.argsort(dists)]


    def removeSegmentedObjects(self):
        om.removeFromObjectModel(om.findObjectByName('segmentation'))
        self.clusterObjects = None
        self.segmentationData = None


    def removeFootstepPlan(self):
        om.removeFromObjectModel(om.findObjectByName('footstep plan'))
        self.footstepPlan = None


    def segmentTableObjects(self):

        tableCentroid = segmentation.computeCentroid(self.tableData.box)
        self.tableData.frame.TransformPoint(tableCentroid, tableCentroid)

        data = segmentation.segmentTableScene(self.getInputPointCloud(), tableCentroid)
        data.clusters = self.sortClustersOnTable(data.clusters)

        self.clusterObjects = vis.showClusterObjects(data.clusters, parent='segmentation')
        self.segmentationData = data


    def getPlanningStartPose(self):
        if self.planFromCurrentRobotState:
            return self.getEstimatedRobotStatePose()
        else:
            if self.plans:
                return robotstate.convertStateMessageToDrakePose(self.plans[-1].plan[-1])
            else:
                return self.getEstimatedRobotStatePose()


    def getRaisedArmPose(self, startPose, side):
        return self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'arm up pregrasp', side)


    def getPreDropHighPose(self, startPose, side):
        return self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'pre drop 1', side)


    def getPreDropLowPose(self, startPose, side):
        return self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'pre drop 2', side)


    def getLoweredArmPose(self, startPose, side):

        #jointNames = robotstate.matchJoints('l_arm' if side == 'left' else 'r_arm')
        #nominalPose = self.ikPlanner.jointController.getPose('q_nom')
        #return self.ikPlanner.mergePostures(startPose, jointNames, nominalPose)

        return self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'arm down thumb in', side)


    def raiseArm(self, side):
        startPose = self.getPlanningStartPose()
        endPose = self.getRaisedArmPose(startPose, side)
        plan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(plan)


    def dropPosturePlanRaise(self, side):
        startPose = self.getPlanningStartPose()
        poseA = self.getRaisedArmPose(startPose, side)
        poseB = self.getPreDropHighPose(startPose, side)
        poseC = self.getPreDropLowPose(startPose, side)

        plan = self.ikPlanner.computeMultiPostureGoal([startPose, poseA, poseB, poseC])
        self.addPlan(plan)

    def dropPosturePlanLower(self, side):
        startPose = self.getPlanningStartPose()
        poseA = self.getPreDropHighPose(startPose, side)
        poseB = self.getRaisedArmPose(startPose, side)
        poseC = self.getLoweredArmPose(startPose, side)

        plan = self.ikPlanner.computeMultiPostureGoal([startPose, poseA, poseB, poseC])
        self.addPlan(plan)


    def dropPosturePlanSwap(self, lowerSide, raiseSide):

        startPose = self.getPlanningStartPose()

        poseA = self.getRaisedArmPose(startPose, raiseSide)
        poseA = self.getPreDropHighPose(poseA, lowerSide)

        poseB = self.getPreDropHighPose(poseA, raiseSide)
        poseB = self.getRaisedArmPose(poseB, lowerSide)

        poseC = self.getPreDropLowPose(poseB, raiseSide)
        poseC = self.getLoweredArmPose(poseC, lowerSide)

        plan = self.ikPlanner.computeMultiPostureGoal([startPose, poseA, poseB, poseC])
        self.addPlan(plan)


    def lowerArmAndStand(self, side):
        startPose = self.getPlanningStartPose()
        endPose = self.getLoweredArmPose(startPose, side)
        endPose, info = self.ikPlanner.computeStandPose(endPose)

        plan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(plan)


    def onRobotModelChanged(self, model):

        for linkName in self.frameSyncs.keys():
            t = self.playbackRobotModel.getLinkFrame(linkName)
            vis.updateFrame(t, '%s frame' % linkName, scale=0.2, visible=False, parent='planning')


    def graspTableObject(self, side):

        linkName = 'l_hand' if side == 'left' else 'r_hand'
        t = self.ikPlanner.getLinkFrameAtPose(linkName, self.getPlanningStartPose())
        linkFrame = vis.updateFrame(t, '%s frame' % linkName, scale=0.2, visible=False, parent='planning')

        _, objFrame = self.getNextTableObject(side)
        frameSync = vis.FrameSync()
        frameSync.addFrame(linkFrame)
        frameSync.addFrame(objFrame)
        self.frameSyncs[linkName] = frameSync

        self.playbackRobotModel.connectModelChanged(self.onRobotModelChanged)


    def dropTableObject(self, side):
        obj, _ = self.getNextTableObject(side)
        obj.setProperty('Visible', False)
        for child in obj.children():
            child.setProperty('Visible', False)


    def getNextTableObject(self, side):

        assert len(self.clusterObjects)
        obj = self.clusterObjects[0] if side == 'left' else self.clusterObjects[-1]
        frameObj = obj.findChild(obj.getProperty('Name') + ' frame')
        return obj, frameObj


    def reachToTableObject(self, side):

        obj, frame = self.getNextTableObject(side)
        startPose = self.getPlanningStartPose()

        constraintSet = self.ikPlanner.planGraspOrbitReachPlan(startPose, side, frame, dist=0.25, lockTorso=False, lockArm=False)


        loweringSide = 'left' if side == 'right' else 'right'
        armPose = self.getLoweredArmPose(startPose, loweringSide)
        armPoseName = 'lowered_arm_pose'
        self.ikPlanner.ikServer.sendPoseToServer(armPose, armPoseName)
        armPostureConstraint = self.ikPlanner.createPostureConstraint(armPoseName, robotstate.matchJoints('l_arm' if loweringSide == 'left' else 'r_arm'))
        armPostureConstraint.tspan = np.array([1.0, 1.0])
        constraintSet.constraints.append(armPostureConstraint)


        constraintSet.runIk()


        armPose = self.getRaisedArmPose(startPose, side)
        armPoseName = 'raised_arm_pose'
        self.ikPlanner.ikServer.sendPoseToServer(armPose, armPoseName)
        armPostureConstraint = self.ikPlanner.createPostureConstraint(armPoseName, robotstate.matchJoints('l_arm' if side == 'left' else 'r_arm'))
        armPostureConstraint.tspan = np.array([0.5, 0.5])
        constraintSet.constraints.append(armPostureConstraint)

        print 'planning reach to'
        plan = constraintSet.runIkTraj()
        self.addPlan(plan)


    def touchTableObject(self, side):

        obj, frame = self.getNextTableObject(side)

        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planGraspOrbitReachPlan(startPose, side, frame, dist=0.05, lockTorso=False)

        constraintSet.constraints[-1].tspan = [-np.inf, np.inf]
        constraintSet.constraints[-2].tspan = [-np.inf, np.inf]

        constraintSet.runIk()

        print 'planning touch'
        plan = constraintSet.runIkTraj()
        self.addPlan(plan)


    def liftTableObject(self, side):

        startPose = self.getPlanningStartPose()


        constraintSet = self.ikPlanner.planEndEffectorDelta(startPose, side, [0.0, 0.0, 0.15])

        constraintSet.constraints[-1].tspan[1] = 1.0

        endPose, info = constraintSet.runIk()
        endPose = self.getRaisedArmPose(endPose, side)

        endPoseName = 'raised_arm_end_pose'
        self.ikPlanner.ikServer.sendPoseToServer(endPose, endPoseName)
        postureConstraint = self.ikPlanner.createPostureConstraint(endPoseName, robotstate.matchJoints('l_arm' if side == 'left' else 'r_arm'))
        postureConstraint.tspan = np.array([2.0, 2.0])
        constraintSet.constraints.append(postureConstraint)

        postureConstraint = self.ikPlanner.createPostureConstraint('q_nom', robotstate.matchJoints('.*_leg_kny'))
        postureConstraint.tspan = np.array([2.0, 2.0])
        constraintSet.constraints.append(postureConstraint)

        postureConstraint = self.ikPlanner.createPostureConstraint('q_nom', robotstate.matchJoints('back'))
        postureConstraint.tspan = np.array([2.0, 2.0])
        constraintSet.constraints.append(postureConstraint)

        print 'planning lift'
        plan = constraintSet.runIkTraj()
        self.addPlan(plan)


    def retractAndRaiseOtherArm(self, lowerSide, raiseSide):
        startPose = self.getPlanningStartPose()
        endPose = self.getLoweredArmPose(startPose, lowerSide)
        endPose = self.getRaisedArmPose(endPose, raiseSide)
        plan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(plan)


    def computeTableStanceFrame(self):
        assert self.tableData

        zGround = 0.0
        tableHeight = self.tableData.frame.GetPosition()[2] - zGround

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Translate(-0.4, self.tableData.dims[1]*0.5, -tableHeight)
        t.Concatenate(self.tableData.frame)

        self.tableStanceFrame = vis.showFrame(t, 'table stance frame', parent=self.tableObj, scale=0.2)


    def computeBinStanceFrame(self):
        assert self.binFrame

        zGround = 0.0
        binHeight = self.binFrame.transform.GetPosition()[2] - zGround

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Translate(-0.45, 0.1, -binHeight)
        t.Concatenate(self.binFrame.transform)

        self.binStanceFrame = vis.showFrame(t, 'bin stance frame', parent=None, scale=0.2)

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.RotateZ(30)
        t.Translate(-0.8, 0.4, -binHeight)
        t.Concatenate(self.binFrame.transform)

        self.startStanceFrame = vis.showFrame(t, 'start stance frame', parent=None, scale=0.2)



    def waitForTableFit(self):
        while not self.tableData:
            yield

    def waitForBinFit(self):
        while not self.binFrame:
            yield

    def moveRobotToStanceFrame(self, frame):
        self.sensorJointController.setPose('q_nom')
        stancePosition = frame.GetPosition()
        stanceOrientation = frame.GetOrientation()

        self.sensorJointController.q[:2] = [stancePosition[0], stancePosition[1]]
        self.sensorJointController.q[5] = math.radians(stanceOrientation[2])
        self.sensorJointController.push()


    def computeFootstepPlan(self, goalFrame):
        startPose = self.getPlanningStartPose()
        request = self.footstepPlanner.constructFootstepPlanRequest(startPose, goalFrame)
        self.footstepPlan = self.footstepPlanner.sendFootstepPlanRequest(request, waitForResponse=True)


    def computeWalkingPlan(self):
        startPose = self.getPlanningStartPose()
        plan = self.footstepPlanner.sendWalkingPlanRequest(self.footstepPlan, startPose, waitForResponse=True)
        self.addPlan(plan)


    def moveRobotToTableStanceFrame(self):
        self.moveRobotToStanceFrame(self.tableStanceFrame.transform)


    def moveRobotToBinStanceFrame(self):
        self.moveRobotToStanceFrame(self.binStanceFrame.transform)


    def moveRobotToStartStanceFrame(self):
        self.moveRobotToStanceFrame(self.startStanceFrame.transform)


    def computeTableFootstepPlan(self):
        self.computeFootstepPlan(self.tableStanceFrame.transform)


    def computeBinFootstepPlan(self):
        self.computeFootstepPlan(self.binStanceFrame.transform)


    def computeStartFootstepPlan(self):
        self.computeFootstepPlan(self.startStanceFrame.transform)


    def getEstimatedRobotStatePose(self):
        return self.sensorJointController.getPose('EST_ROBOT_STATE')


    def cleanupFootstepPlans(self):
        om.removeFromObjectModel(om.findObjectByName('walking goal'))
        om.removeFromObjectModel(om.findObjectByName('footstep plan'))


    def waitForPlanExecution(self, plan):
        planElapsedTime = planplayback.PlanPlayback.getPlanElapsedTime(plan)
        return self.delay(planElapsedTime + 1.0)


    def animateLastPlan(self):
        plan = self.plans[-1]
        #self.ikPlanner.planPlaybackFunction([plan])

        if not self.visOnly:
            self.commitManipPlan()

        return self.waitForPlanExecution(plan)


    def commitFootstepPlan(self):
        self.footstepPlanner.commitFootstepPlan(self.footstepPlan)


    def commitManipPlan(self):
            self.manipPlanner.commitManipPlan(self.plans[-1])


    def getHandDriver(self, side):
        assert side in ('left', 'right')
        return self.lhandDriver if side == 'left' else self.rhandDriver

    def openHand(self, side):
        #self.getHandDriver(side).sendOpen()
        self.getHandDriver(side).sendCustom(0.0, 100.0, 100.0, 0)


    def closeHand(self, side):
        #self.getHandDriver(side).sendClose()
        self.getHandDriver(side).sendCustom(100.0, 100.0, 100.0, 0)


    def sendNeckPitchLookDown(self):
        self.multisenseDriver.setNeckPitch(40)

    def spinLidar(self):
        self.multisenseDriver.setLidarRevolutionTime(10)

    def sendNeckPitchLookForward(self):
        self.multisenseDriver.setNeckPitch(15)


    def waitForAtlasBehaviorAsync(self, behaviorName):
        assert behaviorName in self.atlasDriver.getBehaviorMap().values()

        if self.visOnly:
            return

        while self.atlasDriver.getCurrentBehaviorName() != behaviorName:
            yield


    def printAsync(self, s):
        yield
        print s


    def userPrompt(self, message, force=False):

        if not self.userPromptEnabled and not force:
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


    def pauseQueue(self):
        raise AsyncTaskQueue.PauseException()


    def waitForCleanLidarSweepAsync(self):

        if self.visOnly:
            return

        currentRevolution = self.multisenseDriver.displayedRevolution
        desiredRevolution = currentRevolution + 2
        while self.multisenseDriver.displayedRevolution < desiredRevolution:
            yield


    def addInitTasksToQueue(self, taskQueue):

        taskQueue.addTask(self.printAsync('user fit table'))
        taskQueue.addTask(self.userFitTable)
        taskQueue.addTask(self.waitForTableFit)

        taskQueue.addTask(self.printAsync('user fit bin'))
        taskQueue.addTask(self.userFitBin)
        taskQueue.addTask(self.waitForBinFit)

        taskQueue.addTask(self.computeTableStanceFrame)
        taskQueue.addTask(self.computeBinStanceFrame)


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



    def addTablePickTasksToQueue(self, taskQueue):

        taskQueue.addTask(functools.partial(self.openHand, 'left'))
        taskQueue.addTask(functools.partial(self.openHand, 'right'))

        taskQueue.addTask(self.atlasDriver.sendManipCommand)
        taskQueue.addTask(self.waitForAtlasBehaviorAsync('manip'))


        taskQueue.addTask(self.userPrompt('wait for lidar. continue? y/n: '))
        taskQueue.addTask(self.waitForCleanLidarSweepAsync)

        taskQueue.addTask(self.segmentTableObjects)


        if self.useLeftArm:

            taskQueue.addTask(functools.partial(self.reachToTableObject, 'left'))
            taskQueue.addTask(self.userPrompt('continue? y/n: '))
            taskQueue.addTask(self.animateLastPlan)

            taskQueue.addTask(functools.partial(self.touchTableObject, 'left'))
            taskQueue.addTask(self.userPrompt('continue? y/n: '))
            taskQueue.addTask(self.animateLastPlan)

            taskQueue.addTask(functools.partial(self.closeHand, 'left'))
            taskQueue.addTask(functools.partial(self.graspTableObject, 'left'))

            taskQueue.addTask(functools.partial(self.liftTableObject, 'left'))
            taskQueue.addTask(self.userPrompt('continue? y/n: '))
            taskQueue.addTask(self.animateLastPlan)

        if self.useRightArm:

            taskQueue.addTask(functools.partial(self.reachToTableObject, 'right'))
            taskQueue.addTask(self.userPrompt('continue? y/n: '))
            taskQueue.addTask(self.animateLastPlan)

            taskQueue.addTask(functools.partial(self.touchTableObject, 'right'))
            taskQueue.addTask(self.userPrompt('continue? y/n: '))
            taskQueue.addTask(self.animateLastPlan)

            taskQueue.addTask(functools.partial(self.closeHand, 'right'))
            taskQueue.addTask(functools.partial(self.graspTableObject, 'right'))

            taskQueue.addTask(functools.partial(self.liftTableObject, 'right'))
            taskQueue.addTask(self.animateLastPlan)
            taskQueue.addTask(self.userPrompt('continue? y/n: '))

            taskQueue.addTask(functools.partial(self.lowerArmAndStand, 'right'))
            taskQueue.addTask(self.userPrompt('continue? y/n: '))
            taskQueue.addTask(self.animateLastPlan)


        else:
            taskQueue.addTask(functools.partial(self.lowerArmAndStand, 'left'))
            taskQueue.addTask(self.userPrompt('continue? y/n: '))
            taskQueue.addTask(self.animateLastPlan)


        taskQueue.addTask(functools.partial(self.addTablePickTasksToQueue, taskQueue))



    def addLoopTasksToQueue(self, taskQueue):


        self.ikPlanner.ikServer.usePointwise = True
        self.ikPlanner.ikServer.maxDegreesPerSecond = 20

        taskQueue.addTask(self.userPrompt('neck pitch and open hands. continue? y/n: '))
        taskQueue.addTask(self.sendNeckPitchLookDown)


        self.addWalkingTasksToQueue(taskQueue, self.computeTableFootstepPlan, self.moveRobotToTableStanceFrame)


        self.addTablePickTasksToQueue(taskQueue)


        '''
        self.addWalkingTasksToQueue(taskQueue, self.computeStartFootstepPlan, self.moveRobotToStartStanceFrame)

        self.addWalkingTasksToQueue(taskQueue, self.computeBinFootstepPlan, self.moveRobotToBinStanceFrame)


        taskQueue.addTask(self.atlasDriver.sendManipCommand)
        taskQueue.addTask(self.waitForAtlasBehaviorAsync('manip'))

        taskQueue.addTask(functools.partial(self.dropPosturePlanRaise, 'left'))
        taskQueue.addTask(self.userPrompt('continue? y/n: '))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(functools.partial(self.openHand, 'left'))
        taskQueue.addTask(functools.partial(self.dropTableObject, 'left'))


        taskQueue.addTask(functools.partial(self.dropPosturePlanSwap, 'left', 'right'))
        taskQueue.addTask(self.userPrompt('continue? y/n: '))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(functools.partial(self.openHand, 'right'))
        taskQueue.addTask(functools.partial(self.dropTableObject, 'right'))


        taskQueue.addTask(functools.partial(self.dropPosturePlanLower, 'right'))
        taskQueue.addTask(self.userPrompt('continue? y/n: '))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(self.removeSegmentedObjects)


        taskQueue.addTask(functools.partial(self.closeHand, 'left'))
        taskQueue.addTask(functools.partial(self.closeHand, 'right'))


        self.addWalkingTasksToQueue(taskQueue, self.computeStartFootstepPlan, self.moveRobotToStartStanceFrame)


        taskQueue.addTask(functools.partial(self.addLoopTasksToQueue, taskQueue))

        '''



    def getInitTaskQueue(self):
        taskQueue = AsyncTaskQueue()
        self.addInitTasksToQueue(taskQueue)
        return taskQueue

    def getAutonomousTaskQueue(self):
        taskQueue = AsyncTaskQueue()
        self.addLoopTasksToQueue(taskQueue)
        return taskQueue


    def autonomousExecute(self):

        taskQueue = AsyncTaskQueue()

        self.addInitTasksToQueue(taskQueue)
        self.addLoopTasksToQueue(taskQueue)

        return taskQueue
