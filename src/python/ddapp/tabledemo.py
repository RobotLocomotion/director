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

import ioUtils

class TableDemo(object):

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
        self.useFootstepPlanner = True
        self.visOnly = True
        self.planFromCurrentRobotState = True
        useDevelopment = False
        if (useDevelopment):
            self.visOnly = True
            self.planFromCurrentRobotState = False

        self.userPromptEnabled = False

        self.plans = []
        self.frameSyncs = {}

        self.useLeftArm = False
        self.useRightArm = True

        self.tableData = None
        self.binFrame = None


    def addPlan(self, plan):
        self.plans.append(plan)


    ### Table and Bin Focused Functions
    def userFitTable(self):
        self.tableData = None
        self.picker = PointPicker(self.view, numberOfPoints=2, drawLines=True, callback=self.onSegmentTable)
        self.picker.start()

    def userFitBin(self):
        self.binFrame = None
        self.picker = PointPicker(self.view, numberOfPoints=2, drawLines=True, callback=self.onSegmentBin)
        self.picker.start()

    def waitForTableFit(self):
        while not self.tableData:
            yield

    def waitForBinFit(self):
        while not self.binFrame:
            yield

    def getInputPointCloud(self):
        polyData = segmentation.getCurrentRevolutionData()
        if polyData is None:
            obj = om.findObjectByName('scene')
            if obj:
                polyData = obj.polyData

        return polyData

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


    def cleanupSegmentedObjects(self):
        om.removeFromObjectModel(om.findObjectByName('segmentation'))
        self.clusterObjects = None
        self.segmentationData = None

    def segmentTableObjects(self):

        tableCentroid = segmentation.computeCentroid(self.tableData.box)
        self.tableData.frame.TransformPoint(tableCentroid, tableCentroid)

        data = segmentation.segmentTableScene(self.getInputPointCloud(), tableCentroid)
        data.clusters = self.sortClustersOnTable(data.clusters)

        self.clusterObjects = vis.showClusterObjects(data.clusters, parent='segmentation')
        self.segmentationData = data


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

    # TODO: deprecate this function: (to end of section):
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

    def moveRobotToTableStanceFrame(self):
        self.teleportRobotToStanceFrame(self.tableStanceFrame.transform)

    def moveRobotToBinStanceFrame(self):
        self.teleportRobotToStanceFrame(self.binStanceFrame.transform)

    def moveRobotToStartStanceFrame(self):
        self.teleportRobotToStanceFrame(self.startStanceFrame.transform)

    def planFootstepsToTable(self):
        self.planFootsteps(self.tableStanceFrame.transform)

    def planFootstepsToBin(self):
        self.planFootsteps(self.binStanceFrame.transform)

    def planFootstepsToStart(self):
        self.planFootsteps(self.startStanceFrame.transform)

    ### End Valve Focused Functions ###############################################################
    ### Planning Functions ########################################################################

    def planFootsteps(self, goalFrame):
        startPose = self.getPlanningStartPose()
        request = self.footstepPlanner.constructFootstepPlanRequest(startPose, goalFrame)
        self.footstepPlan = self.footstepPlanner.sendFootstepPlanRequest(request, waitForResponse=True)

    def planWalking(self):
        startPose = self.getPlanningStartPose()
        plan = self.footstepPlanner.sendWalkingPlanRequest(self.footstepPlan, startPose, waitForResponse=True)
        self.addPlan(plan)

    def walkToStance(self, stanceTransform):
        if self.useFootstepPlanner:
            self.planFootsteps(stanceTransform)
            self.planWalking()
        else:
            self.teleportRobotToStanceFrame(stanceTransform)


    def getRaisedArmPose(self, startPose, side):
        return self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'arm up pregrasp', side)

    def getPreDropHighPose(self, startPose, side):
        return self.ikPlanner.getMergedPostureFromDatabase(startPose, 'table clearing', 'pre drop 1', side)

    def getPreDropLowPose(self, startPose, side):
        return self.ikPlanner.getMergedPostureFromDatabase(startPose, 'table clearing', 'pre drop 2', side)

    def getLoweredArmPose(self, startPose, side):
        return self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'handdown', side)


    def planDropPostureRaise(self, side):
        startPose = self.getPlanningStartPose()
        poseA = self.getRaisedArmPose(startPose, side)
        poseB = self.getPreDropHighPose(startPose, side)
        poseC = self.getPreDropLowPose(startPose, side)

        plan = self.ikPlanner.computeMultiPostureGoal([startPose, poseA, poseB, poseC])
        self.addPlan(plan)

    def planDropPostureLower(self, side):
        startPose = self.getPlanningStartPose()
        poseA = self.getPreDropHighPose(startPose, side)
        poseB = self.getRaisedArmPose(startPose, side)
        poseC = self.getLoweredArmPose(startPose, side)

        plan = self.ikPlanner.computeMultiPostureGoal([startPose, poseA, poseB, poseC])
        self.addPlan(plan)

    def planDropPostureSwap(self, lowerSide, raiseSide):

        startPose = self.getPlanningStartPose()

        poseA = self.getRaisedArmPose(startPose, raiseSide)
        poseA = self.getPreDropHighPose(poseA, lowerSide)

        poseB = self.getPreDropHighPose(poseA, raiseSide)
        poseB = self.getRaisedArmPose(poseB, lowerSide)

        poseC = self.getPreDropLowPose(poseB, raiseSide)
        poseC = self.getLoweredArmPose(poseC, lowerSide)

        plan = self.ikPlanner.computeMultiPostureGoal([startPose, poseA, poseB, poseC])
        self.addPlan(plan)


    def planLowerArmAndStand(self, side):
        startPose = self.getPlanningStartPose()
        endPose = self.getLoweredArmPose(startPose, side)
        endPose, info = self.ikPlanner.computeStandPose(endPose)

        plan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(plan)


    def planReachToTableObject(self, side):

        obj, frame = self.getNextTableObject(side)
        startPose = self.getPlanningStartPose()

        constraintSet = self.ikPlanner.planGraspOrbitReachPlan(startPose, side, frame, dist=0.25, lockBase=False, lockBack=False, lockArm=False)

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


    def planTouchTableObject(self, side):

        obj, frame = self.getNextTableObject(side)

        startPose = self.getPlanningStartPose()
        constraintSet = self.ikPlanner.planGraspOrbitReachPlan(startPose, side, frame, dist=0.05, lockBase=False, lockBack=False)

        constraintSet.constraints[-1].tspan = [-np.inf, np.inf]
        constraintSet.constraints[-2].tspan = [-np.inf, np.inf]
        constraintSet.runIk()

        print 'planning touch'
        plan = constraintSet.runIkTraj()
        self.addPlan(plan)


    def planLiftTableObject(self, side):

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


    ### End Planning Functions ####################################################################
    ########## Glue Functions #####################################################################
    def teleportRobotToStanceFrame(self, frame):
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

    def openHand(self, side):
        #self.getHandDriver(side).sendOpen()
        self.getHandDriver(side).sendCustom(0.0, 100.0, 100.0, 0)

    def closeHand(self, side):
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

    def waitForPlanExecution(self, plan):
        planElapsedTime = planplayback.PlanPlayback.getPlanElapsedTime(plan)
        return self.delay(planElapsedTime + 1.0)


    def animateLastPlan(self):
        plan = self.plans[-1]
        if not self.visOnly:
            self.commitManipPlan()

        return self.waitForPlanExecution(plan)

    def onRobotModelChanged(self, model):

        for linkName in self.frameSyncs.keys():
            t = self.playbackRobotModel.getLinkFrame(linkName)
            vis.updateFrame(t, '%s frame' % linkName, scale=0.2, visible=False, parent='planning')


    ######### Nominal Plans and Execution  #################################################################
    def testDemoSequence(self):
        '''
        Running this function should launch a full planning sequence
        to pick to objects, walk and drop.
        Requires footstep footstepPlanner
        '''

        filename = os.path.expanduser('~/drc-testing-data/tabletop/table-and-bin-scene.vtp')
        scene = ioUtils.readPolyData(filename)
        vis.showPolyData(scene,"scene")

        #stanceFrame = transformUtils.frameFromPositionAndRPY([0, 0, 0], [0, 0, 123.0])
        #self.teleportRobotToStanceFrame(stanceFrame)

        self.userFitTable()
        self.onSegmentTable( np.array([-1.72105646,  2.73210716,  0.79449952]), np.array([-1.67336452,  2.63351011,  0.78698605]) )
        self.userFitBin()
        self.onSegmentBin( np.array([-0.02, 2.43, 0.61 ]), np.array([-0.40,  2.79,  0.61964661]) )
        self.computeTableStanceFrame()
        self.computeBinStanceFrame()

        # Actually plan the sequence:
        #self.demoSequence()

    def planSequence(self):
        self.useFootstepPlanner = True
        side = 'both'

        self.cleanupFootstepPlans()
        self.planFromCurrentRobotState = False
        self.segmentTableObjects()
        self.plans = []

        # Go home
        self.walkToStance(self.startStanceFrame.transform)

        # Pick Objects from table:
        self.walkToStance(self.tableStanceFrame.transform)
        if (side == 'left'):
          self.planSequenceTablePick('left')
        elif (side == 'right'):
          self.planSequenceTablePick('right')
        elif (side == 'both'):
          self.planSequenceTablePick('left')
          self.planSequenceTablePick('right')

        # Go home
        self.walkToStance(self.startStanceFrame.transform)

        # Go to Bin
        self.walkToStance(self.binStanceFrame.transform)

        # Drop into the Bin:
        self.planDropPostureRaise('left')
        self.dropTableObject('left')
        self.planDropPostureLower('left')
        self.planDropPostureRaise('right')
        self.dropTableObject('right')
        self.planDropPostureLower('right')

        # Go home
        self.walkToStance(self.startStanceFrame.transform)


    def planSequenceTablePick(self, side):
        self.planReachToTableObject(side)
        self.planTouchTableObject(side)
        self.graspTableObject(side)
        self.planLiftTableObject(side)


    def autonomousExecute(self):

        taskQueue = AsyncTaskQueue()

        self.addInitTasksToQueue(taskQueue)
        self.addLoopTasksToQueue(taskQueue)

        return taskQueue


    def addInitTasksToQueue(self, taskQueue):

        taskQueue.addTask(self.printAsync('user fit table'))
        taskQueue.addTask(self.userFitTable)
        taskQueue.addTask(self.waitForTableFit)

        taskQueue.addTask(self.printAsync('user fit bin'))
        taskQueue.addTask(self.userFitBin)
        taskQueue.addTask(self.waitForBinFit)

        taskQueue.addTask(self.computeTableStanceFrame)
        taskQueue.addTask(self.computeBinStanceFrame)


    def addLoopTasksToQueue(self, taskQueue):


        self.ikPlanner.ikServer.usePointwise = True
        self.ikPlanner.ikServer.maxDegreesPerSecond = 20

        taskQueue.addTask(self.userPrompt('neck pitch and open hands. continue? y/n: '))
        taskQueue.addTask(self.sendNeckPitchLookDown)


        self.addWalkingTasksToQueue(taskQueue, self.planFootstepsToTable, self.moveRobotToTableStanceFrame)


        self.addTablePickTasksToQueue(taskQueue)


        '''
        self.addWalkingTasksToQueue(taskQueue, self.planFootstepsToStart, self.moveRobotToStartStanceFrame)

        self.addWalkingTasksToQueue(taskQueue, self.planFootstepsToBin, self.moveRobotToBinStanceFrame)


        taskQueue.addTask(self.atlasDriver.sendManipCommand)
        taskQueue.addTask(self.waitForAtlasBehaviorAsync('manip'))

        taskQueue.addTask(functools.partial(self.planDropPostureRaise, 'left'))
        taskQueue.addTask(self.userPrompt('continue? y/n: '))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(functools.partial(self.openHand, 'left'))
        taskQueue.addTask(functools.partial(self.dropTableObject, 'left'))


        taskQueue.addTask(functools.partial(self.planDropPostureSwap, 'left', 'right'))
        taskQueue.addTask(self.userPrompt('continue? y/n: '))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(functools.partial(self.openHand, 'right'))
        taskQueue.addTask(functools.partial(self.dropTableObject, 'right'))


        taskQueue.addTask(functools.partial(self.planDropPostureLower, 'right'))
        taskQueue.addTask(self.userPrompt('continue? y/n: '))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(self.cleanupSegmentedObjects)


        taskQueue.addTask(functools.partial(self.closeHand, 'left'))
        taskQueue.addTask(functools.partial(self.closeHand, 'right'))


        self.addWalkingTasksToQueue(taskQueue, self.planFootstepsToStart, self.moveRobotToStartStanceFrame)


        taskQueue.addTask(functools.partial(self.addLoopTasksToQueue, taskQueue))

        '''


    def addWalkingTasksToQueue(self, taskQueue, planFunc, walkFunc):

        if self.useFootstepPlanner:
            taskQueue.addTask(planFunc)

            if self.visOnly:
                taskQueue.addTask(self.planWalking)
                taskQueue.addTask(self.animateLastPlan)
            else:

                taskQueue.addTask(self.userPrompt('send stand command. continue? y/n: '))
                taskQueue.addTask(self.atlasDriver.sendStandCommand)
                taskQueue.addTask(self.waitForAtlasBehaviorAsync('stand'))

                taskQueue.addTask(self.userPrompt('commit footsteps. continue? y/n: '))
                taskQueue.addTask(self.commitFootstepPlan)
                taskQueue.addTask(self.waitForAtlasBehaviorAsync('step'))
                taskQueue.addTask(self.waitForAtlasBehaviorAsync('stand'))

            taskQueue.addTask(self.cleanupFootstepPlans)
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

            taskQueue.addTask(functools.partial(self.planReachToTableObject, 'left'))
            taskQueue.addTask(self.userPrompt('continue? y/n: '))
            taskQueue.addTask(self.animateLastPlan)

            taskQueue.addTask(functools.partial(self.planTouchTableObject, 'left'))
            taskQueue.addTask(self.userPrompt('continue? y/n: '))
            taskQueue.addTask(self.animateLastPlan)

            taskQueue.addTask(functools.partial(self.closeHand, 'left'))
            taskQueue.addTask(functools.partial(self.graspTableObject, 'left'))

            taskQueue.addTask(functools.partial(self.planLiftTableObject, 'left'))
            taskQueue.addTask(self.userPrompt('continue? y/n: '))
            taskQueue.addTask(self.animateLastPlan)

        if self.useRightArm:

            taskQueue.addTask(functools.partial(self.planReachToTableObject, 'right'))
            taskQueue.addTask(self.userPrompt('continue? y/n: '))
            taskQueue.addTask(self.animateLastPlan)

            taskQueue.addTask(functools.partial(self.planTouchTableObject, 'right'))
            taskQueue.addTask(self.userPrompt('continue? y/n: '))
            taskQueue.addTask(self.animateLastPlan)

            taskQueue.addTask(functools.partial(self.closeHand, 'right'))
            taskQueue.addTask(functools.partial(self.graspTableObject, 'right'))

            taskQueue.addTask(functools.partial(self.planLiftTableObject, 'right'))
            taskQueue.addTask(self.animateLastPlan)
            taskQueue.addTask(self.userPrompt('continue? y/n: '))

            taskQueue.addTask(functools.partial(self.planLowerArmAndStand, 'right'))
            taskQueue.addTask(self.userPrompt('continue? y/n: '))
            taskQueue.addTask(self.animateLastPlan)


        else:
            taskQueue.addTask(functools.partial(self.planLowerArmAndStand, 'left'))
            taskQueue.addTask(self.userPrompt('continue? y/n: '))
            taskQueue.addTask(self.animateLastPlan)


        taskQueue.addTask(functools.partial(self.addTablePickTasksToQueue, taskQueue))



