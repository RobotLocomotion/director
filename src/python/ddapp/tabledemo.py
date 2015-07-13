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
from ddapp import vtkNumpy
from numpy import array
from ddapp.uuidutil import newUUID
import affordancepanel
import ioUtils

from ddapp.tasks.taskuserpanel import TaskUserPanel
from ddapp.tasks.taskuserpanel import ImageBasedAffordanceFit

import ddapp.tasks.robottasks as rt


class TableDemo(object):

    def __init__(self, robotStateModel, playbackRobotModel, ikPlanner, manipPlanner, footstepPlanner,
                 atlasDriver, lhandDriver, rhandDriver, multisenseDriver, view, sensorJointController,
                 planPlaybackFunction, teleopPanel):
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
        self.teleopPanel = teleopPanel

        # live operation flags:
        self.useFootstepPlanner = True
        self.visOnly = False
        self.planFromCurrentRobotState = True
        self.useDevelopment = False
        if (self.useDevelopment):
            self.visOnly = True
            self.planFromCurrentRobotState = False
            extraModels = [self.robotStateModel]
            self.affordanceUpdater  = affordanceupdater.AffordanceGraspUpdater(self.playbackRobotModel, self.ikPlanner, extraModels)
        else:
            extraModels = [self.playbackRobotModel]
            self.affordanceUpdater  = affordanceupdater.AffordanceGraspUpdater(self.robotStateModel, self.ikPlanner, extraModels)

        self.optionalUserPromptEnabled = True
        self.requiredUserPromptEnabled = True

        self.plans = []
        self.frameSyncs = {}

        self.graspingHand = 'left' # left, right, both

        self.tableData = None
        self.binFrame = None

        # top level switch between BDI or IHMC (locked base) and MIT (moving base and back)
        self.lockBack = True
        self.lockBase = True

        self.constraintSet = []

        self.reachDist = 0.07

        # Switch indicating whether to use affordances as a collision environment
        self.useCollisionEnvironment = True

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
            extraModels = [self.robotStateModel]
            self.affordanceUpdater  = affordanceupdater.AffordanceGraspUpdater(self.playbackRobotModel, self.ikPlanner, extraModels)
        else:
            print "Setting mode to ROBOT OPERATION"
            self.useDevelopment = False

            extraModels = [self.playbackRobotModel]
            self.affordanceUpdater  = affordanceupdater.AffordanceGraspUpdater(self.robotStateModel, self.ikPlanner, extraModels)


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
            else: # fall back to map in case we used mapping rather than loading of a scene
                obj = om.findObjectByName('map')
                if obj:
                    polyData = obj.polyData

        return polyData

    def onSegmentTable(self, p1, p2):
        print p1
        print p2
        self.picker.stop()
        om.removeFromObjectModel(self.picker.annotationObj)
        self.picker = None

        om.removeFromObjectModel(om.findObjectByName('table demo'))

        self.tableData = segmentation.segmentTableEdge(self.getInputPointCloud(), p1, p2)
        self.tableObj = vis.showPolyData(self.tableData.mesh, 'table', parent='table demo', color=[0,1,0])
        self.tableFrame = vis.showFrame(self.tableData.frame, 'table frame', parent=self.tableObj, scale=0.2)
        self.tableBox = vis.showPolyData(self.tableData.box, 'table box', parent=self.tableObj, color=[0,1,0], visible=False)
        self.tableObj.actor.SetUserTransform(self.tableFrame.transform)
        self.tableBox.actor.SetUserTransform(self.tableFrame.transform)

        if self.useCollisionEnvironment:
            self.addCollisionObject(self.tableObj)

    def onSegmentBin(self, p1, p2):
        print p1
        print p2
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

        #linkName = self.ikPlanner.getHandLink(side)
        #t = self.ikPlanner.getLinkFrameAtPose(linkName, self.getPlanningStartPose())
        #linkFrame = vis.updateFrame(t, '%s frame' % linkName, scale=0.2, visible=False, parent='planning')



        obj, objFrame = self.getNextTableObject(side)
        #frameSync = vis.FrameSync()
        #frameSync.addFrame(linkFrame)
        #frameSync.addFrame(objFrame)
        #self.frameSyncs[linkName] = frameSync

        #self.playbackRobotModel.connectModelChanged(self.onRobotModelChanged)

        self.affordanceUpdater.graspAffordance( obj.getProperty('Name') , side)


    def dropTableObject(self, side='left'):

        obj, _ = self.getNextTableObject(side)
        obj.setProperty('Visible', False)
        for child in obj.children():
            child.setProperty('Visible', False)

        self.clusterObjects.remove(obj) # remove from clusterObjects
        om.removeFromObjectModel(obj) # remove from objectModel

        if self.useCollisionEnvironment:
            objAffordance = om.findObjectByName(obj.getProperty('Name') + ' affordance')
            objAffordance.setProperty('Collision Enabled', False)
            objAffordance.setProperty('Visible', False)

        self.affordanceUpdater.ungraspAffordance(obj.getProperty('Name'))

    def getNextTableObject(self, side='left'):

        assert len(self.clusterObjects)
        obj = self.clusterObjects[0] if side == 'left' else self.clusterObjects[-1]
        frameObj = obj.findChild(obj.getProperty('Name') + ' frame')

        if self.useCollisionEnvironment:
            self.prepCollisionEnvironment()
            collisionObj = om.findObjectByName(obj.getProperty('Name') + ' affordance')
            collisionObj.setProperty('Collision Enabled', False)

        return obj, frameObj

    def computeTableStanceFrame(self):
        assert self.tableData

        zGround = 0.0
        tableHeight = self.tableData.frame.GetPosition()[2] - zGround

        t = transformUtils.copyFrame(self.tableData.frame)
        t.PreMultiply()
        t1 = transformUtils.frameFromPositionAndRPY([-x/2 for x in self.tableData.dims],[0,0,0])
        t.Concatenate(t1)
        t2 = transformUtils.frameFromPositionAndRPY([-0.35, self.tableData.dims[1]*0.5, -tableHeight],[0,0,0])
        t.Concatenate(t2)

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

    # TODO: deprecate this function: (to end of section):
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

    ### End Object Focused Functions ###############################################################
    ### Planning Functions ########################################################################

    def planFootsteps(self, goalFrame):
        startPose = self.getPlanningStartPose()
        request = self.footstepPlanner.constructFootstepPlanRequest(startPose, goalFrame)
        self.footstepPlan = self.footstepPlanner.sendFootstepPlanRequest(request, waitForResponse=True)

    def planWalking(self):
        startPose = self.getPlanningStartPose()
        plan = self.footstepPlanner.sendWalkingPlanRequest(self.footstepPlan, startPose, waitForResponse=True)
        self.addPlan(plan)

    def planWalkToStance(self, stanceTransform):
        if self.useFootstepPlanner:
            self.planFootsteps(stanceTransform)
            self.planWalking()
        else:
            self.teleportRobotToStanceFrame(stanceTransform)

    def planPostureFromDatabase(self, groupName, postureName, side='left'):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, groupName, postureName, side=side)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)
        # TODO: integrate this function with the ones below

    def getRaisedArmPose(self, startPose, side):
        return self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'arm up pregrasp', side)

    def getPreDropHighPose(self, startPose, side):
        return self.ikPlanner.getMergedPostureFromDatabase(startPose, 'table clearing', 'pre drop 1', side)

    def getPreDropLowPose(self, startPose, side):
        return self.ikPlanner.getMergedPostureFromDatabase(startPose, 'table clearing', 'pre drop 2', side)

    def getLoweredArmPose(self, startPose, side):
        return self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'handdown', side)

    def planPreGrasp(self, side='left'):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'arm up pregrasp', side=side)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def planLowerArm(self, side):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'handdown', side=side)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

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


    def planReachToTableObject(self, side='left'):

        obj, frame = self.getNextTableObject(side)
        startPose = self.getPlanningStartPose()

        if self.ikPlanner.fixedBaseArm: # includes reachDist hack instead of in ikPlanner (TODO!)
            f = transformUtils.frameFromPositionAndRPY( np.array(frame.transform.GetPosition())-np.array([self.reachDist,0,0]), [0,0,-90] )
            f.PreMultiply()
            f.RotateY(90)
            f.Update()
            self.constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, side, f, lockBase=False, lockBack=True)
            #newFrame = vis.FrameItem('reach_item', f, self.view)
            #self.constraintSet = self.ikPlanner.planGraspOrbitReachPlan(startPose, side, newFrame, constraints=None, dist=self.reachDist, lockBase=self.lockBase, lockBack=self.lockBack, lockArm=False)
        else:
            self.constraintSet = self.ikPlanner.planGraspOrbitReachPlan(startPose, side, frame, constraints=None, dist=self.reachDist, lockBase=self.lockBase, lockBack=self.lockBack, lockArm=False)
            loweringSide = 'left' if side == 'right' else 'right'
            armPose = self.getLoweredArmPose(startPose, loweringSide)
            armPoseName = 'lowered_arm_pose'
            self.ikPlanner.ikServer.sendPoseToServer(armPose, armPoseName)

            loweringSideJoints = []
            if (loweringSide == 'left'):
              loweringSideJoints += self.ikPlanner.leftArmJoints
            else:
              loweringSideJoints += self.ikPlanner.rightArmJoints

            reachingSideJoints = []
            if (side == 'left'):
                reachingSideJoints += self.ikPlanner.leftArmJoints
            else:
                reachingSideJoints += self.ikPlanner.rightArmJoints


            armPostureConstraint = self.ikPlanner.createPostureConstraint(armPoseName, loweringSideJoints)
            armPostureConstraint.tspan = np.array([1.0, 1.0])
            self.constraintSet.constraints.append(armPostureConstraint)
        
        self.constraintSet.runIk()

        #armPose = self.getRaisedArmPose(startPose, side)
        #armPoseName = 'raised_arm_pose'
        #self.ikPlanner.ikServer.sendPoseToServer(armPose, armPoseName)
        #armPostureConstraint = self.ikPlanner.createPostureConstraint(armPoseName, reachingSideJoints)
        #armPostureConstraint.tspan = np.array([0.5, 0.5])
        #self.constraintSet.constraints.append(armPostureConstraint)

        print 'planning reach to'
        plan = self.constraintSet.runIkTraj()
        self.addPlan(plan)


    def planReachToTableObjectCollisionFree(self, side ='left'):
        # Hard-coded demonstration of collision reaching to object on table
        # Using RRT Connect

        goalFrame = transformUtils.frameFromPositionAndRPY([1.05,0.4,1],[0,90,-90])
        vis.showFrame(goalFrame,'goal frame')
        frameObj = om.findObjectByName( 'goal frame')

        startPose = self.getPlanningStartPose()
        self.constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, side, frameObj.transform, lockBase=self.lockBase, lockBack=self.lockBack)
        self.constraintSet.runIk()

        print 'planning reach to planReachToTableObjectCollisionFree'
        self.constraintSet.ikParameters.usePointwise = False
        self.constraintSet.ikParameters.useCollision = True
        self.teleopPanel.endEffectorTeleop.updateCollisionEnvironment()

        plan = self.constraintSet.runIkTraj()
        self.addPlan(plan)


    def planTouchTableObject(self, side='left'):

        obj, frame = self.getNextTableObject(side)
        startPose = self.getPlanningStartPose()

        if self.ikPlanner.fixedBaseArm: # includes distance hack and currently uses reachDist instead of touchDist (TODO!)
            f = transformUtils.frameFromPositionAndRPY( np.array(frame.transform.GetPosition())-np.array([self.reachDist,0,0]), [0,0,-90] )
            f.PreMultiply()
            f.RotateY(90)
            f.Update()
            item = vis.FrameItem('reach_item', f, self.view)
            self.constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, side, f, lockBase=False, lockBack=True)
        else:
            self.constraintSet = self.ikPlanner.planGraspOrbitReachPlan(startPose, side, frame, dist=0.05, lockBase=self.lockBase, lockBack=self.lockBack)
            self.constraintSet.constraints[-1].tspan = [-np.inf, np.inf]
            self.constraintSet.constraints[-2].tspan = [-np.inf, np.inf]
        
        self.constraintSet.runIk()

        print 'planning touch'
        plan = self.constraintSet.runIkTraj()
        self.addPlan(plan)


    def planLiftTableObject(self, side):

        startPose = self.getPlanningStartPose()
        self.constraintSet = self.ikPlanner.planEndEffectorDelta(startPose, side, [0.0, 0.0, 0.15])

        if not self.ikPlanner.fixedBaseArm:
            self.constraintSet.constraints[-1].tspan[1] = 1.0

        endPose, info = self.constraintSet.runIk()
        
        if not self.ikPlanner.fixedBaseArm:
            endPose = self.getRaisedArmPose(endPose, side)

            reachingSideJoints = []
            if (side == 'left'):
                reachingSideJoints += self.ikPlanner.leftArmJoints
            else:
                reachingSideJoints += self.ikPlanner.rightArmJoints


            endPoseName = 'raised_arm_end_pose'
            self.ikPlanner.ikServer.sendPoseToServer(endPose, endPoseName)
            postureConstraint = self.ikPlanner.createPostureConstraint(endPoseName, reachingSideJoints)
            postureConstraint.tspan = np.array([2.0, 2.0])
            self.constraintSet.constraints.append(postureConstraint)

        #postureConstraint = self.ikPlanner.createPostureConstraint('q_nom', robotstate.matchJoints('.*_leg_kny'))
        #postureConstraint.tspan = np.array([2.0, 2.0])
        #self.constraintSet.constraints.append(postureConstraint)

        #postureConstraint = self.ikPlanner.createPostureConstraint('q_nom', robotstate.matchJoints('back'))
        #postureConstraint.tspan = np.array([2.0, 2.0])
        #self.constraintSet.constraints.append(postureConstraint)

        print 'planning lift'
        plan = self.constraintSet.runIkTraj()
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



    def createCollisionPlanningScene(self, scene=1, moveRobot=False, loadPerception=False):

        if (loadPerception):
            #filename = os.path.expanduser('~/drc-testing-data/collision_scene/collision_scene.vtp')
            #polyData = ioUtils.readPolyData( filename )
            pd = io.readPolyData('/home/mfallon/Desktop/rrt_scene/all.vtp')
            vis.showPolyData(pd,'scene')

        if (scene == 0):
            pose = (array([ 1.20,  0. , 0.8]), array([ 1.,  0.,  0.,  0.]))
            desc = dict(classname='BoxAffordanceItem', Name='scene0-tabletop', uuid=newUUID(), pose=pose, Color=[0.66, 0.66, 0.66], Dimensions=[0.5,1,0.06])
            obj = affordancepanel.panel.affordanceFromDescription(desc)
            pose = (array([ 1.20,  0.5 , 0.4]), array([ 1.,  0.,  0.,  0.]))
            desc = dict(classname='BoxAffordanceItem', Name='scene0-leg1', uuid=newUUID(), pose=pose, Color=[0.66, 0.66, 0.66], Dimensions=[0.5,0.05,0.8])
            obj = affordancepanel.panel.affordanceFromDescription(desc)
            pose = (array([ 1.20,  -0.5 , 0.4]), array([ 1.,  0.,  0.,  0.]))
            desc = dict(classname='BoxAffordanceItem', Name='scene0-leg2', uuid=newUUID(), pose=pose, Color=[0.66, 0.66, 0.66], Dimensions=[0.5,0.05,0.8])
            obj = affordancepanel.panel.affordanceFromDescription(desc)
            pose = (array([ 1.05,  0.3 , 0.98]), array([ 1.,  0.,  0.,  0.]))
            desc = dict(classname='BoxAffordanceItem', Name='scene0-object1', uuid=newUUID(), pose=pose, Color=[0.9, 0.9, 0.1], Dimensions=[0.08,0.08,0.24])
            obj = affordancepanel.panel.affordanceFromDescription(desc)
            pose = (array([ 1.25,  0.1 , 0.98]), array([ 1.,  0.,  0.,  0.]))
            desc = dict(classname='BoxAffordanceItem', Name='scene0-object2', uuid=newUUID(), pose=pose, Color=[0.0, 0.9, 0.0], Dimensions=[0.07,0.07,0.25])
            obj = affordancepanel.panel.affordanceFromDescription(desc)
            pose = (array([ 1.25,  -0.1 , 0.95]), array([ 1.,  0.,  0.,  0.]))
            desc = dict(classname='CylinderAffordanceItem', Name='scene0-object3', uuid=newUUID(), pose=pose, Color=[0.0, 0.9, 0.0], Radius=0.035, Length = 0.22)
            obj = affordancepanel.panel.affordanceFromDescription(desc)
            pose = (array([ 1.05,  -0.2 , 0.95]), array([ 1.,  0.,  0.,  0.]))
            desc = dict(classname='CylinderAffordanceItem', Name='scene0-object4', uuid=newUUID(), pose=pose, Color=[0.9, 0.1, 0.1], Radius=0.045, Length = 0.22)
            obj = affordancepanel.panel.affordanceFromDescription(desc)

            if (moveRobot):
                self.sensorJointController.q[0] = 0.67
                self.sensorJointController.push()

        elif (scene == 1):
            pose = (array([-0.69, -1.50,  0.92]), array([-0.707106781,  0.        ,  0.        ,  0.707106781 ]))
            desc = dict(classname='BoxAffordanceItem', Name='scene1-tabletop', uuid=newUUID(), pose=pose, Color=[0.66, 0.66, 0.66], Dimensions=[0.5,1,0.06])
            obj = affordancepanel.panel.affordanceFromDescription(desc)
            pose = (array([-1.05, -1.10,  0.95]), array([-0.707106781,  0.        ,  0.        ,  0.707106781 ]))
            desc = dict(classname='BoxAffordanceItem', Name='scene1-edge1', uuid=newUUID(), pose=pose, Color=[0.66, 0.66, 0.66], Dimensions=[0.1,0.3,0.05])
            obj = affordancepanel.panel.affordanceFromDescription(desc)
            pose = (array([-0.35, -1.10,  0.95]), array([-0.707106781,  0.        ,  0.        ,  0.707106781 ]))
            desc = dict(classname='BoxAffordanceItem', Name='scene1-edge2', uuid=newUUID(), pose=pose, Color=[0.66, 0.66, 0.66], Dimensions=[0.1,0.3,0.05])
            obj = affordancepanel.panel.affordanceFromDescription(desc)
            pose = (array([-0.6803156 , -1.1826616 ,  1.31299839]), array([-0.707106781,  0.        ,  0.        ,  0.707106781 ]))
            desc = dict(classname='BoxAffordanceItem', Name='scene1-edge3', uuid=newUUID(), pose=pose, Color=[0.66, 0.66, 0.66], Dimensions=[0.14,1.0,0.07])
            obj = affordancepanel.panel.affordanceFromDescription(desc)
            pose = (array([ -0.7,  -1.5 , 1.03]), array([ 1.,  0.,  0.,  0.]))
            desc = dict(classname='BoxAffordanceItem', Name='scene1-object1', uuid=newUUID(), pose=pose, Color=[0.9, 0.9, 0.1], Dimensions=[0.05,0.05,0.14])
            obj = affordancepanel.panel.affordanceFromDescription(desc)

            if (moveRobot):
                self.sensorJointController.q[5] = -1.571
                self.sensorJointController.q[0] = -0.75
                self.sensorJointController.q[1] = -0.85
                self.sensorJointController.push()

        elif (scene == 2):
            pose = (array([-0.98873106,  1.50393395,  0.91420001]), array([ 0.49752312,  0.        ,  0.        ,  0.86745072]))
            desc = dict(classname='BoxAffordanceItem', Name='scene2-tabletop', uuid=newUUID(), pose=pose, Color=[0.66, 0.66, 0.66], Dimensions=[0.5,1,0.06])
            obj = affordancepanel.panel.affordanceFromDescription(desc)
            pose = (array([-0.98873106,  1.50393395,  0.57]), array([ 0.49752312,  0.        ,  0.        ,  0.86745072]))
            desc = dict(classname='BoxAffordanceItem', Name='scene1-object1', uuid=newUUID(), pose=pose, Color=[0.005, 0.005, 0.3], Dimensions=[0.05,0.05,0.14])
            obj = affordancepanel.panel.affordanceFromDescription(desc)

            if (moveRobot):
                self.sensorJointController.q[0] = -0.6
                self.sensorJointController.q[1] = 1.1
                self.sensorJointController.q[5] = 2.1
                self.sensorJointController.push()



    ######### Setup collision environment ####################
    def prepCollisionEnvironment(self):
        assert len(self.clusterObjects)
        
        for obj in self.clusterObjects:
            self.addCollisionObject(obj)

    def addCollisionObject(self, obj):
        if om.getOrCreateContainer('affordances').findChild(obj.getProperty('Name') + ' affordance'):
            return # Affordance has been created previously

        frame = obj.findChild(obj.getProperty('Name') + ' frame')
        (origin, quat) = transformUtils.poseFromTransform(frame.transform)
        (xaxis, yaxis, zaxis) = transformUtils.getAxesFromTransform(frame.transform)

        # TODO: move this into transformUtils as getAxisDimensions or so
        box = obj.findChild(obj.getProperty('Name') + ' box')
        box_np = vtkNumpy.getNumpyFromVtk(box.polyData, 'Points')
        box_min = np.amin(box_np, 0)
        box_max = np.amax(box_np, 0)
        xwidth = np.linalg.norm(box_max[0]-box_min[0])
        ywidth = np.linalg.norm(box_max[1]-box_min[1])
        zwidth = np.linalg.norm(box_max[2]-box_min[2])
        name = obj.getProperty('Name') + ' affordance'

        boxAffordance = segmentation.createBlockAffordance(origin, xaxis, yaxis, zaxis, xwidth, ywidth, zwidth, name, parent='affordances')
        boxAffordance.setSolidColor(obj.getProperty('Color'))
        boxAffordance.setProperty('Alpha', 0.3)

    ######### Nominal Plans and Execution  #################################################################
    def prepKukaTestDemoSequence(self, inputFile='~/drc-testing-data/tabletop/kinect_collision_environment.vtp'):
        filename = os.path.expanduser(inputFile)
        scene = ioUtils.readPolyData(filename)
        vis.showPolyData(scene,"scene")

        self.prepKukaLabScene()


    def prepKukaLabScene(self):
        self.userFitTable()
        self.onSegmentTable( np.array([  0.91544128,  0.06092263,  0.14906664]), np.array([ 0.73494804, -0.21896157,  0.13435645]) )
        self.userFitBin() # TODO: actually fit bin, put bin in picture.
        self.onSegmentBin( np.array([-0.02, 2.43, 0.61 ]), np.array([-0.40,  2.79,  0.61964661]) ) # TODO: fix bin location

        self.segmentTableObjects()

        # Plan sequence
        self.plans = []


    def prepTestDemoSequence(self):
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


    def prepIhmcDemoSequenceFromFile(self):

        filename = os.path.expanduser('~/drc-testing-data/ihmc_table/ihmc_table.vtp')
        polyData = ioUtils.readPolyData( filename )
        vis.showPolyData( polyData,'scene')
        self.prepIhmcDemoSequence()



    def prepIhmcDemoSequence(self):
        self.userFitBin()
        self.onSegmentBin( np.array([ 0.62, -1.33, 0.80]), np.array([ 0.89, -0.87, 0.57]) )
        self.userFitTable()
        self.onSegmentTable( np.array([ 1.11, 0.11, 0.85]), np.array([ 0.97, 0.044, 0.84]) )

        self.segmentTableObjects()
        self.computeBinStanceFrame()
        self.computeTableStanceFrame()


    def planSequence(self):
        self.useFootstepPlanner = True

        self.cleanupFootstepPlans()
        self.planFromCurrentRobotState = False
        self.segmentTableObjects()
        self.plans = []

        # Go home
        self.planWalkToStance(self.startStanceFrame.transform)

        # Pick Objects from table:
        self.planWalkToStance(self.tableStanceFrame.transform)
        if (self.graspingHand == 'both'):
            self.planSequenceTablePick('left')
            self.planSequenceTablePick('right')
        else:
            self.planSequenceTablePick(self.graspingHand)

        # Go home
        self.planWalkToStance(self.startStanceFrame.transform)

        # Go to Bin
        self.planWalkToStance(self.binStanceFrame.transform)

        # Drop into the Bin:
        if (self.graspingHand == 'both'):
            self.planDropPostureRaise('left')
            self.dropTableObject('left')
            self.planDropPostureLower('left')
            self.planDropPostureRaise('right')
            self.dropTableObject('right')
            self.planDropPostureLower('right')
        else:
            self.planDropPostureRaise(self.graspingHand)
            self.dropTableObject(self.graspingHand)
            self.planDropPostureLower(self.graspingHand)

        # Go home
        self.planWalkToStance(self.startStanceFrame.transform)


    def planSequenceTablePick(self, side):
        self.planPreGrasp(side)
        if self.ikPlanner.fixedBaseArm:
            self.planLowerArm(side)
        self.planReachToTableObject(side)
        if not self.ikPlanner.fixedBaseArm:
            self.planTouchTableObject(side) # TODO: distance is handled by reach, hence ignore
        self.graspTableObject(side)
        self.planLiftTableObject(side)


    def autonomousExecute(self):
        '''
        Use global variable self.useDevelopment to switch between simulation and real robot execution
        '''
        #self.ikPlanner.ikServer.usePointwise = True
        #self.ikPlanner.ikServer.maxDegreesPerSecond = 20

        taskQueue = AsyncTaskQueue()
        #self.addTasksToQueueInit(taskQueue)

        # Go home
        if not self.ikPlanner.fixedBaseArm:
            self.addTasksToQueueWalking(taskQueue, self.startStanceFrame.transform, 'Walk to Start')

        for _ in self.clusterObjects:
            # Pick Objects from table:
            if not self.ikPlanner.fixedBaseArm:
                self.addTasksToQueueWalking(taskQueue, self.tableStanceFrame.transform, 'Walk to Table')
            taskQueue.addTask(self.printAsync('Pick with Left Arm'))
            self.addTasksToQueueTablePick(taskQueue, 'left')
            #taskQueue.addTask(self.printAsync('Pick with Right Arm'))
            #self.addTasksToQueueTablePick(taskQueue, 'right')

            # Go home
            if not self.ikPlanner.fixedBaseArm:
                self.addTasksToQueueWalking(taskQueue, self.startStanceFrame.transform, 'Walk to Start')

            # Go to Bin
            if not self.ikPlanner.fixedBaseArm:
                self.addTasksToQueueWalking(taskQueue, self.binStanceFrame.transform, 'Walk to Bin')

            # Drop into the Bin:
            taskQueue.addTask(self.printAsync('Drop from Left Arm'))
            self.addTasksToQueueDropIntoBin(taskQueue, 'left')
            #taskQueue.addTask(self.printAsync('Drop from Right Arm'))
            #self.addTasksToQueueDropIntoBin(taskQueue, 'right')

            # Go home
            if not self.ikPlanner.fixedBaseArm:
                self.addTasksToQueueWalking(taskQueue, self.startStanceFrame.transform, 'Walk to Start')
        
        taskQueue.addTask(self.printAsync('done!'))

        return taskQueue


    def addTasksToQueueInit(self, taskQueue):

        taskQueue.addTask(self.printAsync('user fit table'))
        taskQueue.addTask(self.userFitTable)
        taskQueue.addTask(self.waitForTableFit)

        taskQueue.addTask(self.printAsync('user fit bin'))
        taskQueue.addTask(self.userFitBin)
        taskQueue.addTask(self.waitForBinFit)

        if not self.ikPlanner.fixedBaseArm:
            taskQueue.addTask(self.computeTableStanceFrame)
            taskQueue.addTask(self.computeBinStanceFrame)


    def addTasksToQueueTablePick(self, taskQueue, side):
        taskQueue.addTask(self.requiredUserPrompt('continue? y/n: '))
        taskQueue.addTask(functools.partial(self.planPreGrasp, side))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(self.requiredUserPrompt('continue? y/n: '))
        taskQueue.addTask(functools.partial(self.planReachToTableObject, side))
        taskQueue.addTask(self.animateLastPlan)

        if not self.ikPlanner.fixedBaseArm: # TODO: distance is handled by reach, hence ignore
            taskQueue.addTask(self.requiredUserPrompt('continue? y/n: '))
            taskQueue.addTask(functools.partial(self.planTouchTableObject, side))
            taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(self.requiredUserPrompt('continue? y/n: '))
        taskQueue.addTask(functools.partial(self.closeHand, side))
        taskQueue.addTask(functools.partial(self.graspTableObject, side))

        taskQueue.addTask(self.requiredUserPrompt('continue? y/n: '))
        taskQueue.addTask(functools.partial(self.planLiftTableObject, side))
        taskQueue.addTask(self.animateLastPlan)


    def addTasksToQueueDropIntoBin(self, taskQueue, side):
        taskQueue.addTask(self.requiredUserPrompt('continue? y/n: '))
        taskQueue.addTask(functools.partial(self.planDropPostureRaise, side))
        taskQueue.addTask(self.animateLastPlan)

        taskQueue.addTask(functools.partial(self.openHand, side))
        taskQueue.addTask(functools.partial(self.dropTableObject, side))

        taskQueue.addTask(self.requiredUserPrompt('continue? y/n: '))
        if not self.ikPlanner.fixedBaseArm:
            taskQueue.addTask(functools.partial(self.planDropPostureLower, side))
        else:
            taskQueue.addTask(functools.partial(self.planPreGrasp, side))
        taskQueue.addTask(self.animateLastPlan)


    def addTasksToQueueWalking(self, taskQueue, stanceTransform, message):
        taskQueue.addTask(self.printAsync(message))
        taskQueue.addTask( functools.partial(self.planWalkToStance, stanceTransform ))
        taskQueue.addTask(self.optionalUserPrompt('Send footstep plan. continue? y/n: '))
        taskQueue.addTask(self.commitFootstepPlan)
        #taskQueue.addTask(self.animateLastPlan) # ought to wait until arrival, currently doesnt wait the right amount of time
        taskQueue.addTask(self.requiredUserPrompt('Have you arrived? y/n: '))



class TableTaskPanel(TaskUserPanel):

    def __init__(self, tableDemo):

        TaskUserPanel.__init__(self, windowTitle='Table Task')

        self.tableDemo = tableDemo

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()

    def addButtons(self):

        self.addManualSpacer()
        self.addManualButton('Lower arm', functools.partial(self.tableDemo.planLowerArm, 'left'))
        self.addManualSpacer()
        self.addManualButton('Raise arm', self.tableDemo.planPreGrasp)
        self.addManualSpacer()
        self.addManualButton('Commit Manip', self.tableDemo.commitManipPlan)

    def addDefaultProperties(self):
        self.params.addProperty('Hand', 0,
                                attributes=om.PropertyAttributes(enumNames=['Left', 'Right']))
        self.params.addProperty('Base', 0,
                                attributes=om.PropertyAttributes(enumNames=['Fixed', 'Free']))
        self.params.addProperty('Back', 1,
                                attributes=om.PropertyAttributes(enumNames=['Fixed', 'Free']))
        self._syncProperties()

    def onPropertyChanged(self, propertySet, propertyName):
        self._syncProperties()
        self.taskTree.removeAllTasks()
        self.addTasks()

    def _syncProperties(self):

        self.tableDemo.planFromCurrentRobotState = True

        if self.params.getPropertyEnumValue('Hand') == 'Left':
            self.tableDemo.graspingHand = 'left'
        else:
            self.tableDemo.graspingHand = 'right'

        if self.params.getPropertyEnumValue('Base') == 'Fixed':
            self.tableDemo.lockBase = True
        else:
            self.tableDemo.lockBase = False

        if self.params.getPropertyEnumValue('Back') == 'Fixed':
            self.tableDemo.lockBack = True
        else:
            self.tableDemo.lockBack = False

    def addTasks(self):

        # some helpers
        def addTask(task, parent=None):
            self.taskTree.onAddTask(task, copy=False, parent=parent)

        def addFunc(func, name, parent=None):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)

        def addManipulation(func, name, parent=None):
            group = self.taskTree.addGroup(name, parent=parent)
            addFunc(func, name='plan motion', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'),
                    parent=group)
            addTask(rt.UserPromptTask(name='Confirm execution has finished', message='Continue when plan finishes.'),
                    parent=group)

        v = self.tableDemo

        self.taskTree.removeAllTasks()

        # graspingHand is 'left', side is 'Left'
        side = self.params.getPropertyEnumValue('Hand')

        ###############
        # add the tasks

        # prep
        prep = self.taskTree.addGroup('Preparation')
        addTask(rt.CloseHand(name='close grasp hand', side=side), parent=prep)
        addTask(rt.CloseHand(name='close left hand', side='Left'), parent=prep)
        addTask(rt.CloseHand(name='close right hand', side='Right'), parent=prep)
        addFunc(v.prepIhmcDemoSequenceFromFile, 'prep from file', parent=prep)

        # walk
        walk = self.taskTree.addGroup('Approach Table')
        addTask(rt.RequestFootstepPlan(name='plan walk to table', stanceFrameName='table stance frame'), parent=walk)
        addTask(rt.UserPromptTask(name='approve footsteps',
                                  message='Please approve footstep plan.'), parent=walk)
        addTask(rt.CommitFootstepPlan(name='walk to table',
                                      planName='table grasp stance footstep plan'), parent=walk)
        addTask(rt.SetNeckPitch(name='set neck position', angle=35), parent=walk)
        addTask(rt.WaitForWalkExecution(name='wait for walking'), parent=walk)

        # lift object
        # Not Collision Free:
        addManipulation(functools.partial(v.planPreGrasp, v.graspingHand ), name='raise arm') # seems to ignore arm side?
        addManipulation(functools.partial(v.planReachToTableObject, v.graspingHand), name='reach')
        # Collision Free:
        #addManipulation(functools.partial(v.planReachToTableObjectCollisionFree, v.graspingHand), name='reach')

        addFunc(functools.partial(v.graspTableObject, side=v.graspingHand), 'grasp', parent='reach')
        addManipulation(functools.partial(v.planLiftTableObject, v.graspingHand), name='lift object')

        # walk to start
        walkToStart = self.taskTree.addGroup('Walk to Start')
        addTask(rt.RequestFootstepPlan(name='plan walk to start', stanceFrameName='start stance frame'), parent=walkToStart)
        addTask(rt.UserPromptTask(name='approve footsteps',
                                  message='Please approve footstep plan.'), parent=walkToStart)
        addTask(rt.CommitFootstepPlan(name='walk to start',
                                      planName='start stance footstep plan'), parent=walkToStart)
        addTask(rt.WaitForWalkExecution(name='wait for walking'), parent=walkToStart)

        # walk to bin
        walkToBin = self.taskTree.addGroup('Walk to Bin')
        addTask(rt.RequestFootstepPlan(name='plan walk to bin', stanceFrameName='bin stance frame'), parent=walkToBin)
        addTask(rt.UserPromptTask(name='approve footsteps',
                                  message='Please approve footstep plan.'), parent=walkToBin)
        addTask(rt.CommitFootstepPlan(name='walk to start',
                                      planName='bin stance footstep plan'), parent=walkToBin)
        addTask(rt.WaitForWalkExecution(name='wait for walking'), parent=walkToBin)

        # drop in bin
        addManipulation(functools.partial(v.planDropPostureRaise, v.graspingHand), name='drop: raise arm') # seems to ignore arm side?
        addFunc(functools.partial(v.dropTableObject, side=v.graspingHand), 'drop', parent='drop: release')
        addManipulation(functools.partial(v.planDropPostureLower, v.graspingHand), name='drop: lower arm')
