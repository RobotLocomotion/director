import os
import copy
import math
import functools
import numpy as np

import drcargs

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
from ddapp import sceneloader

from ddapp.debugVis import DebugData
from ddapp import affordanceitems
from ddapp import ikplanner
from ddapp import vtkNumpy
from numpy import array
from ddapp.uuidutil import newUUID
from ddapp import lcmUtils
import ioUtils

from ddapp.tasks.taskuserpanel import TaskUserPanel
from ddapp.tasks.taskuserpanel import ImageBasedAffordanceFit

import ddapp.tasks.robottasks as rt


class TableDemo(object):

    def __init__(self, robotStateModel, playbackRobotModel, ikPlanner, manipPlanner, footstepPlanner,
                 atlasDriver, lhandDriver, rhandDriver, multisenseDriver, view, sensorJointController,
                 planPlaybackFunction, teleopPanel, playbackPanel, jointLimitChecker):
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
        self.affordanceManager = segmentation.affordanceManager
        self.playbackPanel = playbackPanel
        self.jointLimitChecker = jointLimitChecker

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

        self.affordanceManager.setAffordanceUpdater(self.affordanceUpdater)
        self.optionalUserPromptEnabled = True
        self.requiredUserPromptEnabled = True

        self.plans = []
        self.clusterObjects = []
        self.frameSyncs = {}

        self.graspingHand = 'left' # left, right, both

        self.tableData = None
        self.binFrame = None

        # top level switch between BDI or IHMC (locked base) and MIT (moving base and back)
        self.lockBack = True
        self.lockBase = True
        self.planner = None

        self.constraintSet = []

        self.reachDist = 0.07

        # Switch indicating whether to use affordances as a collision environment
        self.useCollisionEnvironment = True

        self.sceneID = None
        self.sceneName = None

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
                else: # fall back to kinect source and get a frame copy
                    obj = om.findObjectByName('kinect source')
                    if obj:
                        polyData = obj.polyData

        return polyData

    def onSegmentTable(self, p1, p2):
        print p1
        print p2
        self.picker.stop()
        om.removeFromObjectModel(self.picker.annotationObj)
        self.picker = None

        tableData = segmentation.segmentTableEdge(self.getInputPointCloud(), p1, p2)

        pose = transformUtils.poseFromTransform(tableData.frame)
        desc = dict(classname='MeshAffordanceItem', Name='table', Color=[0,1,0], pose=pose)
        aff = self.affordanceManager.newAffordanceFromDescription(desc)
        aff.setPolyData(tableData.mesh)

        self.tableData = tableData

        tableBox = vis.showPolyData(tableData.box, 'table box', parent=aff, color=[0,1,0], visible=False)
        tableBox.actor.SetUserTransform(tableData.frame)

        if self.useCollisionEnvironment:
            self.addCollisionObject(aff)


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

        pose = transformUtils.poseFromTransform(t)
        desc = dict(classname='BoxAffordanceItem', Name='bin', uuid=newUUID(), pose=pose, Color=[1, 0, 0], Dimensions=[0.02,0.02,0.02])
        obj = self.affordanceManager.newAffordanceFromDescription(desc)


        #self.binFrame = vis.showFrame(t, 'bin frame', parent=None, scale=0.2)

    def sortClustersOnTable(self, clusters):
        '''
        returns list copy of clusters, sorted left to right using the
        table coordinate system.  (Table y axis points right to left)
        '''
        tableTransform = om.findObjectByName('table').getChildFrame().transform

        tableYAxis = transformUtils.getAxesFromTransform(tableTransform)[1]
        tableOrigin = np.array(tableTransform.GetPosition())

        origins = [np.array(c.frame.GetPosition()) for c in clusters]
        dists = [np.dot(origin-tableOrigin, -tableYAxis) for origin in origins]

        return [clusters[i] for i in np.argsort(dists)]


    def cleanupSegmentedObjects(self):
        om.removeFromObjectModel(om.findObjectByName('segmentation'))
        self.clusterObjects = None
        self.segmentationData = None

    def segmentTableObjects(self):
        tableFrame = om.findObjectByName('table').getChildFrame()

        #tableCentroid = segmentation.computeCentroid(self.tableData.box)
        #self.tableData.frame.TransformPoint(tableCentroid, tableFrame)

        data = segmentation.segmentTableScene(self.getInputPointCloud(), tableFrame.transform.GetPosition() )
        data.clusters = self.sortClustersOnTable(data.clusters)

        objects = vis.showClusterObjects(data.clusters, parent='affordances')
        self.segmentationData = data

        self.clusterObjects = []
        for i, cluster in enumerate(objects):
            affObj = affordanceitems.MeshAffordanceItem.promotePolyDataItem(cluster)
            self.affordanceManager.registerAffordance(affObj)
            self.clusterObjects.append(affObj)



    def graspTableObject(self, side='left'):

        obj, objFrame = self.getNextTableObject(side)
            
        self.affordanceUpdater.graspAffordance(obj.getProperty('Name'), side)


    def dropTableObject(self, side='left'):

        obj, _ = self.getNextTableObject(side)
        obj.setProperty('Visible', False)
        for child in obj.children():
            child.setProperty('Visible', False)

        self.clusterObjects.remove(obj) # remove from clusterObjects
        om.removeFromObjectModel(obj) # remove from objectModel

        self.affordanceUpdater.ungraspAffordance(obj.getProperty('Name'))

        if self.ikPlanner.fixedBaseArm: # if we're dealing with the real world, open hand
            self.openHand(side)
            return self.delay(5)
#        elif self.planner == 'RRT*':
#        self.ikPlanner.ikServer.removeAffordanceFromLink( self.ikPlanner.getHandLink(side) , obj.getProperty('Name'))


    def getNextTableObject(self, side='left'):

        assert len(self.clusterObjects)
        obj = self.clusterObjects[0] if side == 'left' else self.clusterObjects[-1]
        frameObj = obj.findChild(obj.getProperty('Name') + ' frame')
        
        if self.planner != 'RRT*':
            if self.useCollisionEnvironment:
                self.prepCollisionEnvironment()
                collisionObj = om.findObjectByName(obj.getProperty('Name') + ' affordance')
                collisionObj.setProperty('Collision Enabled', False)
        else:
            obj.setProperty('Collision Enabled', False)

        return obj, frameObj

    def computeTableStanceFrame(self, relativeStance):
        tableTransform = om.findObjectByName('table').getChildFrame().transform
        zGround = 0.0
        tableHeight = tableTransform.GetPosition()[2] - zGround

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Translate(relativeStance.GetPosition()[0], relativeStance.GetPosition()[1], -tableHeight)
        t.Concatenate(tableTransform)
        vis.showFrame(t, 'table stance frame', parent=om.findObjectByName('table'), scale=0.2)

    def computeCollisionGoalFrame(self, relativeFrame):
        tableTransform = om.findObjectByName('table').getChildFrame().transform

        #t = vtk.vtkTransform()
        #t.PostMultiply()
        #t.Translate(relativeStance.GetPosition()[0], relativeStance.GetPosition()[1], -tableHeight)
        relativeFrame.Concatenate(tableTransform)
        vis.showFrame(relativeFrame, 'table goal frame', parent=om.findObjectByName('table'), scale=0.2)

    def computeBinStanceFrame(self):
        binTransform = om.findObjectByName('bin').getChildFrame().transform
        zGround = 0.0
        binHeight = binTransform.GetPosition()[2] - zGround

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Translate(-0.45, 0.1, -binHeight)
        t.Concatenate(binTransform)
        vis.showFrame(t, 'bin stance frame', parent=om.findObjectByName('bin'), scale=0.2)

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.RotateZ(30)
        t.Translate(-0.8, 0.4, -binHeight)
        t.Concatenate(binTransform)
        vis.showFrame(t, 'start stance frame', parent=om.findObjectByName('bin'), scale=0.2)

    # TODO: deprecate this function: (to end of section):
    def moveRobotToTableStanceFrame(self):
        self.teleportRobotToStanceFrame(om.findObjectByName('table stance frame').transform)

    def moveRobotToBinStanceFrame(self):
        self.teleportRobotToStanceFrame(om.findObjectByName('bin stance frame').transform)

    def moveRobotToStartStanceFrame(self):
        self.teleportRobotToStanceFrame(om.findObjectByName('start stance frame').transform)

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
        
    def disableJointChecker(self):
        self.jointLimitChecker.automaticallyExtendLimits = True

    def planLowerArm(self, side = 'default'):
        startPose = self.getPlanningStartPose()
        if side == 'default':
            endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'handsdown both')
        else:
            endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'handdown', side=side)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)
        self.commitManipPlan()

    def planDropPostureRaise(self, side):        
        if self.planner == 'RRT*':
            binAff = om.findObjectByName('bin')
            binAff.setProperty('Collision Enabled', False)
            self.teleopPanel.endEffectorTeleop.updateCollisionEnvironment()
            
            startPose = self.getPlanningStartPose()
            
            print 'planning drop raise'
            
            plan = self.ikPlanner.runMultiRRT(list(startPose),
                          list(np.append(binAff.getPose()[0], binAff.getPose()[1])), ikParameters=None)
        else:
            startPose = self.getPlanningStartPose()
            poseA = self.getRaisedArmPose(startPose, side)
            poseB = self.getPreDropHighPose(startPose, side)
            poseC = self.getPreDropLowPose(startPose, side)
    
            plan = self.ikPlanner.computeMultiPostureGoal([startPose, poseA, poseB, poseC])
        self.addPlan(plan)

    def planDropPostureLower(self, side):
        startPose = self.getPlanningStartPose()
        # removed this as it duplicated calls to rrtStar planner:
        #if self.planner == 'RRT*':
        #    stanceFrame = list(self.footstepPlanner.getFeetMidPoint(self.robotStateModel).GetPosition())
        #    stanceFrame.extend(self.footstepPlanner.getFeetMidPoint(self.robotStateModel).GetOrientationWXYZ())
        #    plan = self.ikPlanner.ikServer.planNominalPose(self.graspingHand, list(startPose), list(stanceFrame))
        #else:
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
            f = transformUtils.frameFromPositionAndRPY( np.array(frame.transform.GetPosition())-np.array([self.reachDist+.15,0,-.03]), [0,0,-90] )
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
        if (self.lockBase is True):
            if (self.lockBack is False):
                print "Currently the combination Base Fixed, Back Free doesn't work"
                print "setting to Base Free, Back Free for collision planning"
                self.lockBase=False
                self.lockBack=False

        frameObj = om.findObjectByName( 'table goal frame')

        print 'planning reach to table object (Collision Free)'

        startPose = self.getPlanningStartPose()   
        
        if self.planner != 'RRT*':
            self.constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, side, frameObj.transform, lockBase=self.lockBase, lockBack=self.lockBack)
            self.constraintSet.runIk()
            self.constraintSet.ikParameters.usePointwise = False
            self.constraintSet.ikParameters.useCollision = True
            
        self.teleopPanel.endEffectorTeleop.updateCollisionEnvironment()
            
        obj, objFrame = self.getNextTableObject(side)
        
        plan = self.constraintSet.runIkTraj()
        self.addPlan(plan)

    def planMotionFromFinalPose(self):
        self.teleopPanel.endEffectorTeleop.terminateFinalPosePlanning()
        self.syncIkPlannerOptions()
        plan = self.constraintSet.runIkTraj()
        self.addPlan(plan)
    
    def initFinalPose(self, func):
        self.syncIkPlannerOptions()
        self.lockBase=False
        self.lockBack=False
        self.teleopPanel.endEffectorTeleop.initFinalPosePlanning()
        ee = om.findObjectByName('Final Pose End Effector')
        transform = func()
        ee.getChildFrame().copyFrame(transform)

    def getTableObjectFrame(self):
        obj, frame = self.getNextTableObject(self.graspingHand)
        transform = transformUtils.copyFrame(frame.transform)
        transform.PreMultiply()
        transform.Concatenate(transformUtils.frameFromPositionAndRPY([0,0,0],[0,90,-90]))
        return transform
    
    def getLiftOffsetFrame(self):
        eeFrame = self.ikPlanner.getLinkFrameAtPose(self.ikPlanner.getHandLink('left'), self.getPlanningStartPose())
        t = transformUtils.copyFrame(eeFrame)
        t.PreMultiply()
        t.Concatenate(self.ikPlanner.getPalmToHandLink())
        t.PostMultiply()
        t.Translate(0, 0, 0.03)
        return t

    def getWithdrawFrame(self):
        eeFrame = om.findObjectByName('Final Pose End Effector frame')
        t = transformUtils.copyFrame(eeFrame.transform)
        t.PostMultiply()
        if self.graspingHand == 'left':
            t.Translate(-0.1, 0.3, 0)
        else:            
            t.Translate(-0.1, -0.3, 0)
        return t
    
    def syncIkPlannerOptions(self):
        if self.planner == 'RRT*':
            ikplanner.getIkOptions().setProperty('Use collision', 'RRT*')
        elif self.planner == 'RRT-Connect':
            ikplanner.getIkOptions().setProperty('Use collision', 'RRT Connect')
        
        if self.graspingHand == 'left':
            ikplanner.getIkOptions().setProperty('RRT hand', 'left')
        else:
            ikplanner.getIkOptions().setProperty('RRT hand', 'right')
    
    def createConstraintSet(self):
        eeTeleop = self.teleopPanel.endEffectorTeleop
        ikPlanner = self.ikPlanner

        startPoseName = 'reach_start'
        startPose = self.getPlanningStartPose()
        ikPlanner.addPose(startPose, startPoseName)

        constraints = []
        constraints.append(ikPlanner.createQuasiStaticConstraint())
        constraints.append(ikPlanner.createLockedNeckPostureConstraint(startPoseName))
        constraints.append(ikPlanner.createFixedLinkConstraints(startPoseName, ikPlanner.leftFootLink, tspan=[0.0, 1.0], lowerBound=-0.0001*np.ones(3), upperBound=0.0001*np.ones(3), angleToleranceInDegrees=0.1))
        constraints.append(ikPlanner.createFixedLinkConstraints(startPoseName, ikPlanner.rightFootLink, tspan=[0.0, 1.0], lowerBound=-0.0001*np.ones(3), upperBound=0.0001*np.ones(3), angleToleranceInDegrees=0.1))
        if self.lockBack:
            constraints.append(ikPlanner.createLockedBackPostureConstraint(startPoseName))
            ikPlanner.setBackLocked(True)
        else:
            constraints.append(ikPlanner.createMovingBackPostureConstraint())
            ikPlanner.setBackLocked(False)

        if self.lockBase:
            constraints.append(ikPlanner.createLockedBasePostureConstraint(startPoseName, lockLegs=False))
            ikPlanner.setBaseLocked(True)
        else:
            constraints.append(ikPlanner.createKneePostureConstraint(drcargs.getDirectorConfig()['kneeJointLimits']))
            ikPlanner.setBaseLocked(False)
        
        self.constraintSet = ikplanner.ConstraintSet(ikPlanner, constraints, 'reach_end', startPoseName)

        handLinks = []
        for handModel in ikPlanner.handModels: handLinks.append(handModel.handLinkName)

        for constraint in constraints:
            if hasattr(constraint, 'linkName') and constraint.linkName in handLinks:
                continue

            if isinstance(constraint, ikplanner.ik.PositionConstraint):
                frameObj = self.getGoalFrame(constraint.linkName)
                if frameObj:
                    constraint.referenceFrame = frameObj.transform

            elif isinstance(constraint, ikplanner.ik.QuatConstraint):
                frameObj = self.getGoalFrame(constraint.linkName)
                if frameObj:
                    constraint.quaternion = frameObj.transform

            elif isinstance(constraint, ikplanner.ik.WorldGazeDirConstraint):
                frameObj = self.getGoalFrame(constraint.linkName)
                if frameObj:
                    constraint.targetFrame = frameObj.transform

        om.removeFromObjectModel(eeTeleop.getConstraintFolder())
        folder = eeTeleop.getConstraintFolder()
        
        from ddapp.teleoppanel import ConstraintItem
        for i, pc in enumerate(constraints):
            constraintItem = ConstraintItem(pc)
            om.addToObjectModel(constraintItem, parentObj=folder)
    
    def searchFinalPose(self):
        self.createConstraintSet()
        self.teleopPanel.endEffectorTeleop.updateCollisionEnvironment()
        frame = om.findObjectByName('Final Pose End Effector frame')
        handTransform = transformUtils.copyFrame(frame.transform)
        handTransform.PreMultiply()
        palmToHand = self.ikPlanner.getPalmToHandLink(self.graspingHand)
        palmToHand = palmToHand.GetLinearInverse()
        handTransform.Concatenate(palmToHand)
        endPose, info = self.constraintSet.searchFinalPose(self.graspingHand, handTransform)
        if info == 1:
            self.teleopPanel.showPose(self.constraintSet.endPose)
        else:
            'No final pose found. Please try with a different end effector pose'
        
    def planTouchTableObject(self, side='left'):

        obj, frame = self.getNextTableObject(side)
        startPose = self.getPlanningStartPose()

        if self.ikPlanner.fixedBaseArm: # includes distance hack and currently uses reachDist instead of touchDist (TODO!)
            f = transformUtils.frameFromPositionAndRPY( np.array(frame.transform.GetPosition())-np.array([self.reachDist+.05,0,-0.03]), [0,0,-90] )
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


    def planLiftTableObject(self, side='left'):

        startPose = self.getPlanningStartPose()
        
        print 'planning lift'  
        
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

        plan = self.constraintSet.runIkTraj()
        self.addPlan(plan)


    def planWithdrawTableObject(self):
        endPose = self.ikPlanner.getMergedPostureFromDatabase(self.getPlanningStartPose(), 'General', 'handsdown incl back')
        endPose = self.ikPlanner.getMergedPostureFromDatabase(endPose, 'General', 'arm up pregrasp', self.graspingHand)
        self.createConstraintSet()
        self.ikPlanner.addPose(endPose, 'reach_end')
        self.planMotionFromFinalPose()

#        t = vtk.vtkTransform()
#        t.PostMultiply()
#        if self.graspingHand == 'left':
#            t.Translate(0.1, 0.5, 1)
#        else:
#            t.Translate(0.1, -0.5, 1)
#        t.Concatenate(om.findObjectByName('table stance frame').transform)
#
#        print 'planning Withdraw'
#
#        plan = self.ikPlanner.runMultiRRT(list(startPose),
#                                          list(np.append(t.GetPosition(), t.GetOrientationWXYZ())), True, ikParameters=None)
#        self.addPlan(plan)


    ### End Planning Functions ####################################################################
    ########## Glue Functions #####################################################################
    def teleportRobotToStanceFrame(self, frame):
        if self.planner != 'RRT*':
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

    def createCollisionPlanningScene(self, scene=0, loadPerception=True, moveRobot=False):
        self.createCollisionPlanningSceneMain(self.sceneID,loadPerception,moveRobot)
        filename= os.environ['DRC_BASE'] + '/software/models/worlds/directorAffordances.sdf'
        import ipab
        msg=ipab.scs_api_command_t()
        msg.command="loadSDF "+filename+"\nsimulate"
        lcmUtils.publish('SCS_API_CONTROL', msg)

    def createCollisionPlanningSceneMain(self, scene=0, loadPerception=True, moveRobot=False):
        om.removeFromObjectModel(om.findObjectByName('affordances'))
        om.removeFromObjectModel(om.findObjectByName('segmentation'))

        if (self.sceneID is not None):
            # use variable if one exists
            scene = self.sceneID


        if (scene == 4):
            filename = os.environ['DRC_BASE'] + '/../drc-testing-data/ihmc_table/ihmc_table.vtp'
            polyData = ioUtils.readPolyData( filename )
            vis.showPolyData( polyData,'scene')
            self.segmentIhmcScene()

            relativeStance = transformUtils.frameFromPositionAndRPY([-0.6, 0, 0],[0,0,0])
            relativeReachGoal = transformUtils.frameFromPositionAndRPY([-0.19,0.4,0.16],[90,90,0])
            self.computeTableStanceFrame(relativeStance)
            self.computeCollisionGoalFrame(relativeReachGoal)

            if (moveRobot):
                self.moveRobotToTableStanceFrame()
            return

        elif (scene == 0):
            pose = (array([ 1.20,  0. , 0.8]), array([ 1.,  0.,  0.,  0.]))
            desc = dict(classname='BoxAffordanceItem', Name='table', uuid=newUUID(), pose=pose, Color=[0.66, 0.66, 0.66], Dimensions=[0.5,1,0.06])
            obj = self.affordanceManager.newAffordanceFromDescription(desc)
            pose = (array([ 1.20,  0.5 , 0.4]), array([ 1.,  0.,  0.,  0.]))
            desc = dict(classname='BoxAffordanceItem', Name='scene0_leg1', uuid=newUUID(), pose=pose, Color=[0.66, 0.66, 0.66], Dimensions=[0.5,0.05,0.8])
            obj = self.affordanceManager.newAffordanceFromDescription(desc)
            pose = (array([ 1.20,  -0.5 , 0.4]), array([ 1.,  0.,  0.,  0.]))
            desc = dict(classname='BoxAffordanceItem', Name='scene0_leg2', uuid=newUUID(), pose=pose, Color=[0.66, 0.66, 0.66], Dimensions=[0.5,0.05,0.8])
            obj = self.affordanceManager.newAffordanceFromDescription(desc)
            pose = (array([ 1.05,  0.3 , 0.98]), array([ 1.,  0.,  0.,  0.]))
            desc = dict(classname='BoxAffordanceItem', Name='scene0_object1', uuid=newUUID(), pose=pose, Color=[0.9, 0.9, 0.1], Dimensions=[0.08,0.08,0.24])
            obj1 = self.affordanceManager.newAffordanceFromDescription(desc)
            pose = (array([ 1.25,  0.1 , 0.98]), array([ 1.,  0.,  0.,  0.]))
            desc = dict(classname='BoxAffordanceItem', Name='scene0_object2', uuid=newUUID(), pose=pose, Color=[0.0, 0.9, 0.0], Dimensions=[0.07,0.07,0.25])
            obj2 = self.affordanceManager.newAffordanceFromDescription(desc)
            pose = (array([ 1.25,  -0.1 , 0.95]), array([ 1.,  0.,  0.,  0.]))
            desc = dict(classname='CylinderAffordanceItem', Name='scene0_object3', uuid=newUUID(), pose=pose, Color=[0.0, 0.9, 0.0], Radius=0.035, Length = 0.22)
            obj3 = self.affordanceManager.newAffordanceFromDescription(desc)
            pose = (array([ 1.05,  -0.2 , 0.95]), array([ 1.,  0.,  0.,  0.]))
            desc = dict(classname='CylinderAffordanceItem', Name='scene0_object4', uuid=newUUID(), pose=pose, Color=[0.9, 0.1, 0.1], Radius=0.045, Length = 0.22)
            obj4 = self.affordanceManager.newAffordanceFromDescription(desc)

            self.clusterObjects = [obj1,obj2, obj3, obj4]
            relativeStance = transformUtils.frameFromPositionAndRPY([-0.58, 0, 0],[0,0,0])
            relativeReachGoal = transformUtils.frameFromPositionAndRPY([-0.15,0.4,0.2],[90,90,0])
            self.computeTableStanceFrame(relativeStance)
            self.computeCollisionGoalFrame(relativeReachGoal)

        elif (scene == 1):
            pose = (array([-0.98873106,  1.50393395,  0.91420001]), array([ 0.49752312,  0.        ,  0.        ,  0.86745072]))
            desc = dict(classname='BoxAffordanceItem', Name='table', uuid=newUUID(), pose=pose, Color=[0.66, 0.66, 0.66], Dimensions=[0.5,1,0.06])
            obj = self.affordanceManager.newAffordanceFromDescription(desc)
            pose = (array([-0.98873106,  1.50393395,  0.57]), array([ 0.49752312,  0.        ,  0.        ,  0.86745072]))
            desc = dict(classname='BoxAffordanceItem', Name='scene1_object1', uuid=newUUID(), pose=pose, Color=[0.005, 0.005, 0.3], Dimensions=[0.05,0.05,0.14])
            obj1 = self.affordanceManager.newAffordanceFromDescription(desc)

            self.clusterObjects = [obj1]
            relativeStance = transformUtils.frameFromPositionAndRPY([-0.6, 0, 0],[0,0,0])
            relativeReachGoal = transformUtils.frameFromPositionAndRPY([0,0.1,-0.35],[90,90,0])
            self.computeTableStanceFrame(relativeStance)
            self.computeCollisionGoalFrame(relativeReachGoal)

        elif (scene == 2):
            pose = (array([ 0.49374956,  1.51828255,  0.84852654]), array([ 0.86198582,  0.        ,  0.        ,  0.50693238]))
            desc = dict(classname='BoxAffordanceItem', Name='table', uuid=newUUID(), pose=pose, Color=[0.66, 0.66, 0.66], Dimensions=[0.5,1,0.06])
            obj = affordancepanel.panel.affordanceFromDescription(desc)
            pose = (array([ 0.57555491,  1.6445656 ,  1.02993633]), array([ 0.86280979,  0.        ,  0.        ,  0.50552871]))
            desc = dict(classname='BoxAffordanceItem', Name='scene2_object', uuid=newUUID(), pose=pose, Color=[0.005, 0.005, 0.3], Dimensions=[0.05,0.05,0.3])
            obj1 = affordancepanel.panel.affordanceFromDescription(desc)

            pose = (array([ 0.38458635,  1.32625758,  1.47444768]), array([ 0.86314205,  0.        ,  0.        ,  0.50496119]))
            desc = dict(classname='BoxAffordanceItem', Name='scene2_wall1', uuid=newUUID(), pose=pose, Color=[0.66, 0.66, 0.66], Dimensions=[0.05,1.0,0.4])
            obj = affordancepanel.panel.affordanceFromDescription(desc)

            pose = (array([ 0.08282192,  1.49589397,  1.07518917]), array([ 0.86314205,  0.        ,  0.        ,  0.50496119]))
            desc = dict(classname='BoxAffordanceItem', Name='scene2_wall2', uuid=newUUID(), pose=pose, Color=[0.66, 0.66, 0.66], Dimensions=[0.05,0.3,0.39])
            obj = affordancepanel.panel.affordanceFromDescription(desc)

            pose = (array([ 0.69532105,  1.15157858,  1.07518917]), array([ 0.86314205,  0.        ,  0.        ,  0.50496119]))
            desc = dict(classname='BoxAffordanceItem', Name='scene2_wall3', uuid=newUUID(), pose=pose, Color=[0.66, 0.66, 0.66], Dimensions=[0.05,0.3,0.39])
            obj = affordancepanel.panel.affordanceFromDescription(desc)

            self.clusterObjects = [obj1]
            relativeStance = transformUtils.frameFromPositionAndRPY([-0.65, -0.3, 0],[0,0,0])
            relativeReachGoal = transformUtils.frameFromPositionAndRPY([0.15,0.07,0.14],[90,90,0])
            self.computeTableStanceFrame(relativeStance)
            self.computeCollisionGoalFrame(relativeReachGoal)

        elif (scene == 3):
            pose = (array([-0.69, -1.50,  0.92]), array([-0.707106781,  0.        ,  0.        ,  0.707106781 ]))
            desc = dict(classname='BoxAffordanceItem', Name='table', uuid=newUUID(), pose=pose, Color=[0.66, 0.66, 0.66], Dimensions=[0.5,1,0.06])
            obj = self.affordanceManager.newAffordanceFromDescription(desc)
            pose = (array([-1.05, -1.10,  0.95]), array([-0.707106781,  0.        ,  0.        ,  0.707106781 ]))
            desc = dict(classname='BoxAffordanceItem', Name='scene3_edge1', uuid=newUUID(), pose=pose, Color=[0.66, 0.66, 0.66], Dimensions=[0.1,0.3,0.05])
            obj = self.affordanceManager.newAffordanceFromDescription(desc)
            pose = (array([-0.35, -1.10,  0.95]), array([-0.707106781,  0.        ,  0.        ,  0.707106781 ]))
            desc = dict(classname='BoxAffordanceItem', Name='scene3_edge2', uuid=newUUID(), pose=pose, Color=[0.66, 0.66, 0.66], Dimensions=[0.1,0.3,0.05])
            obj = self.affordanceManager.newAffordanceFromDescription(desc)
            pose = (array([-0.6803156 , -1.1826616 ,  1.31299839]), array([-0.707106781,  0.        ,  0.        ,  0.707106781 ]))
            desc = dict(classname='BoxAffordanceItem', Name='scene3_edge3', uuid=newUUID(), pose=pose, Color=[0.66, 0.66, 0.66], Dimensions=[0.14,1.0,0.07])
            obj = self.affordanceManager.newAffordanceFromDescription(desc)
            pose = (array([ -0.7,  -1.5 , 1.03]), array([ 1.,  0.,  0.,  0.]))
            desc = dict(classname='BoxAffordanceItem', Name='scene3_object1', uuid=newUUID(), pose=pose, Color=[0.9, 0.9, 0.1], Dimensions=[0.05,0.05,0.14])
            obj1 = self.affordanceManager.newAffordanceFromDescription(desc)

            self.clusterObjects = [obj1]
            relativeStance = transformUtils.frameFromPositionAndRPY([-0.7, -0.1, 0],[0,0,0])
            relativeReachGoal = transformUtils.frameFromPositionAndRPY([0.0,0.07,0.14],[90,90,0])
            self.computeTableStanceFrame(relativeStance)
            self.computeCollisionGoalFrame(relativeReachGoal)

        self.userFitBin()
        self.onSegmentBin( np.array([ 0.62, -1.33, 0.80]), np.array([ 0.89, -0.87, 0.57]) )
        self.computeBinStanceFrame()

        if (moveRobot):
            self.moveRobotToTableStanceFrame()

        if (loadPerception):
            filename = os.environ['DRC_BASE'] + '/../drc-testing-data/ihmc_table/'+str(scene)+'.vtp'
            pd = ioUtils.readPolyData( filename )
            vis.showPolyData(pd,'scene')
        
        sc = sceneloader.SceneLoader()
        sc.generateSDFfromAffordances()

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
#        box = obj.findChild(obj.getProperty('Name') + ' box')
        box = obj
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
    def loadSDFFileAndRunSim(self):
        filename= os.environ['DRC_BASE'] + '/software/models/worlds/tabledemo.sdf'
        sc=sceneloader.SceneLoader()
        sc.loadSDF(filename)
        import ipab
        msg=ipab.scs_api_command_t()
        msg.command="loadSDF "+filename+"\nsimulate"
        lcmUtils.publish('SCS_API_CONTROL', msg)

    def prepGetSceneFrame(self, createNewObj=False):
        if createNewObj:
            objScene = vis.showPolyData(self.getInputPointCloud(), 'scene', colorByName='rgb_colors')
        else:
            objScene = vis.updatePolyData(self.getInputPointCloud(), 'scene', colorByName='rgb_colors')


    def prepKukaTestDemoSequence(self, inputFile=None):
        if inputFile is None:
            inputFile = os.environ['DRC_BASE'] + '/../drc-testing-data/tabletop/kinect_collision_environment.vtp'
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

        filename = os.environ['DRC_BASE'] + '/../drc-testing-data/tabletop/table-and-bin-scene.vtp'
        scene = ioUtils.readPolyData(filename)
        vis.showPolyData(scene,"scene")

        #stanceFrame = transformUtils.frameFromPositionAndRPY([0, 0, 0], [0, 0, 123.0])
        #self.teleportRobotToStanceFrame(stanceFrame)

        self.userFitTable()
        self.onSegmentTable( np.array([-1.72105646,  2.73210716,  0.79449952]), np.array([-1.67336452,  2.63351011,  0.78698605]) )
        self.userFitBin()
        self.onSegmentBin( np.array([-0.02, 2.43, 0.61 ]), np.array([-0.40,  2.79,  0.61964661]) )

        relativeStance = transformUtils.frameFromPositionAndRPY([-0.65, 0, 0],[0,0,0])
        self.computeTableStanceFrame(relativeStance)
        self.computeBinStanceFrame()

        # Actually plan the sequence:
        #self.demoSequence()


    def segmentIhmcScene(self):
        self.userFitBin()
        self.onSegmentBin( np.array([ 0.62, -1.33, 0.80]), np.array([ 0.89, -0.87, 0.57]) )
        self.userFitTable()
        self.onSegmentTable( np.array([ 1.11, 0.11, 0.85]), np.array([ 0.97, 0.044, 0.84]) )

        self.segmentTableObjects()
        self.computeBinStanceFrame()

    def planSequence(self):
        self.useFootstepPlanner = True

        self.cleanupFootstepPlans()
        self.planFromCurrentRobotState = False
        self.segmentTableObjects()
        self.plans = []

        # Go home
        self.planWalkToStance(om.findObjectByName('start stance frame').transform)

        # Pick Objects from table:
        self.planWalkToStance(om.findObjectByName('table stance frame').transform)
        if (self.graspingHand == 'both'):
            self.planSequenceTablePick('left')
            self.planSequenceTablePick('right')
        else:
            self.planSequenceTablePick(self.graspingHand)

        # Go home
        self.planWalkToStance(om.findObjectByName('start stance frame').transform)

        # Go to Bin
        self.planWalkToStance(om.findObjectByName('bin stance frame').transform)

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
        self.planWalkToStance(om.findObjectByName('start stance frame').transform)


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
            self.addTasksToQueueWalking(taskQueue, om.findObjectByName('start stance frame').transform, 'Walk to Start')

        for _ in self.clusterObjects:
            # Pick Objects from table:
            if not self.ikPlanner.fixedBaseArm:
                self.addTasksToQueueWalking(taskQueue, om.findObjectByName('table stance frame').transform, 'Walk to Table')
            taskQueue.addTask(self.printAsync('Pick with Left Arm'))
            self.addTasksToQueueTablePick(taskQueue, 'left')
            #taskQueue.addTask(self.printAsync('Pick with Right Arm'))
            #self.addTasksToQueueTablePick(taskQueue, 'right')

            # Go home
            if not self.ikPlanner.fixedBaseArm:
                self.addTasksToQueueWalking(taskQueue, om.findObjectByName('start stance frame').transform, 'Walk to Start')

            # Go to Bin
            if not self.ikPlanner.fixedBaseArm:
                self.addTasksToQueueWalking(taskQueue, om.findObjectByName('bin stance frame').transform, 'Walk to Bin')

            # Drop into the Bin:
            taskQueue.addTask(self.printAsync('Drop from Left Arm'))
            self.addTasksToQueueDropIntoBin(taskQueue, 'left')
            #taskQueue.addTask(self.printAsync('Drop from Right Arm'))
            #self.addTasksToQueueDropIntoBin(taskQueue, 'right')

            # Go home
            if not self.ikPlanner.fixedBaseArm:
                self.addTasksToQueueWalking(taskQueue, om.findObjectByName('start stance frame').transform, 'Walk to Start')
        
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
            taskQueue.addTask( om.findObjectByName('table stance frame').transform )
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

'''
Tabledemo Image Fit for live-stream of webcam
'''
class TableImageFitter(ImageBasedAffordanceFit):

    def __init__(self, tableDemo):
        ImageBasedAffordanceFit.__init__(self, numberOfPoints=1)
        self.tableDemo = tableDemo

    def fit(self, polyData, points):
        pass

'''
Table Task Panel
'''
class TableTaskPanel(TaskUserPanel):

    def __init__(self, tableDemo):

        TaskUserPanel.__init__(self, windowTitle='Table Task')

        self.tableDemo = tableDemo
        self.tableDemo.planFromCurrentRobotState = True

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()

        self.fitter = TableImageFitter(self.tableDemo)
        self.initImageView(self.fitter.imageView, activateAffordanceUpdater=False)

    def addButtons(self):

        self.addManualSpacer()
        self.addManualButton('Lower arm', functools.partial(self.tableDemo.planLowerArm, self.tableDemo.graspingHand))
        self.addManualSpacer()
        self.addManualButton('Raise arm', self.tableDemo.planPreGrasp)
        self.addManualSpacer()
        self.addManualButton('Commit Manip', self.tableDemo.commitManipPlan)
        self.addManualSpacer()
        self.addManualButton('Open Hand', functools.partial(self.tableDemo.openHand, self.tableDemo.graspingHand))
        self.addManualSpacer()
        self.addManualButton('Close Hand', functools.partial(self.tableDemo.closeHand, self.tableDemo.graspingHand))
        self.addManualSpacer()
        self.addManualButton('Read SDF & Sim', self.tableDemo.loadSDFFileAndRunSim)

    def addDefaultProperties(self):
        # TODO: if there is a way not to display a property where there is only one value, that'd be great

        if len(drcargs.getDirectorConfig()['handCombinations']) > 1:  # more than one hand
            self.params.addProperty('Hand', 0,
                                attributes=om.PropertyAttributes(enumNames=['Left', 'Right']))
        else:
            # Hard-coded left for now per convention, can also be generic per drcargs.getDirectorConfig()['handCombinations']['side']
            self.params.addProperty('Hand', 0,
                                attributes=om.PropertyAttributes(enumNames=['Left']))

        if self.tableDemo.ikPlanner.fixedBaseArm:
            self.params.addProperty('Base', 0,
                                attributes=om.PropertyAttributes(enumNames=['Fixed']))
            self.params.addProperty('Back', 0,
                                attributes=om.PropertyAttributes(enumNames=['Fixed']))
        else: # floating base
            self.params.addProperty('Base', 1,
                                attributes=om.PropertyAttributes(enumNames=['Fixed', 'Free']))
            self.params.addProperty('Back', 1,
                                    attributes=om.PropertyAttributes(enumNames=['Fixed', 'Free']))

        # Hand control for Kuka LWR / Schunk SDH
        if 'userConfig' in drcargs.getDirectorConfig() and 'useKuka' in drcargs.getDirectorConfig()['userConfig']:
            self.params.addProperty('Hand Engaged (Powered)', False)

        # If we're dealing with humanoids, offer the scene selector
        if not self.tableDemo.ikPlanner.fixedBaseArm:
            self.params.addProperty('Scene', 0, attributes=om.PropertyAttributes(enumNames=['Objects on table','Object below table','Object through slot','Object at depth','Objects on table (fit)']))
            self.params.addProperty('Planner', 1, attributes=om.PropertyAttributes(enumNames=['RRT-Connect', 'RRT*']))

        # Init values as above
        self.tableDemo.graspingHand = self.getSide()
        self.tableDemo.lockBase = self.getLockBase()
        self.tableDemo.lockBack = self.getLockBack()
        self.tableDemo.sceneID = self.getSceneId()
        self.tableDemo.sceneName = self.getSceneName()
        self.tableDemo.planner = self.getPlanner()
        if 'userConfig' in drcargs.getDirectorConfig() and 'useKuka' in drcargs.getDirectorConfig()['userConfig']:
            self.handEngaged = self.getHandEngaged() # WARNING: does not check current state [no status message]

    def getSide(self):
        return self.params.getPropertyEnumValue('Hand').lower()

    def getLockBase(self):
        return True if self.params.getPropertyEnumValue('Base') == 'Fixed' else False

    def getLockBack(self):
        return True if self.params.getPropertyEnumValue('Back') == 'Fixed' else False

    def getHandEngaged(self):
        return self.params.getProperty('Hand Engaged (Powered)')

    def getSceneId(self):
        return self.params.getProperty('Scene') if self.params.hasProperty('Scene') else None

    def getSceneName(self):
        return self.params.getPropertyEnumValue('Scene') if self.params.hasProperty('Scene') else None

    def getPlanner(self):
        return self.params.getPropertyEnumValue('Planner') if self.params.hasProperty('Planner') else None

    def onPropertyChanged(self, propertySet, propertyName):
        propertyName = str(propertyName)

        if propertyName == 'Hand':
            self.tableDemo.graspingHand = self.getSide()
            self.taskTree.removeAllTasks()
            self.addTasks()

        elif propertyName == 'Base':
            self.tableDemo.lockBase = self.getLockBase()

        elif propertyName == 'Back':
            self.tableDemo.lockBack = self.getLockBack()

        elif propertyName == 'Hand Engaged (Powered)':
            if self.handEngaged: # was engaged, hence deactivate
                self.tableDemo.getHandDriver(self.getSide()).sendDeactivate() # deactivate hand
            else: # was disenaged, hence activate
                self.tableDemo.getHandDriver(self.getSide()).sendActivate() # activate hand
            self.handEngaged = self.getHandEngaged()

        elif propertyName == 'Scene':
            self.tableDemo.sceneID = self.params.getProperty('Scene')
            self.tableDemo.sceneName = self.params.getPropertyEnumValue('Scene')
		
        elif propertyName == 'Planner':
            self.tableDemo.planner = self.params.getPropertyEnumValue('Planner')
        self.syncIkPlannerOptions()

    def pickupMoreObjects(self):
        if len(self.tableDemo.clusterObjects) > 0: # There is still sth on the table, let's do it again!
            print "There are more objects on the table, going at it again!"
            self.taskTree.selectTaskByName('reach')
            self.onPause()
            self.onContinue()
        else:
            print "Table clear, sir!"

    def addTasks(self):

        # some helpers
        def addTask(task, parent=None):
            self.taskTree.onAddTask(task, copy=False, parent=parent)

        def addFunc(func, name, parent=None, confirm=False):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)
            if confirm:
                addTask(rt.UserPromptTask(name='Confirm execution has finished', message='Continue when plan finishes.'), parent=parent)

        def addManipulation(func, name, parent=None, confirm=False):
            group = self.taskTree.addGroup(name, parent=parent)
            addFunc(func, name='plan motion', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            if self.tableDemo.planner != 1:
                addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'), parent=group)
                if confirm:
                   addTask(rt.UserPromptTask(name='Confirm execution has finished', message='Continue when plan finishes.'), parent=group)
                        
        def addManipulationWithFinalPose(func, name, parent=None, confirm=False):
            group = self.taskTree.addGroup(name, parent=parent)
            addFunc(functools.partial(v.initFinalPose, func), 'init final pose planning', parent=group)
            addTask(rt.UserPromptTask(name='approve end-effector pose', message='Please approve end-effector pose.'), parent=group)
            addFunc(v.searchFinalPose, 'search final pose', parent=group)
            addFunc(v.planMotionFromFinalPose, name='plan motion', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            if self.tableDemo.planner != 1:
                addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'), parent=group)
                if confirm:
                   addTask(rt.UserPromptTask(name='Confirm execution has finished', message='Continue when plan finishes.'), parent=group)

        def addGrasping(mode, name, parent=None, confirm=False):
            assert mode in ('open', 'close')
            group = self.taskTree.addGroup(name, parent=parent)
            side = self.params.getPropertyEnumValue('Hand')

            checkStatus = False  # whether to confirm that there is an object in the hand when closed
            if 'userConfig' in drcargs.getDirectorConfig() and 'useKuka' in drcargs.getDirectorConfig()['userConfig']:
                checkStatus = True

            if mode == 'open':
                addTask(rt.OpenHand(name='open grasp hand', side=side, CheckStatus=checkStatus), parent=group)
            else:
                addTask(rt.CloseHand(name='close grasp hand', side=side, CheckStatus=checkStatus), parent=group)
            if confirm:
                addTask(rt.UserPromptTask(name='Confirm grasping has succeeded', message='Continue when grasp finishes.'),
                        parent=group)


        v = self.tableDemo

        self.taskTree.removeAllTasks()

        # graspingHand is 'left', side is 'Left'
        side = self.params.getPropertyEnumValue('Hand')

        ###############
        # add the tasks

        # pre-prep
        if v.ikPlanner.fixedBaseArm:
            if not v.useDevelopment:
                addManipulation(functools.partial(v.planPostureFromDatabase, 'roomMapping', 'p3_down', side='left'), 'go to pre-mapping pose')
        # TODO(wxm): mapping

        # prep
        prep = self.taskTree.addGroup('Preparation')
        if v.ikPlanner.fixedBaseArm:
            addGrasping('open', name='open hand', parent=prep, confirm=False)
            if v.useDevelopment:
                addFunc(v.prepKukaTestDemoSequence, 'prep from file', parent=prep)
            else:
                # get one frame from camera, segment on there
                addFunc(v.prepGetSceneFrame, 'capture scene frame', parent=prep)
                addFunc(v.prepKukaLabScene, 'prep kuka lab scene', parent=prep)
        else:
            addFunc(v.disableJointChecker, 'disable joint checker', parent =  prep)
            addFunc(v.createCollisionPlanningScene, 'prep from file', parent=prep)
            if v.planner != 'RRT*':
                addTask(rt.CloseHand(name='close grasp hand', side=side), parent=prep)
                addTask(rt.CloseHand(name='close left hand', side='Left'), parent=prep)
                addTask(rt.CloseHand(name='close right hand', side='Right'), parent=prep)
            else:
                addFunc(v.planLowerArm, 'set arms down', parent=prep)

        # walk
        if not v.ikPlanner.fixedBaseArm:
            walk = self.taskTree.addGroup('Approach Table')
            addTask(rt.RequestFootstepPlan(name='plan walk to table', stanceFrameName='table stance frame'), parent=walk)
            addTask(rt.UserPromptTask(name='approve footsteps', message='Please approve footstep plan.'), parent=walk)
            addTask(rt.CommitFootstepPlan(name='walk to table', planName='table grasp stance footstep plan'), parent=walk)
            addTask(rt.WaitForWalkExecution(name='wait for walking'), parent=walk)
            addTask(rt.SetNeckPitch(name='set neck position', angle=35), parent=walk)

        # lift object
        if v.ikPlanner.fixedBaseArm:
            addManipulation(functools.partial(v.planPreGrasp, v.graspingHand ), name='raise arm')
            addManipulation(functools.partial(v.planReachToTableObject, v.graspingHand), name='reach')
            addManipulation(functools.partial(v.planTouchTableObject, v.graspingHand), name='touch')
        elif v.planner == 'RRT*':
            addManipulationWithFinalPose(v.getTableObjectFrame, name='reach')
            addFunc(functools.partial(v.graspTableObject, side=v.graspingHand), 'grasp', parent='reach', confirm=False)
#            addManipulationWithFinalPose(v.getLiftOffsetFrame, name='lift object')
#            addFunc(v.planWithdrawTableObject, 'withdraw object', parent='lift object', confirm=False)
        else:
            addManipulation(functools.partial(v.planReachToTableObjectCollisionFree, v.graspingHand), name='reach')
            addFunc(functools.partial(v.graspTableObject, side=v.graspingHand), 'grasp', parent='reach', confirm=True)
            addManipulation(functools.partial(v.planLiftTableObject, v.graspingHand), name='lift object')

        # walk to start
        if not v.ikPlanner.fixedBaseArm and v.planner != 'RRT*':
            walkToStart = self.taskTree.addGroup('Walk to Start')
            addTask(rt.RequestFootstepPlan(name='plan walk to start', stanceFrameName='start stance frame'), parent=walkToStart)
            addTask(rt.UserPromptTask(name='approve footsteps',
                                      message='Please approve footstep plan.'), parent=walkToStart)
            addTask(rt.CommitFootstepPlan(name='walk to start',
                                          planName='start stance footstep plan'), parent=walkToStart)
            addTask(rt.WaitForWalkExecution(name='wait for walking'), parent=walkToStart)

        # walk to bin
        '''
        if not v.ikPlanner.fixedBaseArm:
            walkToBin = self.taskTree.addGroup('Walk to Bin')
            if v.planner != 'RRT*':
                addTask(rt.RequestFootstepPlan(name='plan walk to bin', stanceFrameName='bin stance frame'), parent=walkToBin)
                addTask(rt.UserPromptTask(name='approve footsteps', message='Please approve footstep plan.'), parent=walkToBin)
                addTask(rt.CommitFootstepPlan(name='walk to start', planName='bin stance footstep plan'), parent=walkToBin)
                addTask(rt.WaitForWalkExecution(name='wait for walking'), parent=walkToBin)
            addFunc(self.tableDemo.moveRobotToBinStanceFrame, 'walk to Bin', parent = walkToBin)

        # drop in bin
        if not v.ikPlanner.fixedBaseArm: # v.ikPlanner.pushToMatlab: # latter statement doesnt work since not yet set when initialising
            addManipulation(functools.partial(v.planDropPostureRaise, v.graspingHand), name='drop: raise arm') # seems to ignore arm side?
        if v.planner == 'RRT*':
            addFunc(functools.partial(v.dropTableObject, side=v.graspingHand), 'drop', parent='drop: release', confirm=False)
        else:
            addManipulation(functools.partial(v.planPostureFromDatabase, 'table clearing', 'pre drop 2', side=v.graspingHand), 'drop: lower arm')
            addFunc(functools.partial(v.dropTableObject, side=v.graspingHand), 'drop', parent='drop: release', confirm=True)

        if not v.ikPlanner.fixedBaseArm:
            addManipulation(functools.partial(v.planDropPostureLower, v.graspingHand), name='drop: lower arm')
        else:
            addManipulation(functools.partial(v.planPreGrasp, v.graspingHand), name='go home')

        addFunc(self.pickupMoreObjects, 'clear until table empty', parent='clear until table empty folder')
        '''
