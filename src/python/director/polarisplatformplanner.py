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
from director import ikplanner
from director import ioUtils
from director.simpletimer import SimpleTimer
from director.utime import getUtime
from director import affordanceitems
from director import robotstate
from director import robotplanlistener
from director import planplayback
from director import affordanceupdater
from director import segmentationpanel
from director import segmentation
from director import terraintask
from director import footstepsdriverpanel
from director.footstepsdriver import FootstepRequestGenerator
from director import vtkNumpy as vnp


from director.tasks.taskuserpanel import TaskUserPanel
from director.tasks.taskuserpanel import ImageBasedAffordanceFit

import director.tasks.robottasks as rt
import director.tasks.taskmanagerwidget as tmw

import drc as lcmdrc
import copy

from PythonQt import QtCore, QtGui





class PolarisPlatformPlanner(object):
    def __init__(self, ikServer, robotSystem):
        self.ikServer = ikServer
        self.robotSystem = robotSystem
        self.terrainTask = terraintask.TerrainTask(robotSystem)
        self.initializedFlag = False
        self.plans = []

    def initialize(self):
        self.initializedFlag = True
        self.updateFramesAndAffordance()
        self.setFoostepData()
        self.setFootstepDataForwards()
        self.footstepRequestGenerator = FootstepRequestGenerator(self.robotSystem.footstepsDriver)
        #define the frames we need relative to the box frame etc
        self.numFootsteps = 2
        self.minHoldTime = 2

    def updateAffordance(self):
        self.platform = om.findObjectByName('running board')
        self.dimensions = np.array(self.platform.getProperty('Dimensions'))

    def updateFramesAndAffordance(self):
        self.updateAffordance()
        self.getPlanningFrame()
        self.requestRaycastTerrain()

    # this method should set the planning frame
    def getPlanningFrame(self):
        platformToWorld = self.platform.getChildFrame().transform
        worldToPlatform = platformToWorld.GetLinearInverse()
        f = self.robotSystem.footstepsDriver.getFeetMidPoint(self.robotSystem.robotStateModel)
        footPosition = f.GetPosition()
        footPosInPlatformFrame = worldToPlatform.TransformPoint(footPosition)

        planFramePosInPlatformFrame = self.dimensions/2.0
        planFramePosInPlatformFrame[1] = footPosInPlatformFrame[1]
        planFramePosInWorld = platformToWorld.TransformPoint(planFramePosInPlatformFrame)
        # now we want to find the homogeneous transform for the planning Frame
        _,quat = transformUtils.poseFromTransform(platformToWorld)
        self.planToWorld = transformUtils.transformFromPose(planFramePosInWorld,quat)

    # returns the f to plan transform, given fToWorld transform
    def getTransformToPlanningFrame(self,fToWorld):
        fToPlan = fToWorld
        fToPlan.PostMultiply()
        fToPlan.Concatenate(self.planToWorld.GetLinearInverse())
        return fToPlan

    def getFootstepRelativeTransform(self):
        self.footstepsToPlan = []
        for n in range(1,self.numFootsteps + 1):
            stepFrameName = 'step ' + str(n) + ' frame'
            fToWorld = transformUtils.copyFrame(om.findObjectByName(stepFrameName).transform)
            fToPlan = self.getTransformToPlanningFrame(fToWorld)
            self.footstepsToPlan.append(fToPlan)

    def visFootstepFrames(self):
        for n in range(1,self.numFootsteps + 1):
            fToPlan = self.footstepsToPlan[n-1]
            fToWorld = fToPlan
            fToWorld.PostMultiply()
            fToWorld.Concatenate(self.planToWorld)
            frameName = 'step_'+str(n)+'ToWorld'
            vis.updateFrame(fToWorld,frameName)

    def setFoostepData(self):
        self.footstepPosition = []

        self.footstepPosition.append(np.array([-0.19052019522393965, -0.16752527574088918, 0.07678844136281959 ]))
        self.footstepPosition.append(np.array([-0.2111150611750166, 0.1621390575248655, 0.07571540514427666]))
        self.footstepPosition.append(np.array([ 0.19315482042625953, -0.2866541182385602, 0.016873465171285976]))
        self.footstepPosition.append(np.array([ 0.13708399594888881, 0.1522408848113495, 0.008706862136780541 ]))

        # note that this is in radians, transform to degrees
        self.footstepYaw = np.array([-0.77413819, -0.57931552, -0.75042088, -0.68140433])
        self.footstepYaw = np.rad2deg(self.footstepYaw)

    def setFootstepDataForwards(self):
        self.footstepPositionForwards = []
        self.footstepPositionForwards.append(np.array([-0.08774644,  0.0635555 ,  0.07771066])) # narrow first step
        # self.footstepPositionForwards.append(np.array([-0.06954156,  0.14726368,  0.07522517])) # normal first step
        self.footstepPositionForwards.append(np.array([ 0.18256867, -0.11692981,  0.01602283]))
        self.footstepPositionForwards.append(np.array([ 0.31539397,  0.15317327,  0.04011487]))

        self.footstepYawForwards = np.array([0, 0, 0])
        self.footstepYawForwards = np.rad2deg(self.footstepYawForwards)


    def planTurn(self):
        # request footsteps 1 and 2
        footstepsToWorldList = self.getFootstepToWorldTransforms([0,1])
        q = self.robotSystem.robotStateJointController.q
        request = self.footstepRequestGenerator.makeFootstepRequest(q, footstepsToWorldList, 'right', snapToTerrain=True)
        request.params.map_mode = lcmdrc.footstep_plan_params_t.TERRAIN_HEIGHTS_AND_NORMALS
        request = self.setMapModeToTerrainAndNormals(request)
        self.robotSystem.footstepsDriver.sendFootstepPlanRequest(request)

    def planStepDown(self):
        footstepsToWorldList = self.getFootstepToWorldTransforms([3])
        q = self.robotSystem.robotStateJointController.q
        request = self.footstepRequestGenerator.makeFootstepRequest(q, footstepsToWorldList, 'left', snapToTerrain=True)
        self.robotSystem.footstepsDriver.sendFootstepPlanRequest(request)

    def planStepOff(self):
        footstepsToWorldList = self.getFootstepToWorldTransforms([2])
        q = self.robotSystem.robotStateJointController.q
        request = self.footstepRequestGenerator.makeFootstepRequest(q, footstepsToWorldList, 'right', snapToTerrain=True)
        self.robotSystem.footstepsDriver.sendFootstepPlanRequest(request)

    def planWeightShift(self):
        ikPlanner = self.robotSystem.ikPlanner
        startPoseName = 'plan_start'
        endPoseName = 'plan_end'
        startPose = self.robotSystem.robotStateJointController.q
        ikPlanner.addPose(startPose, startPoseName)
        constraints = ikPlanner.createMovingBodyConstraints(startPoseName, lockBack=True, lockLeftArm=True, lockRightArm=True)
        constraints[0].rightFootEnabled = False
        constraints[0].shrinkFactor=0.1
        constraints.append(ikPlanner.createKneePostureConstraint([1, 2.5]))
        cs = ikplanner.ConstraintSet(ikPlanner, constraints, endPoseName, startPoseName)
        endPose, info = cs.runIk()
        ikPlanner.computeMultiPostureGoal([startPose, endPose])

    # maybe should update the frame and affordance everytime we call a method?
    def planStepDownForwards(self):
        # need to make us step left foot forwards
        # set the walking defaults to be what we want
        q = self.getPlanningStartPose()
        footstepsToWorldList = self.getFootstepToWorldTransforms([0,1], stepOffDirection='forwards')
        request = self.footstepRequestGenerator.makeFootstepRequest(q, footstepsToWorldList, 'left', snapToTerrain=True)
        request.goal_steps[0].params.support_contact_groups = lcmdrc.footstep_params_t.SUPPORT_GROUPS_HEEL_MIDFOOT
        self.robotSystem.footstepsDriver.sendFootstepPlanRequest(request)

    def planStepOffForwards(self):
        q = self.getPlanningStartPose()
        footstepsToWorldList = self.getFootstepToWorldTransforms([2], stepOffDirection='forwards')
        request = self.footstepRequestGenerator.makeFootstepRequest(q, footstepsToWorldList, 'left', snapToTerrain=True)
        self.robotSystem.footstepsDriver.sendFootstepPlanRequest(request)

    def planWeightShiftForwards(self):
        pass

    def setMinHoldTime(self, request, minHoldTime):


        for stepMessages in request.goal_steps:
            stepMessages.params.drake_min_hold_time = minHoldTime

        return request

    def switchToPolarisPlatformParameters(self):
        self.robotSystem.footstepsDriver.params.setProperty('Defaults', 'Polaris Platform')

    #get the footsteps to world transform from footstepsToPlan transform
    def getFootstepToWorldTransforms(self,footstepIdx, stepOffDirection='sideways'):
        self.updateFramesAndAffordance()
        footstepsToWorldList = []
        for j in footstepIdx:
            if stepOffDirection == 'sideways':
                rpy = np.array([0,0,self.footstepYaw[j]])
                position = self.footstepPosition[j]
            else:
                rpy = np.array([0,0,0], self.footstepYawForwards[j])
                position = self.footstepPositionForwards[j]

            footstepToPlan = transformUtils.frameFromPositionAndRPY(position,rpy)
            footstepToWorld = footstepToPlan
            footstepToWorld.PostMultiply();
            footstepToWorld.Concatenate(self.planToWorld)
            footstepsToWorldList.append(footstepToWorld)

        return footstepsToWorldList


    def setMapModeToTerrainAndNormals(self,request):
        request.params.map_mode = lcmdrc.footstep_plan_params_t.TERRAIN_HEIGHTS_AND_NORMALS
        return request


    def spawnRunningBoardAffordance(self):

        boxDimensions = [0.4, 1.0, 0.05]
        stanceFrame = self.robotSystem.footstepsDriver.getFeetMidPoint(self.robotSystem.robotStateModel, useWorldZ=False)

        boxFrame = transformUtils.copyFrame(stanceFrame)
        boxFrame.PreMultiply()
        boxFrame.Translate(0.0, 0.0, -boxDimensions[2]/2.0)

        box = om.findObjectByName('running board')
        if not box:
            pose = transformUtils.poseFromTransform(boxFrame)
            desc = dict(classname='BoxAffordanceItem', Name='running board', Dimensions=boxDimensions, pose=pose)
            box = self.robotSystem.affordanceManager.newAffordanceFromDescription(desc)

        return box

    def fitRunningBoardAtFeet(self):

        # get stance frame
        startPose = self.getPlanningStartPose()
        stanceFrame = self.robotSystem.footstepsDriver.getFeetMidPoint(self.robotSystem.robotStateModel, useWorldZ=False)
        stanceFrameAxes = transformUtils.getAxesFromTransform(stanceFrame)

        # get pointcloud and extract search region covering the running board
        polyData = segmentation.getCurrentRevolutionData()
        polyData = segmentation.applyVoxelGrid(polyData, leafSize=0.01)
        _, polyData = segmentation.removeGround(polyData)
        polyData = segmentation.cropToBox(polyData, stanceFrame, [1.0, 1.0, 0.1])

        if not polyData.GetNumberOfPoints():
            print('empty search region point cloud')
            return

        vis.updatePolyData(polyData, 'running board search points', parent=segmentation.getDebugFolder(), color=[0,1,0], visible=False)

        # extract maximal points along the stance x axis
        perpAxis = stanceFrameAxes[0]
        edgeAxis = stanceFrameAxes[1]
        edgePoints = segmentation.computeEdge(polyData, edgeAxis, perpAxis)
        edgePoints = vnp.getVtkPolyDataFromNumpyPoints(edgePoints)
        vis.updatePolyData(edgePoints, 'edge points', parent=segmentation.getDebugFolder(), visible=True)

        # ransac fit a line to the edge points
        linePoint, lineDirection, fitPoints = segmentation.applyLineFit(edgePoints)
        if np.dot(lineDirection, stanceFrameAxes[1]) < 0:
            lineDirection = -lineDirection

        linePoints = segmentation.thresholdPoints(fitPoints, 'ransac_labels', [1.0, 1.0])
        dists = np.dot(vnp.getNumpyFromVtk(linePoints, 'Points')-linePoint, lineDirection)
        p1 = linePoint + lineDirection*np.min(dists)
        p2 = linePoint + lineDirection*np.max(dists)
        vis.updatePolyData(fitPoints, 'line fit points', parent=segmentation.getDebugFolder(), colorByName='ransac_labels', visible=False)


        # compute a new frame that is in plane with the stance frame
        # and matches the orientation and position of the detected edge
        origin = np.array(stanceFrame.GetPosition())
        normal = np.array(stanceFrameAxes[2])

        # project stance origin to edge, then back to foot frame
        originProjectedToEdge = linePoint + lineDirection*np.dot(origin - linePoint, lineDirection)
        originProjectedToPlane = segmentation.projectPointToPlane(originProjectedToEdge, origin, normal)
        zaxis = np.array(stanceFrameAxes[2])
        yaxis = np.array(lineDirection)
        xaxis = np.cross(yaxis, zaxis)
        xaxis /= np.linalg.norm(xaxis)
        yaxis = np.cross(zaxis, xaxis)
        yaxis /= np.linalg.norm(yaxis)

        d = DebugData()
        d.addSphere(p1, radius=0.005)
        d.addSphere(p2, radius=0.005)
        d.addLine(p1, p2)
        d.addSphere(originProjectedToEdge, radius=0.001, color=[1,0,0])
        d.addSphere(originProjectedToPlane, radius=0.001, color=[0,1,0])
        d.addLine(originProjectedToPlane, origin, color=[0,1,0])
        d.addLine(originProjectedToEdge, origin, color=[1,0,0])
        vis.updatePolyData(d.getPolyData(), 'running board edge', parent=segmentation.getDebugFolder(), colorByName='RGB255', visible=False)

        # update the running board box affordance position and orientation to
        # fit the detected edge
        box = self.spawnRunningBoardAffordance()
        boxDimensions = box.getProperty('Dimensions')
        t = transformUtils.getTransformFromAxesAndOrigin(xaxis, yaxis, zaxis, originProjectedToPlane)
        t.PreMultiply()
        t.Translate(-boxDimensions[0]/2.0, 0.0, -boxDimensions[2]/2.0)
        box.getChildFrame().copyFrame(t)

        self.initialize()

    #passthrough methods to the terrain task
    # should force updating the affordance before doing this
    def requestRaycastTerrain(self):
        self.terrainTask.requestRaycastTerrain()

    def spawnGroundAffordance(self):
        self.terrainTask.spawnGroundAffordance()

    def spawnFootplaneGroundAffordance(self):
        self.terrainTask.spawnFootplaneGroundAffordance('right')

    def planArmsUp(self, stepOffDirection):
        ikPlanner = self.robotSystem.ikPlanner
        startPose = self.getPlanningStartPose()
        if stepOffDirection == 'forwards':
            endPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'hands-forward', side='left')
            endPose = ikPlanner.getMergedPostureFromDatabase(endPose, 'General', 'hands-forward', side='right')
        else:
            endPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'polaris_step_arm_safe', side='left')
            endPose = ikPlanner.getMergedPostureFromDatabase(endPose, 'General', 'polaris_step_arm_safe', side='right')
        plan = ikPlanner.computeMultiPostureGoal([startPose, endPose])
        self.addPlan(plan)

    def addPlan(self, plan):
        self.plans.append(plan)

    def commitManipPlan(self):
        self.robotSystem.manipPlanner.commitManipPlan(self.plans[-1])

    def getPlanningStartPose(self):
        return self.robotSystem.robotStateJointController.q.copy()

    def planNominal(self):
        ikPlanner = self.robotSystem.ikPlanner
        startPose = self.getPlanningStartPose()
        endPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'safe nominal')
        endPose, info = ikPlanner.computeStandPose(endPose)
        newPlan = ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def addPlan(self, plan):
        self.plans.append(plan)


class PolarisPlatformPlannerPanel(TaskUserPanel):

    def __init__(self, robotSystem):

        TaskUserPanel.__init__(self, windowTitle='Platform Task')

        self.robotSystem = robotSystem
        self.platformPlanner = PolarisPlatformPlanner(robotSystem.ikServer, robotSystem)
        self.addButtons()
        self.addDefaultProperties()
        self.addTasks()

    def addButtons(self):
        self.addManualButton('Fit Platform Affordance', self.onFitPlatformAffordance)
        self.addManualButton('Spawn Ground Affordance', self.onSpawnGroundAffordance)
        self.addManualButton('Raycast Terrain', self.onRaycastTerrain)
        self.addManualButton('Start', self.onStart)
        self.addManualButton('Update Affordance', self.onUpdateAffordance)
        self.addManualButton('Arms Up',self.onArmsUp)
        self.addManualButton('Plan Turn', self.onPlanTurn)
        self.addManualButton('Plan Step Down', self.onPlanStepDown)
        self.addManualButton('Plan Weight Shift', self.onPlanWeightShift)
        self.addManualButton('Plan Step Off', self.onPlanStepOff)

    def addDefaultProperties(self):
        self.params.addProperty('Step Off Direction', 0, attributes=om.PropertyAttributes(enumNames=['Forwards','Sideways']))
        self._syncProperties()

    def _syncProperties(self):
        self.stepOffDirection = self.params.getPropertyEnumValue('Step Off Direction').lower()

    def onFitPlatformAffordance(self):
        self.platformPlanner.fitRunningBoardAtFeet()

    def onSpawnGroundAffordance(self):
        self.platformPlanner.spawnGroundAffordance()

    def onArmsUp(self):
        self.platformPlanner.planArmsUp(self.stepOffDirection)

    def onRaycastTerrain(self):
        self.platformPlanner.requestRaycastTerrain()

    def onStart(self):
        self.platformPlanner.initialize()

    def onUpdateAffordance(self):
        self.platformPlanner.updateFramesAndAffordance()

    def onPlanTurn(self):
        self._syncProperties()
        self.platformPlanner.planTurn()

    def onPlanStepDown(self):
        self._syncProperties()
        if self.stepOffDirection == 'forwards':
            self.platformPlanner.planStepDownForwards()
        else:
            self.platformPlanner.planStepDown()

    def onPlanWeightShift(self):
        self._syncProperties()
        if self.stepOffDirection == 'forwards':
            self.platformPlanner.planWeightShiftForwards()
        else:
            self.platformPlanner.planWeightShift()

    def onPlanStepOff(self):
        self._syncProperties()
        if self.stepOffDirection == 'forwards':
            self.platformPlanner.planStepOffForwards()
        else:
            self.platformPlanner.planStepOff()

    def addTasks(self):

        # some helpers
        self.folder = None
        def addTask(task, parent=None):
            parent = parent or self.folder
            self.taskTree.onAddTask(task, copy=False, parent=parent)
        def addFunc(func, name, parent=None):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)
        def addFolder(name, parent=None):
            self.folder = self.taskTree.addGroup(name, parent=parent)
            return self.folder
