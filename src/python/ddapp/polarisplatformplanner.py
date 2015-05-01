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
from ddapp import affordanceupdater
from ddapp import segmentationpanel
from ddapp import segmentation
from ddapp import terraintask
from ddapp import footstepsdriverpanel
from ddapp.footstepsdriver import FootstepRequestGenerator

import ddapp.terrain

from ddapp.tasks.taskuserpanel import TaskUserPanel
from ddapp.tasks.taskuserpanel import ImageBasedAffordanceFit

import ddapp.tasks.robottasks as rt
import ddapp.tasks.taskmanagerwidget as tmw

import drc as lcmdrc
import copy

from PythonQt import QtCore, QtGui





class PolarisPlatformPlanner(object):
    def __init__(self, ikServer, robotSystem):
        self.ikServer = ikServer
        self.robotSystem = robotSystem
        self.terrainTask = terraintask.TerrainTask(robotSystem)
        self.initializedFlag = False

    def initialize(self):
        self.initializedFlag = True
        self.updateFramesAndAffordance()
        self.setFoostepData()
        self.footstepRequestGenerator = FootstepRequestGenerator(self.robotSystem.footstepsDriver)
        #define the frames we need relative to the box frame etc
        self.numFootsteps = 2

    def updateAffordance(self):
        self.platform = om.findObjectByName('box')
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
        for n in xrange(1,self.numFootsteps + 1):
            stepFrameName = 'step ' + str(n) + ' frame'
            fToWorld = transformUtils.copyFrame(om.findObjectByName(stepFrameName).transform)
            fToPlan = self.getTransformToPlanningFrame(fToWorld)
            self.footstepsToPlan.append(fToPlan)

    def visFootstepFrames(self):
        for n in xrange(1,self.numFootsteps + 1):
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
        request = self.setMapModeToTerrainAndNormals(request)
        self.robotSystem.footstepsDriver.sendFootstepPlanRequest(request)
        
    def planStepOff(self):
        footstepsToWorldList = self.getFootstepToWorldTransforms([2])
        q = self.robotSystem.robotStateJointController.q
        request = self.footstepRequestGenerator.makeFootstepRequest(q, footstepsToWorldList, 'right', snapToTerrain=True)
        request = self.setMapModeToTerrainAndNormals(request)
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


    #get the footsteps to world transform from footstepsToPlan transform
    def getFootstepToWorldTransforms(self,footstepIdx):
        self.updateFramesAndAffordance()
        footstepsToWorldList = []
        for j in footstepIdx:
            rpy = np.array([0,0,self.footstepYaw[j]])
            position = self.footstepPosition[j]
            footstepToPlan = transformUtils.frameFromPositionAndRPY(position,rpy)
            footstepToWorld = footstepToPlan
            footstepToWorld.PostMultiply();
            footstepToWorld.Concatenate(self.planToWorld)
            footstepsToWorldList.append(footstepToWorld)

        return footstepsToWorldList


    def setMapModeToTerrainAndNormals(self,request):
        request.params.map_mode = lcmdrc.footstep_plan_params_t.TERRAIN_HEIGHTS_AND_NORMALS
        return request


    def segmentPlatform(self):
        segmentation.startInteractiveLineDraw([0.4,0.3])

    #passthrough methods to the terrain task
    # should force updating the affordance before doing this
    def requestRaycastTerrain(self):
        self.terrainTask.requestRaycastTerrain()

    def spawnGroundAffordance(self):
        self.terrainTask.spawnGroundAffordance()

    def planArmsUp(self):
        ikPlanner = self.robotSystem.ikPlanner
        startPose = self.getPlanningStartPose()
        # endPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'door', 'hand up tuck', side='left')
        # endPose = ikPlanner.getMergedPostureFromDatabase(endPose, 'door', 'hand up tuck', side='right')
        endPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'general', 'polaris_step_arm_safe', side='left')
        endPose = ikPlanner.getMergedPostureFromDatabase(endPose, 'door', 'polaris_step_arm_safe', side='right')
        ikPlanner.computeMultiPostureGoal([startPose, endPose])

    def getPlanningStartPose(self):
        return self.robotSystem.robotStateJointController.q.copy()


class PolarisPlatformPlannerPanel(TaskUserPanel):
        
    def __init__(self, robotSystem):

        TaskUserPanel.__init__(self, windowTitle='Platform Task')

        self.robotSystem = robotSystem
        self.platformPlanner = PolarisPlatformPlanner(robotSystem.ikServer, robotSystem)
        self.addButtons()
        self.addTasks()

    def addButtons(self):
        self.addManualButton('Fit Platform Affordance', self.onFitPlatformAffordance)
        self.addManualButton('Spawn Ground Affordance', self.onSpawnGroundAffordance)
        self.addManualButton('Raycast Terrain', self.onRaycastTerrain)
        self.addManualButton('Start', self.onStart)
        self.addManualButton('Update Affordance', self.onUpdateAffordance)
        self.addManualButton('Arms Up',self.platformPlanner.planArmsUp)
        self.addManualButton('Plan Turn', self.onPlanTurn)
        self.addManualButton('Plan Step Down', self.onPlanStepDown)
        self.addManualButton('Plan Weight Shift', self.onPlanWeightShift)
        self.addManualButton('Plan Step Off', self.onPlanStepOff)

    def onFitPlatformAffordance(self):
        self.platformPlanner.segmentPlatform()

    def onSpawnGroundAffordance(self):
        self.platformPlanner.spawnGroundAffordance()

    def onRaycastTerrain(self):
        self.platformPlanner.requestRaycastTerrain()

    def onStart(self):
        self.platformPlanner.initialize()

    def onUpdateAffordance(self):
        self.platformPlanner.updateFramesAndAffordance()

    def onPlanTurn(self):
        self.platformPlanner.planTurn()

    def onPlanStepDown(self):
        self.platformPlanner.planStepDown()

    def onPlanWeightShift(self):
        self.platformPlanner.planWeightShift()

    def onPlanStepOff(self):
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