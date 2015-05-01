import os
import sys
import vtkAll as vtk
from ddapp import botpy
import math
import time
import types
import functools
import numpy as np
from collections import defaultdict

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



blockWidth = (15 + 3/8.0) * 0.0254
blockLength = (15 + 5/8.0) * 0.0254
blockHeight = (5 + 5/8.0) * 0.0254

blockSafetyMargin = [0.03, 0.05]

class TerrainTask(object):

    def __init__(self, robotSystem):
        self.robotSystem = robotSystem


    def requestRaycastTerrain(self):
        affs = self.robotSystem.affordanceManager.getCollisionAffordances()
        xy = self.robotSystem.robotStateJointController.q[:2]
        self.robotSystem.raycastDriver.requestRaycast(affs, xy-5, xy+5)


    def walkToTiltedCinderblocks(self):
        frame = om.findObjectByName('cinderblock stance frame')
        assert frame

        frameCopy = transformUtils.copyFrame(frame.transform)
        footstepsdriverpanel.panel.onNewWalkingGoal(frameCopy)


    def spawnGroundAffordance(self):

        polyData = segmentation.getCurrentRevolutionData()
        groundPoints, normal = segmentation.segmentGroundPoints(polyData)
        groundOrigin = segmentation.computeCentroid(groundPoints)

        stanceFrame = FootstepRequestGenerator.getRobotStanceFrame(self.robotSystem.robotStateModel)
        #stanceFrame.PreMultiply()
        #stanceFrame.Translate(2.0, 0.0, 0.0)
        origin = np.array(stanceFrame.GetPosition())

        origin = segmentation.projectPointToPlane(origin, groundOrigin, normal)

        zaxis = normal
        xaxis = transformUtils.getAxesFromTransform(stanceFrame)[0]

        yaxis = np.cross(zaxis, xaxis)
        yaxis /= np.linalg.norm(yaxis)
        xaxis = np.cross(yaxis, zaxis)
        xaxis /= np.linalg.norm(xaxis)

        t = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
        t.PostMultiply()
        t.Translate(origin)


        om.removeFromObjectModel(om.findObjectByName('ground affordance'))
        pose = transformUtils.poseFromTransform(t)
        desc = dict(classname='BoxAffordanceItem', Name='ground affordance', Dimensions=[10, 10, 0.01], pose=pose)
        aff = segmentation.affordanceManager.newAffordanceFromDescription(desc)
        aff.setProperty('Visible', False)
        aff.setProperty('Alpha', 0.2)


    def getPlanningStartPose(self):
        return self.robotSystem.robotStateJointController.q.copy()

    def spawnTiltedCinderblocks(self):

        for obj in self.findBlockObjects():
            om.removeFromObjectModel(obj)

        stanceFrame = FootstepRequestGenerator.getRobotStanceFrame(self.robotSystem.robotStateModel)
        stanceFrame.PreMultiply()
        stanceFrame.Translate(0.25, 0.0, 0.0)

        f = vis.showFrame(stanceFrame, 'cinderblock stance frame', scale=0.2)
        frameSync = vis.FrameSync()
        frameSync.addFrame(f)
        f.frameSync = frameSync

        startPose = self.robotSystem.robotStateJointController.q.copy()

        forwardOffset = 0.25

        relativeFrame = transformUtils.frameFromPositionAndRPY([0.0, blockWidth/2.0, 0.0], [0.0, 0.0, 0.0])
        relativeFrame.PostMultiply()
        relativeFrame.Concatenate(stanceFrame)

        self.spawnTiltedCinderblocksRow(relativeFrame, startSequence=0, numberOfBlocks=4)

        relativeFrame = transformUtils.frameFromPositionAndRPY([0.0, -blockWidth/2.0, 0.0], [0.0, 0.0, 0.0])
        relativeFrame.PostMultiply()
        relativeFrame.Concatenate(stanceFrame)

        self.spawnTiltedCinderblocksRow(relativeFrame, startSequence=3, numberOfBlocks=4)

        for block in self.findBlockObjects():
            frameSync.addFrame(block.getChildFrame(), ignoreIncoming=True)

    def findBlockObjects(self):

        blocks = []
        for obj in om.getObjects():
            if obj.getProperty('Name').startswith('cinderblock ') and obj.getChildFrame():
                blocks.append(obj)

        return sorted(blocks, key=lambda x: x.getProperty('Name'))

    def reorientBlocks(self):
        stanceFrame = FootstepRequestGenerator.getRobotStanceFrame(self.robotSystem.robotStateModel)
        forward = transformUtils.getAxesFromTransform(stanceFrame)[0]

        blocks = self.findBlockObjects()
        for block in blocks:

            blockFrame = block.getChildFrame().transform
            axes = transformUtils.getAxesFromTransform(blockFrame)
            origin = blockFrame.GetPosition()
            axes = [np.array(axis) for axis in axes]
            dims = block.getProperty('Dimensions')
            axisIndex, axis, sign = transformUtils.findTransformAxis(blockFrame, forward)
            if axisIndex == 2:
                continue

            if axisIndex == 0 and sign < 0:
                axes = [-axes[0], -axes[1], axes[2]]
            elif axisIndex == 1:
                dims = [dims[1], dims[0], dims[2]]
                if sign > 0:
                    axes = [axes[1], -axes[0], axes[2]]
                else:
                    axes = [-axes[1], axes[0], axes[2]]

            t = transformUtils.getTransformFromAxesAndOrigin(axes[0], axes[1], axes[2], origin)
            block.getChildFrame().copyFrame(t)
            block.setProperty('Dimensions', dims)
        self.renameAndReorderBlocks(stanceFrame)

    def renameAndReorderBlocks(self, stanceFrame):
        """
        Sort the blocks into row and column bins using the robot's stance frame, and rename the accordingly
        """
        blocks = self.findBlockObjects()
        blockDescriptions = [block.getDescription() for block in blocks]

        blockRows = defaultdict(lambda: [])

        T = stanceFrame.GetLinearInverse()
        blockXYZInStance = np.vstack((T.TransformPoint(block.getChildFrame().transform.GetPosition()) for block in blocks))
        minBlockX = blockXYZInStance[:,0].min()

        # Bin by x (in stance frame)
        for i, block in enumerate(blocks):
            om.removeFromObjectModel(block)
            blockRows[int(round((blockXYZInStance[i,0] - minBlockX) / blockLength))].append(i)
        # Then sort by y (in stance frame)
        for row_id, row in blockRows.iteritems():
            yInLocal = [blockXYZInStance[i, 1] for i in row]
            blockRows[row_id] = [blockDescriptions[row[i]] for i in np.argsort(yInLocal)]

        for row_id in sorted(blockRows.keys()):
            for col_id, block in enumerate(blockRows[row_id]):
                block['Name'] = 'cinderblock ({:d},{:d})'.format(row_id, col_id)
                self.robotSystem.affordanceManager.newAffordanceFromDescription(block)


    def computeSafeRegions(self):

        om.removeFromObjectModel(om.findObjectByName('Safe terrain regions'))
        blocks = self.findBlockObjects()
        for block in blocks:

            d = np.array(block.getProperty('Dimensions'))/2.0
            d[0] -= blockSafetyMargin[0]
            d[1] -= blockSafetyMargin[1]

            t = block.getChildFrame().transform

            pts = [
              [d[0], d[1], d[2]],
              [d[0], -d[1], d[2]],
              [-d[0], -d[1], d[2]],
              [-d[0], d[1], d[2]]
            ]
            #print 'pts:', pts
            pts = np.array([np.array(t.TransformPoint(p)) for p in pts])
            rpySeed = transformUtils.rollPitchYawFromTransform(t)
            #print 'tx pts:', pts
            #print 'rpy seed:', rpySeed

            self.convertStepToSafeRegion(pts, rpySeed)


    def computeManualFootsteps(self):

        leadingFoot = 'right'

        blockIds = [4,0,5,1,6,2,7,3]

        blocks = self.findBlockObjects()
        blocks = [blocks[i] for i in blockIds]

        f = 0.04
        w = 0.04

        offsets = [
          [0.0, w],
          [0.0, -w],
          [f, w],
          [0.0, -w],
          [-f, w],
          [f, -w],
          [0.0, w],
          [0.0, -w],
        ]

        stepFrames = []

        for block, offset in zip(blocks, offsets):
            d = np.array(block.getProperty('Dimensions'))/2.0
            t = transformUtils.copyFrame(block.getChildFrame().transform)
            pt = offset[0], offset[1], d[2]
            t.PreMultiply()
            t.Translate(pt)
            stepFrames.append(t)
            #obj = vis.showFrame(t, '%s step frame' % block.getProperty('Name'), parent='step frames', scale=0.2)

        stanceFrame = FootstepRequestGenerator.getRobotStanceFrame(self.robotSystem.robotStateModel)
        startPose = self.getPlanningStartPose()

        helper = FootstepRequestGenerator(self.robotSystem.footstepsDriver)
        request = helper.makeFootstepRequest(startPose, stepFrames, leadingFoot)

        self.robotSystem.footstepsDriver.sendFootstepPlanRequest(request, waitForResponse=True)


    def convertStepToSafeRegion(self, step, rpySeed):
        assert step.shape[0] >= 3
        assert step.shape[1] == 3

        shapeVertices = np.array(step).transpose()[:2,:]
        s = ddapp.terrain.PolygonSegmentationNonIRIS(shapeVertices, bot_pts=ddapp.terrain.DEFAULT_FOOT_CONTACTS)

        stepCenter = np.mean(step, axis=0)
        startSeed = np.hstack([stepCenter, rpySeed])

        r = s.findSafeRegion(startSeed)

        if r is not None:
            # draw step
            d = DebugData()
            for p1, p2 in zip(step, step[1:]):
                d.addLine(p1, p2)
            d.addLine(step[-1], step[0])

            folder = om.getOrCreateContainer('Safe terrain regions')
            obj = vis.showPolyData(d.getPolyData(), 'step region %d' % len(folder.children()), parent=folder)
            obj.properties.addProperty('Enabled for Walking', True)
            obj.safe_region = r


    def spawnCinderblocksAtFeet(self):

        footSoleToOrigin = np.mean(self.robotSystem.footstepsDriver.getContactPts(), axis=0)
        startPose = self.getPlanningStartPose()

        for linkName in ['l_foot', 'r_foot']:
            blockFrame = self.robotSystem.ikPlanner.getLinkFrameAtPose(linkName, startPose)
            blockFrame.PreMultiply()
            blockFrame.Translate(footSoleToOrigin)
            blockFrame.Translate(0.0, 0.0, -blockHeight/2.0)

            blockId = len(self.findBlockObjects())
            pose = transformUtils.poseFromTransform(blockFrame)
            desc = dict(classname='BoxAffordanceItem', Name='cinderblock %d' % blockId, Dimensions=[blockLength, blockWidth, blockHeight], pose=pose)
            block = self.robotSystem.affordanceManager.newAffordanceFromDescription(desc)


    def spawnTiltedCinderblocksRow(self, relativeFrame, startSequence, numberOfBlocks):

        blocks = []

        tiltAngle = 15

        baseVerticalOffset = blockHeight/2.0 + np.sin(np.radians(tiltAngle))*blockLength/2.0

        forwardOffset = blockLength
        offset = np.array([0.0, 0.0, 0.0])

        footFrames = []
        for i in xrange(numberOfBlocks):

            if i == 2:
                verticalOffset = blockHeight
            else:
                verticalOffset = 0.0

            stepSequence = (i + startSequence) % 4
            if stepSequence == 0:
                l = blockLength
                w = blockWidth
                tiltX = 0.0
                tiltY = -tiltAngle
            elif stepSequence == 1:
                l = blockWidth
                w = blockLength
                tiltX = tiltAngle
                tiltY = 0.0
            elif stepSequence == 2:
                l = blockLength
                w = blockWidth
                tiltX = 0.0
                tiltY = tiltAngle
            elif stepSequence == 3:
                l = blockWidth
                w = blockLength
                tiltX = -tiltAngle
                tiltY = 0.0

            offsetFrame = transformUtils.frameFromPositionAndRPY([forwardOffset*(i+1), 0.0, verticalOffset+baseVerticalOffset], [tiltX, tiltY, 0.0])

            offsetFrame.PostMultiply()
            offsetFrame.Concatenate(relativeFrame)

            #vis.showFrame(offsetFrame, 'cinderblock %d' % i)

            '''
            footOffsetFrame = transformUtils.frameFromPositionAndRPY([0.0, 0.0, blockHeight/2.0 + 0.07], [0.0, 0.0, 0.0])
            footOffsetFrame.PostMultiply()
            footOffsetFrame.Concatenate(offsetFrame)
            footFrames.append(offsetFrame)
            vis.showFrame(footOffsetFrame, 'footstep %d' % i)
            '''

            blockId = len(self.findBlockObjects())
            pose = transformUtils.poseFromTransform(offsetFrame)
            desc = dict(classname='BoxAffordanceItem', Name='cinderblock %d' % blockId, Dimensions=[l, w, blockHeight], pose=pose)
            block = self.robotSystem.affordanceManager.newAffordanceFromDescription(desc)
            blocks.append(block)

        return blocks


    def planArmsUp(self):
        ikPlanner = self.robotSystem.ikPlanner
        startPose = self.getPlanningStartPose()
        endPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'hands-forward', side='left')
        endPose = ikPlanner.getMergedPostureFromDatabase(endPose, 'General', 'hands-forward', side='right')
        ikPlanner.computeMultiPostureGoal([startPose, endPose])


class TerrainImageFitter(ImageBasedAffordanceFit):

    def __init__(self, drillDemo):
        ImageBasedAffordanceFit.__init__(self, numberOfPoints=1)
        self.drillDemo = drillDemo

    def fit(self, polyData, points):
        pass


class TerrainTaskPanel(TaskUserPanel):

    def __init__(self, robotSystem):

        TaskUserPanel.__init__(self, windowTitle='Terrain Task')

        self.robotSystem = robotSystem
        self.terrainTask = TerrainTask(robotSystem)

        self.fitter = TerrainImageFitter(self.terrainTask)
        self.initImageView(self.fitter.imageView)

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()

    def addButtons(self):
        self.addManualButton('Spawn tilted steps', self.terrainTask.spawnTiltedCinderblocks)
        self.addManualButton('Spawn blocks at feet', self.terrainTask.spawnCinderblocksAtFeet)

        self.addManualButton('Walk to tilted steps', self.terrainTask.walkToTiltedCinderblocks)
        self.addManualButton('Spawn manual footsteps', self.terrainTask.computeManualFootsteps)
        self.addManualSpacer()
        self.addManualButton('Fit ground affordance', self.terrainTask.spawnGroundAffordance)
        self.addManualButton('Raycast terrain', self.terrainTask.requestRaycastTerrain)
        self.addManualSpacer()
        self.addManualButton('Reorient blocks to robot', self.terrainTask.reorientBlocks)
        self.addManualButton('Compute safe regions', self.terrainTask.computeSafeRegions)
        self.addManualSpacer()
        self.addManualButton('Arms up', self.terrainTask.planArmsUp)


    def addDefaultProperties(self):
        self._syncProperties()

    def onPropertyChanged(self, propertySet, propertyName):
        self._syncProperties()

    def _syncProperties(self):
        pass

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

