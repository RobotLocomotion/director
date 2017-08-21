import os
import sys
from . import vtkAll as vtk
import math
import time
import types
import functools
import numpy as np
from collections import defaultdict

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
from director import segmentation
from director import planplayback
from director import affordanceupdater
from director import segmentationpanel
from director import footstepsdriver
from director import footstepsdriverpanel

from director import pointpicker

from director.footstepsdriver import FootstepRequestGenerator

import director.terrain

from director.tasks.taskuserpanel import TaskUserPanel
from director.tasks.taskuserpanel import ImageBasedAffordanceFit

import director.tasks.robottasks as rt
import director.tasks.taskmanagerwidget as tmw

import drc as lcmdrc
import copy
import re
import glob

from PythonQt import QtCore, QtGui



blockWidth = (15 + 3/8.0) * 0.0254
blockLength = (15 + 5/8.0) * 0.0254
blockHeight = (5 + 5/8.0) * 0.0254
blockDiagonal = np.linalg.norm([blockWidth, blockLength, blockHeight])

blockSafetyMargin = [0.03, 0.05]

class TerrainTask(object):

    def __init__(self, robotSystem):
        self.robotSystem = robotSystem
        self.cinderblockPrefix = 'cinderblock'
        self.stairPrefix = 'stair'

        self.currentRow = {'left':-1, 'right':-1}
        self.stanceBlocks = {'left':None, 'right':None}
        self.useTextures = False
        self.constrainBlockSize = True
        self.blockFitAlgo = 1

        self.timer = TimerCallback(targetFps=10)
        self.timer.callback = self.updateBlockState

        self.defaultLeadingFoot = 'right'
        self.removeDuringOrganize = False

        self.numberOfPrefabSteps = -1
        self.prefabStepStartOffset = 0

        self.relativeStartPos = np.array([0,0])
        self.relativeStartYaw = 0

        # terrain config file stuff
        self.terrainConfigDir = os.path.join(director.getDRCBaseDir(), 'software','config','terrain')
        files = glob.glob(os.path.join(self.terrainConfigDir,'*.py'))
        self.terrainConfigList = []
        for f in files:
            self.terrainConfigList.append(os.path.basename(f).split('.')[0])
        if len(self.terrainConfigList)>0:
            self.terrainConfigList.sort()
            self.loadTerrainConfig(self.terrainConfigList[0])

    def startBlockUpdater(self):
        self.timer.start()


    def stopBlockUpdater(self):
        self.timer.stop()


    def requestRaycastTerrain(self):
        affs = self.robotSystem.affordanceManager.getCollisionAffordances()
        xy = self.robotSystem.robotStateJointController.q[:2]
        self.robotSystem.raycastDriver.requestRaycast(affs, xy-4.5, xy+4.5)


    def walkToTiltedCinderblocks(self):
        frame = om.findObjectByName('cinderblock stance frame')
        assert frame

        frameCopy = transformUtils.copyFrame(frame.transform)
        footstepsdriverpanel.panel.onNewWalkingGoal(frameCopy)

    def requestBlockFit(self):
        for block in self.getIndexedBlockAffordances():
            om.removeFromObjectModel(block)
        blockSize = self.terrainConfig['blockSize'].tolist()
        msg = lcmdrc.block_fit_request_t()
        msg.utime = getUtime()
        if self.constrainBlockSize:
            msg.dimensions = blockSize
        else:
            msg.dimensions = [0, 0, blockSize[2]]
        msg.name_prefix = self.terrainConfig['blockName']
        msg.algorithm = self.blockFitAlgo
        lcmUtils.publish('BLOCK_FIT_TRIGGER', msg)

    def spawnGroundAffordance(self):

        polyData = segmentation.getCurrentRevolutionData()
        groundOrigin, normal, groundPoints, _ = segmentation.segmentGround(polyData)

        stanceFrame = FootstepRequestGenerator.getRobotStanceFrame(self.robotSystem.robotStateModel)
        origin = np.array(stanceFrame.GetPosition())

        origin = segmentation.projectPointToPlane(origin, groundOrigin, normal)

        zaxis = normal
        xaxis = transformUtils.getAxesFromTransform(stanceFrame)[0]

        yaxis = np.cross(zaxis, xaxis)
        yaxis /= np.linalg.norm(yaxis)
        xaxis = np.cross(yaxis, zaxis)
        xaxis /= np.linalg.norm(xaxis)

        boxThickness = 0.01

        t = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
        t.PreMultiply()
        t.Translate(0.0, 0.0, -boxThickness/2.0)
        t.PostMultiply()
        t.Translate(origin)

        om.removeFromObjectModel(om.findObjectByName('ground affordance'))
        pose = transformUtils.poseFromTransform(t)
        desc = dict(classname='BoxAffordanceItem', Name='ground affordance', Dimensions=[100, 100, boxThickness], pose=pose)
        aff = segmentation.affordanceManager.newAffordanceFromDescription(desc)
        aff.setProperty('Visible', False)
        aff.setProperty('Alpha', 0.2)

    def spawnFootplaneGroundAffordance(self, side):
        boxThickness = 0.01
        footFrame = self.getFootFrameAtSole(self.sideToFootLinkName(side))
        t = transformUtils.copyFrame(footFrame)
        t.PreMultiply()
        t.Translate(0.0, 0.0, -boxThickness/2.0)

        om.removeFromObjectModel(om.findObjectByName('ground affordance'))
        pose = transformUtils.poseFromTransform(t)
        desc = dict(classname='BoxAffordanceItem', Name='ground affordance', Dimensions=[100, 100, boxThickness], pose=pose)
        aff = segmentation.affordanceManager.newAffordanceFromDescription(desc)
        aff.setProperty('Visible', False)
        aff.setProperty('Alpha', 0.2)


    def getPlanningStartPose(self):
        return self.robotSystem.robotStateJointController.q.copy()


    #
    # TERRAIN SPAWNING STUFF STARTS HERE
    #

    def loadTerrainConfig(self, terrainType):
        # first clear out existing affordances
        if hasattr(self, 'terrainConfig') and self.terrainConfig is not None:
            blocks = self.getAllBlockAffordances()
            if blocks is not None:
                for obj in blocks:
                    om.removeFromObjectModel(obj)

        # load file
        configFile = os.path.join(self.terrainConfigDir, terrainType+'.py')
        self.terrainConfig = {}
        exec(compile(open(configFile).read(), configFile, 'exec'), self.terrainConfig)

    def getAllBlockAffordances(self):
        return [obj for obj in om.getObjects() if obj.getProperty('Name').startswith(self.terrainConfig['blockName'])]

    def getTabularBlockAffordances(self):
        blocks = []
        for obj in om.getObjects():
            name = obj.getProperty('Name')
            if re.match('^%s \(\d+,\d+\)$' % self.terrainConfig['blockName'], name):
                blocks.append(obj)
        return blocks

    def getIndexedBlockAffordances(self):
        blocks = []
        for obj in om.getObjects():
            name = obj.getProperty('Name')
            if re.match(r'^%s \d+$' % self.terrainConfig['blockName'], name):
                blocks.append(obj)
        return blocks

    def createStartingGoal(self):
        blockTable = self.createBlockObjectTable()
        if len(blockTable)>1 and len(blockTable[0])>0:
            firstBlockFrame = transformUtils.copyFrame(blockTable[0][0].getChildFrame().transform)
            secondBlockFrame = transformUtils.copyFrame(blockTable[1][0].getChildFrame().transform)

            secondBlockDir = np.array(secondBlockFrame.GetPosition()) - np.array(firstBlockFrame.GetPosition())
            blockXDir = transformUtils.getAxesFromTransform(firstBlockFrame)[0]
            yawOffset = np.arctan2(secondBlockDir[1],secondBlockDir[0]) - np.arctan2(blockXDir[1],blockXDir[0])
            yawDiff = self.relativeYawOffset - yawOffset
            yawAngle = np.radians(90) * np.round(yawDiff / np.radians(90))

            startingFrame = transformUtils.copyFrame(self.relativeStartFrame)
            startingFrame.PostMultiply()
            startingFrame.RotateZ(np.degrees(-yawAngle))
            startingFrame.Concatenate(firstBlockFrame)

            goalFrame = startingFrame
            footstepsdriverpanel.panel.onNewWalkingGoal(goalFrame)
        else:
            print('error: no blocks defined; use Spawn Terrain button')
            return

    def correctFrameYaw(self, t1, t2):
        R1 = np.array(transformUtils.getAxesFromTransform(t1)).T
        R2 = np.array(transformUtils.getAxesFromTransform(t2)).T
        R = R1.dot(np.round(R1.T.dot(R2)))
        t = transformUtils.getTransformFromAxesAndOrigin(R[:,0],R[:,1],R[:,2],t1.GetPosition())
        return t


    def createFootstepsForTerrain(self):
        footstepData = self.terrainConfig['footstepData']

        # check that we have required data
        blockObjectTable = self.createBlockObjectTable()
        if len(blockObjectTable) == 0:
            print('error: no blocks available for footsteps')
            return

        stanceFrame = FootstepRequestGenerator.getRobotStanceFrame(self.robotSystem.robotStateModel)
        stanceAxes = transformUtils.getAxesFromTransform(stanceFrame)

        # get coord frame of first block
        firstBlockFrame = transformUtils.copyFrame(blockObjectTable[0][0].getChildFrame().transform)
        firstBlockFrame = self.correctFrameYaw(firstBlockFrame, stanceFrame)
        toFirstBlock = firstBlockFrame.GetLinearInverse()

        # get current frames of feet
        leftFootFrame = self.getFootFrameAtSole(self.sideToFootLinkName('left'))
        rightFootFrame = self.getFootFrameAtSole(self.sideToFootLinkName('right'))
        leftPos = np.array(leftFootFrame.GetPosition()[0:2])
        leftLine = np.array(transformUtils.getAxesFromTransform(leftFootFrame)[0][0:2])

        leftLine = np.append(leftLine, -np.dot(leftLine,leftPos))
        rightPos = np.array(rightFootFrame.GetPosition()[0:2])
        rightLine = np.array(transformUtils.getAxesFromTransform(rightFootFrame)[0][0:2])
        rightLine = np.append(rightLine, -np.dot(rightLine,rightPos))

        # generate new footstep frames
        steps = []
        for foot, blockIndex, offset, supportType in footstepData:
            block = blockObjectTable[blockIndex[0]][blockIndex[1]]
            if block is None:
                print('error: no block for footstep (%d,%d)' % blockIndex)
                return
            z = np.array(block.getProperty('Dimensions'))[2]/2.0
            pt = np.array([offset[0], offset[1], z])
            t = transformUtils.copyFrame(block.getChildFrame().transform)
            # TODO: should feet be aligned with forward direction or block?
            blockAxes = transformUtils.getAxesFromTransform(t)
            x = np.round(np.dot(stanceAxes[0], blockAxes[0]))
            y = np.round(np.dot(stanceAxes[0], blockAxes[1]))
            theta = np.degrees(np.arctan2(y,x))

            # compute and apply lateral offset if desired
            if self.terrainConfig['forceZeroLateralFootstepOffset']:
                curBlockFrame = self.correctFrameYaw(t, stanceFrame)
                curBlockToFirst = transformUtils.copyFrame(curBlockFrame)
                R = np.array(transformUtils.getAxesFromTransform(curBlockToFirst)).T
                curBlockToFirst.PostMultiply()
                curBlockToFirst.Concatenate(toFirstBlock)
                p = np.array(curBlockToFirst.GetPosition())
                yDir = np.array(transformUtils.getAxesFromTransform(curBlockToFirst)[1])
                dist = p[1]/yDir[1]
                pt[1] -= dist

            # compute new foot frame
            t.PreMultiply()
            t.RotateZ(theta);
            t.Translate(pt)

            # only append footstep if it is forward of current foot
            pos = np.array(t.GetPosition()[0:2])
            if foot=='left':
                dist = leftLine[0:2].dot(pos)+leftLine[2]
            else:
                dist = rightLine[0:2].dot(pos)+rightLine[2]
            steps.append({'transform':t, 'foot':foot, 'support':supportType, 'dist':dist})
            #obj = vis.showFrame(t, '%s step frame' % block.getProperty('Name'), parent='step frames', scale=0.2)

        # find first valid start step
        startIndex = 0
        for step in steps:
            if (step['dist'] < 0.1):
                startIndex += 1
            else:
                break

        # adjust start index by manually specified offset
        startIndex += self.prefabStepStartOffset
        startIndex = max(0, startIndex)
        startIndex = min(len(steps), startIndex)
        steps = steps[startIndex:]

        # remove footsteps beyond specified limit
        if self.numberOfPrefabSteps >= 0:
            endIndex = min(len(steps), self.numberOfPrefabSteps)
            steps = steps[0:endIndex]

        # send footstep request
        if len(steps) > 0:
            leadingFoot = steps[0]['foot']
        else:
            print('error: no footsteps')
            return
        startPose = self.getPlanningStartPose()
        helper = FootstepRequestGenerator(self.robotSystem.footstepsDriver)
        stepFrames = [step['transform'] for step in steps]
        request = helper.makeFootstepRequest(startPose, stepFrames, leadingFoot,  snapToTerrain=False)
        for i in range(len(stepFrames)):
            request.goal_steps[i].params.support_contact_groups = steps[i]['support']
        self.robotSystem.footstepsDriver.sendFootstepPlanRequest(request, waitForResponse=True)

    def createBlockObjectTable(self):
        types = self.terrainConfig['blockTypes']
        table = [[None for i in range(len(types[0]))] for j in range(len(types))]
        for obj in self.getTabularBlockAffordances():
            name = obj.getProperty('Name')
            m = re.match('%s \((\d+)\,(\d+)\)' % self.terrainConfig['blockName'], name)
            if m is not None:
                row = int(m.group(1))
                col = int(m.group(2))
                table[row][col] = obj
        return table


    def spawnCinderblockTerrain(self, cols=[]):

        # clear old affordances
        for obj in self.getTabularBlockAffordances():
            om.removeFromObjectModel(obj)

        blockTypes = self.terrainConfig['blockTypes']
        blockLevels = self.terrainConfig['blockLevels']
        blockAngleMap = self.terrainConfig['blockAngleMap']
        blockSize = self.terrainConfig['blockSize']

        if len(cols) == 0:
            cols = list(range(len(blockTypes[0])))

        # create frame for starting position
        yaw = self.terrainConfig['startingYaw']
        pos = np.copy(self.terrainConfig['startingPosition'])
        pos[1] -= blockSize[0]*np.mean(np.array(cols))
        offsetFrame = transformUtils.frameFromPositionAndRPY(pos, np.array([0,0,yaw]))

        stanceFrame = FootstepRequestGenerator.getRobotStanceFrame(self.robotSystem.robotStateModel)
        blockFrame = transformUtils.copyFrame(stanceFrame)
        blockFrame.PreMultiply()
        blockFrame.Concatenate(offsetFrame.GetLinearInverse())

        firstBlockFrame = None
        secondBlockFrame = None
        for row in range(len(blockTypes)):
            for col in cols:
                blockType = blockTypes[row][col]
                blockPitch = np.radians(blockAngleMap[blockType][0])
                blockYaw = np.radians(blockAngleMap[blockType][1])
                blockLevel = blockLevels[row][col]

                verticalOffset = blockSize[2]/2.0 + np.sin(blockPitch)*blockSize[1]/2.0

                rpy = np.degrees([0.0, blockPitch, blockYaw])
                pos = [blockSize[1]*row, -blockSize[0]*col, blockSize[2]*blockLevel + verticalOffset]

                offsetFrame = transformUtils.frameFromPositionAndRPY(pos, rpy)
                offsetFrame.PostMultiply()
                offsetFrame.Concatenate(blockFrame)

                if row==0 and col==0:
                    firstBlockFrame = offsetFrame
                if row==1 and col==0:
                    secondBlockFrame = offsetFrame

                pose = transformUtils.poseFromTransform(offsetFrame)
                desc = dict(classname='BoxAffordanceItem', Name='%s (%d,%d)' % (self.terrainConfig['blockName'], row, col), Dimensions=blockSize.tolist(), pose=pose, Color=self.terrainConfig['blockColor'])
                block = self.robotSystem.affordanceManager.newAffordanceFromDescription(desc)

        if firstBlockFrame is not None and secondBlockFrame is not None:
            relativeTransform = transformUtils.copyFrame(stanceFrame)
            relativeTransform.PostMultiply()
            relativeTransform.Concatenate(firstBlockFrame.GetLinearInverse())
            self.relativeStartFrame = relativeTransform
            firstBlockPos = np.array(firstBlockFrame.GetPosition())
            secondBlockPos = np.array(secondBlockFrame.GetPosition())
            secondBlockDir = (secondBlockPos - firstBlockPos)[0:2]
            blockXDir = transformUtils.getAxesFromTransform(firstBlockFrame)[0]
            yawOffset = np.arctan2(secondBlockDir[1],secondBlockDir[0]) - np.arctan2(blockXDir[1],blockXDir[0])
            self.relativeYawOffset = yawOffset

        #for block in self.getCinderblockAffordances():
        #    frameSync.addFrame(block.getChildFrame(), ignoreIncoming=True)

    def showBlocks(self, blocks, visible):
        for b in blocks:
            b.setProperty('Visible', visible)
            b.setProperty('Collision Enabled', visible)


    def assignBlocks(self):
        # get list of ideal blocks and detected blocks
        idealBlocks = self.getTabularBlockAffordances()
        detectedBlocks = self.getIndexedBlockAffordances()

        # check that we have both spawned and detected blocks
        if len(detectedBlocks) == 0:
            print('error: no blocks detected; use Fit Blocks button')
            return
        elif len(idealBlocks) == 0:
            print('error: no blocks spawned; use Spawn Terrain button')
            return

        # hide ideal blocks, show detected blocks
        self.showBlocks(idealBlocks, False)
        self.showBlocks(detectedBlocks, True)

        # pick point in detected blocks
        picker = pointpicker.AffordancePicker(app.getDRCView(), self.robotSystem.affordanceManager, filterFunc=lambda x: x in detectedBlocks)
        picker.callbackFunc = functools.partial(self.assignBlocks2)
        picker.start()

    def assignBlocks2(self, pickedDetectedBlock):
        proceed = len(pickedDetectedBlock)==1

        # get list of ideal blocks and detected blocks
        idealBlocks = self.getTabularBlockAffordances()
        detectedBlocks = self.getIndexedBlockAffordances()

        # hide detected blocks, show ideal blocks
        self.showBlocks(detectedBlocks, not proceed)
        self.showBlocks(idealBlocks, True)

        # TODO: make an abort function
        if not proceed:
            print('cannot proceed: no spawned block was clicked')
            return

        # pick point in ideal blocks
        pickedDetectedBlock = pickedDetectedBlock[0]
        picker = pointpicker.AffordancePicker(app.getDRCView(), self.robotSystem.affordanceManager, filterFunc=lambda x: x in idealBlocks)
        picker.callbackFunc = functools.partial(self.assignBlocks3, pickedDetectedBlock)
        picker.start()

    def assignBlocks3(self, pickedDetectedBlock, pickedIdealBlock):
        proceed = len(pickedIdealBlock)==1
        if not proceed:
            return
        pickedIdealBlock = pickedIdealBlock[0]

        # get list of ideal blocks and detected blocks
        idealBlocks = self.getTabularBlockAffordances()
        detectedBlocks = self.getIndexedBlockAffordances()

        # grab poses of selected blocks
        t1 = transformUtils.copyFrame(pickedIdealBlock.getChildFrame().transform)
        t2 = transformUtils.copyFrame(pickedDetectedBlock.getChildFrame().transform)

        # due to symmetry, find minimal of the 4 possible rotations
        axes1 = transformUtils.getAxesFromTransform(t1)
        axes2 = transformUtils.getAxesFromTransform(t2)
        R1 = np.array(axes1).T
        R2 = np.array(axes2).T
        R = R2.T.dot(R1)
        permIndices = np.argmax(np.fabs(R),axis=1)
        R2 = R2[:,permIndices]
        values = [R[permIndices[0]][0], R[permIndices[1]][1], R[permIndices[2]][2]]
        R2 = R2.dot(np.sign(np.diag(np.array(values))))
        rot = R2.T.dot(R1)

        # find translation so that tops of blocks align
        d1 = pickedIdealBlock.getProperty('Dimensions')
        d2 = pickedDetectedBlock.getProperty('Dimensions')
        pos1 = np.array(t1.GetPosition()) + np.array(axes1[2])*d1[2]/2
        pos2 = np.array(t2.GetPosition()) + np.array(axes2[2])*d2[2]/2

        # compute initial (minimal) transform between selected blocks
        correction = vtk.vtkTransform()
        correction.PostMultiply()
        correction.Translate(-np.array(pos1))
        rotTransform = transformUtils.getTransformFromAxes(rot[0,:],rot[1,:],rot[2,:])
        correction.Concatenate(rotTransform)
        correction.Translate(pos2)

        # move all ideal blocks and reset color
        for b in idealBlocks:
            t = b.getChildFrame().transform
            t.PostMultiply()
            t.Concatenate(correction)
            t.Modified()
            b.setProperty('Color', self.terrainConfig['blockColor'])

        # associate blocks (only use xy distances)
        matches = []
        blockSize = self.terrainConfig['blockSize']
        for b in detectedBlocks:
            t1 = b.getChildFrame().transform
            p1 = np.array(t1.GetPosition())
            normal1 = np.array(transformUtils.getAxesFromTransform(t1)[2])
            bestMatch = None
            minDist = 1e10
            for ib in idealBlocks:
                t2 = ib.getChildFrame().transform
                p2 = np.array(t2.GetPosition())
                dist = np.linalg.norm(p2[0:2]-p1[0:2])
                if dist < minDist:
                    minDist = dist
                    bestMatch = ib
            if minDist < np.linalg.norm(blockSize[0:2])/4:
                t2 = bestMatch.getChildFrame().transform
                normal2 = np.array(transformUtils.getAxesFromTransform(t2)[2])
                if np.arccos(np.dot(normal1,normal2)) > np.radians(20):
                    print('warning: normal mismatch between %s and %s' % (b.getProperty('Name'), bestMatch.getProperty('Name')))
                    continue
                matches.append((b,bestMatch))
                bestMatch.setProperty('Color', self.terrainConfig['blockColorMatched'])


        # compute and apply transform using all matches
        correction = vtk.vtkTransform()
        #if False:
        if len(matches) > 2:
            pts1 = np.zeros((len(matches),3))
            pts2 = np.zeros((len(matches),3))
            norms1 = np.zeros((0,3))
            norms2 = np.zeros((0,3))
            for i in range(len(matches)):
                match = matches[i]
                t1 = transformUtils.copyFrame(match[1].getChildFrame().transform)
                t2 = transformUtils.copyFrame(match[0].getChildFrame().transform)
                t2 = self.correctFrameYaw(t2,t1)
                t1.PreMultiply()
                t1.Translate(0, 0, -match[1].getProperty('Dimensions')[2])
                t2.PreMultiply()
                t2.Translate(0, 0, -match[0].getProperty('Dimensions')[2])
                pts1[i,:] = np.array(t1.GetPosition())
                pts2[i,:] = np.array(t2.GetPosition())
                norms1 = np.vstack((norms1,np.array(transformUtils.getAxesFromTransform(t1))))
                norms2 = np.vstack((norms2,np.array(transformUtils.getAxesFromTransform(t2))))
            mean1 = np.mean(pts1,axis=0)
            mean2 = np.mean(pts2,axis=0)
            rays1 = pts1-mean1
            rays1 = np.vstack((rays1,norms1))
            rays2 = pts2-mean2
            rays2 = np.vstack((rays2,norms2))
            u,_,v = np.linalg.svd(rays2.T.dot(rays1))
            rot = u.dot(v)
            if np.sign(np.linalg.det(rot)) < 0:
                rot = u.dot(np.diag(np.array([1,1,-1]))).dot(v)
            correction.PostMultiply()
            correction.Translate(-mean1)
            rotTransform = transformUtils.getTransformFromAxes(rot[:,0],rot[:,1],rot[:,2])
            correction.Concatenate(rotTransform)
            correction.Translate(mean2)
        for b in idealBlocks:
            t = b.getChildFrame().transform
            t.PostMultiply()
            t.Concatenate(correction)
            t.Modified()

        # adjust matched blocks
        for match in matches:
            t1 = transformUtils.copyFrame(match[1].getChildFrame().transform)
            t2 = transformUtils.copyFrame(match[0].getChildFrame().transform)
            # adjust frames to be centered at the top face
            t1.PreMultiply()
            t1.Translate(0, 0, -match[1].getProperty('Dimensions')[2])
            t2.PreMultiply()
            t2.Translate(0, 0, -match[0].getProperty('Dimensions')[2])
            correction = vtk.vtkTransform()
            correction.PreMultiply()
            correction.Concatenate(t2)
            correction.Concatenate(t1.GetLinearInverse())
            t = match[1].getChildFrame().transform
            t.PostMultiply()
            t.Concatenate(correction)
            t.Modified()

    #
    # TERRAIN SPAWNING STUFF ENDS HERE
    #


    def spawnTiltedCinderblocks(self):

        for obj in self.getCinderblockAffordances():
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

        for block in self.getCinderblockAffordances():
            frameSync.addFrame(block.getChildFrame(), ignoreIncoming=True)

    def getBoxAffordancesWithNamePrefix(self, prefix):
        affs = []
        for obj in om.getObjects():
            if isinstance(obj, affordanceitems.BoxAffordanceItem) and obj.getProperty('Name').startswith(prefix):
                affs.append(obj)

        return sorted(affs, key=lambda x: x.getProperty('Name'))

    def getStairAffordances(self):
        return self.getBoxAffordancesWithNamePrefix(self.stairPrefix)

    def getAllCinderblockAffordances(self):
        return self.getBoxAffordancesWithNamePrefix(self.cinderblockPrefix)

    def getAffordanceDistanceToFrame(self, affordance, frame):
        return np.linalg.norm(affordance.getChildFrame().transform.GetPosition() - np.array(frame.GetPosition()))

    def sortAffordancesByDistanceToFrame(self, affordances, frame):
        dists = [self.getAffordanceDistanceToFrame(aff, frame) for aff in affordances]
        return [affordances[i] for i in np.argsort(dists)]

    def getAffordanceClosestToFoot(self, affordances, side):
        '''
        Given a list of affordances and string 'left' or 'right', returns
        the affordance closest to the foot frame.  Also returns the foot frame
        and the distance between foot and affordance frames.
        '''
        linkName = {'left':'l_foot', 'right':'r_foot'}[side]
        footFrame = self.getFootFrameAtSole(linkName)

        vis.updateFrame(footFrame, 'foot frame ' + side, scale=0.2, visible=False)

        aff = self.sortAffordancesByDistanceToFrame(affordances, footFrame)[0]
        return aff, footFrame, self.getAffordanceDistanceToFrame(aff, footFrame)


    def getCinderblockUnderFoot(self, affordances, side):
        if not affordances:
            return None

        aff, footFrame, distance = self.getAffordanceClosestToFoot(affordances, side)

        blockFrame = transformUtils.copyFrame(aff.getChildFrame().transform)

        footToBlock = transformUtils.concatenateTransforms([footFrame, blockFrame.GetLinearInverse()])

        xyz = footToBlock.GetPosition()


        #d = DebugData()
        #d.addLine(blockFrame.GetPosition(), footFrame.GetPosition(), color=self.getFootColor(side))
        #vis.updatePolyData(d.getPolyData(), 'foot to block debug %s' % side, colorByName='RGB255')

        if not (-blockHeight*0.5 < xyz[2] < blockHeight):
            return None

        #print 'xyz:', xyz
        #print 'max xy offset:', max([abs(xyz[0]), abs(xyz[1])])
        #print 'max block dim:', max([blockLength/2, blockWidth/2])

        if max(abs(xyz[0]), abs(xyz[1])) > max(blockLength/2.0, blockWidth/2.0):
            return None

        return aff


    def getFootColor(self, side):
        if side == 'left':
            return footstepsdriver.getLeftFootColor()
        else:
            return footstepsdriver.getRightFootColor()


    def getBlockRowColumn(self, blockAffordance):
        return [int(s) for s in re.findall('\d+', blockAffordance.getProperty('Name'))]

    def sideToFootLinkName(self, side):
        return {'left':'l_foot', 'right':'r_foot'}[side]

    def sideToColumn(self, side):
        return ['right', 'left'].index(side)

    def columnToSide(self, column):
        return ['right', 'left'][column]


    def getRearBlocks(self, side):
        row = self.currentRow[side]
        column = self.sideToColumn(side)
        blocks = []
        for block in self.getCinderblockAffordances():
            r, c = self.getBlockRowColumn(block)
            if c == column and r < row:
                blocks.append(block)
        return blocks


    def getCinderblockAffordances(self):
        blocks = []
        for obj in om.getObjects():
            name = obj.getProperty('Name')
            if re.match('^%s \(\d,\d\)$' % self.cinderblockPrefix, name):
                blocks.append(obj)

        return blocks

    def getFitCinderblockAffordances(self):

        blocks = []
        for obj in om.getObjects():
            name = obj.getProperty('Name')
            if re.match('^%s \d$' % self.cinderblockPrefix, name):
                blocks.append(obj)

        return blocks


    def organizeFitBlocks(self):

        self.updateBlockState()

        blocks = self.getFitCinderblockAffordances()

        if not blocks:
            return

        # rename with temp name
        originalPrefix = self.cinderblockPrefix
        tempPrefix = 'temp_%s' % originalPrefix
        self.cinderblockPrefix = tempPrefix

        for i, block in enumerate(blocks):
            name = '%s %d' % (tempPrefix, i)
            block.rename(name)

        # reorient, sort and rename
        self.reorientBlocks(blocks)
        self.sortAndRenameBlocks(blocks)

        # remove blocks under the feet or behind the robot

        blocks = self.getCinderblockAffordances()

        if self.removeDuringOrganize:
            for side in ['left', 'right']:
                if not self.stanceBlocks[side]:
                    continue

                nearBlock = self.getCinderblockUnderFoot(blocks, side)
                om.removeFromObjectModel(nearBlock)
                for block in self.getRearBlocks(side):
                    om.removeFromObjectModel(block)

        # add current row to names and remove temp name
        blocks = self.getCinderblockAffordances()
        if blocks:

            minRow = min([self.getBlockRowColumn(block)[0] for block in blocks])

            for block in blocks:
                r, c = self.getBlockRowColumn(block)
                side = self.columnToSide(c)
                newRow = (r - minRow) + self.currentRow[side] + 1
                block.rename('%s (%d,%d)' % (originalPrefix, newRow , c))

        self.cinderblockPrefix = originalPrefix
        self.updateBlockState()

    def getFrontBlocks(self, side):
        row = self.currentRow[side]
        column = self.sideToColumn(side)
        blocks = []
        for block in self.getCinderblockAffordances():
            r, c = self.getBlockRowColumn(block)
            if c == column and r > row:
                blocks.append(block)
        return blocks


    def hideRearBlocks(self, side):
        for block in self.getRearBlocks(side):
            block.setProperty('Alpha', 0.1)

    def textureFrontBlocks(self, side):
        for block in self.getFrontBlocks(side):
            block.setProperty('Camera Texture Enabled', True)

    def highlightStanceBlock(self, side):
        block = self.stanceBlocks[side]
        if block:
            block.setProperty('Color', self.getFootColor(side))

    def resetBlockRows(self):
        for side in ['left', 'right']:
            self.currentRow[side] = -1
            self.stanceBlocks[side] = None

    def updateCurrentBlockRow(self, side):

        blocks = self.getCinderblockAffordances()
        block = self.getCinderblockUnderFoot(blocks, side)
        self.stanceBlocks[side] = block

        if not block:
            return False

        blockRow, blockColumn = self.getBlockRowColumn(block)

        if blockColumn != self.sideToColumn(side):
            print('matched wrong block for side %s %s' % (side, block.getProperty('Name')))
            return False

        if blockRow > self.currentRow[side]:
            self.currentRow[side] = blockRow
            return True
        return False


    def deleteFrontBlocks(self):

        self.updateBlockState()

        for side in ['left', 'right']:
            for block in self.getFrontBlocks(side):
                om.removeFromObjectModel(block)

    def updateBlockState(self):

        self.resetCinderblockVisualizationProperties()

        for side in ['left', 'right']:
            self.updateCurrentBlockRow(side)
            self.hideRearBlocks(side)
            self.highlightStanceBlock(side)

            if self.useTextures:
                self.textureFrontBlocks(side)


    def getCinderblockAffordanceWithRowColumn(self, row, column):
        name = '%s (%d,%d)' % (self.cinderblockPrefix, row, column)
        return om.findObjectByName(name)


    def spawnFootstepsForCinderblocks(self):

        def flipSide(side):
            return 'right' if side == 'left' else 'left'


        leadingFoot = self.defaultLeadingFoot

        if self.currentRow[flipSide(leadingFoot)] < self.currentRow[leadingFoot]:
            leadingFoot = flipSide(leadingFoot)


        stepFrames = []

        sideToRow = dict(self.currentRow)
        nextSide = leadingFoot

        om.removeFromObjectModel(om.findObjectByName('debug step frames'))

        while True:

            side = nextSide
            nextSide = flipSide(side)
            row = sideToRow[side] + 1
            sideToRow[side] = row

            #print '------------------------'
            #print 'step side:', side
            #print 'step row:', row

            block = self.getCinderblockAffordanceWithRowColumn(row, self.sideToColumn(side))
            neighborBlock = self.getCinderblockAffordanceWithRowColumn(row, self.sideToColumn(flipSide(side)))
            if not block or not neighborBlock:
                break

            d = np.array(block.getProperty('Dimensions'))/2.0
            t = transformUtils.copyFrame(block.getChildFrame().transform)

            yOffset = 0.06
            xOffset = 0.05
            yOffsetSign = -1 if side == 'left' else 1
            xOffsetSign = -1 if side == leadingFoot else 1

            stepOffset = [xOffset*xOffsetSign, yOffset*yOffsetSign]

            pt = stepOffset[0], stepOffset[1], d[2]
            t.PreMultiply()
            t.Translate(pt)
            stepFrames.append(t)

            #obj = vis.showFrame(t, '%s step frame' % block.getProperty('Name'), parent='step frames', scale=0.2)


        startPose = self.getPlanningStartPose()

        helper = FootstepRequestGenerator(self.robotSystem.footstepsDriver)
        request = helper.makeFootstepRequest(startPose, stepFrames, leadingFoot, snapToTerrain=True)

        self.robotSystem.footstepsDriver.sendFootstepPlanRequest(request, waitForResponse=True)


    def printCinderblocksUnderFoot(self):

        blocks = self.getCinderblockAffordances()

        for side in ['left', 'right']:
            block = self.getCinderblockUnderFoot(blocks, side)
            print(side, block.getProperty('Name') if block else None)


    def getFootstepObjects(self):
        folder = om.findObjectByName('footstep plan')
        if not folder:
            return []
        steps = []
        for obj in folder.children():
            if re.match('^step \d$', obj.getProperty('Name')):
                steps.append(obj)
        return steps


    def printFootstepOffsets(self):

        blocks = self.getCinderblockAffordances()
        steps = self.getFootstepObjects()

        for step in steps:
            stepFrame = step.getChildFrame().transform

            block = self.sortAffordancesByDistanceToFrame(blocks, stepFrame)[0]
            print('%s  --> %s' % (step.getProperty('Name'), block.getProperty('Name')))


    def resetCinderblockVisualizationProperties(self):
        for obj in self.getCinderblockAffordances():

            obj.setProperty('Alpha', 1.0)
            obj.setProperty('Camera Texture Enabled', False)
            if not self.useTextures:
                obj.setProperty('Color', [0.8, 0.8, 0.8])

    def reorientBlocks(self, blocks):

        stanceFrame = FootstepRequestGenerator.getRobotStanceFrame(self.robotSystem.robotStateModel)
        forward = transformUtils.getAxesFromTransform(stanceFrame)[0]

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

    def sortAndRenameBlocks(self, blocks):
        """
        Sort the blocks into row and column bins using the robot's stance frame, and rename the accordingly
        """

        '''

        if not blocks:
            return

        blockDescriptions = [block.getDescription() for block in blocks]

        blockRows = defaultdict(lambda: [])

        stanceFrame = FootstepRequestGenerator.getRobotStanceFrame(self.robotSystem.robotStateModel)
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
                block['Name'] = '%s (%d,%d)' % (self.cinderblockPrefix, row_id, col_id)
                del block['uuid']
                self.robotSystem.affordanceManager.newAffordanceFromDescription(block)

        '''

        if not blocks:
            return

        stanceFrame = FootstepRequestGenerator.getRobotStanceFrame(self.robotSystem.robotStateModel)

        T = stanceFrame.GetLinearInverse()
        blockXYZInStance = np.vstack((T.TransformPoint(block.getChildFrame().transform.GetPosition()) for block in blocks))
        minBlockX = blockXYZInStance[:,0].min()

        rowToBlocks = defaultdict(lambda: [])


        # Bin by x (in stance frame)
        for i, block in enumerate(blocks):
            row = int(round((blockXYZInStance[i,0] - minBlockX) / blockLength))
            rowToBlocks[row].append(block)
            print('block %s row bin: %d' % (block.getProperty('Name'), row))

        # Then sort by y (in stance frame)
        for row, rowBlocks in rowToBlocks.items():
            print('---------')
            print('sorting row', row)
            for block in rowBlocks:
                print('  %s' % block.getProperty('Name'))

            yInLocal = [blockXYZInStance[blocks.index(block), 1] for block in rowBlocks]
            print(yInLocal)

            rowBlocks = [rowBlocks[i] for i in np.argsort(yInLocal)]

            print(rowBlocks)
            rowToBlocks[row] = rowBlocks
            print(rowToBlocks[row])


        for row in sorted(rowToBlocks.keys()):
            print('renaming by row,col for row:', row)
            print(rowToBlocks[row])
            for col, block in enumerate(rowToBlocks[row]):
                newName = '%s (%d,%d)' % (self.cinderblockPrefix, row, col)
                print('renaming %s to: %s' % (block.getProperty('Name'), newName))
                block.rename(newName)


    def computeSafeRegions(self):

        om.removeFromObjectModel(om.findObjectByName('Safe terrain regions'))

        blocks = self.getCinderblockAffordances() + self.getStairAffordances()
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

        blocks = self.getCinderblockAffordances()
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

        startPose = self.getPlanningStartPose()

        helper = FootstepRequestGenerator(self.robotSystem.footstepsDriver)
        request = helper.makeFootstepRequest(startPose, stepFrames, leadingFoot)

        self.robotSystem.footstepsDriver.sendFootstepPlanRequest(request, waitForResponse=True)


    def convertStepToSafeRegion(self, step, rpySeed):
        assert step.shape[0] >= 3
        assert step.shape[1] == 3

        shapeVertices = np.array(step).transpose()[:2,:]
        s = director.terrain.PolygonSegmentationNonIRIS(shapeVertices, bot_pts=director.terrain.DEFAULT_FOOT_CONTACTS)

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

    def getFootFrameAtSole(self, linkName):
        leftPoints, rightPoints = self.robotSystem.footstepsDriver.getContactPts()
        footSoleToOrigin = np.mean(leftPoints, axis=0)
        startPose = self.getPlanningStartPose()
        footFrame = self.robotSystem.ikPlanner.getLinkFrameAtPose(linkName, startPose)
        footFrame.PreMultiply()
        footFrame.Translate(footSoleToOrigin)
        return footFrame


    def snapCinderblocksAtFeet(self):

        blocks = self.getBoxAffordancesWithNamePrefix('')

        for side in ['left', 'right']:
            block = self.getCinderblockUnderFoot(blocks, side)

            print('%s block: %s' % (side, block.getProperty('Name') if block else None))
            if not block:
                continue

            blockFrame = block.getChildFrame()
            footFrame = self.getFootFrameAtSole(self.sideToFootLinkName(side))

            footRpy = transformUtils.rollPitchYawFromTransform(footFrame)
            blockRpy =  transformUtils.rollPitchYawFromTransform(blockFrame.transform)
            blockRpy[0] = footRpy[0]
            blockRpy[1] = footRpy[1]

            newBlockFrame = transformUtils.frameFromPositionAndRPY(blockFrame.transform.GetPosition(), np.degrees(blockRpy))
            blockSurfaceFrame = transformUtils.concatenateTransforms([transformUtils.frameFromPositionAndRPY([0.0, 0.0, blockHeight/2.0], [0.0, 0.0, 0.0]), newBlockFrame])

            blockSurfaceOrigin = blockSurfaceFrame.GetPosition()
            blockSurfaceNormal = transformUtils.getAxesFromTransform(blockSurfaceFrame)[2]
            footIntersection = segmentation.intersectLineWithPlane(np.array(footFrame.GetPosition()), np.array([0,0,1]), np.array(blockSurfaceOrigin), np.array(blockSurfaceNormal))
            zoffset = footIntersection[2] - footFrame.GetPosition()[2]

            newBlockFrame.PostMultiply()
            newBlockFrame.Translate(0.0, 0.0, -zoffset)

            blockFrame.copyFrame(newBlockFrame)


    def spawnCinderblocksAtFeet(self):

        for linkName in ['l_foot', 'r_foot']:
            blockFrame = self.getFootFrameAtSole(linkName)
            blockFrame.PreMultiply()
            blockFrame.Translate(0.0, 0.0, -blockHeight/2.0)

            blockId = len(self.getFitCinderblockAffordances())
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
        for i in range(numberOfBlocks):

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

            blockId = len(self.getCinderblockAffordances())
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

    def planArmsUpPre(self):
        ikPlanner = self.robotSystem.ikPlanner
        startPose = self.getPlanningStartPose()
        endPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'arm up pregrasp', side='left')
        endPose = ikPlanner.getMergedPostureFromDatabase(endPose, 'General', 'arm up pregrasp', side='right')
        ikPlanner.computeMultiPostureGoal([startPose, endPose])

    def planArmsUpNarrow(self):
        ikPlanner = self.robotSystem.ikPlanner
        startPose = self.getPlanningStartPose()
        endPose = ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'hands-forward-narrow', side='left')
        endPose = ikPlanner.getMergedPostureFromDatabase(endPose, 'General', 'hands-forward-narrow', side='right')
        ikPlanner.computeMultiPostureGoal([startPose, endPose])

    def commitLastManipPlan(self):
        self.robotSystem.manipPlanner.commitManipPlan(self.robotSystem.manipPlanner.lastManipPlan)

    def commitLastFootstepPlan(self):
        self.robotSystem.footstepsDriver.commitFootstepPlan(self.robotSystem.footstepsDriver.lastFootstepPlan)

    def switchToTerrainParameters(self):
        self.robotSystem.footstepsDriver.params.setProperty('Defaults', 'Terrain')

    def switchToStairsParameters(self):
        self.robotSystem.footstepsDriver.params.setProperty('Defaults', 'Stairs')

    def switchToNominalParameters(self):
        self.robotSystem.footstepsDriver.params.setProperty('Defaults', 'Drake Nominal')

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
        self.addManualButton('Fit Blocks', self.terrainTask.requestBlockFit)
        self.addManualButton('Spawn Terrain', self.terrainTask.spawnCinderblockTerrain)
        self.addManualButton('Assign Blocks', self.terrainTask.assignBlocks)
        self.addManualButton('Prefab Footsteps', self.terrainTask.createFootstepsForTerrain)
        self.addManualButton('Set Starting Goal', self.terrainTask.createStartingGoal)
        self.addManualSpacer()

        #self.addManualButton('Walk to tilted steps', self.terrainTask.walkToTiltedCinderblocks)
        self.addManualButton('Spawn blocks at feet', self.terrainTask.spawnCinderblocksAtFeet)
        self.addManualSpacer()

        self.addManualButton('Organize fit blocks', self.terrainTask.organizeFitBlocks)
        self.addManualButton('Fit ground affordance', self.terrainTask.spawnGroundAffordance)
        self.addManualButton('Raycast terrain', self.terrainTask.requestRaycastTerrain)
        self.addManualButton('Generate footsteps', self.generateFootsteps)
        self.addManualButton('Print footstep offsets', self.terrainTask.printFootstepOffsets)
        self.addManualSpacer()
        self.addManualButton('Delete front blocks', self.terrainTask.deleteFrontBlocks)
        self.addManualButton('Snap foot blocks', self.terrainTask.snapCinderblocksAtFeet)



        #self.addManualSpacer()
        #self.addManualButton('Reorient blocks to robot', self.terrainTask.reorientBlocks)
        #self.addManualButton('Compute safe regions', self.terrainTask.computeSafeRegions)
        self.addManualSpacer()
        self.addManualButton('Arms up', self.terrainTask.planArmsUp)


    def addDefaultProperties(self):
        self.params.addProperty('Terrain Type', 0, attributes=om.PropertyAttributes(enumNames=self.terrainTask.terrainConfigList))
        self.params.addProperty('Number of Steps', self.terrainTask.numberOfPrefabSteps, attributes=om.PropertyAttributes(minimum=-1, maximum=100))
        self.params.addProperty('Start Step Offset', self.terrainTask.prefabStepStartOffset, attributes=om.PropertyAttributes(minimum=-100, maximum=100))
        self.params.addProperty('Block Fit Algo', self.terrainTask.blockFitAlgo, attributes=om.PropertyAttributes(enumNames=['MinArea', 'ClosestSize']))
        self.params.addProperty('Constrain Block Size', self.terrainTask.constrainBlockSize)
        self.params.addProperty('Manual Steps Leading Foot', 1, attributes=om.PropertyAttributes(enumNames=['Left', 'Right']))
        self.params.addProperty('Camera Texture', False, attributes=om.PropertyAttributes(hidden=False))
        self.params.addProperty('Auto Organize', False, attributes=om.PropertyAttributes(hidden=False))


    def onPropertyChanged(self, propertySet, propertyName):
        if propertyName == 'Camera Texture':
            self.terrainTask.useTextures = self.params.getProperty(propertyName)
        elif propertyName == 'Number of Steps':
            self.terrainTask.numberOfPrefabSteps = self.params.getProperty(propertyName)
        elif propertyName == 'Start Step Offset':
            self.terrainTask.prefabStepStartOffset = self.params.getProperty(propertyName)
        elif propertyName == 'Terrain Type':
            terrainConfig = self.terrainTask.terrainConfigList[self.params.getProperty(propertyName)];
            self.terrainTask.loadTerrainConfig(terrainConfig)
            self.addTasks()
        elif propertyName == 'Block Fit Algo':
            self.terrainTask.blockFitAlgo = self.params.getProperty(propertyName)
        elif propertyName == 'Constrain Block Size':
            self.terrainTask.constrainBlockSize = self.params.getProperty(propertyName)
        elif propertyName == 'Manual Steps Leading Foot':
            self.terrainTask.defaultLeadingFoot = self.params.getPropertyEnumValue(propertyName).lower()
        elif propertyName == 'Auto Organize':
            if self.params.getProperty(propertyName):
                self.terrainTask.startBlockUpdater()
            else:
                self.terrainTask.stopBlockUpdater()

    def generateFootsteps(self):

        self.terrainTask.spawnFootstepsForCinderblocks()


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

        self.taskTree.removeAllTasks()

        isStairs = 'stairs' in self.terrainTask.terrainConfigList[self.params.properties.terrain_type]

        def addFit():
            fit = self.taskTree.addGroup("Fit")
            addTask(rt.UserPromptTask(name='Wait for lidar sweep',
                                      message='Please wait for lidar sweep.'), parent=fit)
            addFunc(self.terrainTask.requestBlockFit, "Fit blocks", parent=fit)
            addTask(rt.UserPromptTask(name='approve blocks',
                                      message='Please wait for fit blocks.'), parent=fit)
            addFunc(self.terrainTask.spawnCinderblockTerrain, "Spawn terrain", parent=fit)
            addFunc(self.terrainTask.assignBlocks, "Assign blocks", parent=fit)
            addTask(rt.UserPromptTask(name='Wait for block assignments',
                                      message='Please assign and adjust blocks.'), parent=fit)
            addFunc(self.terrainTask.requestRaycastTerrain, "Request raycast", parent=fit)

        def addManipExecution(planFunc, name, parent):
            addFunc(planFunc, "Plan " + name, parent=parent)
            addTask(rt.UserPromptTask(name='approve manip plan',
                                      message='Please approve manipulation plan.'), parent=parent)
            addFunc(self.terrainTask.commitLastManipPlan, "Commit " + name, parent=parent)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'), parent=parent)

        def addFootsteps(numSteps=-1):
            footsteps = self.taskTree.addGroup("Footsteps")
            if isStairs:
                addManipExecution(self.terrainTask.planArmsUpNarrow, "arms up narrow", footsteps)
            addFunc(functools.partial(self.params.setProperty, "Number of Steps", numSteps), "Set num steps to {:d}".format(numSteps), parent=footsteps)
            addTask(rt.UserPromptTask(name='Number of steps',
                                      message='Please confirm number of steps.'),
                                      parent=footsteps)
            addFunc(self.terrainTask.createFootstepsForTerrain, "Prefab steps", parent=footsteps)
            addTask(rt.UserPromptTask(name='approve footsteps',
                                      message='Please adjust and approve footstep plan.'), parent=footsteps)
            addFunc(self.terrainTask.commitLastFootstepPlan, "Commit footsteps", parent=footsteps)
            addTask(rt.WaitForWalkExecution(name='wait for walking'), parent=footsteps)
            if isStairs:
                addManipExecution(self.terrainTask.planArmsUp, "arms up", footsteps)

        def addApproach():
            approach = self.taskTree.addGroup("Approach")
            if isStairs:
                addManipExecution(self.terrainTask.planArmsUpNarrow, "Arms up narrow", approach)
            else:
                addManipExecution(self.terrainTask.planArmsUp, "Arms up", approach)
            addFunc(self.terrainTask.createStartingGoal, "plan approach to terrain", parent=approach)
            addTask(rt.UserPromptTask(name='approve footsteps',
                                      message='Please approve footstep plan.'), parent=approach)
            addFunc(self.terrainTask.commitLastFootstepPlan, "Commit footsteps", parent=approach)
            addTask(rt.WaitForWalkExecution(name='wait for walking'), parent=approach)
            addTask(rt.UserPromptTask(name='Confirm approach complete',
                                      message='Please confirm approach is complete.'))
            if isStairs:
                addManipExecution(self.terrainTask.planArmsUp, "Arms up", approach)
                addFunc(self.terrainTask.switchToStairsParameters, "Switch to stairs params")
            else:
                addFunc(self.terrainTask.switchToTerrainParameters, "Switch to terrain params")
            addTask(rt.UserPromptTask(name='Disable pressure control',
                                      message='Please disable pressure control & recovery.'), parent=approach)
            addTask(rt.UserPromptTask(name='Set high pressure',
                                      message='Please set high pressure.'), parent=approach)

        addTask(rt.SetNeckPitch(name='set neck position', angle=45), parent=None)
        addFit()
        addApproach()

        for n in self.terrainTask.terrainConfig["numSteps"]:
            addFit()
            addFootsteps(n)

        finish = self.taskTree.addGroup("Finish")
        addTask(rt.UserPromptTask(name='Confirm terrain complete',
                                  message='Please confirm terrain is complete.'), parent=finish)
        addFunc(self.terrainTask.switchToNominalParameters, "Switch to nominal footstep params", parent=finish)
        addTask(rt.UserPromptTask(name='Enable recovery',
                                  message='Please enable recovery.'), parent=finish)
        addTask(rt.UserPromptTask(name='Enable pressure control',
                                  message='Please enable pressure control.'), parent=finish)




