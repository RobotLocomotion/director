import os
import math
import numpy as np
import operator
from . import vtkAll as vtk
from . import vtkNumpy
import functools
import ihmc

from director import lcmUtils
from director import ioUtils
from director import segmentation
from director import objectmodel as om
from director import visualization as vis
from director.debugVis import DebugData
from director import transformUtils
from director import footstepsdriver
from director.debugVis import DebugData
from director import ikplanner
from director import applogic
from director.tasks.taskuserpanel import TaskUserPanel
from director.lcmframe import positionMessageFromFrame

import director.terrain
import director.tasks.robottasks as rt

import drc as lcmdrc
import bot_core
import atlas

from .thirdparty import qhull_2d
from .thirdparty import min_bounding_rect

from PythonQt import QtCore,QtGui


class BlockTop():
    def __init__(self, cornerTransform, rectDepth, rectWidth, rectArea):
        self.cornerTransform = cornerTransform # location of far right corner
        self.rectDepth = rectDepth # length of face away from robot
        self.rectWidth = rectWidth # length of face perpendicular to robot's toes
        self.rectArea = rectArea

    def getCorners(self):
        '''
        Return a 4x3 numpy array representing the world xyz positions of the
        four corners of the block top.  Corners are listed clockwise from far right.
        '''


        width = self.rectWidth
        depth = self.rectDepth

        width = max(width, 0.39)
        #depth = max(depth, 0.38)

        xaxis, yaxis, zaxis = transformUtils.getAxesFromTransform(self.cornerTransform)
        xedge = np.array(xaxis)*depth
        yedge = np.array(yaxis)*width

        c1 = np.array(self.cornerTransform.GetPosition()) + (np.array(yaxis)*self.rectWidth*0.5) - yedge*0.5
        c2 = c1 - xedge
        c3 = c1 - xedge + yedge
        c4 = c1 + yedge

        return np.array([c3, c4, c1, c2])


class Footstep():
    def __init__(self, transform, is_right_foot):
        self.transform = transform
        self.is_right_foot = is_right_foot


class ContinousWalkingDemo(object):
    FOOTSIZE_REDUCTION = 0.04
    FOOT_LENGTH = 0.25 - FOOTSIZE_REDUCTION
    FOOT_WIDTH = 0.15 - FOOTSIZE_REDUCTION
    BACK_FOOT_CONTACT_POINTS = np.array([[-0.5*FOOT_LENGTH, -0.5*FOOT_LENGTH, 0.166666667*0.25, 0.166666667*0.25],
                                  [0.5*FOOT_WIDTH, -0.5*FOOT_WIDTH, 0.5*FOOT_WIDTH, -0.5*FOOT_WIDTH]])

    FRONT_FOOT_CONTACT_POINTS = np.array([[-0.166666667*0.25, -0.166666667*0.25, 0.5*FOOT_LENGTH, 0.5*FOOT_LENGTH],
                                  [0.5*FOOT_WIDTH, -0.5*FOOT_WIDTH, 0.5*FOOT_WIDTH, -0.5*FOOT_WIDTH]])

    FULL_FOOT_CONTACT_POINTS = np.array([[-0.5*FOOT_LENGTH, -0.5*FOOT_LENGTH, 0.5*FOOT_LENGTH, 0.5*FOOT_LENGTH],
                                  [0.5*FOOT_WIDTH, -0.5*FOOT_WIDTH, 0.5*FOOT_WIDTH, -0.5*FOOT_WIDTH]])

    
    def __init__(self, robotStateModel, footstepsPanel, footstepsDriver, playbackPanel, robotStateJointController, ikPlanner, teleopJointController, navigationPanel, cameraView):
        self.footstepsPanel = footstepsPanel
        self.footstepsDriver = footstepsDriver
        self.playbackPanel = playbackPanel
        self.robotStateModel = robotStateModel
        self.robotStateJointController = robotStateJointController
        self.ikPlanner = ikPlanner
        self.teleopJointController = teleopJointController
        self.navigationPanel = navigationPanel
        self.cameraView = cameraView

        # live operation flags
        self.leadingFootByUser = 'Left'
        self.automaticContinuousWalkingEnabled = True
        self.planFromCurrentRobotState = False
        self.chosenTerrain = 'simple'
        self.supportContact = lcmdrc.footstep_params_t.SUPPORT_GROUPS_HEEL_TOE

        self.plans = []
        self.planned_footsteps = []
        self.footStatus = []
        self.footStatus_right = []
        self.footStatus_left = []
        self.tf_robotStatus = None
        self.transforms_series = []
        self.blocks_series = []

        self.new_status = False
        self.footstep_index = -1

        # Smooth Stereo or Raw or Lidar?
        self.processContinuousStereo = False
        self.processRawStereo = False
        self.committedStep = None
        self.useManualFootstepPlacement = False
        self.queryPlanner = True

        self._setupComplete = False


    def _setupOnce(self):
        '''
        This is setup code that is called that first time continuous walking is
        executed.
        '''

        if self._setupComplete:
            return

        # use a different classifier to scott:
        #footContactSubContinuous = lcmUtils.addSubscriber('FOOT_CONTACT_ESTIMATE_SLOW', lcmdrc.foot_contact_estimate_t, self.onFootContactContinuous)
        #footContactSubContinuous.setSpeedLimit(60)

        lcmUtils.addSubscriber('FOOTSTEP_PLAN_RESPONSE', lcmdrc.footstep_plan_t, self.onFootstepPlanContinuous)# additional git decode stuff removed
        lcmUtils.addSubscriber('IHMC_FOOTSTEP_STATUS', ihmc.footstep_status_t, self.onFootstepStatus)
        lcmUtils.addSubscriber('EST_ROBOT_STATE', bot_core.robot_state_t, self.onRobotStatus)
        stepParamsSub = lcmUtils.addSubscriber('ATLAS_STEP_PARAMS', atlas.behavior_step_params_t, self.onAtlasStepParams)
        stepParamsSub.setSpeedLimit(60)

        self.footstepsPanel.driver.applyDefaults('BDI')


    def getRecedingTerrainRegion(self, polyData, linkFrame):
        ''' Find the point cloud in front of the foot frame'''

        #polyData = shallowCopy(polyData)
        points = vtkNumpy.getNumpyFromVtk(polyData, 'Points')
        #vtkNumpy.addNumpyToVtk(polyData, points[:,0].copy(), 'x')
        #vtkNumpy.addNumpyToVtk(polyData, points[:,1].copy(), 'y')
        #vtkNumpy.addNumpyToVtk(polyData, points[:,2].copy(), 'z')

        viewOrigin = linkFrame.TransformPoint([0.0, 0.0, 0.0])
        viewX = linkFrame.TransformVector([1.0, 0.0, 0.0])
        viewY = linkFrame.TransformVector([0.0, 1.0, 0.0])
        viewZ = linkFrame.TransformVector([0.0, 0.0, 1.0])
        polyData = segmentation.labelPointDistanceAlongAxis(polyData, viewX, origin=viewOrigin, resultArrayName='distance_along_foot_x')
        polyData = segmentation.labelPointDistanceAlongAxis(polyData, viewY, origin=viewOrigin, resultArrayName='distance_along_foot_y')
        polyData = segmentation.labelPointDistanceAlongAxis(polyData, viewZ, origin=viewOrigin, resultArrayName='distance_along_foot_z')

        if self.chosenTerrain == 'stairs':
            polyData = segmentation.thresholdPoints(polyData, 'distance_along_foot_x', [0.30, 1.6])
            polyData = segmentation.thresholdPoints(polyData, 'distance_along_foot_y', [-0.45, 0.45])
            polyData = segmentation.thresholdPoints(polyData, 'distance_along_foot_z', [-0.4, 0.9])
        else:
            polyData = segmentation.thresholdPoints(polyData, 'distance_along_foot_x', [0.12, 1.6])
            polyData = segmentation.thresholdPoints(polyData, 'distance_along_foot_y', [-0.4, 0.4])
            polyData = segmentation.thresholdPoints(polyData, 'distance_along_foot_z', [-0.4, 0.4])

        vis.updatePolyData( polyData, 'walking snapshot trimmed', parent='cont debug', visible=True)
        return polyData



    def extractBlocksFromSurfaces(self, clusters, linkFrame):
        ''' find the corners of the minimum bounding rectangles '''
        om.removeFromObjectModel(om.findObjectByName('block corners'))
        om.removeFromObjectModel(om.findObjectByName('foot placements'))
        om.removeFromObjectModel(om.findObjectByName('steps'))
        om.getOrCreateContainer('block corners',om.getOrCreateContainer('continuous'))
        om.getOrCreateContainer('foot placements',om.getOrCreateContainer('continuous'))
        om.getOrCreateContainer('steps',om.getOrCreateContainer('continuous'))

        print('got %d clusters' % len(clusters))

        # get the rectangles from the clusters:
        blocks = []
        for i, cluster in enumerate(clusters):
                cornerTransform, rectDepth, rectWidth, rectArea = segmentation.findMinimumBoundingRectangle( cluster, linkFrame )
                #print 'min bounding rect:', rectDepth, rectWidth, rectArea, cornerTransform.GetPosition()

                block = BlockTop(cornerTransform, rectDepth, rectWidth, rectArea)
                blocks.append(block)

        # filter out blocks that are too big or small
        # TODO: pull out these parameters
        blocksGood = []
        groundPlane = None
        if self.chosenTerrain == 'stairs':
            ground_width_thresh = 0.90
            ground_depth_thresh = 0.90
            step_width_thresh = 0.30
            step_depth_thresh = 0.125
        else:
            ground_width_thresh = 0.45
            ground_depth_thresh = 0.90
            step_width_thresh = 0.30
            step_depth_thresh = 0.20

        for i, block in enumerate(blocks):
            if ((block.rectWidth>ground_width_thresh) or (block.rectDepth>ground_depth_thresh)):
                #print " ground plane",i,block.rectWidth,block.rectDepth
                groundPlane = block
            elif ((block.rectWidth<step_width_thresh) or (block.rectDepth<step_depth_thresh)): # was 0.34 and 0.30 for 13 block successful walk with lidar
                #print "removed block",i,block.rectWidth,block.rectDepth
                foobar=[]
            else:
                blocksGood.append(block)
                #print "keeping block",i,block.rectWidth,block.rectDepth
        blocks = blocksGood

        # order by distance from robot's foot
        for i, block in enumerate(blocks):
            block.distToRobot = np.linalg.norm(np.array(linkFrame.GetPosition()) - np.array(block.cornerTransform.GetPosition()))
        blocks.sort(key=operator.attrgetter('distToRobot'))

        # merge blocks if represent multiple pieces of same step
        for i, block in enumerate(blocks):
            j = i+1
            while j < len(blocks):
                block_pos = block.cornerTransform.GetPosition()
                next_block_pos = blocks[j].cornerTransform.GetPosition()
                frames_dist = pow(pow(block_pos[0]-next_block_pos[0],2)+pow(block_pos[2]-next_block_pos[2],2),0.5)
                if frames_dist < 0.05:
                    print('CORRECTION: Merging segments representing same step.')
                    if (block_pos[1] < next_block_pos[1]):  
                        block.rectWidth = block_pos[1] - next_block_pos[1] + blocks[j].rectWidth
                        #block.rectWidth = block.rectWidth+blocks[j].rectWidth
                        block.rectArea = block.rectDepth*block.rectWidth 
                    else:
                        block.cornerTransform = blocks[j].cornerTransform
                        block.rectWidth = block_pos[1] - next_block_pos[1] + blocks[j].rectWidth
                        #block.rectWidth = block.rectWidth+blocks[j].rectWidth
                        block.rectArea = block.rectDepth*block.rectWidth
                    blocks.pop(j)
                    j=j-1
                    #i=i-1            
                j = j+1


        # populate global blocks list
        tmp_dist = 1000
        match_idx = -1
        if len(blocks) > 0:
            if len(self.blocks_series) == 0:
                self.blocks_series.extend(blocks)
            else:
                for i, stored_block in enumerate(self.blocks_series):
                    #curr_dist = np.linalg.norm(np.array(blocks[0].cornerTransform.GetPosition()) - np.array(stored_block.cornerTransform.GetPosition()))
                    block_pos = blocks[0].cornerTransform.GetPosition()
                    stored_block_pos = stored_block.cornerTransform.GetPosition()
                    curr_dist = pow(pow(block_pos[0]-stored_block_pos[0],2)+pow(block_pos[2]-stored_block_pos[2],2),0.5)
                    if curr_dist < tmp_dist:
                        tmp_dist = curr_dist
                        match_idx = i
                if tmp_dist < 0.05:
                    self.blocks_series = self.blocks_series[:match_idx]
                else:
                    match_idx = match_idx + 1
                self.blocks_series.extend(blocks)

        # draw global blocks list
        om.removeFromObjectModel(om.findObjectByName('blocks_list'))
        blocksFolder=om.getOrCreateContainer('blocks_list',om.getOrCreateContainer('continuous'))
        for i, block in enumerate(self.blocks_series):

            blockCenter = transformUtils.frameFromPositionAndRPY([-block.rectDepth/2,block.rectWidth/2,0.0], [0,0,0])
            blockCenter.Concatenate(block.cornerTransform)

            d = DebugData()
            d.addCube([ block.rectDepth, block.rectWidth,0.005],[0,0,0])
            obj = vis.showPolyData(d.getPolyData(),'block %d' % i, color=[0,1,1],parent=blocksFolder)
            obj.actor.SetUserTransform(blockCenter)

        # draw blocks including the ground plane:
        om.removeFromObjectModel(om.findObjectByName('blocks'))
        blocksFolder=om.getOrCreateContainer('blocks',om.getOrCreateContainer('continuous'))
        for i, block in enumerate(blocks):
            vis.updateFrame(block.cornerTransform, 'block corners %d' % i , parent='block corners', scale=0.2, visible=True)

            blockCenter = transformUtils.frameFromPositionAndRPY([-block.rectDepth/2,block.rectWidth/2,0.0], [0,0,0])
            blockCenter.Concatenate(block.cornerTransform)

            d = DebugData()
            d.addCube([ block.rectDepth, block.rectWidth,0.005],[0,0,0])
            obj = vis.showPolyData(d.getPolyData(),'block %d' % i, color=[1,0,1],parent=blocksFolder)
            obj.actor.SetUserTransform(blockCenter)

        if (groundPlane is not None):
            vis.updateFrame(groundPlane.cornerTransform, 'ground plane', parent='block corners', scale=0.2, visible=True)

            blockCenter = transformUtils.frameFromPositionAndRPY([-groundPlane.rectDepth/2,groundPlane.rectWidth/2,0.0], [0,0,0])
            blockCenter.Concatenate(groundPlane.cornerTransform)

            d = DebugData()
            d.addCube([ groundPlane.rectDepth, groundPlane.rectWidth,0.005],[0,0,0])
            obj = vis.showPolyData(d.getPolyData(),'ground plane', color=[1,1,0],alpha=0.1, parent=blocksFolder)
            obj.actor.SetUserTransform(blockCenter)

        return blocks,match_idx,groundPlane


    def placeStepsOnBlocks(self, blocks, groundPlane, standingFootName, standingFootFrame, removeFirstLeftStep = True):

        footsteps = []
        for i, block in enumerate(blocks):
            # move back less for stereo:
            # lidar: -0.27 and -0.23
            if self.processContinuousStereo or self.processRawStereo:
                nextLeftTransform = transformUtils.frameFromPositionAndRPY([-0.24,0.29,0.08], [0,0,0])
                nextRightTransform = transformUtils.frameFromPositionAndRPY([-0.20,0.1,0.08], [0,0,0])
            else:
                nextLeftTransform = transformUtils.frameFromPositionAndRPY([-0.27,0.29,0.08], [0,0,0])
                nextRightTransform = transformUtils.frameFromPositionAndRPY([-0.23,0.1,0.08], [0,0,0])

            nextLeftTransform.Concatenate(block.cornerTransform)
            footsteps.append(Footstep(nextLeftTransform,False))

            nextRightTransform.Concatenate(block.cornerTransform)
            footsteps.append(Footstep(nextRightTransform,True))

        #footOnGround = False
        #if (groundPlane):
        #    # TODO: 0.08 is distance from foot frames to sole. remove hard coding!
        #    distOffGround = abs(groundPlane.cornerTransform.GetPosition()[2]-standingFootFrame.GetPosition()[2] + 0.08)
        #    #print "distOffGround",distOffGround
        #    footOnGround = (distOffGround < 0.05)
        #    if (footOnGround):
        #        # the robot is standing on the ground plane
        #        nextRightTransform = transformUtils.frameFromPositionAndRPY([(-0.23-0.38),0.1,0.08-0.13], [0,0,0])
        #        nextRightTransform.Concatenate(blocks[0].cornerTransform)
        #        footsteps = [Footstep(nextRightTransform,True)] + footsteps

        #if (footOnGround is False):
        #  # if we are standing on right foot, we can see the next block.
        #  # but the next left step has been committed - so remove it from the the list

        if (removeFirstLeftStep is True):
            if (standingFootName is self.ikPlanner.rightFootLink ):
                footsteps = footsteps[1:]
              #print "removing the first left step"

        return footsteps


    def getMeshAndColor(self,is_right_foot):
        if is_right_foot:
            mesh = footstepsdriver.getRightFootMesh()
            color = footstepsdriver.getRightFootColor()
        else:
            mesh = footstepsdriver.getLeftFootMesh()
            color = footstepsdriver.getLeftFootColor()

        return mesh,color


    def drawFittedSteps(self, footsteps):
        ''' Draw the footsteps fitted to the blocks
        These are NOT the steps placed by the planner
        '''
        left_color=None
        right_color=None

        for i, footstep in enumerate(footsteps):
            mesh,color = self.getMeshAndColor(footstep.is_right_foot)

            vis.updateFrame(footstep.transform, 'foot placement %d' % i , parent='foot placements', scale=0.2, visible=False)
            obj = vis.showPolyData(mesh, 'step %d' % i, color=color, alpha=1.0, parent='steps')
            #frameObj = vis.showFrame(footstepTransform, stepName + ' frame', parent=obj, scale=0.3, visible=False)
            obj.actor.SetUserTransform(footstep.transform)



    def computeFootstepPlanSafeRegions(self, blocks, robotPose, standingFootName):

        print('planning with safe regions.  %d blocks.' % len(blocks))

        folder = om.getOrCreateContainer('Safe terrain regions')
        om.removeFromObjectModel(folder)

        footsteps = []

        for i, block in enumerate(blocks):
            corners = block.getCorners()
            rpy = np.radians(block.cornerTransform.GetOrientation())

            self.convertStepToSafeRegion(corners, rpy)

        lastBlock = blocks[-1]

        goalFrame = transformUtils.copyFrame(lastBlock.cornerTransform)
        goalOffset = vtk.vtkTransform()
        goalOffset.Translate(0.3, lastBlock.rectWidth/2.0, 0.0)
        goalFrame.PreMultiply()
        goalFrame.Concatenate(goalOffset)
        goalPosition = np.array(goalFrame.GetPosition())

        if len(blocks) > 1:
            goalFrame = transformUtils.copyFrame(blocks[-2].cornerTransform)
            goalFrame.Translate(goalPosition - np.array(goalFrame.GetPosition()))

        vis.updateFrame(goalFrame, 'footstep plan goal', scale=0.2)

        request = self.footstepsPanel.driver.constructFootstepPlanRequest(robotPose, goalFrame)

        assert standingFootName in (self.ikPlanner.leftFootLink, self.ikPlanner.rightFootLink)
        if standingFootName == self.ikPlanner.rightFootLink:
            leadingFoot = lcmdrc.footstep_plan_params_t.LEAD_RIGHT
        else:
            leadingFoot = lcmdrc.footstep_plan_params_t.LEAD_LEFT

        request.params.leading_foot = leadingFoot
        request.params.max_forward_step = 0.5
        request.params.nom_forward_step = 0.12
        request.params.nom_step_width = 0.22
        request.params.max_num_steps = 8 #2*len(blocks)
        request.default_step_params.support_contact_groups = self.supportContact

        plan = self.footstepsPanel.driver.sendFootstepPlanRequest(request, waitForResponse=True)

        if not plan:
            return []

        print('received footstep plan with %d steps.' % len(plan.footsteps))

        footsteps = []
        for i, footstep in enumerate(plan.footsteps):
            footstepTransform = self.transformFromFootstep(footstep)
            footsteps.append(Footstep(footstepTransform, footstep.is_right_foot))

        return footsteps[2:] #returns the list from footsteps[2] to the end of the list 


    def transformFromFootstep(self, footstep):
        #print 'footstep (trans and quat) :'
        trans = footstep.pos.translation
        trans = [trans.x, trans.y, trans.z]
        #print (trans)
        quat = footstep.pos.rotation
        quat = [quat.w, quat.x, quat.y, quat.z]
        #print (quat)
        return transformUtils.transformFromPose(trans, quat)


    def convertStepToSafeRegion(self, step, rpySeed):
        assert step.shape[0] >= 3
        assert step.shape[1] == 3

        shapeVertices = np.array(step).transpose()[:2,:]

        if self.supportContact == lcmdrc.footstep_params_t.SUPPORT_GROUPS_HEEL_MIDFOOT:
            footContactPoints = self.BACK_FOOT_CONTACT_POINTS
        elif self.supportContact == lcmdrc.footstep_params_t.SUPPORT_GROUPS_MIDFOOT_TOE:
            footContactPoints = self.FRONT_FOOT_CONTACT_POINTS
        else:
            footContactPoints = self.FULL_FOOT_CONTACT_POINTS

        s = director.terrain.PolygonSegmentationNonIRIS(shapeVertices, bot_pts=footContactPoints)

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


    def replanFootsteps(self, polyData, standingFootName, removeFirstLeftStep=True, doStereoFiltering=True, nextDoubleSupportPose=None):

        obj = om.getOrCreateContainer('continuous')
        om.getOrCreateContainer('cont debug', obj)

        vis.updatePolyData( polyData, 'walking snapshot', parent='cont debug', visible=False)

        standingFootFrame = self.robotStateModel.getLinkFrame(standingFootName)
        vis.updateFrame(standingFootFrame, standingFootName, parent='cont debug', visible=False)
        # TODO: remove the pitch and roll of this frame to support it being on uneven ground

        # Step 1: filter the data down to a box in front of the robot:
        polyData = self.getRecedingTerrainRegion(polyData, footstepsdriver.FootstepsDriver.getFeetMidPoint(self.robotStateModel))
        if (doStereoFiltering is True):
            # used for stereo data:
            polyData = segmentation.applyVoxelGrid(polyData, leafSize=0.01)
            polyData = segmentation.labelOutliers(polyData, searchRadius=0.06, neighborsInSearchRadius=15) # 0.06 and 10 originally
            vis.updatePolyData(polyData, 'voxel plane points', parent='cont debug', colorByName='is_outlier', visible=False)
            polyData = segmentation.thresholdPoints(polyData, 'is_outlier', [0, 0])
            vis.updatePolyData( polyData, 'walking snapshot trimmed', parent='cont debug', visible=True)

        # Step 2: find all the surfaces in front of the robot (about 0.75sec)
        clusters = segmentation.findHorizontalSurfaces(polyData, removeGroundFirst=False, normalEstimationSearchRadius=0.05,
                                                       clusterTolerance=0.025, distanceToPlaneThreshold=0.0025, normalsDotUpRange=[0.95, 1.0])
        if clusters is None:
            print("No cluster found, stop walking now!")
            return

        # Step 3: find the corners of the minimum bounding rectangles
        blocks,match_idx,groundPlane = self.extractBlocksFromSurfaces(clusters, standingFootFrame)


        # Step 4: reduce list of blocks to those which represent either going_up stairs or going_down stairs
        heights = []
        safety_thresh = 0.05
        blocks_big_enough = True
        for i, block in enumerate(blocks):
            corners = block.getCorners()
            h_mean = (corners[0,2]+corners[1,2]+corners[2,2]+corners[3,2])/4
            heights.append(h_mean)
            if block.rectDepth <= (self.FOOT_LENGTH+safety_thresh):
                blocks_big_enough = False

        if not blocks_big_enough:
            cut_idx = -1
            for i, height in enumerate(heights):
                if i == 0 and len(self.footStatus) != 0:
                    t1_lastContact = self.footStatus[len(self.footStatus)-1].transform
                    [t1_lastContact_pos, t1_lastContact_ori] = transformUtils.poseFromTransform(t1_lastContact)
                    t2_lastContact = self.footStatus[len(self.footStatus)-2].transform
                    [t2_lastContact_pos, t2_lastContact_ori] = transformUtils.poseFromTransform(t2_lastContact)
                    previous_height1 = t1_lastContact_pos[2]
                    previous_height2 = t2_lastContact_pos[2]
                    if (heights[i] > previous_height1) or (heights[i] > previous_height2):  
                        up = True
                    else:
                        up = False
                elif len(self.footStatus) != 0 and cut_idx == -1:
                    # Stairs are switching up/down or down/up 
                    if (heights[i] > heights[i-1] and not up) or (heights[i] < heights[i-1] and up):
                        cut_idx = i

            if cut_idx != -1:
                blocks = blocks[:cut_idx]
                heights = heights[:cut_idx]

            if len(self.footStatus) != 0 and len(heights) != 0:
                if up: 
                    self.supportContact = lcmdrc.footstep_params_t.SUPPORT_GROUPS_MIDFOOT_TOE
                else:
                    self.supportContact = lcmdrc.footstep_params_t.SUPPORT_GROUPS_HEEL_MIDFOOT
        else:
            self.supportContact = lcmdrc.footstep_params_t.SUPPORT_GROUPS_HEEL_TOE

        # Assumption: going down the stairs visible segmented area is part of a larger step (occluded by previous step) 
        # Increase step depth depending on the corner of the previous step:
        if match_idx > 0:
            corners_prev = self.blocks_series[match_idx-1].getCorners()
            h_mean_prev = (corners_prev[0,2]+corners_prev[1,2]+corners_prev[2,2]+corners_prev[3,2])/4
            corners_next = blocks[0].getCorners()
            h_mean_next = (corners_next[0,2]+corners_next[1,2]+corners_next[2,2]+corners_next[3,2])/4
            if h_mean_next < h_mean_prev:
                for i, block in enumerate(blocks):
                    x_mean_prev = (corners_prev[1,0]+corners_prev[2,0])/2
                    x_mean_next = (corners_next[1,0]+corners_next[2,0])/2
                    depth_before = block.rectDepth
                    block.rectDepth = abs(x_mean_next - x_mean_prev) # length of face away from robot
                    depth_after = block.rectDepth
                    block.rectArea = depth_after * block.rectWidth 
                    if (match_idx+1)<len(self.blocks_series) and (i+1)<len(blocks):
                        corners_prev = self.blocks_series[match_idx+i].getCorners()
                        corners_next = blocks[i+1].getCorners()    


        # Step 5: Footsteps planning
        self.displayExpectedPose(nextDoubleSupportPose)

        if not self.useManualFootstepPlacement and self.queryPlanner:
            footsteps = self.computeFootstepPlanSafeRegions(blocks, nextDoubleSupportPose, standingFootName)
        else:
            footsteps = self.placeStepsOnBlocks(blocks, groundPlane, standingFootName, standingFootFrame, removeFirstLeftStep)

            if not len(footsteps):
                return

            if self.queryPlanner:
                self.sendPlanningRequest(footsteps, nextDoubleSupportPose)
            else:
                self.drawFittedSteps(footsteps)

        if (len(footsteps) > 2):
            self.planned_footsteps[:] = []
            self.planned_footsteps.extend(footsteps)
            self.footstep_index = -1
            self.new_status = False
            self.new_first_double_supp = False
        

    def sendPlanningRequest(self, footsteps, nextDoubleSupportPose):

        goalSteps = []

        #for i in range(flist_shape[0]):
        for i, footstep in enumerate(footsteps):
            #step_t = vtk.vtkTransform()
            #step_t.PostMultiply()
            #step_t.Concatenate(transformUtils.frameFromPositionAndRPY(flist[i,0:3] , flist[i,3:6]))
            #step_t.Concatenate(foot_to_sole)
            #step_t.Concatenate(frame_pt_to_centerline)
            step_t = footstep.transform

            step = lcmdrc.footstep_t()
            step.pos = positionMessageFromFrame(step_t)
            step.is_right_foot =  footstep.is_right_foot # flist[i,6] # is_right_foot
            step.params = self.footstepsPanel.driver.getDefaultStepParams()

            # Visualization via triads
            #vis.updateFrame(step_t, str(i), parent="navigation")
            goalSteps.append(step)

        #nextDoubleSupportPose = self.robotStateJointController.q
        request = self.footstepsPanel.driver.constructFootstepPlanRequest(nextDoubleSupportPose)
        request.num_goal_steps = len(goalSteps)
        request.goal_steps = goalSteps

        # force correct planning parameters:
        request.params.leading_foot = goalSteps[0].is_right_foot
        request.params.planning_mode = lcmdrc.footstep_plan_params_t.MODE_SPLINE
        request.params.nom_forward_step = 0.38
        request.params.map_mode = lcmdrc.footstep_plan_params_t.TERRAIN_HEIGHTS_Z_NORMALS
        request.params.max_num_steps = len(goalSteps)
        request.params.min_num_steps = len(goalSteps)
        request.default_step_params.support_contact_groups = self.supportContact

        lcmUtils.publish('FOOTSTEP_PLAN_REQUEST', request)

        if (self.automaticContinuousWalkingEnabled):
            print("Requested Footstep Plan, it will be AUTOMATICALLY EXECUTED")
        else:
            print("Requested Footstep Plan, it will be not be executed")

    def onAtlasStepParams(self,msg):
        if (msg.desired_step_spec.foot_index ==1):
            is_right_foot = True
        else:
            is_right_foot = False
        #print msg.desired_step_spec.foot_index , " and " , is_right_foot
        foot = msg.desired_step_spec.foot
        footTransform  = transformUtils.frameFromPositionAndRPY( foot.position , [0, 0, foot.yaw*180/math.pi])

        mesh,color = self.getMeshAndColor(is_right_foot)
        #color[1] = 0.75 ; color[2] = 0.25
        vis.updateFrame(footTransform, 'bdi foot', parent='foot placements', scale=0.2, visible=False)

        #bdi_step_mesh = om.findObjectByName('bdi step')
        #om.removeFromObjectModel(bdi_step_mesh)
        obj = vis.updatePolyData(mesh, 'bdi step', color=color, alpha=1.0, parent='foot placements', visible=False)
        obj.setProperty('Color',QtGui.QColor(color[0]*255.0,color[1]*255.0,color[2]*255.0))
        obj.actor.SetUserTransform(footTransform)


    def makeReplanRequest(self, standingFootName, removeFirstLeftStep = False, nextDoubleSupportPose=None):

        if (self.processContinuousStereo):
            polyData = self.cameraView.getStereoPointCloud(2,'CAMERA_FUSED', cameraName='CAMERA_TSDF', removeSize=4000)
            doStereoFiltering = True
            print("makeReplanRequest processContinuousStereo")
        elif (self.processRawStereo):
            polyData = self.cameraView.getStereoPointCloud(2,'MULTISENSE_CAMERA')
            doStereoFiltering = True
            print("makeReplanRequest processRawStereo")
        else:
            polyData = segmentation.getCurrentRevolutionData()
            doStereoFiltering = False

        self.replanFootsteps(polyData, standingFootName, removeFirstLeftStep, doStereoFiltering, nextDoubleSupportPose)

    def startContinuousWalking(self, leadFoot=None):
        
        if (leadFoot is None):
            if self.leadingFootByUser == 'Right':
                leadFoot=self.ikPlanner.rightFootLink #'r_foot'
            else:
                leadFoot=self.ikPlanner.leftFootLink #'l_foot'

        self._setupOnce()

        self.committedStep = None

        startPose = self.robotStateJointController.getPose('EST_ROBOT_STATE')

        self.makeReplanRequest(leadFoot, removeFirstLeftStep = False, nextDoubleSupportPose=startPose)

    def testContinuousWalking(self):

        leadFoot=self.ikPlanner.leftFootLink

        if (self.chosenTerrain == 'stairs'):
            filename =  director.getDRCBaseDir() + '/../drc-testing-data/terrain/terrain_stairs_ihmc.vtp'
            polyData = ioUtils.readPolyData( filename )
            vis.showPolyData(polyData,'terrain_stairs_ihmc.vtp')
        else:
            filename =  director.getDRCBaseDir() + '/../drc-testing-data/terrain/terrain_simple_ihmc.vtp'
            polyData = ioUtils.readPolyData( filename )
            vis.showPolyData(polyData,'terrain_simple_ihmc.vtp')

        startPose = self.robotStateJointController.getPose('EST_ROBOT_STATE')

        self.replanFootsteps(polyData, leadFoot, removeFirstLeftStep= True, nextDoubleSupportPose = startPose)


    def onFootstepPlanContinuous(self, msg):

        if msg.num_steps <= 2:
            return
        
        if self.automaticContinuousWalkingEnabled:
            print("Committing Footstep Plan for AUTOMATIC EXECUTION")
            lcmUtils.publish('COMMITTED_FOOTSTEP_PLAN', msg)
    
    def onRobotStatus(self, msg):
        x = msg.pose.translation.x
        y = msg.pose.translation.y
        z = msg.pose.translation.z
        q1 = msg.pose.rotation.w
        q2 = msg.pose.rotation.x
        q3 = msg.pose.rotation.y
        q4 = msg.pose.rotation.z 
        self.tf_robotStatus = transformUtils.transformFromPose([x,y,z], [q1,q2,q3,q4])
        '''
        print 'pos and ori:'
        print ([x,y,z])
        print ([q1,q2,q3,q4])
        '''

    def onFootstepStatus(self, msg):
        if not self.automaticContinuousWalkingEnabled:
            return

        import ihmc
        x = msg.actual_foot_position_in_world[0]
        y = msg.actual_foot_position_in_world[1]
        z = msg.actual_foot_position_in_world[2]
        q1 = msg.actual_foot_orientation_in_world[0]
        q2 = msg.actual_foot_orientation_in_world[1]
        q3 = msg.actual_foot_orientation_in_world[2]
        q4 = msg.actual_foot_orientation_in_world[3] 

        if msg.status == 1 and self.new_status:	#valid message
      
            self.new_status = False

            tf_footStatus = transformUtils.transformFromPose([x,y,z], [q1,q2,q3,q4])
            self.transforms_series[:] = []
            self.transforms_series.append(tf_footStatus) 
            self.transforms_series.append(self.tf_robotStatus.GetInverse())
            tf_foot_robot = transformUtils.concatenateTransforms(self.transforms_series)
            #vis.showFrame(self.tf_robotStatus,'tf_robotStatus')
            #vis.showFrame(tf_foot_robot,'tf_foot_robot')
       
            # Increases at each contact     
            self.footstep_index = self.footstep_index + 1   

            [robot_pos, robot_ori] = transformUtils.poseFromTransform(self.tf_robotStatus)
            [current_pos, current_ori] = transformUtils.poseFromTransform(tf_foot_robot)  
            if (current_pos[1] > 0):  
                current_left = True
            else:
                current_left = False  
            
            if (self.leadingFootByUser == 'Left'):
                # I want to take the first status for the LEFT foot
                if current_left:
                    # left foot in contact (reached left single support)
                    self.footStatus.append(Footstep(tf_footStatus, 0))
                    self.new_first_double_supp = True
                    # right foot expected pose (from planning)   
                    if self.footstep_index+1 < len(self.planned_footsteps):
                        if (self.planned_footsteps[self.footstep_index+1].is_right_foot):
                            self.footStatus.append(self.planned_footsteps[self.footstep_index+1])
                        else:
                            self.footStatus.append(self.planned_footsteps[self.footstep_index])
                    else:
                        self.footStatus.append(self.footStatus_right[len(self.footStatus_right)-1])
                    self.footStatus_left.append(Footstep(tf_footStatus, 0))
                else:
                    # right foot in contact
                	self.footStatus_right.append(Footstep(tf_footStatus, 1))
            else:
                # I want to take the first status for the RIGHT foot
                if not current_left:
                    self.new_first_double_supp = True
                    # left foot expected pose (from planning)   
                    if self.footstep_index+1 < len(self.planned_footsteps):
                        if not (self.planned_footsteps[self.footstep_index+1].is_right_foot):
                            self.footStatus.append(self.planned_footsteps[self.footstep_index+1])
                        else:
                            self.footStatus.append(self.planned_footsteps[self.footstep_index])
                    else:
                        self.footStatus.append(self.footStatus_left[len(self.footStatus_left)-1])
                    self.footStatus_right.append(Footstep(tf_footStatus, 1))
                else:
                    # left foot in contact
                    self.footStatus_left.append(Footstep(tf_footStatus, 0))
                # right foot in contact (reached right single support)
                self.footStatus.append(Footstep(tf_footStatus, 1))
        elif msg.status == 0: 
            self.new_status = True

        if self.footstep_index != -1 and len(self.footStatus) > 0 and self.new_first_double_supp:
            self.new_first_double_supp = False

            t1 = self.footStatus[len(self.footStatus)-2].transform
            t2 = self.footStatus[len(self.footStatus)-1].transform
            [t1_pos, t1_ori] = transformUtils.poseFromTransform(t1)
            [t2_pos, t2_ori] = transformUtils.poseFromTransform(t2)
            distance = pow(pow(t1_pos[0]-t2_pos[0],2)+pow(t1_pos[1]-t2_pos[1],2)+pow(t1_pos[2]-t2_pos[2],2),0.5)
            '''
            print 'distance:'
            print distance
            print 't1 and t2:' 
            print (transformUtils.poseFromTransform(t1))
            print (transformUtils.poseFromTransform(t2)) 
            '''

            if (distance > 0.65):
                t1 = self.footStatus_left[len(self.footStatus_left)-1].transform
                t2 = self.footStatus_right[len(self.footStatus_right)-1].transform
            '''
            if self.footStatus[len(self.footStatus)-2].is_right_foot:
                t1, t2 = t2, t1
            '''    
            pose = self.getNextDoubleSupportPose(t1, t2)

            standingFootName = self.ikPlanner.rightFootLink if self.leadingFootByUser == 'Right' else self.ikPlanner.leftFootLink
            self.makeReplanRequest(standingFootName, removeFirstLeftStep = False, nextDoubleSupportPose=pose)

    def testDouble():

        lfootTransform  = transformUtils.frameFromPositionAndRPY( [0.1, 0.13, 0.08], [0, 0, 0])
        rfootTransform  = transformUtils.frameFromPositionAndRPY( [-0.1, -0.16, 0.08], [0, 0, 0])
        nextDoubleSupportPose = self.getNextDoubleSupportPose(lfootTransform, rfootTransform)
        displayExpectedPose(self, nextDoubleSupportPose)


    def getNextDoubleSupportPose(self, lfootTransform, rfootTransform):

        vis.updateFrame(lfootTransform, 'lfootTransform', visible=True, scale=0.2)
        vis.updateFrame(rfootTransform, 'rfootTransform', visible=True, scale=0.2)

        startPose = self.robotStateJointController.getPose('EST_ROBOT_STATE')
        startPoseName = 'stride_start'
        self.ikPlanner.addPose(startPose, startPoseName)

        constraints = []
        # lock everything except the feet, constrain the feet
        constraints.append(self.ikPlanner.createQuasiStaticConstraint())
        constraints.append(self.ikPlanner.createMovingBackPostureConstraint())
        constraints.append(self.ikPlanner.createMovingBasePostureConstraint(startPoseName))
        constraints.append(self.ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
        constraints.append(self.ikPlanner.createLockedRightArmPostureConstraint(startPoseName))

        nullFrame = vtk.vtkTransform()
        positionConstraint, orientationConstraint = self.ikPlanner.createPositionOrientationConstraint(self.ikPlanner.rightFootLink, rfootTransform, nullFrame)
        positionConstraint.tspan = [1.0, 1.0]
        orientationConstraint.tspan = [1.0, 1.0]
        constraints.append(positionConstraint)
        constraints.append(orientationConstraint)

        positionConstraint, orientationConstraint = self.ikPlanner.createPositionOrientationConstraint(self.ikPlanner.leftFootLink, lfootTransform, nullFrame)
        positionConstraint.tspan = [1.0, 1.0]
        orientationConstraint.tspan = [1.0, 1.0]
        constraints.append(positionConstraint)
        constraints.append(orientationConstraint)

        constraintSet = ikplanner.ConstraintSet(self.ikPlanner, constraints, 'stride_end', startPoseName)
        nextDoubleSupportPose, info = constraintSet.runIk()
        return nextDoubleSupportPose


    def displayExpectedPose(self, nextDoubleSupportPose):
        self.teleopJointController.setPose('double_support_pose', nextDoubleSupportPose)
        om.getOrCreateContainer('teleop model').setProperty('Visible',True)
        om.getOrCreateContainer('teleop model').setProperty('Color Mode', 0)


    def makeDebugRegions(self):

        stepWidth = (15 + 3/8.0) * 0.0254
        stepLength = (15 + 5/8.0) * 0.0254
        stepHeight = (5 + 5/8.0) * 0.0254

        stepPoints = np.array([
          [-stepLength/2.0, -stepWidth/2.0, 0.0],
          [-stepLength/2.0, stepWidth/2.0, 0.0],
          [stepLength/2.0, stepWidth/2.0, 0.0],
          [stepLength/2.0, -stepWidth/2.0, 0.0]
        ])

        t = vtk.vtkTransform()
        t.Translate(0.0, 0.0, 0.0)
        t.RotateZ(4.5)

        for i in range(len(stepPoints)):
            stepPoints[i] = np.array(t.TransformPoint(stepPoints[i]))

        stepOffset = np.array([stepLength, 0.0, stepHeight])

        numSteps = 5

        goalFrame = transformUtils.frameFromPositionAndRPY([0.4, 0.0, 0.1], [0,0,0])
        vis.showFrame(goalFrame, 'goal frame', scale=0.2)

        rpySeed = np.radians(goalFrame.GetOrientation())
        for i in range(numSteps):

            step = stepPoints + (i+1)*stepOffset
            self.convertStepToSafeRegion(step, rpySeed)

        self.footstepsPanel.onNewWalkingGoal(goalFrame)

    def loadSDFFileAndRunSim(self):
        from director import sceneloader

        if self.chosenTerrain == 'simple':
            filename= os.environ['DRC_BASE'] + '/software/models/worlds/terrain_simple.sdf'
        elif self.chosenTerrain == 'simple_nogaps':
            filename= os.environ['DRC_BASE'] + '/software/models/worlds/terrain_simple_nogaps.sdf'
        elif self.chosenTerrain == 'simple_flagstones':
            filename= os.environ['DRC_BASE'] + '/software/models/worlds/terrain_simple_flagstones.sdf'
        elif self.chosenTerrain == 'uneven':
            filename= os.environ['DRC_BASE'] + '/software/models/worlds/terrain_uneven.sdf'
        elif self.chosenTerrain == 'stairs':
            filename= os.environ['DRC_BASE'] + '/software/models/worlds/terrain_stairs.sdf'
        elif self.chosenTerrain == 'boxroom':
            filename= os.environ['DRC_BASE'] + '/software/models/worlds/terrain_boxroom.sdf'
        
        sc=sceneloader.SceneLoader()
        sc.loadSDF(filename)
        msg=lcmdrc.scs_api_command_t()
        msg.command="loadSDF "+filename+"\nsimulate"
        lcmUtils.publish('SCS_API_CONTROL', msg)


    def executeManipPlan(self):
        self.playbackPanel.executePlan()

    def addPlan(self, plan):
        self.plans.append(plan)

    # Planning Functions #######################################################

    # These are operational conveniences:
    def planHandsDown(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'handsdown both')
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def planHandsUp(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'handsup both')
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def planNeckDown(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'neckdown')
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def planNeckUp(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'neckup')
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    # Glue Functions ###########################################################

    def getEstimatedRobotStatePose(self):
        return self.robotStateJointController.getPose('EST_ROBOT_STATE')

    def getPlanningStartPose(self):
        if self.planFromCurrentRobotState:
            return self.robotStateJointController.getPose('EST_ROBOT_STATE')
        else:
            if self.plans:
                return robotstate.convertStateMessageToDrakePose(
                    self.plans[-1].plan[-1])
            else:
                return self.getEstimatedRobotStatePose()

class ContinuousWalkingTaskPanel(TaskUserPanel):

    def __init__(self, continuousWalkingDemo):

        TaskUserPanel.__init__(self, windowTitle='Walking Task')

        self.continuousWalkingDemo = continuousWalkingDemo

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()

    def addButtons(self):

        self.addManualButton('Neck Up', self.continuousWalkingDemo.planNeckUp)
        self.addManualButton('Neck Down', self.continuousWalkingDemo.planNeckDown)
        self.addManualSpacer()
        self.addManualButton('Arms Up', self.continuousWalkingDemo.planHandsUp)
        self.addManualButton('Arms Down', self.continuousWalkingDemo.planHandsDown) 
        self.addManualSpacer() 
        self.addManualSpacer()
        self.addManualButton('RUN Test', self.continuousWalkingDemo.testContinuousWalking)    

    def addDefaultProperties(self):
        self.params.addProperty('Terrain Type', 0, attributes=om.PropertyAttributes(enumNames=['Simple', 'Simple, no Gaps', 'Simple Flagstones',
                                                                                       'Uneven', 'Stairs', 'Box Room']))
        self.params.addProperty('Sensor', 0, attributes=om.PropertyAttributes(enumNames=['Lidar',
                                                                                       'Stereo']))
        self.params.addProperty('Leading Foot', 0, attributes=om.PropertyAttributes(enumNames=['Left',
                                                                                       'Right']))
        self.params.addProperty('Support Contact Groups', 0, attributes=om.PropertyAttributes(enumNames=['Whole Foot', 'Front 2/3', 'Back 2/3']))
        self.params.addProperty('Continuous Walking', 0, attributes=om.PropertyAttributes(enumNames=['Enabled',
                                                                                       'Disabled']))
        self._syncProperties()

    def onPropertyChanged(self, propertySet, propertyName):
        self._syncProperties()

    def _syncProperties(self):
        if self.params.getPropertyEnumValue('Terrain Type') == 'Simple':
            self.continuousWalkingDemo.chosenTerrain = 'simple'
        elif self.params.getPropertyEnumValue('Terrain Type') == 'Simple, no Gaps':
            self.continuousWalkingDemo.chosenTerrain = 'simple_nogaps'
        elif self.params.getPropertyEnumValue('Terrain Type') == 'Uneven':
            self.continuousWalkingDemo.chosenTerrain = 'uneven'
        elif self.params.getPropertyEnumValue('Terrain Type') == 'Simple Flagstones':
            self.continuousWalkingDemo.chosenTerrain = 'simple_flagstones'
        elif self.params.getPropertyEnumValue('Terrain Type') == 'Stairs':
            self.continuousWalkingDemo.chosenTerrain = 'stairs'         
        else:
            self.continuousWalkingDemo.chosenTerrain = 'boxroom'

        if self.params.getPropertyEnumValue('Sensor') == 'Stereo':
            self.continuousWalkingDemo.processContinuousStereo = True
        else:
            self.continuousWalkingDemo.processContinuousStereo = False

        if self.params.getPropertyEnumValue('Leading Foot') == 'Left':
            self.continuousWalkingDemo.leadingFootByUser = 'Left'
        else:
            self.continuousWalkingDemo.leadingFootByUser = 'Right'

        if self.params.getPropertyEnumValue('Support Contact Groups') == 'Whole Foot':
            self.continuousWalkingDemo.supportContact = lcmdrc.footstep_params_t.SUPPORT_GROUPS_HEEL_TOE
        elif self.params.getPropertyEnumValue('Support Contact Groups') == 'Front 2/3':
            self.continuousWalkingDemo.supportContact = lcmdrc.footstep_params_t.SUPPORT_GROUPS_MIDFOOT_TOE
        else:
            self.continuousWalkingDemo.supportContact = lcmdrc.footstep_params_t.SUPPORT_GROUPS_HEEL_MIDFOOT

        if self.params.getPropertyEnumValue('Continuous Walking') == 'Enabled':
            self.continuousWalkingDemo.automaticContinuousWalkingEnabled = True
        else:
            self.continuousWalkingDemo.automaticContinuousWalkingEnabled = False
     
        self.continuousWalkingDemo.planFromCurrentRobotState = True

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

        cw = self.continuousWalkingDemo

        ###############
        # add the tasks

        # load
        load = self.taskTree.addGroup('Loading')
        addFunc(functools.partial(cw.loadSDFFileAndRunSim), 'load scenario', parent=load)

        # prep
        prep = self.taskTree.addGroup('Preparation')
        addTask(rt.SetArmsPosition(name='set arms position'), parent=prep)
        addFunc(functools.partial(cw.executeManipPlan), 'execute arms plan', parent=prep)
        addTask(rt.SetNeckPitch(name='set neck position', angle=50), parent=prep)

        # plan walking
        load = self.taskTree.addGroup('Planning')
        addFunc(functools.partial(cw.startContinuousWalking), 'plan footsteps', parent=load)

