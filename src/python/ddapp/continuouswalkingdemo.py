import math
import numpy as np
import operator
import vtkAll as vtk
import vtkNumpy

from ddapp import lcmUtils
from ddapp import segmentation
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp.debugVis import DebugData
from ddapp import transformUtils
from ddapp import footstepsdriver
from ddapp.debugVis import DebugData
from ddapp import ikplanner

import drc as lcmdrc

from thirdparty import qhull_2d
from thirdparty import min_bounding_rect



def get2DAsPolyData(xy_points):
    '''
    Convert a 2D np array to a 3D polydata by appending z=0
    '''
    d = np.vstack((xy_points.T, np.zeros( xy_points.shape[0]) )).T
    d2=d.copy()
    return vtkNumpy.getVtkPolyDataFromNumpyPoints( d2 )


class BlockTop():
    def __init__(self, cornerTransform, rectDepth, rectWidth, rectArea):
        self.cornerTransform = cornerTransform # location of far right corner
        self.rectDepth = rectDepth # length of face away from robot
        self.rectWidth = rectWidth # length of face perpendicular to robot's toes
        self.rectArea = rectArea


class Footstep():
    def __init__(self, transform, is_right_foot):
        self.transform = transform
        self.is_right_foot = is_right_foot


class ContinousWalkingDemo(object):

    def __init__(self, robotStateModel, footstepsPanel, robotStateJointController, ikPlanner, teleopJointController, navigationPanel, cameraView):
        self.footstepsPanel = footstepsPanel
        self.robotStateModel = robotStateModel
        self.robotStateJointController = robotStateJointController
        self.ikPlanner = ikPlanner
        self.teleopJointController = teleopJointController
        self.navigationPanel = navigationPanel
        self.cameraView = cameraView

        self.lastContactState = "none"
        # Smooth Stereo or Raw or Lidar?
        self.processContinuousStereo = False
        self.processRawStereo = False
        self.committedStep = None

        # overwrite all of the above with the yaw of the robt when it was standing in front of the blocks:
        self.fixBlockYawWithInitial = False
        self.initialRobotYaw = np.Inf

        if (self.footstepsPanel is not None):
            self.queryPlanner = True
            self.footstepsPanel.driver.applyDefaults('BDI')
        else:
            self.queryPlanner = False

        # For development - dont need to query planner
        #self.queryPlanner = False

        self._setupSubscriptions()


    def _setupSubscriptions(self):
        # use a different classifier to scott:
        footContactSubContinuous = lcmUtils.addSubscriber('FOOT_CONTACT_ESTIMATE_SLOW', lcmdrc.foot_contact_estimate_t, self.onFootContactContinuous)
        footContactSubContinuous.setSpeedLimit(60)

        lcmUtils.addSubscriber('FOOTSTEP_PLAN_RESPONSE', lcmdrc.footstep_plan_t, self.onFootstepPlanContinuous)# additional git decode stuff removed

        stepParamsSub = lcmUtils.addSubscriber('ATLAS_STEP_PARAMS', lcmdrc.atlas_behavior_step_params_t, self.onAtlasStepParams)
        stepParamsSub.setSpeedLimit(60)

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

        polyData = segmentation.thresholdPoints(polyData, 'distance_along_foot_x', [0.20, 1.3])
        polyData = segmentation.thresholdPoints(polyData, 'distance_along_foot_y', [-0.4, 0.4])
        polyData = segmentation.thresholdPoints(polyData, 'distance_along_foot_z', [-0.4, 0.4])

        vis.updatePolyData( polyData, 'walking snapshot trimmed', parent='cont debug', visible=True)
        return polyData


    def findFarRightCorner(self, polyData, linkFrame):
        '''
        Within a point cloud find the point to the far right from the link
        The input is typically the 4 corners of a minimum bounding box
        '''

        diagonalTransform = transformUtils.frameFromPositionAndRPY([0,0,0], [0,0,45])
        diagonalTransform.Concatenate(linkFrame)
        vis.updateFrame(diagonalTransform, 'diagonal frame', parent='cont debug', visible=False)

        #polyData = shallowCopy(polyData)
        points = vtkNumpy.getNumpyFromVtk(polyData, 'Points')
        #vtkNumpy.addNumpyToVtk(polyData, points[:,0].copy(), 'x')
        #vtkNumpy.addNumpyToVtk(polyData, points[:,1].copy(), 'y')
        #vtkNumpy.addNumpyToVtk(polyData, points[:,2].copy(), 'z')

        viewOrigin = diagonalTransform.TransformPoint([0.0, 0.0, 0.0])
        viewX = diagonalTransform.TransformVector([1.0, 0.0, 0.0])
        viewY = diagonalTransform.TransformVector([0.0, 1.0, 0.0])
        viewZ = diagonalTransform.TransformVector([0.0, 0.0, 1.0])
        #polyData = segmentation.labelPointDistanceAlongAxis(polyData, viewX, origin=viewOrigin, resultArrayName='distance_along_foot_x')
        polyData = segmentation.labelPointDistanceAlongAxis(polyData, viewY, origin=viewOrigin, resultArrayName='distance_along_foot_y')
        #polyData = segmentation.labelPointDistanceAlongAxis(polyData, viewZ, origin=viewOrigin, resultArrayName='distance_along_foot_z')

        vis.updatePolyData( polyData, 'cornerPoints', parent='cont debug', visible=False)
        farRightIndex = vtkNumpy.getNumpyFromVtk(polyData, 'distance_along_foot_y').argmin()
        points = vtkNumpy.getNumpyFromVtk(polyData, 'Points')
        return points[farRightIndex,:]


    def findMinimumBoundingRectangle(self, polyData, linkFrame):
        '''
        Find minimum bounding rectangle.
        The input is assumed to be a rectangular point cloud of cinder blocks
        Returns transform of far right corner (pointing away from robot)
        '''
        # TODO: for non-z up surfaces, this needs work
        # TODO: return other parameters

        # Originally From: https://github.com/dbworth/minimum-area-bounding-rectangle
        polyData = segmentation.applyVoxelGrid(polyData, leafSize=0.02)
        #vis.updatePolyData( polyData, 'block top', parent='cont debug', visible=False)
        polyDataCentroid = segmentation.computeCentroid(polyData)
        pts =vtkNumpy.getNumpyFromVtk( polyData , 'Points' )

        xy_points =  pts[:,[0,1]]
        vis.updatePolyData( get2DAsPolyData(xy_points) , 'xy_points', parent='cont debug', visible=False)
        hull_points = qhull_2d.qhull2D(xy_points)
        vis.updatePolyData( get2DAsPolyData(hull_points) , 'hull_points', parent='cont debug', visible=False)
        # Reverse order of points, to match output from other qhull implementations
        hull_points = hull_points[::-1]
        # print 'Convex hull points: \n', hull_points, "\n"

        # Find minimum area bounding rectangle
        (rot_angle, rectArea, rectDepth, rectWidth, center_point, corner_points_ground) = min_bounding_rect.minBoundingRect(hull_points)
        vis.updatePolyData( get2DAsPolyData(corner_points_ground) , 'corner_points_ground', parent='cont debug', visible=False)
        cornerPoints = np.vstack((corner_points_ground.T, polyDataCentroid[2]*np.ones( corner_points_ground.shape[0]) )).T
        cornerPolyData = vtkNumpy.getVtkPolyDataFromNumpyPoints(cornerPoints)

        # Create a frame at the far right point - which points away from the robot
        farRightCorner = self.findFarRightCorner(cornerPolyData , linkFrame)
        viewDirection = segmentation.SegmentationContext.getGlobalInstance().getViewDirection()
        robotYaw = math.atan2( viewDirection[1], viewDirection[0] )*180.0/np.pi
        blockAngle =  rot_angle*(180/math.pi)
        #print "robotYaw   ", robotYaw
        #print "blockAngle ", blockAngle
        blockAngleAll = np.array([blockAngle , blockAngle+90 , blockAngle+180, blockAngle+270])
        #print blockAngleAll
        for i in range(0,4):
            if(blockAngleAll[i]>180):
              blockAngleAll[i]=blockAngleAll[i]-360
        #print blockAngleAll
        values = abs(blockAngleAll - robotYaw)
        #print values
        min_idx = np.argmin(values)
        if ( (min_idx==1) or (min_idx==3) ):
            #print "flip rectDepth and rectWidth as angle is not away from robot"
            temp = rectWidth ; rectWidth = rectDepth ; rectDepth = temp

        #print "best angle", blockAngleAll[min_idx]
        rot_angle = blockAngleAll[min_idx]*math.pi/180.0


        # Optional: overwrite all of the above with the yaw of the robt when it was standing in front of the blocks:
        if (self.fixBlockYawWithInitial):
            rot_angle = self.initialRobotYaw

        cornerTransform = transformUtils.frameFromPositionAndRPY( farRightCorner , [0,0, np.rad2deg(rot_angle) ] )

        #print "Minimum area bounding box:"
        #print "Rotation angle:", rot_angle, "rad  (", rot_angle*(180/math.pi), "deg )"
        #print "rectDepth:", rectDepth, " rectWidth:", rectWidth, "  Area:", rectArea
        #print "Center point: \n", center_point # numpy array
        #print "Corner points: \n", cornerPoints, "\n"  # numpy array
        return cornerTransform, rectDepth, rectWidth, rectArea


    def extractBlocksFromSurfaces(self, clusters, linkFrame):
        ''' find the corners of the minimum bounding rectangles '''
        om.removeFromObjectModel(om.findObjectByName('block corners'))
        om.removeFromObjectModel(om.findObjectByName('foot placements'))
        om.removeFromObjectModel(om.findObjectByName('steps'))
        om.getOrCreateContainer('block corners',om.getOrCreateContainer('continuous'))
        om.getOrCreateContainer('foot placements',om.getOrCreateContainer('continuous'))
        om.getOrCreateContainer('steps',om.getOrCreateContainer('continuous'))

        # get the rectangles from the clusters:
        blocks = []
        for i, cluster in enumerate(clusters):
                cornerTransform, rectDepth, rectWidth, rectArea = self.findMinimumBoundingRectangle( cluster, linkFrame )
                block = BlockTop(cornerTransform, rectDepth, rectWidth, rectArea)
                blocks.append(block)

        # filter out blocks that are too big or small
        # TODO: pull out these parameters
        blocksGood = []
        groundPlane = None
        for i, block in enumerate(blocks):
            if ((block.rectWidth>0.45) or (block.rectDepth>0.45)):
                #print " ground plane",i,block.rectWidth,block.rectDepth
                groundPlane = block
            elif ((block.rectWidth<0.30) or (block.rectDepth<0.20)): # was 0.34 and 0.30 for 13 block successful walk with lidar
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

        return blocks,groundPlane


    def placeStepsOnBlocks(self, blocks, groundPlane, standingFootName, standingFootFrame, removeFirstLeftStep = True):

        footsteps = []
        for i, block in enumerate(blocks):
            nextLeftTransform = transformUtils.frameFromPositionAndRPY([-0.27,0.29,0.08], [0,0,0])
            nextLeftTransform.Concatenate(block.cornerTransform)
            footsteps.append(Footstep(nextLeftTransform,False))

            nextRightTransform = transformUtils.frameFromPositionAndRPY([-0.23,0.1,0.08], [0,0,0])
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
            if (standingFootName is 'r_foot'):
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


    def replanFootsteps(self, polyData, standingFootName, removeFirstLeftStep=True, doStereoFiltering=True):
        obj = om.getOrCreateContainer('continuous')
        om.getOrCreateContainer('cont debug', obj)

        vis.updatePolyData( polyData, 'walking snapshot', parent='cont debug', visible=False)

        standingFootFrame = self.robotStateModel.getLinkFrame(standingFootName)
        vis.updateFrame(standingFootFrame, standingFootName, parent='cont debug', visible=False)
        # TODO: remove the pitch and roll of this frame to support it being on uneven ground

        # Step 1: filter the data down to a box in front of the robot:
        polyData = self.getRecedingTerrainRegion(polyData, standingFootFrame)
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
        if (clusters is None):
            print "No cluster found, stop walking now!"
            return

        # Step 3: find the corners of the minimum bounding rectangles
        blocks,groundPlane = self.extractBlocksFromSurfaces(clusters, standingFootFrame)

        footsteps = self.placeStepsOnBlocks(blocks, groundPlane, standingFootName, standingFootFrame, removeFirstLeftStep)

        if (len(footsteps) ==0):
            print "no steps to plan, returning"
            return

        # Step 5: Find the two foot positions we should plan with: the next committed tool and the current standing foot
        if (self.committedStep is not None):
          #print "i got a committedStep. is_right_foot?" , self.committedStep.is_right_foot
          if (self.committedStep.is_right_foot):
              standingFootTransform = self.robotStateModel.getLinkFrame('l_foot')
              nextDoubleSupportPose = self.getNextDoubleSupportPose(standingFootTransform, self.committedStep.transform)
          else:
              standingFootTransform = self.robotStateModel.getLinkFrame('r_foot')
              nextDoubleSupportPose = self.getNextDoubleSupportPose(self.committedStep.transform, standingFootTransform)

          comm_mesh,comm_color = self.getMeshAndColor(self.committedStep.is_right_foot)
          comm_color[1] = 0.75 ; comm_color[2] = 0.25
          stand_mesh, stand_color = self.getMeshAndColor( not self.committedStep.is_right_foot )
          stand_color[1] = 0.75 ; stand_color[2] = 0.25
          vis.updateFrame(self.committedStep.transform, 'committed foot', parent='foot placements', scale=0.2, visible=False)
          obj = vis.showPolyData(comm_mesh, 'committed step', color=comm_color, alpha=1.0, parent='steps')
          obj.actor.SetUserTransform(self.committedStep.transform)
          vis.updateFrame(standingFootTransform, 'standing foot', parent='foot placements', scale=0.2, visible=False)
          obj = vis.showPolyData(stand_mesh, 'standing step', color=stand_color, alpha=1.0, parent='steps')
          obj.actor.SetUserTransform(standingFootTransform)

        else:
            # don't have a committed footstep, assume we are standing still
            nextDoubleSupportPose = self.robotStateJointController.getPose('EST_ROBOT_STATE')

        self.displayExpectedPose(nextDoubleSupportPose)

        if (self.queryPlanner):
            self.sendPlanningRequest(footsteps, nextDoubleSupportPose)
        else:
            self.drawFittedSteps(footsteps)

        # retain the first step as it will be the committed step for execution
        if (len(footsteps) > 0):
            self.committedStep = Footstep(footsteps[0].transform, footsteps[0].is_right_foot )

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
            step.pos = transformUtils.positionMessageFromFrame(step_t)
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
        request.params.planning_mode = 1
        request.params.nom_forward_step = 0.28
        request.params.map_mode = 1 #  2 footplane, 0 h+n, 1 h+zup, 3 hori
        request.params.max_num_steps = len(goalSteps)
        request.params.min_num_steps = len(goalSteps)

        lcmUtils.publish('FOOTSTEP_PLAN_REQUEST', request)

        #if (self.navigationPanel.automaticContinuousWalkingEnabled):
        #    print "Requested Footstep Plan, it will be AUTOMATICALLY EXECUTED"
        #else:
        #    print "Requested Footstep Plan, it will be not be executed"


    def onFootContactContinuous(self,msg):

        leftInContact = msg.left_contact > 0.0
        rightInContact = msg.right_contact > 0.0

        if (leftInContact and rightInContact):
            contactState="both"
        elif (leftInContact and not rightInContact):
            contactState="left"
        elif (not leftInContact and rightInContact):
            contactState="right"
        else:
            contactState="none"
            #print "No foot contacts. Error!"

        replanNow = False
        if (self.lastContactState is "both") and (contactState is "left"):
            replanNow = True
            standingFootName = 'l_foot'
        if (self.lastContactState is "both") and (contactState is "right"):
            replanNow = True
            standingFootName = 'r_foot'

        if (replanNow):
            #print "contact: ", self.lastContactState, " to ", contactState
            if (self.navigationPanel.automaticContinuousWalkingEnabled):
                self.makeReplanRequest(standingFootName)
            else:
                print "not enabled, not planning"

        self.lastContactState = contactState


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

        bdi_step_mesh = om.findObjectByName('bdi step')
        om.removeFromObjectModel(bdi_step_mesh)
        obj = vis.showPolyData(mesh, 'bdi step', color=color, alpha=1.0, parent='steps')
        obj.actor.SetUserTransform(footTransform)


    def makeReplanRequest(self, standingFootName, removeFirstLeftStep = True):

        if (self.processContinuousStereo):
            polyData = self.cameraView.getStereoPointCloud(2,'CAMERA_FUSED')
            #polyData = segmentation.applyVoxelGrid(polyData, leafSize=0.01)
            doStereoFiltering = True
        elif (self.processRawStereo):
            polyData = self.cameraView.getStereoPointCloud(2,'CAMERA')
            doStereoFiltering = True
            print "makeReplanRequest processRawStereo"
            #polyData = segmentation.applyVoxelGrid(polyData, leafSize=0.01)
        else:
            polyData = segmentation.getCurrentRevolutionData()
            doStereoFiltering = False

        self.replanFootsteps(polyData, standingFootName, removeFirstLeftStep, doStereoFiltering)


    def startContinuousWalking(self):
        self.committedStep = None
        self.makeReplanRequest('r_foot', removeFirstLeftStep = False)

        if (self.fixBlockYawWithInitial):
            self.initialRobotYaw = self.robotStateJointController.q[5]

    # i think this should be removed, according to robin
    def onFootstepPlanContinuous(self, msg):
        if (self.navigationPanel.automaticContinuousWalkingEnabled):
            print "Committing Footstep Plan for AUTOMATIC EXECUTION"
            lcmUtils.publish('COMMITTED_FOOTSTEP_PLAN', msg)

        else:
            print "Received Footstep Plan, it will be not be executed"





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
        positionConstraint, orientationConstraint = self.ikPlanner.createPositionOrientationConstraint('r_foot', rfootTransform, nullFrame)
        positionConstraint.tspan = [1.0, 1.0]
        orientationConstraint.tspan = [1.0, 1.0]
        constraints.append(positionConstraint)
        constraints.append(orientationConstraint)

        positionConstraint, orientationConstraint = self.ikPlanner.createPositionOrientationConstraint('l_foot', lfootTransform, nullFrame)
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
