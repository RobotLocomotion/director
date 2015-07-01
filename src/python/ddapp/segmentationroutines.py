''' Routines and Fitting algorithms
   Fitting: means where ALL other non-object points
   have been removed, determining the transform frame
   of the object

   Segment: means seperating clusters from a single cloud
'''


from ddapp.filterUtils import *
import ddapp.visualization as vis
from ddapp import objectmodel as om
from ddapp.transformUtils import getTransformFromAxes
from ddapp import vtkAll as vtk


#from ddapp.footstepsdriver import FootstepsDriver

import vtkNumpy
import numpy as np
from shallowCopy import shallowCopy
from debugVis import DebugData



class SegmentationContext(object):
    '''
       Maintains an abstraction between the fitting scene and a robot
       Assumes point cloud is world aligned, with z up
       Provides access to (1) ground height,
       (2) location of the head frame, (3) view direction

       Can be configured:
       (a) Default mode: populated continously by EST_ROBOT_STATE
           (2) and (3) set seperately
       (b) Autonomy: where (2) gives (3)
       (c) Populated programmatically. e.g:
           - for unit testing
           - where ground plane from feet cannot be used
    '''

    def __init__(self, groundHeightProvider, viewProvider):
        self.groundHeightProvider = groundHeightProvider
        self.viewProvider = viewProvider

    def getGroundHeight(self):
        return self.groundHeightProvider.getGroundHeight()

    def getViewFrame(self):
        return self.viewProvider.getViewFrame()

    def getViewOrigin(self):
        return self.viewProvider.getViewOrigin()

    def getViewDirection(self):
        return self.viewProvider.getViewDirection()

    '''
    These static methods are provided for convenience to initialize
    a globalally accessible instance of the SegmentationContext.
    '''

    _globalSegmentationContext = None

    @staticmethod
    def installGlobalInstance(inst):
        if SegmentationContext._globalSegmentationContext is not None:
            raise Exception('Error, a global segmentation context instance is already installed.')

        SegmentationContext._globalSegmentationContext = inst

    @staticmethod
    def getGlobalInstance():
        if SegmentationContext._globalSegmentationContext is None:
            raise Exception('Error, the global segmentation context instance has not been initialized.')
        return SegmentationContext._globalSegmentationContext

    @staticmethod
    def initWithRobot(model):
        sc = SegmentationContext(RobotModelGroundHeightProvider(model), RobotModelViewProvider(model))
        SegmentationContext.installGlobalInstance(sc)

    @staticmethod
    def initWithCamera(camera, userGroundHeight):
        sc = SegmentationContext(UserGroundHeightProvider(userGroundHeight), CameraViewProvider(camera))
        SegmentationContext.installGlobalInstance(sc)

    @staticmethod
    def initWithUser(userGroundHeight, userViewFrame):
        sc = SegmentationContext(UserGroundHeightProvider(userGroundHeight), UserViewProvider(userViewFrame))
        SegmentationContext.installGlobalInstance(sc)


class RobotModelGroundHeightProvider(object):

    def __init__(self, model):
        self.model = model

    def getGroundHeight(self):
        return FootstepsDriver.getFeetMidPoint(self.model).GetPosition()[2]


class RobotModelViewProvider(object):

    def __init__(self, model):
        self.model = model

    def getViewFrame(self):
        return self.model.getLinkFrame('head')

    def getViewOrigin(self):
        headFrame = self.model.getLinkFrame('head')
        return np.array(headFrame.GetPosition())

    def getViewDirection(self):
        headFrame = self.model.getLinkFrame('head')
        viewDirection = [1,0,0]
        headFrame.TransformVector(viewDirection, viewDirection)
        return np.array(viewDirection)

class UserGroundHeightProvider(object):

    def __init__(self, groundHeight):
        self.groundHeight = groundHeight

    def getGroundHeight():
        return self.groundHeight

class UserViewProvider(object):

    def __init__(self, viewFrame):
        self.viewFrame = viewFrame

    def getViewFrame(self):
        return self.viewFrame

    def getViewOrigin(self):
        return np.array( self.viewFrame.GetPosition())

    def getViewDirection(self):
        viewDirection = [1,0,0]
        self.viewFrame.TransformVector(viewDirection, viewDirection)
        return np.array(viewDirection)

class CameraViewProvider(object):

    def __init__(self, camera):
        self.camera = camera

    def getViewFrame(self):
        return  self.camera.GetViewTransformObject()

    def getViewOrigin(self):
        return np.array(self.camera.GetViewPosition())

    def getViewDirection(self):
        return np.array(self.camera.GetViewDirection())



def getDebugFolder():
    obj = om.findObjectByName('debug')
    if obj is None:
        obj = om.getOrCreateContainer('debug', om.getOrCreateContainer('segmentation'))
        om.collapse(obj)
    return obj


def applyLineFit(dataObj, distanceThreshold=0.02):

    f = vtk.vtkPCLSACSegmentationLine()
    f.SetInput(dataObj)
    f.SetDistanceThreshold(distanceThreshold)
    f.Update()
    origin = np.array(f.GetLineOrigin())
    direction = np.array(f.GetLineDirection())

    return origin, direction, shallowCopy(f.GetOutput())


def projectPointToPlane(point, origin, normal):
    projectedPoint = np.zeros(3)
    vtk.vtkPlane.ProjectPoint(point, origin, normal, projectedPoint)
    return projectedPoint


def intersectLineWithPlane(line_point, line_ray, plane_point, plane_normal ):
    '''
    Find the intersection between a line and a plane
    http://www.scratchapixel.com/lessons/3d-basic-lessons/lesson-7-intersecting-simple-shapes/ray-plane-and-ray-disk-intersection/
    '''

    line_point = np.asarray(line_point)
    line_ray = np.asarray(line_ray)
    plane_point = np.asarray(plane_point)
    plane_normal = np.asarray(plane_normal)

    denom = np.dot( plane_normal , line_ray )

    # TODO: implement this check
    #if (denom > 1E-6):
    #    # ray is very close to parallel to plane
    #    return None

    p0l0 = plane_point - line_point
    t = np.dot(p0l0, plane_normal) / denom

    intersection_point = line_point + t*line_ray
    return intersection_point


def labelPointDistanceAlongAxis(polyData, axis, origin=None, resultArrayName='distance_along_axis'):

    points = vtkNumpy.getNumpyFromVtk(polyData, 'Points')
    if origin is not None:
        points = points - origin
    distanceValues = np.dot(points, axis)
    if origin is None:
        distanceValues -= np.nanmin(distanceValues)
    newData = shallowCopy(polyData)
    vtkNumpy.addNumpyToVtk(newData, distanceValues, resultArrayName)
    return newData


def applyEuclideanClustering(dataObj, clusterTolerance=0.05, minClusterSize=100, maxClusterSize=1e6):

    f = vtk.vtkPCLEuclideanClusterExtraction()
    f.SetInput(dataObj)
    f.SetClusterTolerance(clusterTolerance)
    f.SetMinClusterSize(int(minClusterSize))
    f.SetMaxClusterSize(int(maxClusterSize))
    f.Update()
    return shallowCopy(f.GetOutput())


def extractClusters(polyData, clusterInXY=False, **kwargs):
    ''' Segment a single point cloud into smaller clusters
        using Euclidean Clustering
     '''

    if not polyData.GetNumberOfPoints():
        return []

    if (clusterInXY == True):
        ''' If Points are seperated in X&Y, then cluster outside this '''
        polyDataXY = vtk.vtkPolyData()
        polyDataXY.DeepCopy(polyData)
        points=vtkNumpy.getNumpyFromVtk(polyDataXY , 'Points') # shared memory
        points[:,2] = 0.0
        #showPolyData(polyDataXY, 'polyDataXY', visible=False, parent=getDebugFolder())
        polyDataXY = applyEuclideanClustering(polyDataXY, **kwargs)
        clusterLabels = vtkNumpy.getNumpyFromVtk(polyDataXY, 'cluster_labels')
        vtkNumpy.addNumpyToVtk(polyData, clusterLabels, 'cluster_labels')

    else:
        polyData = applyEuclideanClustering(polyData, **kwargs)
        clusterLabels = vtkNumpy.getNumpyFromVtk(polyData, 'cluster_labels')


    clusters = []
    for i in xrange(1, clusterLabels.max() + 1):
        cluster = thresholdPoints(polyData, 'cluster_labels', [i, i])
        clusters.append(cluster)
    return clusters


def applyVoxelGrid(polyData, leafSize=0.01):

    v = vtk.vtkPCLVoxelGrid()
    v.SetLeafSize(leafSize, leafSize, leafSize)
    v.SetInput(polyData)
    v.Update()
    return shallowCopy(v.GetOutput())


def labelOutliers(dataObj, searchRadius=0.03, neighborsInSearchRadius=10):

    f = vtk.vtkPCLRadiusOutlierRemoval()
    f.SetInput(dataObj)
    f.SetSearchRadius(searchRadius)
    f.SetNeighborsInSearchRadius(int(neighborsInSearchRadius))
    f.Update()
    return shallowCopy(f.GetOutput())


def sparsifyStereoCloud(polyData):
    ''' Take in a typical Stereo Camera Point Cloud
    Filter it down to about the density of a lidar point cloud
    and remove outliers
    '''

    # >>> strips color out <<<
    polyData = applyVoxelGrid(polyData, leafSize=0.01)

    # remove outliers
    polyData = labelOutliers(polyData)
    vis.showPolyData(polyData, 'is_outlier', colorByName='is_outlier', visible=False, parent=getDebugFolder())
    polyData = thresholdPoints(polyData, 'is_outlier', [0.0, 0.0])
    return polyData

def fitDrillBarrel ( drillPoints, forwardDirection, plane_origin, plane_normal):
    ''' Given a point cloud which ONLY contains points from a barrell drill, standing upright
        and the equations of a table its resting on, and the general direction of the robot
        Fit a barrell drill
     '''

    if not drillPoints.GetNumberOfPoints():
        return

    vis.updatePolyData(drillPoints, 'drill cluster', parent=getDebugFolder(), visible=False)
    drillBarrelPoints = thresholdPoints(drillPoints, 'dist_to_plane', [0.177, 0.30])

    if not drillBarrelPoints.GetNumberOfPoints():
        return


    # fit line to drill barrel points
    linePoint, lineDirection, _ = applyLineFit(drillBarrelPoints, distanceThreshold=0.5)

    if np.dot(lineDirection, forwardDirection) < 0:
        lineDirection = -lineDirection

    vis.updatePolyData(drillBarrelPoints, 'drill barrel points', parent=getDebugFolder(), visible=False)


    pts = vtkNumpy.getNumpyFromVtk(drillBarrelPoints, 'Points')

    dists = np.dot(pts-linePoint, lineDirection)

    p1 = linePoint + lineDirection*np.min(dists)
    p2 = linePoint + lineDirection*np.max(dists)

    p1 = projectPointToPlane(p1, plane_origin, plane_normal)
    p2 = projectPointToPlane(p2, plane_origin, plane_normal)


    d = DebugData()
    d.addSphere(p1, radius=0.01)
    d.addSphere(p2, radius=0.01)
    d.addLine(p1, p2)
    vis.updatePolyData(d.getPolyData(), 'drill debug points', color=[0,1,0], parent=getDebugFolder(), visible=False)


    drillToBasePoint = np.array([-0.07,  0.0  , -0.12])

    zaxis = plane_normal
    xaxis = lineDirection
    xaxis /= np.linalg.norm(xaxis)
    yaxis = np.cross(zaxis, xaxis)
    yaxis /= np.linalg.norm(yaxis)
    xaxis = np.cross(yaxis, zaxis)
    xaxis /= np.linalg.norm(xaxis)

    t = getTransformFromAxes(xaxis, yaxis, zaxis)
    t.PreMultiply()
    t.Translate(-drillToBasePoint)
    t.PostMultiply()
    t.Translate(p1)

    return t
