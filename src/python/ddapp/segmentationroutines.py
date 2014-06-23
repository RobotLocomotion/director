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
import vtkNumpy
import numpy as np
import vtkPCLFiltersPython as pcl
from shallowCopy import shallowCopy
from debugVis import DebugData


def getDebugFolder():
    obj = om.findObjectByName('debug')
    if obj is None:
        obj = om.getOrCreateContainer('debug', om.getOrCreateContainer('segmentation'))
        om.collapse(obj)
    return obj


def applyLineFit(dataObj, distanceThreshold=0.02):

    f = pcl.vtkPCLSACSegmentationLine()
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

    f = pcl.vtkPCLEuclideanClusterExtraction()
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