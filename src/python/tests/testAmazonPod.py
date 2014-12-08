import os
import math
import vtk
from ddapp.consoleapp import ConsoleApp
from ddapp import ioUtils
from ddapp import segmentation
from ddapp import segmentationroutines
from ddapp import applogic
from ddapp import visualization as vis
from ddapp import transformUtils
from ddapp import filterUtils
from ddapp import roboturdf
from ddapp import vtkNumpy as vnp

import numpy as np

def removePlaneAndBeyond(polyData, expectedNormal=[1,0,0], filterRange=[-np.inf, -0.03], whichAxis=1, whichAxisLetter='y', percentile = 95):

    yvalues = vnp.getNumpyFromVtk(polyData, 'Points')[:, whichAxis]
    backY = np.percentile(yvalues, percentile)

    if ( percentile > 50):
        searchRegion = segmentation.thresholdPoints(polyData, whichAxisLetter, [backY - 0.1, np.inf])
    else:
        searchRegion = segmentation.thresholdPoints(polyData, whichAxisLetter, [-np.inf, backY + 0.1])

    vis.updatePolyData(searchRegion, 'search region', parent="segmentation", colorByName=whichAxisLetter, visible=False)

    # find the plane of the back wall, remove it and the points behind it:
    _, origin, normal = segmentation.applyPlaneFit(searchRegion, distanceThreshold=0.02, expectedNormal=expectedNormal, perpendicularAxis=expectedNormal, returnOrigin=True)

    points = vnp.getNumpyFromVtk(polyData, 'Points')
    dist = np.dot(points - origin, normal)
    vnp.addNumpyToVtk(polyData, dist, 'dist_to_plane')

    backFrame = transformUtils.getTransformFromOriginAndNormal(origin, normal, normalAxis=2)
    vis.updateFrame(backFrame, 'back frame', parent='segmentation', scale=0.15 , visible=False)
    vis.updatePolyData(polyData, 'dist to back', parent='segmentation', visible=False)

    polyData = segmentation.thresholdPoints(polyData, 'dist_to_plane', filterRange)
    vis.updatePolyData(polyData, 'back off and all', parent='segmentation', visible=False)

    return polyData


def fitObjectsOnShelf(polyData, maxHeight = 0.25):
    # find the shelf plane:
    polyDataWithoutFront, _ = segmentation.removeMajorPlane(polyData, distanceThreshold=0.02)
    polyDataPlaneFit, origin, normal = segmentation.applyPlaneFit(polyDataWithoutFront,  expectedNormal=np.array([0.0,0.0,1.0]), perpendicularAxis=np.array([0.0,0.0,1.0]), returnOrigin=True)
    vis.updatePolyData(polyDataPlaneFit, 'polyDataPlaneFit', parent='segmentation', visible=False)

    shelfSurfacePoints = segmentation.thresholdPoints(polyDataPlaneFit, 'dist_to_plane', [-0.01, 0.01])
    shelfCenter = segmentation.computeCentroid(shelfSurfacePoints)
    shelfFrame = transformUtils.getTransformFromOriginAndNormal(shelfCenter, normal, normalAxis=2)
    vis.showFrame(shelfFrame, 'shelfFrame', parent='segmentation', scale=0.15 , visible=False)

    # find the points near to the shelf plane and find objects on it:
    points = vnp.getNumpyFromVtk(polyData, 'Points')
    dist = np.dot(points - origin, normal)
    vnp.addNumpyToVtk(polyData, dist, 'dist_to_plane')
    shelfPoints = segmentation.thresholdPoints(polyData, 'dist_to_plane', [-0.01, maxHeight])
    vis.updatePolyData(shelfPoints, 'shelf', parent='segmentation', visible=False)

    data = segmentation.segmentTableScene(shelfPoints, shelfCenter, filterClustering = False )
    vis.showClusterObjects(data.clusters + [data.table], parent='segmentation')

    # remove the points that we considered from the orginal cloud
    dists = vnp.getNumpyFromVtk(polyData, 'dist_to_plane')
    diffShelf = ( ((dists > maxHeight) + (dists < -0.01))) + 0.1 -0.1
    vnp.addNumpyToVtk(polyData, diffShelf, 'diff_shelf')
    polyData = segmentation.thresholdPoints(polyData, 'diff_shelf', [1, 1])

    vis.updatePolyData(polyData, 'rest', parent='segmentation', visible=False)
    return polyData


app = ConsoleApp()

# create a view
view = app.createView()
segmentation._defaultSegmentationView = view

robotStateModel, robotStateJointController = roboturdf.loadRobotModel('robot state model', view, parent='sensors', color=roboturdf.getRobotGrayColor(), visible=True)
segmentationroutines.SegmentationContext.initWithRobot(robotStateModel)

# load poly data
dataDir = app.getTestingDataDirectory()
polyData = ioUtils.readPolyData(os.path.join(dataDir, 'amazon-pod/01-small-changes.vtp'))
vis.showPolyData(polyData, 'pointcloud snapshot', visible=False)

# remove ground and clip to just the pod:
groundPoints, polyData = segmentation.removeGround(polyData)
vis.showPolyData(polyData, 'scene', visible=False)
polyData = segmentation.addCoordArraysToPolyData(polyData)
polyData = segmentation.thresholdPoints(polyData, 'y', [1, 1.6])
polyData = segmentation.thresholdPoints(polyData, 'x', [-1.2, 0.5])
vis.showPolyData(polyData, 'clipped', visible=False)

# remove outliers
polyData = segmentation.labelOutliers(polyData, searchRadius=0.03, neighborsInSearchRadius=40)
polyData = segmentation.thresholdPoints(polyData, 'is_outlier', [0, 0])
vis.showPolyData(polyData, 'inliers', visible=False)

# remove walls, and points behind temp:
polyData = removePlaneAndBeyond(polyData, expectedNormal=[0,1,0], filterRange=[-np.inf, -0.03], whichAxis=1, whichAxisLetter='y', percentile = 95)
polyData = removePlaneAndBeyond(polyData, expectedNormal=[1,0,0], filterRange=[-np.inf, -0.03], whichAxis=0, whichAxisLetter='x', percentile = 95)
polyData = removePlaneAndBeyond(polyData, expectedNormal=[1,0,0], filterRange=[0.03, np.inf], whichAxis=0, whichAxisLetter='x', percentile = 5)

vis.updatePolyData(polyData, 'only shelves', parent='segmentation', visible=False)


polyData = fitObjectsOnShelf(polyData)
polyData = fitObjectsOnShelf(polyData)
polyData = fitObjectsOnShelf(polyData)
polyData = fitObjectsOnShelf(polyData, maxHeight=0.2)



if app.getTestingInteractiveEnabled():
    view.show()
    app.showObjectModel()
    app.start()
