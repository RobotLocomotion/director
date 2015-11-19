import os
import math
from director.consoleapp import ConsoleApp
from director import ioUtils
from director import segmentation
from director import segmentationroutines
from director import applogic
from director import visualization as vis
from director import vtkNumpy as vnp
from director import objectmodel as om
from director import screengrabberpanel
from director import cameracontrol
from director import filterUtils
from director.fieldcontainer import FieldContainer
from director.shallowCopy import shallowCopy
from director.debugVis import DebugData
from director import vtkAll as vtk

import numpy as np
import shutil

app = ConsoleApp()

# create a view
view = app.createView()
segmentation._defaultSegmentationView = view


# load poly data
dataDir = app.getTestingDataDirectory()

filename = os.path.join(dataDir, 'terrain/tilted_steps_lidar.pcd')
#filename = os.path.join(dataDir, 'BigBIRD/tapatio_hot_sauce/meshes/poisson.ply')
#filename = os.path.join(dataDir, 'BigBIRD/cheez_it_white_cheddar/meshes/poisson.ply')

name = 'terrain'
polyData = ioUtils.readPolyData(filename)
vis.showPolyData(polyData, name, visible=False)


useTerrainOptions = True

if useTerrainOptions:
    removeGround = True
    useVoxelGrid = True
    reorientNormals = False
    testNormals = False
    showGlyphs = False
    testPlaneSegmentation = True

else:
    removeGround = False
    useVoxelGrid = False
    reorientNormals = True
    testNormals = False
    showGlyphs = False
    testPlaneSegmentation = True


if reorientNormals:
    polyData = filterUtils.computeNormals(polyData)


if removeGround:
    groundPoints, scenePoints =  segmentation.removeGround(polyData)
    vis.showPolyData(groundPoints, 'ground', visible=False)
    polyData = scenePoints


if useVoxelGrid:
    polyData = segmentation.applyVoxelGrid(polyData, leafSize=0.02)

if testNormals:

    print 'computing normals...'
    f = vtk.vtkRobustNormalEstimator()
    f.SetInput(polyData)
    f.SetMaxIterations(100)
    f.SetMaxEstimationError(0.01)
    f.SetMaxCenterError(0.02)
    f.SetComputeCurvature(True)
    f.SetRadius(0.1)
    f.Update()
    polyData = shallowCopy(f.GetOutput())
    print 'done.'


    # filter points without normals
    normals = vnp.getNumpyFromVtk(polyData, 'normals')

    segmentation.flipNormalsWithViewDirection(polyData, [1, -1, -1])

    normalsValid = np.any(normals, axis=1)
    vnp.addNumpyToVtk(polyData, np.array(normalsValid, dtype=np.int32), 'normals_valid')

    vis.showPolyData(polyData, 'scene points', colorByName='normals_valid', visible=False)

    numPoints = polyData.GetNumberOfPoints()

    polyData = segmentation.thresholdPoints(polyData, 'normals_valid', [1, 1])
    vis.showPolyData(polyData, 'cloud normals', colorByName='curvature', visible=True)

    print 'number of filtered points:', numPoints - polyData.GetNumberOfPoints()

    if showGlyphs:
        polyData.GetPointData().SetNormals(polyData.GetPointData().GetArray('normals'))
        arrows = segmentation.applyArrowGlyphs(polyData, computeNormals=False)
        disks = segmentation.applyDiskGlyphs(polyData, computeNormals=False)
        polyData.GetPointData().SetNormals(None)

        vis.showPolyData(arrows, 'arrows')
        vis.showPolyData(disks, 'disks')

        #arrows2 = segmentation.applyArrowGlyphs(polyData)
        #vis.showPolyData(arrows2, 'arrows pcl normals')


def getMergedConvexHullsMesh(chulls):
    d = DebugData()
    for i, chull in enumerate(chulls):
        d.addPolyData(chull.convexHull, color=segmentation.getRandomColor())
    return d.getPolyData()


def saveConvexHulls(chulls, outputDir):

    if os.path.isdir(outputDir):
        print 'removing directory:', outputDir
        shutil.rmtree(outputDir)

    print 'making directory:', outputDir
    os.makedirs(outputDir)

    for i, chull in enumerate(chulls):

        chull, plane = chull.convexHull, chull.plane
        origin = plane.GetOrigin()
        normal = plane.GetNormal()
        chullPoints = vnp.getNumpyFromVtk(chull, 'Points')

        assert np.allclose(np.linalg.norm(normal), 1.0)

        output = np.vstack([normal, chullPoints])
        outputFilename = os.path.join(outputDir, 'plane_%04d.txt' % i)

        np.savetxt(outputFilename, output)

    ioUtils.writePolyData(getMergedConvexHullsMesh(chulls),  os.path.join(outputDir, 'merged_planes.ply'))


def computePlanarConvexHull(polyData, expectedNormal=None):

    plane = vtk.vtkPlane()
    vtk.vtkSurfaceFitter.ComputePlane(polyData, plane)

    if expectedNormal is not None:
        planeNormal = plane.GetNormal()
        if np.dot(planeNormal, expectedNormal) < 0:
            plane.SetNormal(-1*np.array(planeNormal))

    chull = vtk.vtkPolyData()
    vtk.vtkSurfaceFitter.ComputeConvexHull(polyData, plane, chull)
    return FieldContainer(points=polyData, convexHull=chull, plane=plane)


if testPlaneSegmentation:

    if reorientNormals:
        inputNormals = vnp.getNumpyFromVtk(polyData, 'Normals').copy()
        inputNormals *= -1 # the normals from the BigBIRD meshes are inverted for some reason (incorrect triangle winding?)
        vnp.addNumpyToVtk(polyData, inputNormals, 'original_normals')

    f = vtk.vtkSurfaceFitter()
    f.SetInput(polyData)

    if useTerrainOptions:
        f.SetMaxError(0.07)
        f.SetMaxAngle(np.radians(8))
        f.SetSearchRadius(0.06)
        f.SetMinimumNumberOfPoints(100)
    else:
        f.SetMaxError(0.005)
        f.SetMaxAngle(np.radians(15))
        f.SetSearchRadius(0.01)
        f.SetMinimumNumberOfPoints(10)

        ne = f.GetRobustNormalEstimator()
        ne.SetRadius(0.01)
        ne.SetMaxEstimationError(0.01)
        ne.SetMaxCenterError(0.01)

    f.Update()
    polyData = shallowCopy(f.GetOutput())

    labels = vnp.getNumpyFromVtk(polyData, 'plane_segmentation_labels')
    maxLabel = labels.max()

    polyData = segmentation.thresholdPoints(polyData, 'plane_segmentation_labels', [1, maxLabel])
    vis.showPolyData(polyData, 'segmented cloud', colorByName='plane_segmentation_labels', visible=False)

    chulls = []

    for i in xrange(1, maxLabel+1):

        planePoints = segmentation.thresholdPoints(polyData, 'plane_segmentation_labels', [i, i])

        if reorientNormals:
            normals = vnp.getNumpyFromVtk(planePoints, 'original_normals')
            expectedNormal = normals[0]
        else:
            expectedNormal = None

        chulls.append(computePlanarConvexHull(planePoints, expectedNormal=expectedNormal))


    if removeGround:
        chulls.append(computePlanarConvexHull(groundPoints, expectedNormal=[0,0,1]))


    d = DebugData()

    for i, result in enumerate(chulls):

        planePoints, chull, plane = result.points, result.convexHull, result.plane

        c = segmentation.getRandomColor()
        vis.showPolyData(planePoints, 'plane %d' % i, color=c)
        chull = vis.showPolyData(chull, 'convex hull %d' % i, color=c)
        chull.setProperty('Surface Mode', 'Surface with edges')
        chull.actor.GetProperty().SetLineWidth(3)

        center = segmentation.computeCentroid(chull.polyData)
        chullPoints = vnp.getNumpyFromVtk(chull.polyData, 'Points')
        d.addLine(plane.GetOrigin(), np.array(plane.GetOrigin()) + 0.005 * np.array(plane.GetNormal()), radius=0.0001, color=[0,0,0])
        #d.addArrow(plane.GetOrigin(), np.array(plane.GetOrigin()) + 0.01 * np.array(plane.GetNormal()), headRadius=0.001, tubeRadius=0.0002)
        #d.addSphere(chullPoints[0], radius=0.001, color=[1,0,0])
        #d.addSphere(chullPoints[1], radius=0.001, color=[0,1,0])


    vis.showPolyData(d.getPolyData(), 'plane normals', colorByName='RGB255')

    #saveConvexHulls(chulls, name)
    #vis.showPolyData(ioUtils.readPolyData(os.path.join(name, 'merged_planes.ply')), 'merged_planes')


applogic.resetCamera([1,1,0])
applogic.setBackgroundColor([1,1,1])
view.orientationMarkerWidget().Off()
app.gridObj.setProperty('Color', [0,0,0])
app.gridObj.setProperty('Surface Mode', 'Surface with edges')

orbit = cameracontrol.OrbitController(view)
screenGrabberPanel = screengrabberpanel.ScreenGrabberPanel(view)

if app.getTestingInteractiveEnabled():
    view.show()
    view.resize(1280, 1024)
    app.start()
