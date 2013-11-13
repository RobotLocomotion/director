import os
import sys
import math
import vtk
import colorsys
import time
import functools
import PythonQt
from PythonQt import QtCore, QtGui
import ddapp.applogic as app
from ddapp import objectmodel as om
from ddapp import perception
from ddapp import lcmUtils
from ddapp import transformUtils
from ddapp.transformUtils import getTransformFromAxes
from ddapp.timercallback import TimerCallback
from ddapp.visualization import *

import numpy as np
import vtkNumpy
from debugVis import DebugData
from shallowCopy import shallowCopy
import affordance
import ioUtils
import pointCloudUtils

import vtkPCLFiltersPython as pcl

import drc as lcmdrc
import bot_core as lcmbotcore


eventFilters = {}

_spindleAxis = None


def getSegmentationView():
    return app.getViewManager().findView('Segmentation View')


def getDRCView():
    return app.getViewManager().findView('DRC View')


def switchToView(viewName):
    app.getViewManager().switchToView(viewName)


def getCurrentView():
    return app.getViewManager().currentView()


def getDebugFolder():
    obj = om.findObjectByName('debug')
    if obj is None:
        obj = om.getOrCreateContainer('debug', om.findObjectByName('segmentation'))
        om.collapse(obj)
    return obj


def thresholdPoints(polyData, arrayName, thresholdRange):
    assert(polyData.GetPointData().GetArray(arrayName))
    f = vtk.vtkThresholdPoints()
    f.SetInput(polyData)
    f.ThresholdBetween(thresholdRange[0], thresholdRange[1])
    f.SetInputArrayToProcess(0,0,0, vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS, arrayName)
    f.Update()
    return shallowCopy(f.GetOutput())


def transformPolyData(polyData, transform):

    t = vtk.vtkTransformPolyDataFilter()
    t.SetTransform(transform)
    t.SetInput(shallowCopy(polyData))
    t.Update()
    return shallowCopy(t.GetOutput())


def cropToLineSegment(polyData, point1, point2):

    line = point2 - point1
    length = np.linalg.norm(line)
    axis = line / length

    polyData = pointCloudUtils.labelPointDistanceAlongAxis(polyData, axis, origin=point1, resultArrayName='dist_along_line')
    return thresholdPoints(polyData, 'dist_along_line', [0.0, length])



'''
icp programmable filter

import vtkFiltersGeneralPython as filtersGeneral

points = inputs[0]
block = inputs[1]

print points.GetNumberOfPoints()
print block.GetNumberOfPoints()

if points.GetNumberOfPoints() < block.GetNumberOfPoints():
    block, points = points, block

icp = vtk.vtkIterativeClosestPointTransform()
icp.SetSource(points.VTKObject)
icp.SetTarget(block.VTKObject)
icp.GetLandmarkTransform().SetModeToRigidBody()
icp.Update()

t = filtersGeneral.vtkTransformPolyDataFilter()
t.SetInput(points.VTKObject)
t.SetTransform(icp)
t.Update()

output.ShallowCopy(t.GetOutput())
'''


def getRandomColor():
    '''
    Return a random color as a list of RGB values between 0.0 and 1.0.
    '''
    return colorsys.hsv_to_rgb(np.random.rand(), 1.0, 0.9)



def extractLargestCluster(polyData):

    polyData = applyEuclideanClustering(polyData)
    return thresholdPoints(polyData, 'cluster_labels', [1, 1])


def extractClusters(polyData, **kwargs):

    polyData = applyEuclideanClustering(polyData, **kwargs)
    clusterLabels = vtkNumpy.getNumpyFromVtk(polyData, 'cluster_labels')
    clusters = []
    for i in xrange(1, clusterLabels.max() + 1):
        cluster = thresholdPoints(polyData, 'cluster_labels', [i, i])
        clusters.append(cluster)
    return clusters



def segmentGroundPoints(polyData):

    zvalues = vtkNumpy.getNumpyFromVtk(polyData, 'Points')[:,2]
    groundHeight = np.percentile(zvalues, 5)
    polyData = thresholdPoints(polyData, 'z', [groundHeight - 0.3, groundHeight + 0.3])

    polyData, normal = applyPlaneFit(polyData, distanceThreshold=0.005, expectedNormal=[0,0,1])
    groundPoints = thresholdPoints(polyData, 'dist_to_plane', [-0.01, 0.01])

    return groundPoints, normal


def segmentGroundPlane():

    inputObj = om.findObjectByName('pointcloud snapshot')
    inputObj.setProperty('Visible', False)
    polyData = shallowCopy(inputObj.polyData)

    zvalues = vtkNumpy.getNumpyFromVtk(polyData, 'Points')[:,2]
    groundHeight = np.percentile(zvalues, 5)
    searchRegion = thresholdPoints(polyData, 'z', [groundHeight - 0.3, groundHeight + 0.3])

    updatePolyData(searchRegion, 'ground search region', parent=getDebugFolder(), colorByName='z', visible=False)

    _, origin, normal = applyPlaneFit(searchRegion, distanceThreshold=0.02, expectedNormal=[0,0,1], perpendicularAxis=[0,0,1], returnOrigin=True)

    points = vtkNumpy.getNumpyFromVtk(polyData, 'Points')
    dist = np.dot(points - origin, normal)
    vtkNumpy.addNumpyToVtk(polyData, dist, 'dist_to_plane')

    groundPoints = thresholdPoints(polyData, 'dist_to_plane', [-0.01, 0.01])
    scenePoints = thresholdPoints(polyData, 'dist_to_plane', [0.05, 10])

    updatePolyData(groundPoints, 'ground points', alpha=0.3)
    updatePolyData(scenePoints, 'scene points', alpha=0.3)

    #scenePoints = applyEuclideanClustering(scenePoints, clusterTolerance=0.10, minClusterSize=100, maxClusterSize=1e6)
    #updatePolyData(scenePoints, 'scene points', colorByName='cluster_labels')


def getMajorPlanes(polyData, useVoxelGrid=True):

    voxelGridSize = 0.01
    distanceToPlaneThreshold = 0.02

    if useVoxelGrid:
        polyData = applyVoxelGrid(polyData, leafSize=voxelGridSize)

    polyDataList = []

    minClusterSize = 100

    while len(polyDataList) < 25:

        polyData, normal = applyPlaneFit(polyData, distanceToPlaneThreshold)
        outliers = thresholdPoints(polyData, 'ransac_labels', [0, 0])
        inliers = thresholdPoints(polyData, 'ransac_labels', [1, 1])
        largestCluster = extractLargestCluster(inliers)

        #i = len(polyDataList)
        #showPolyData(inliers, 'inliers %d' % i, color=getRandomColor(), parent='major planes')
        #showPolyData(outliers, 'outliers %d' % i, color=getRandomColor(), parent='major planes')
        #showPolyData(largestCluster, 'cluster %d' % i, color=getRandomColor(), parent='major planes')

        if largestCluster.GetNumberOfPoints() > minClusterSize:
            polyDataList.append(largestCluster)
            polyData = outliers
        else:
            break

    return polyDataList


def showMajorPlanes():

    inputObj = om.findObjectByName('pointcloud snapshot')
    inputObj.setProperty('Visible', False)
    polyData = inputObj.polyData

    om.removeFromObjectModel(om.findObjectByName('major planes'))
    folderObj = om.findObjectByName('segmentation')
    folderObj = om.getOrCreateContainer('major planes', folderObj)

    polyData = thresholdPoints(polyData, 'distance', [1, 4])
    polyDataList = getMajorPlanes(polyData)


    for i, polyData in enumerate(polyDataList):
        obj = showPolyData(polyData, 'plane %d' % i, color=getRandomColor(), visible=True, parent='major planes')
        obj.setProperty('Point Size', 3)


def cropToBox(polyData, params, expansionDistance=0.1):


    origin = params['origin']

    xwidth = params['xwidth']
    ywidth = params['ywidth']
    zwidth = params['zwidth']

    xaxis = params['xaxis']
    yaxis = params['yaxis']
    zaxis = params['zaxis']


    for axis, width in ((xaxis, xwidth), (yaxis, ywidth), (zaxis, zwidth)):
        cropAxis = axis*(width/2.0 + expansionDistance)
        polyData = cropToLineSegment(polyData, origin - cropAxis, origin + cropAxis)

    updatePolyData(polyData, 'cropped')


def cropToSphere(polyData, origin, radius):
    polyData = labelDistanceToPoint(polyData, origin)
    return thresholdPoints(polyData, 'distance_to_point', [0, radius])


def applyEuclideanClustering(dataObj, clusterTolerance=0.05, minClusterSize=100, maxClusterSize=1e6):

    f = pcl.vtkPCLEuclideanClusterExtraction()
    f.SetInput(dataObj)
    f.SetClusterTolerance(clusterTolerance)
    f.SetMinClusterSize(int(minClusterSize))
    f.SetMaxClusterSize(int(maxClusterSize))
    f.Update()
    return shallowCopy(f.GetOutput())


def labelOutliers(dataObj, searchRadius=0.03, neighborsInSearchRadius=10):

    f = pcl.vtkPCLRadiusOutlierRemoval()
    f.SetInput(dataObj)
    f.SetSearchRadius(searchRadius)
    f.SetNeighborsInSearchRadius(int(neighborsInSearchRadius))
    f.Update()
    return shallowCopy(f.GetOutput())


def applyPlaneFit(polyData, distanceThreshold=0.02, expectedNormal=None, perpendicularAxis=None, angleEpsilon=0.2, returnOrigin=False, searchOrigin=None, searchRadius=None):

    expectedNormal = expectedNormal if expectedNormal is not None else [-1,0,0]

    fitInput = polyData
    if searchOrigin is not None:
        assert searchRadius
        fitInput = cropToSphere(fitInput, searchOrigin, searchRadius)

    # perform plane segmentation
    f = pcl.vtkPCLSACSegmentationPlane()
    f.SetInput(fitInput)
    f.SetDistanceThreshold(distanceThreshold)
    if perpendicularAxis is not None:
        f.SetPerpendicularConstraintEnabled(True)
        f.SetPerpendicularAxis(perpendicularAxis)
        f.SetAngleEpsilon(angleEpsilon)
    f.Update()
    origin = f.GetPlaneOrigin()
    normal = np.array(f.GetPlaneNormal())

    # flip the normal if needed
    if np.dot(normal, expectedNormal) < 0:
        normal = -normal

    # for each point, compute signed distance to plane

    polyData = shallowCopy(polyData)
    points = vtkNumpy.getNumpyFromVtk(polyData, 'Points')
    dist = np.dot(points - origin, normal)
    vtkNumpy.addNumpyToVtk(polyData, dist, 'dist_to_plane')

    if returnOrigin:
        return polyData, origin, normal
    else:
        return polyData, normal


def applyLineFit(dataObj, distanceThreshold=0.02):

    f = pcl.vtkPCLSACSegmentationLine()
    f.SetInput(dataObj)
    f.SetDistanceThreshold(distanceThreshold)
    f.Update()
    origin = np.array(f.GetLineOrigin())
    direction = np.array(f.GetLineDirection())

    return origin, direction, shallowCopy(f.GetOutput())


def addCoordArraysToPolyData(polyData):
    polyData = shallowCopy(polyData)
    points = vtkNumpy.getNumpyFromVtk(polyData, 'Points')
    vtkNumpy.addNumpyToVtk(polyData, points[:,0].copy(), 'x')
    vtkNumpy.addNumpyToVtk(polyData, points[:,1].copy(), 'y')
    vtkNumpy.addNumpyToVtk(polyData, points[:,2].copy(), 'z')

    bodyFrame = perception._multisenseItem.model.getFrame('body')
    bodyOrigin = bodyFrame.TransformPoint([0.0, 0.0, 0.0])
    bodyX = bodyFrame.TransformVector([1.0, 0.0, 0.0])
    bodyY = bodyFrame.TransformVector([0.0, 1.0, 0.0])
    bodyZ = bodyFrame.TransformVector([0.0, 0.0, 1.0])
    polyData = pointCloudUtils.labelPointDistanceAlongAxis(polyData, bodyX, origin=bodyOrigin, resultArrayName='distance_along_robot_x')
    polyData = pointCloudUtils.labelPointDistanceAlongAxis(polyData, bodyY, origin=bodyOrigin, resultArrayName='distance_along_robot_y')
    polyData = pointCloudUtils.labelPointDistanceAlongAxis(polyData, bodyZ, origin=bodyOrigin, resultArrayName='distance_along_robot_z')

    return polyData


def getDebugRevolutionData():
    dataDir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../drc-data'))
    #filename = os.path.join(dataDir, 'valve_wall.vtp')
    #filename = os.path.join(dataDir, 'bungie_valve.vtp')
    #filename = os.path.join(dataDir, 'cinder-blocks.vtp')
    #filename = os.path.join(dataDir, 'cylinder_table.vtp')
    #filename = os.path.join(dataDir, 'debris.vtp')
    #filename = os.path.join(dataDir, 'rev1.vtp')
    filename = os.path.join(dataDir, 'drill-in-hand.vtp')

    return addCoordArraysToPolyData(ioUtils.readPolyData(filename))


def getCurrentRevolutionData():
    revPolyData = perception._multisenseItem.model.revPolyData
    if not revPolyData or not revPolyData.GetNumberOfPoints():
        return None

    if useVoxelGrid:
        revPolyData = applyVoxelGrid(revPolyData, leafSize=0.015)

    return addCoordArraysToPolyData(revPolyData)


def getCurrentMapServerData():
    mapServer = om.findObjectByName('Map Server')
    polyData = None
    if mapServer and mapServer.getProperty('Visible'):
        polyData = mapServer.source.polyData

    if not polyData or not polyData.GetNumberOfPoints():
        return None

    return addCoordArraysToPolyData(polyData)


def getOrCreateSegmentationView():

    viewManager = app.getViewManager()
    segmentationView = viewManager.findView('Segmentation View')
    if not segmentationView:
        segmentationView = viewManager.createView('Segmentation View')
        installEventFilter(segmentationView, segmentationViewEventFilter)

    viewManager.switchToView('Segmentation View')
    return segmentationView


def getTransformFromAxes(xaxis, yaxis, zaxis):

    t = vtk.vtkTransform()
    m = vtk.vtkMatrix4x4()

    axes = [xaxis, yaxis, zaxis]
    for r in xrange(3):
        for c in xrange(3):
            # transpose on assignment
            m.SetElement(r, c, axes[c][r])

    t.SetMatrix(m)
    return t


useVoxelGrid = False

def applyVoxelGrid(polyData, leafSize=0.01):

    v = pcl.vtkPCLVoxelGrid()
    v.SetLeafSize(leafSize, leafSize, leafSize)
    v.SetInput(polyData)
    v.Update()
    return shallowCopy(v.GetOutput())


def getLumberDimensions(lumberId):

    dimensions = [
                  [0.089, 0.038], # 2x4
                  [0.140, 0.038], # 2x6
                  [0.089, 0.089], # 4x4
                 ]

    return dimensions[lumberId]


class SegmentationPanel(object):

    def __init__(self):
        self.panel = QtGui.QWidget()
        self.taskSelection = PythonQt.dd.ddTaskSelection()
        self.debrisWizard = self._makeDebrisWizard()
        self.terrainWizard = self._makeTerrainWizard()
        self.firehoseWizard = self._makeFirehoseWizard()
        self.drillWizard = self._makeDrillWizard()

        self.taskSelection.connect('taskSelected(int)', self.onTaskSelected)

        l = QtGui.QVBoxLayout(self.panel)
        l.addWidget(self.taskSelection)
        l.addWidget(self.debrisWizard)
        l.addWidget(self.terrainWizard)
        l.addWidget(self.firehoseWizard)
        l.addWidget(self.drillWizard)
        self.debrisWizard.hide()
        self.terrainWizard.hide()
        self.firehoseWizard.hide()
        self.drillWizard.hide()

    def _makeDebrisWizard(self):
        debrisWizard = QtGui.QWidget()
        lumberSelection = PythonQt.dd.ddLumberSelection()
        lumberSelection.connect('lumberSelected(int)', self.onDebrisLumberSelected)
        l = QtGui.QVBoxLayout(debrisWizard)
        l.addWidget(self._makeBackButton())
        l.addWidget(lumberSelection)
        l.addStretch()
        return debrisWizard

    def _makeFirehoseWizard(self):
        firehoseWizard = QtGui.QWidget()
        segmentButton = QtGui.QToolButton()
        segmentButton.setIcon(QtGui.QIcon(':/images/wye.png'))
        segmentButton.setIconSize(QtCore.QSize(60,60))
        segmentButton.connect('clicked()', self.onSegmentWye)
        l = QtGui.QVBoxLayout(firehoseWizard)
        l.addWidget(self._makeBackButton())
        l.addWidget(segmentButton)
        l.addStretch()
        return firehoseWizard


    def _makeButton(self, text, func):

        b = QtGui.QPushButton(text)
        b.connect('clicked()', func)
        return b

    def _makeDrillWizard(self):
        drillWizard = QtGui.QWidget()

        #segmentButton = QtGui.QToolButton()
        #segmentButton.setIcon(QtGui.QIcon(':/images/wye.png'))
        #segmentButton.setIconSize(QtCore.QSize(60,60))
        #segmentButton.connect('clicked()', self.onSegmentWye)

        l = QtGui.QVBoxLayout(drillWizard)
        l.addWidget(self._makeBackButton())
        l.addWidget(self._makeButton('segment drill on table', startDrillAutoSegmentation))
        l.addWidget(self._makeButton('segment drill in hand', startDrillInHandSegmentation))
        l.addWidget(self._makeButton('segment wall', startDrillWallSegmentation))
        l.addStretch()
        return drillWizard


    def _makeTerrainWizard(self):
        terrainWizard = QtGui.QWidget()

        self.cinderBlockButton = QtGui.QToolButton()
        self.cinderBlock2Button = QtGui.QToolButton()
        self.cinderBlockButton.setIcon(QtGui.QIcon(':/images/cinderblock.png'))
        self.cinderBlock2Button.setIcon(QtGui.QIcon(':/images/cinderblock_double.png'))
        self.cinderBlockButton.setIconSize(QtCore.QSize(60,60))
        self.cinderBlock2Button.setIconSize(QtCore.QSize(60,60))

        self.cinderBlockButton.connect('clicked()', functools.partial(self.onTerrainCinderblockSelected, self.cinderBlockButton))
        self.cinderBlock2Button.connect('clicked()', functools.partial(self.onTerrainCinderblockSelected, self.cinderBlock2Button))

        buttons = QtGui.QWidget()
        l = QtGui.QHBoxLayout(buttons)
        l.addStretch()
        l.addWidget(self.cinderBlockButton)
        l.addWidget(self.cinderBlock2Button)
        l.addStretch()

        l = QtGui.QVBoxLayout(terrainWizard)
        l.addWidget(self._makeBackButton())
        l.addWidget(buttons)
        l.addStretch()
        return terrainWizard

    def _makeBackButton(self):
        w = QtGui.QPushButton()
        w.setIcon(QtGui.QApplication.style().standardIcon(QtGui.QStyle.SP_ArrowBack))
        w.connect('clicked()', self.onBackButton)

        frame = QtGui.QWidget()
        l = QtGui.QHBoxLayout(frame)
        l.addWidget(w)
        l.addStretch()
        return frame


    def onBackButton(self):
        self.cancelCurrentTask()

    def onSegmentWye(self):
        startWyeSegmentation()

    def onDebrisLumberSelected(self, lumberId):
        blockDimensions = getLumberDimensions(lumberId)
        startInteractiveLineDraw(blockDimensions)

    def onTerrainCinderblockSelected(self, button):
        if button == self.cinderBlockButton:
            blockDimensions = [0.1905, 0.149225]
        elif button == self.cinderBlock2Button:
            blockDimensions = [0.1905, 0.29845]
        startInteractiveLineDraw(blockDimensions)

    def startDebrisTask(self):
        self.debrisWizard.show()
        self.taskSelection.hide()

    def startTerrainTask(self):
        self.terrainWizard.show()
        self.taskSelection.hide()

    def startFirehoseTask(self):
        self.firehoseWizard.show()
        self.taskSelection.hide()

    def startDrillTask(self):
        self.drillWizard.show()
        self.taskSelection.hide()

    def cancelCurrentTask(self):
        self.debrisWizard.hide()
        self.terrainWizard.hide()
        self.firehoseWizard.hide()
        self.drillWizard.hide()
        self.taskSelection.show()

    def onTaskSelected(self, taskId):

        taskFunctions = {
                         2:self.startTerrainTask,
                         4:self.startDebrisTask,
                         8:self.startFirehoseTask,
                         6:self.startDrillTask,
                        }

        taskFunction = taskFunctions.get(taskId+1)

        if taskFunction:
            taskFunction()
        else:
            app.showInfoMessage('Sorry, not impemented yet.')



def createDockWidget():
    global _segmentationPanel
    try: _segmentationPanel
    except NameError:
        _segmentationPanel = SegmentationPanel()
        app.addWidgetToDock(_segmentationPanel.panel)


def activateSegmentationMode(debug=False):

    if debug:
        polyData = getDebugRevolutionData()
    else:
        polyData = getCurrentMapServerData() or getCurrentRevolutionData()

    if not polyData:
        return

    global _spindleAxis
    _spindleAxis = perception._multisenseItem.model.getSpindleAxis()

    segmentationView = getOrCreateSegmentationView()

    perspective()

    initICPCallback()

    thresholdWorkspace = False
    doRemoveGround = False

    if thresholdWorkspace:
        polyData = thresholdPoints(polyData, 'distance_along_robot_x', [0.3, 2.0])
        polyData = thresholdPoints(polyData, 'distance_along_robot_y', [-3.0, 3.0])
        polyData = thresholdPoints(polyData, 'distance_along_robot_z', [-10.0, 1.5])

    if doRemoveGround:
        groundPoints, polyData = removeGround(polyData)
        segmentationObj = updatePolyData(groundPoints, 'ground', alpha=0.3, visible=False)

    segmentationObj = updatePolyData(polyData, 'pointcloud snapshot', alpha=0.3)
    segmentationObj.headAxis = perception._multisenseItem.model.getAxis('head', [1,0,0])

    segmentationView.camera().DeepCopy(app.getDRCView().camera())
    segmentationView.render()

    createDockWidget()


def segmentGroundPlanes():

    objs = []
    for obj in om.objects.values():
        name = obj.getProperty('Name')
        if name.startswith('pointcloud snapshot'):
            objs.append(obj)

    objs = sorted(objs, key=lambda x: x.getProperty('Name'))

    d = DebugData()

    prevHeadAxis = None
    for obj in objs:
        name = obj.getProperty('Name')
        print '----- %s---------' % name
        print  'head axis:', obj.headAxis
        groundPoints, normal = segmentGroundPoints(obj.polyData)
        print 'ground normal:', normal
        showPolyData(groundPoints, name + ' ground points', visible=False)
        a = np.array([0,0,1])
        b = np.array(normal)
        diff = math.degrees(math.acos(np.dot(a,b) / (np.linalg.norm(a) * np.linalg.norm(b))))
        if diff > 90:
            print 180 - diff
        else:
            print diff

        if prevHeadAxis is not None:
            a = prevHeadAxis
            b = np.array(obj.headAxis)
            diff = math.degrees(math.acos(np.dot(a,b) / (np.linalg.norm(a) * np.linalg.norm(b))))
            if diff > 90:
                print 180 - diff
            else:
                print diff
        prevHeadAxis = np.array(obj.headAxis)

        d.addLine([0,0,0], normal)

    updatePolyData(d.getPolyData(), 'normals')


def extractCircle(polyData, distanceThreshold=0.04, radiusLimit=None):

    circleFit = pcl.vtkPCLSACSegmentationCircle()
    circleFit.SetDistanceThreshold(distanceThreshold)
    circleFit.SetInput(polyData)
    if radiusLimit is not None:
        circleFit.SetRadiusLimit(radiusLimit)
        circleFit.SetRadiusConstraintEnabled(True)
    circleFit.Update()

    polyData = thresholdPoints(circleFit.GetOutput(), 'ransac_labels', [1.0, 1.0])
    return polyData, circleFit


def removeMajorPlane(polyData, distanceThreshold=0.02):

    # perform plane segmentation
    f = pcl.vtkPCLSACSegmentationPlane()
    f.SetInput(polyData)
    f.SetDistanceThreshold(distanceThreshold)
    f.Update()

    polyData = thresholdPoints(f.GetOutput(), 'ransac_labels', [0.0, 0.0])
    return polyData, f


def removeGround(polyData, groundThickness=0.02, sceneHeightFromGround=0.05):

    searchRegionThickness = 0.5

    zvalues = vtkNumpy.getNumpyFromVtk(polyData, 'z')
    assert zvalues is not None
    groundHeight = np.percentile(zvalues, 5)
    searchRegion = thresholdPoints(polyData, 'z', [groundHeight - searchRegionThickness/2.0, groundHeight + searchRegionThickness/2.0])

    updatePolyData(searchRegion, 'ground search region', parent=getDebugFolder(), colorByName='z', visible=False)

    _, origin, normal = applyPlaneFit(searchRegion, distanceThreshold=0.02, expectedNormal=[0,0,1], perpendicularAxis=[0,0,1], returnOrigin=True)

    points = vtkNumpy.getNumpyFromVtk(polyData, 'Points')
    dist = np.dot(points - origin, normal)
    vtkNumpy.addNumpyToVtk(polyData, dist, 'dist_to_plane')

    groundPoints = thresholdPoints(polyData, 'dist_to_plane', [-groundThickness/2.0, groundThickness/2.0])
    scenePoints = thresholdPoints(polyData, 'dist_to_plane', [sceneHeightFromGround, 100])

    return groundPoints, scenePoints


def generateFeetForValve():

    aff = om.findObjectByName('valve affordance')
    assert aff


    params = aff.params

    origin = np.array(params['origin'])
    origin[2] = 0.0

    xaxis = -params['axis']
    zaxis = np.array([0,0,1])
    yaxis = np.cross(zaxis, xaxis)
    xaxis = np.cross(yaxis, zaxis)

    stanceWidth = 0.2
    stanceRotation = 25.0
    stanceOffset = [-1.0, -0.5, 0.0]

    valveFrame = getTransformFromAxes(xaxis, yaxis, zaxis)
    valveFrame.PostMultiply()
    valveFrame.Translate(origin)

    stanceFrame, lfootFrame, rfootFrame = getFootFramesFromReferenceFrame(valveFrame, stanceWidth, stanceRotation, stanceOffset)

    showFrame(boardFrame, 'board ground frame', parent=aff, scale=0.15, visible=False)
    showFrame(lfootFrame, 'lfoot frame', parent=aff, scale=0.15)
    showFrame(rfootFrame, 'rfoot frame', parent=aff, scale=0.15)

    #d = DebugData()
    #d.addLine(valveFrame.GetPosition(), stanceFrame.GetPosition())
    #updatePolyData(d.getPolyData(), 'stance debug')
    #publishSteppingGoal(lfootFrame, rfootFrame)


def generateFeetForDebris():

    aff = om.findObjectByName('board A')
    if not aff:
        return

    params = aff.params

    origin = np.array(params['origin'])

    origin = origin + params['zaxis']*params['zwidth']/2.0 - params['xaxis']*params['xwidth']/2.0
    origin[2] = 0.0

    yaxis = params['zaxis']
    zaxis = np.array([0,0,1])
    xaxis = np.cross(yaxis, zaxis)

    stanceWidth = 0.35
    stanceRotation = 0.0
    stanceOffset = [-0.48, -0.08, 0]

    boardFrame = getTransformFromAxes(xaxis, yaxis, zaxis)
    boardFrame.PostMultiply()
    boardFrame.Translate(origin)

    stanceFrame, lfootFrame, rfootFrame = getFootFramesFromReferenceFrame(boardFrame, stanceWidth, stanceRotation, stanceOffset)

    showFrame(boardFrame, 'board ground frame', parent=aff, scale=0.15, visible=False)
    lfoot = showFrame(lfootFrame, 'lfoot frame', parent=aff, scale=0.15)
    rfoot = showFrame(rfootFrame, 'rfoot frame', parent=aff, scale=0.15)

    for obj in [lfoot, rfoot]:
        obj.addToView(app.getDRCView())

    #d = DebugData()
    #d.addLine(valveFrame.GetPosition(), stanceFrame.GetPosition())
    #updatePolyData(d.getPolyData(), 'stance debug')
    #publishSteppingGoal(lfootFrame, rfootFrame)


def generateFeetForWye():

    aff = om.findObjectByName('wye points')
    if not aff:
        return

    params = aff.params

    origin = np.array(params['origin'])
    origin[2] = 0.0

    yaxis = params['xaxis']
    xaxis = -params['zaxis']
    zaxis = np.cross(xaxis, yaxis)

    stanceWidth = 0.20
    stanceRotation = 0.0
    stanceOffset = [-0.48, -0.08, 0]

    affGroundFrame = getTransformFromAxes(xaxis, yaxis, zaxis)
    affGroundFrame.PostMultiply()
    affGroundFrame.Translate(origin)

    stanceFrame, lfootFrame, rfootFrame = getFootFramesFromReferenceFrame(affGroundFrame, stanceWidth, stanceRotation, stanceOffset)

    showFrame(affGroundFrame, 'affordance ground frame', parent=aff, scale=0.15, visible=False)
    lfoot = showFrame(lfootFrame, 'lfoot frame', parent=aff, scale=0.15)
    rfoot = showFrame(rfootFrame, 'rfoot frame', parent=aff, scale=0.15)

    for obj in [lfoot, rfoot]:
        obj.addToView(app.getDRCView())

    publishStickyFeet(lfootFrame, rfootFrame)


def getFootFramesFromReferenceFrame(referenceFrame, stanceWidth, stanceRotation, stanceOffset):

    footHeight=0.0745342

    ref = vtk.vtkTransform()
    ref.SetMatrix(referenceFrame.GetMatrix())

    stanceFrame = vtk.vtkTransform()
    stanceFrame.PostMultiply()
    stanceFrame.RotateZ(stanceRotation)
    stanceFrame.Translate(stanceOffset)
    stanceFrame.Concatenate(ref)

    lfootFrame = vtk.vtkTransform()
    lfootFrame.PostMultiply()
    lfootFrame.Translate(0, stanceWidth/2.0, footHeight)
    lfootFrame.Concatenate(stanceFrame)

    rfootFrame = vtk.vtkTransform()
    rfootFrame.PostMultiply()
    rfootFrame.Translate(0, -stanceWidth/2.0, footHeight)
    rfootFrame.Concatenate(stanceFrame)

    return stanceFrame, lfootFrame, rfootFrame


def poseFromFrame(frame):

    trans = lcmdrc.vector_3d_t()
    trans.x, trans.y, trans.z = frame.GetPosition()

    wxyz = range(4)
    perception.drc.vtkMultisenseSource.GetBotQuaternion(frame, wxyz)
    quat = lcmdrc.quaternion_t()
    quat.w, quat.x, quat.y, quat.z = wxyz

    pose = lcmdrc.position_3d_t()
    pose.translation = trans
    pose.rotation = quat
    return pose


def publishStickyFeet(lfootFrame, rfootFrame):

    worldAffordanceId = affordance.publishWorldAffordance()

    m = lcmdrc.traj_opt_constraint_t()
    m.utime = int(time.time() * 1e6)
    m.robot_name = worldAffordanceId
    m.num_links = 2
    m.link_name = ['l_foot', 'r_foot']
    m.link_timestamps = [m.utime, m.utime]
    m.num_joints = 0
    m.link_origin_position = [poseFromFrame(lfootFrame), poseFromFrame(rfootFrame)]
    #lcmUtils.publish('DESIRED_FOOT_STEP_SEQUENCE', m)
    lcmUtils.publish('AFF_TRIGGERED_CANDIDATE_STICKY_FEET', m)


def publishStickyHand(handFrame, affordanceItem=None):

    worldAffordanceId = affordance.publishWorldAffordance()

    m = lcmdrc.desired_grasp_state_t()
    m.utime = 0
    m.robot_name = 'atlas'
    m.object_name = worldAffordanceId
    m.geometry_name = 'box_0'
    m.unique_id = 3
    m.grasp_type = m.IROBOT_RIGHT
    m.power_grasp = False

    m.l_hand_pose = poseFromFrame(vtk.vtkTransform())
    m.r_hand_pose = poseFromFrame(handFrame)

    m.num_l_joints = 0
    m.l_joint_name = []
    m.l_joint_position = []

    m.num_r_joints = 8
    m.r_joint_name = [
        'right_finger[0]/joint_base_rotation',
        'right_finger[0]/joint_base',
        'right_finger[0]/joint_flex',
        'right_finger[1]/joint_base_rotation',
        'right_finger[1]/joint_base',
        'right_finger[1]/joint_flex',
        'right_finger[2]/joint_base',
        'right_finger[2]/joint_flex',
        ]

    m.r_joint_position = [
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0,
        ]

    lcmUtils.publish('CANDIDATE_GRASP', m)


def cropToPlane(polyData, origin, normal, threshold):
    polyData = shallowCopy(polyData)
    normal = normal/np.linalg.norm(normal)
    points = vtkNumpy.getNumpyFromVtk(polyData, 'Points')
    dist = np.dot(points - origin, normal)
    vtkNumpy.addNumpyToVtk(polyData, dist, 'dist_to_plane')
    cropped = thresholdPoints(polyData, 'dist_to_plane', threshold)
    return cropped, polyData


def createLine(blockDimensions, p1, p2):


    sliceWidth = np.array(blockDimensions).max()/2.0 + 0.02
    sliceThreshold =  [-sliceWidth, sliceWidth]


    # require p1 to be point on left
    if p1[0] > p2[0]:
        p1, p2 = p2, p1

    _, worldPt1 = getRayFromDisplayPoint(getSegmentationView(), p1)
    _, worldPt2 = getRayFromDisplayPoint(getSegmentationView(), p2)

    cameraPt = np.array(getSegmentationView().camera().GetPosition())

    leftRay = worldPt1 - cameraPt
    rightRay = worldPt2 - cameraPt
    middleRay = (leftRay + rightRay) / 2.0


    d = DebugData()
    d.addLine(cameraPt, worldPt1)
    d.addLine(cameraPt, worldPt2)
    d.addLine(worldPt1, worldPt2)
    d.addLine(cameraPt, cameraPt + middleRay)
    updatePolyData(d.getPolyData(), 'line annotation', parent=getDebugFolder(), visible=False)

    inputObj = om.findObjectByName('pointcloud snapshot')
    polyData = shallowCopy(inputObj.polyData)

    origin = cameraPt

    normal = np.cross(rightRay, leftRay)
    leftNormal = np.cross(normal, leftRay)
    rightNormal = np.cross(rightRay, normal)

    normal /= np.linalg.norm(normal)
    leftNormal /= np.linalg.norm(leftNormal)
    rightNormal /= np.linalg.norm(rightNormal)
    middleRay /= np.linalg.norm(middleRay)

    cropped, polyData = cropToPlane(polyData, origin, normal, sliceThreshold)

    updatePolyData(polyData, 'slice dist', parent=getDebugFolder(), colorByName='dist_to_plane', colorByRange=[-0.5, 0.5], visible=False)
    updatePolyData(cropped, 'slice',  parent=getDebugFolder(), colorByName='dist_to_plane', visible=False)

    cropped, _ = cropToPlane(cropped, origin, leftNormal, [-1e6, 0])
    cropped, _ = cropToPlane(cropped, origin, rightNormal, [-1e6, 0])

    updatePolyData(cropped, 'slice segment', parent=getDebugFolder(), colorByName='dist_to_plane', visible=False)

    planePoints, planeNormal = applyPlaneFit(cropped, distanceThreshold=0.005, perpendicularAxis=middleRay, angleEpsilon=math.radians(60))
    planePoints = thresholdPoints(planePoints, 'dist_to_plane', [-0.005, 0.005])
    updatePolyData(planePoints, 'board segmentation', parent=getDebugFolder(), color=getRandomColor(), visible=False)

    names = ['board A', 'board B', 'board C', 'board D', 'board E', 'board F', 'board G', 'board H', 'board I']
    for name in names:
        if not om.findObjectByName(name):
            break
    else:
        name = 'board'

    segmentBlockByTopPlane(planePoints, blockDimensions, expectedNormal=middleRay, expectedXAxis=middleRay, edgeSign=-1, name=name)


def startInteractiveLineDraw(blockDimensions):

    picker = LineDraw(getSegmentationView())
    addViewPicker(picker)
    picker.enabled = True
    picker.start()
    picker.annotationFunc = functools.partial(createLine, blockDimensions)


def segmentValveByWallPlane(expectedValveRadius, point1, point2):


    centerPoint = (point1 + point2) / 2.0

    inputObj = om.findObjectByName('pointcloud snapshot')
    polyData = inputObj.polyData

    cameraPos = np.array(getSegmentationView().camera().GetPosition())

    #bodyX = perception._multisenseItem.model.getAxis('body', [1.0, 0.0, 0.0])
    bodyX = centerPoint - cameraPos
    bodyX /= np.linalg.norm(bodyX)

    polyData, origin, normal = applyPlaneFit(polyData, expectedNormal=-bodyX, searchOrigin=point1, searchRadius=0.2, returnOrigin=True)


    perpLine = np.cross(point2 - point1, normal)
    #perpLine /= np.linalg.norm(perpLine)
    #perpLine * np.linalg.norm(point2 - point1)/2.0
    point3, point4 = centerPoint + perpLine/2.0, centerPoint - perpLine/2.0

    d = DebugData()
    d.addLine(point1, point2)
    d.addLine(point3, point4)
    updatePolyData(d.getPolyData(), 'crop lines', parent=getDebugFolder(), visible=False)

    wallPoints = thresholdPoints(polyData, 'dist_to_plane', [-0.01, 0.01])
    updatePolyData(wallPoints, 'valve wall', parent=getDebugFolder(), visible=False)

    searchRegion = thresholdPoints(polyData, 'dist_to_plane', [0.05, 0.4])
    searchRegion = cropToLineSegment(searchRegion, point1, point2)
    searchRegion = cropToLineSegment(searchRegion, point3, point4)

    updatePolyData(searchRegion, 'valve search region', parent=getDebugFolder(), visible=False)


    searchRegion, origin, _  = applyPlaneFit(searchRegion, expectedNormal=normal, perpendicularAxis=normal, returnOrigin=True)
    searchRegion = thresholdPoints(searchRegion, 'dist_to_plane', [-0.015, 0.015])

    updatePolyData(searchRegion, 'valve search region 2', parent=getDebugFolder(), visible=False)


    largestCluster = extractLargestCluster(searchRegion)

    updatePolyData(largestCluster, 'valve cluster', parent=getDebugFolder(), visible=False)


    #radiusLimit = [expectedValveRadius - 0.01, expectedValveRadius + 0.01] if expectedValveRadius else None
    radiusLimit = None

    polyData, circleFit = extractCircle(largestCluster, distanceThreshold=0.01, radiusLimit=radiusLimit)
    updatePolyData(polyData, 'circle fit', parent=getDebugFolder(), visible=False)


    #polyData, circleFit = extractCircle(polyData, distanceThreshold=0.01)
    #showPolyData(polyData, 'circle fit', colorByName='z')


    radius = circleFit.GetCircleRadius()
    origin = np.array(circleFit.GetCircleOrigin())
    circleNormal = np.array(circleFit.GetCircleNormal())
    circleNormal = circleNormal/np.linalg.norm(circleNormal)

    if np.dot(circleNormal, normal) < 0:
        circleNormal *= -1

    # force use of the plane normal
    circleNormal = normal
    radius = expectedValveRadius

    d = DebugData()
    d.addLine(origin - normal*radius, origin + normal*radius)
    d.addCircle(origin, circleNormal, radius)
    updatePolyData(d.getPolyData(), 'valve axes', parent=getDebugFolder(), visible=False)


    zaxis = circleNormal
    xaxis = bodyX
    yaxis = np.cross(zaxis, xaxis)
    xaxis = np.cross(yaxis, zaxis)
    xaxis /= np.linalg.norm(xaxis)
    yaxis /= np.linalg.norm(yaxis)
    t = getTransformFromAxes(xaxis, yaxis, zaxis)
    t.PostMultiply()
    t.Translate(origin)

    zwidth = 0.03

    d = DebugData()
    d.addLine(np.array([0,0,-zwidth/2.0]), np.array([0,0,zwidth/2.0]), radius=radius)


    obj = updatePolyData(d.getPolyData(), 'valve affordance', cls=CylinderAffordanceItem, parent='affordances')
    obj.actor.SetUserTransform(t)
    obj.addToView(app.getDRCView())

    params = dict(axis=zaxis, radius=radius, length=zwidth, origin=origin, xaxis=xaxis, yaxis=yaxis, zaxis=zaxis, xwidth=radius, ywidth=radius, zwidth=zwidth)

    obj.setAffordanceParams(params)
    obj.updateParamsFromActorTransform()




_icpSub = None
_icpCallbacks = []
_icpTransforms = []

def onICPCorrection(messageData):

    messageClass = lcmbotcore.rigid_transform_t
    m = messageClass.decode(messageData.data())
    t = transformUtils.transformFromPose(m.trans, m.quat)

    _icpTransforms.append(t)

    print 'appending icp transform %d' % len(_icpTransforms)

    for func in _icpCallbacks:
        func(t)


def initICPCallback():

    global _icpSub
    lcmThread = lcmUtils.getGlobalLCMThread()
    _icpSub = PythonQt.dd.ddLCMSubscriber('MAP_LOCAL_CORRECTION', lcmThread)
    _icpSub.connect('messageReceived(const QByteArray&)', onICPCorrection)
    lcmThread.addSubscriber(_icpSub)




def applyICP(source, target):

    icp = vtk.vtkIterativeClosestPointTransform()
    icp.SetSource(source)
    icp.SetTarget(target)
    icp.GetLandmarkTransform().SetModeToRigidBody()
    icp.Update()
    t = vtk.vtkTransform()
    t.SetMatrix(icp.GetMatrix())
    return t



def segmentWye(point1, point2):


    inputObj = om.findObjectByName('pointcloud snapshot')
    polyData = inputObj.polyData

    viewPlaneNormal = np.array(getSegmentationView().camera().GetViewPlaneNormal())

    polyData, origin, normal = applyPlaneFit(polyData, expectedNormal=viewPlaneNormal, searchOrigin=point1, searchRadius=0.2, angleEpsilon=0.7, returnOrigin=True)


    wallPoints = thresholdPoints(polyData, 'dist_to_plane', [-0.01, 0.01])
    updatePolyData(wallPoints, 'wall points', parent=getDebugFolder(), visible=False)

    searchRegion = thresholdPoints(polyData, 'dist_to_plane', [0.03, 0.4])
    searchRegion = cropToSphere(searchRegion, point2, 0.30)
    wyePoints = extractLargestCluster(searchRegion)
    updatePolyData(wyePoints, 'wye cluster', parent=getDebugFolder(), visible=False)

    wyeMesh = ioUtils.readPolyData(os.path.join(app.getDRCBase(), 'software/models/otdf/wye.obj'))

    wyeMeshPoint = np.array([0.0, 0.0, 0.005])
    wyeMeshLeftHandle = np.array([0.032292, 0.02949, 0.068485])

    xaxis = -normal
    zaxis = [0,0,1]
    yaxis = np.cross(zaxis, xaxis)

    t = getTransformFromAxes(xaxis, yaxis, zaxis)
    t.PreMultiply()
    t.Translate(-wyeMeshPoint)
    t.PostMultiply()
    t.Translate(point2)

    d = DebugData()
    d.addSphere(point2, radius=0.005)
    updatePolyData(d.getPolyData(), 'wye pick point', parent=getDebugFolder(), visible=False)

    wyeObj = showPolyData(wyeMesh, 'wye', cls=FrameAffordanceItem, color=[0,1,0], visible=True)
    wyeObj.actor.SetUserTransform(t)
    wyeObj.addToView(app.getDRCView())
    showFrame(t, 'wye frame', parent=wyeObj, visible=False)

    params = dict(origin=origin, xaxis=xaxis, yaxis=yaxis, zaxis=zaxis, xwidth=0.1, ywidth=0.1, zwidth=0.1, friendly_name='wye', otdf_type='wye')
    wyeObj.setAffordanceParams(params)
    wyeObj.updateParamsFromActorTransform()


def segmentDrillWall(point1, point2, point3):


    inputObj = om.findObjectByName('pointcloud snapshot')
    polyData = inputObj.polyData


    d = DebugData()

    points = [point1, point2, point3]


    viewPlaneNormal = np.array(getSegmentationView().camera().GetViewPlaneNormal())
    expectedNormal = np.cross(point2 - point1, point3 - point1)
    expectedNormal /= np.linalg.norm(expectedNormal)
    if np.dot(expectedNormal, viewPlaneNormal) < 0:
        expectedNormal *= -1.0

    polyData, origin, normal = applyPlaneFit(polyData, expectedNormal=expectedNormal, searchOrigin=(point1 + point2 + point3)/3.0, searchRadius=0.3, angleEpsilon=0.3, returnOrigin=True)

    points = [projectPointToPlane(point, origin, normal) for point in points]

    # draw points
    for p in points:
        d.addSphere(p, radius=0.01)

    for a, b in zip(points, points[1:] + [points[0]]):
        d.addLine(a, b)

    obj = updatePolyData(d.getPolyData(), 'drill wall points', parent=getDebugFolder(), color=[0,1,0])

    xaxis = -normal
    zaxis = [0, 0, 1]
    yaxis = np.cross(zaxis, xaxis)

    t = getTransformFromAxes(xaxis, yaxis, zaxis)
    t.PostMultiply()
    t.Translate(point1)

    d = DebugData()
    pointsInWallFrame = []
    for p in points:
        pp = np.zeros(3)
        t.GetLinearInverse().TransformPoint(p, pp)
        pointsInWallFrame.append(pp)
        print pp
        d.addSphere(pp, radius=0.01)

    points = pointsInWallFrame

    for a, b in zip(points, points[1:] + [points[0]]):
        d.addLine(a, b)

    aff = showPolyData(d.getPolyData(), 'drill targets', cls=FrameAffordanceItem, color=[0,1,0], visible=True)
    aff.actor.SetUserTransform(t)
    showFrame(t, 'wall frame', parent=aff, visible=False)

    params = dict(origin=origin, xaxis=xaxis, yaxis=yaxis, zaxis=zaxis, xwidth=0.1, ywidth=0.1, zwidth=0.1,
                  p1y=points[0][1], p1z=points[0][2],
                  p2y=points[1][1], p2z=points[1][2],
                  p3y=points[2][1], p3z=points[2][2],
                  friendly_name='drill_wall', otdf_type='drill_wall')

    aff.setAffordanceParams(params)
    aff.updateParamsFromActorTransform()


def segmentDrill(point1, point2, point3):


    inputObj = om.findObjectByName('pointcloud snapshot')
    polyData = inputObj.polyData

    viewPlaneNormal = np.array(getSegmentationView().camera().GetViewPlaneNormal())

    polyData, origin, normal = applyPlaneFit(polyData, expectedNormal=viewPlaneNormal, searchOrigin=point1, searchRadius=0.2, angleEpsilon=0.7, returnOrigin=True)


    tablePoints = thresholdPoints(polyData, 'dist_to_plane', [-0.01, 0.01])
    updatePolyData(tablePoints, 'table plane points', parent=getDebugFolder(), visible=False)


    searchRegion = thresholdPoints(polyData, 'dist_to_plane', [0.03, 0.4])
    searchRegion = cropToSphere(searchRegion, point2, 0.30)
    drillPoints = extractLargestCluster(searchRegion)


    zaxis = normal
    yaxis = point3 - point2
    yaxis /= np.linalg.norm(yaxis)
    xaxis = np.cross(yaxis, zaxis)
    xaxis /= np.linalg.norm(xaxis)
    yaxis = np.cross(zaxis, xaxis)

    t = getTransformFromAxes(xaxis, yaxis, zaxis)
    t.PostMultiply()

    drillToTopPoint = np.array([-0.002904, -0.010029, 0.153182])
    drillToButton = np.array([0.034091, 0.007616, -0.060168])
    t.Translate(point2 - drillToTopPoint)

    drillMesh = ioUtils.readPolyData(os.path.join(app.getDRCBase(), 'software/models/otdf/dewalt_button.obj'))

    aff = showPolyData(drillMesh, 'drill', cls=FrameAffordanceItem, visible=True)
    aff.actor.SetUserTransform(t)
    showFrame(t, 'drill frame', parent=aff, visible=False)

    params = dict(origin=origin, xaxis=xaxis, yaxis=yaxis, zaxis=zaxis, xwidth=0.1, ywidth=0.1, zwidth=0.1, friendly_name='dewalt_button', otdf_type='dewalt_button')
    aff.setAffordanceParams(params)
    aff.updateParamsFromActorTransform()
    aff.addToView(app.getDRCView())


def segmentDrillAuto(point1):


    inputObj = om.findObjectByName('pointcloud snapshot')
    polyData = inputObj.polyData

    expectedNormal = np.array([0.0, 0.0, 1.0])

    polyData, origin, normal = applyPlaneFit(polyData, expectedNormal=expectedNormal, perpendicularAxis=expectedNormal, searchOrigin=point1, searchRadius=0.4, angleEpsilon=0.2, returnOrigin=True)


    tablePoints = thresholdPoints(polyData, 'dist_to_plane', [-0.01, 0.01])
    updatePolyData(tablePoints, 'table plane points', parent=getDebugFolder(), visible=False)

    tablePoints = labelDistanceToPoint(tablePoints, point1)
    tablePointsClusters = extractClusters(tablePoints)
    tablePointsClusters.sort(key=lambda x: vtkNumpy.getNumpyFromVtk(x, 'distance_to_point').min())

    tablePoints = tablePointsClusters[0]
    updatePolyData(tablePoints, 'table points', parent=getDebugFolder(), visible=False)

    searchRegion = thresholdPoints(polyData, 'dist_to_plane', [0.03, 0.4])
    searchRegion = cropToSphere(searchRegion, point1, 0.30)
    drillPoints = extractLargestCluster(searchRegion)


    # determine drill orientation (rotation about z axis)

    centroids = computeCentroids(drillPoints, axis=normal)

    centroidsPolyData = vtkNumpy.getVtkPolyDataFromNumpyPoints(centroids)
    d = DebugData()
    updatePolyData(centroidsPolyData, 'cluster centroids', parent=getDebugFolder(), visible=False)


    '''
    f = pcl.vtkAnnotateOBBs()
    f.SetInputArrayToProcess(0,0,0, vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS, 'cluster_labels')
    f.SetInput(drillPoints)
    f.Update()

    nBoxes = f.GetNumberOfBoundingBoxes()
    assert nBoxes == 1

    edges = np.array((3,3))
    for i, edge in enumerate(edges):
        f.GetBoundingBoxEdge(0, i, edge)

    sort(edges, key=lambda x: np.linalg.norm(x))
    '''

    zaxis = normal
    yaxis = centroids[0] - centroids[-1]
    yaxis /= np.linalg.norm(yaxis)
    xaxis = np.cross(yaxis, zaxis)
    xaxis /= np.linalg.norm(xaxis)
    yaxis = np.cross(zaxis, xaxis)

    t = getTransformFromAxes(xaxis, yaxis, zaxis)
    t.PostMultiply()

    drillToTopPoint = np.array([-0.002904, -0.010029, 0.153182])
    drillToButton = np.array([0.034091, 0.007616, -0.060168])
    t.Translate(centroids[-1] - drillToTopPoint)

    drillMesh = ioUtils.readPolyData(os.path.join(app.getDRCBase(), 'software/models/otdf/dewalt_button.obj'))

    aff = showPolyData(drillMesh, 'drill', cls=FrameAffordanceItem, visible=True)
    aff.actor.SetUserTransform(t)
    showFrame(t, 'drill frame', parent=aff, visible=False)

    params = dict(origin=origin, xaxis=xaxis, yaxis=yaxis, zaxis=zaxis, xwidth=0.1, ywidth=0.1, zwidth=0.1, friendly_name='dewalt_button', otdf_type='dewalt_button')
    aff.setAffordanceParams(params)
    aff.updateParamsFromActorTransform()
    aff.addToView(app.getDRCView())



def segmentDrillInHand(p1, p2):

    inputObj = om.findObjectByName('pointcloud snapshot')
    polyData = inputObj.polyData

    distanceToLineThreshold = 0.05

    polyData = labelDistanceToLine(polyData, p1, p2)
    polyData = thresholdPoints(polyData, 'distance_to_line', [0.0, distanceToLineThreshold])

    lineSegment = p2 - p1
    lineLength = np.linalg.norm(lineSegment)

    cropped, polyData = cropToPlane(polyData, p1, lineSegment/lineLength, [-0.03, lineLength + 0.03])

    updatePolyData(cropped, 'drill cluster', parent=getDebugFolder(), visible=False)


    drillPoints = cropped
    normal = lineSegment/lineLength

    centroids = computeCentroids(drillPoints, axis=normal)

    centroidsPolyData = vtkNumpy.getVtkPolyDataFromNumpyPoints(centroids)
    d = DebugData()
    updatePolyData(centroidsPolyData, 'cluster centroids', parent=getDebugFolder(), visible=False)


    zaxis = normal
    yaxis = centroids[0] - centroids[-1]
    yaxis /= np.linalg.norm(yaxis)
    xaxis = np.cross(yaxis, zaxis)
    xaxis /= np.linalg.norm(xaxis)
    yaxis = np.cross(zaxis, xaxis)

    t = getTransformFromAxes(xaxis, yaxis, zaxis)

    drillToTopPoint = np.array([-0.002904, -0.010029, 0.153182])
    drillToButton = np.array([0.034091, 0.007616, -0.060168])

    t.PreMultiply()
    t.Translate(-drillToTopPoint)
    t.PostMultiply()
    t.Translate(p2)

    drillMesh = ioUtils.readPolyData(os.path.join(app.getDRCBase(), 'software/models/otdf/dewalt_button.obj'))

    aff = showPolyData(drillMesh, 'drill', cls=FrameAffordanceItem, visible=True)
    aff.actor.SetUserTransform(t)
    showFrame(t, 'drill frame', parent=aff, visible=False)

    params = dict(origin=t.GetPosition(), xaxis=xaxis, yaxis=yaxis, zaxis=zaxis, xwidth=0.1, ywidth=0.1, zwidth=0.1, friendly_name='dewalt_button', otdf_type='dewalt_button')
    aff.setAffordanceParams(params)
    aff.updateParamsFromActorTransform()
    aff.addToView(app.getDRCView())


def pickDataSet(displayPoint, tolerance=0.01):

    view = app.getCurrentRenderView()
    assert view

    picker = vtk.vtkPointPicker()
    picker.SetTolerance(tolerance)
    picker.Pick(displayPoint[0], displayPoint[1], 0, view.renderer())
    pickPoints = picker.GetPickedPositions()
    return picker.GetDataSet(), np.array(pickPoints.GetPoint(0)) if pickPoints.GetNumberOfPoints() else None


def pickPoint(objName, displayPoint, tolerance=0.01):

    view = app.getCurrentRenderView()
    assert view

    picker = vtk.vtkPointPicker()

    if objName:
        obj = om.findObjectByName(objName)
        assert obj
        picker.AddPickList(obj.actor)
        picker.PickFromListOn()

    picker.SetTolerance(tolerance)
    picker.Pick(displayPoint[0], displayPoint[1], 0, view.renderer())
    pickPoints = picker.GetPickedPositions()
    return np.array(pickPoints.GetPoint(0)) if pickPoints.GetNumberOfPoints() else None


def mapMousePosition(widget, mouseEvent):
    mousePosition = mouseEvent.pos()
    return mousePosition.x(), widget.height - mousePosition.y()


class PointPicker(TimerCallback):

    def __init__(self, numberOfPoints=3):
        TimerCallback.__init__(self)
        self.targetFps = 30
        self.enabled = False
        self.numberOfPoints = numberOfPoints
        self.annotationObj = None
        self.drawLines = True
        self.clear()

    def clear(self):
        self.points = [None for i in xrange(self.numberOfPoints)]
        self.hoverPos = None
        self.annotationFunc = None
        self.lastMovePos = [0, 0]

    def onMouseMove(self, displayPoint, modifiers=None):
        self.lastMovePos = displayPoint

    def onMousePress(self, displayPoint, modifiers=None):

        #print 'mouse press:', modifiers
        #if not modifiers:
        #    return

        for i in xrange(self.numberOfPoints):
            if self.points[i] is None:
                self.points[i] = self.hoverPos
                break

        if self.points[-1] is not None:
            self.finish()

    def finish(self):

        self.enabled = False
        om.removeFromObjectModel(self.annotationObj)

        points = [p.copy() for p in self.points]
        if self.annotationFunc is not None:
            self.annotationFunc(*points)

        removeViewPicker(self)

    def handleRelease(self, displayPoint):
        pass

    def draw(self):

        d = DebugData()

        points = [p if p is not None else self.hoverPos for p in self.points]

        # draw points
        for p in points:
            if p is not None:
                d.addSphere(p, radius=0.01)

        if self.drawLines:
            # draw lines
            for a, b in zip(points, points[1:]):
                if b is not None:
                    d.addLine(a, b)

            # connect end points
            if points[-1] is not None:
                d.addLine(points[0], points[-1])


        self.annotationObj = updatePolyData(d.getPolyData(), 'annotation', parent=getDebugFolder())
        self.annotationObj.setProperty('Color', QtGui.QColor(0, 255, 0))


    def tick(self):

        if not self.enabled:
            return

        self.hoverPos = pickPoint('pointcloud snapshot', self.lastMovePos)
        self.draw()


class LineDraw(TimerCallback):

    def __init__(self, view):
        TimerCallback.__init__(self)
        self.targetFps = 30
        self.enabled = False
        self.view = view
        self.renderer = view.renderer()
        self.line = vtk.vtkLeaderActor2D()
        self.line.SetArrowPlacementToNone()
        self.line.GetPositionCoordinate().SetCoordinateSystemToViewport()
        self.line.GetPosition2Coordinate().SetCoordinateSystemToViewport()
        self.line.GetProperty().SetLineWidth(2)
        self.line.SetPosition(0,0)
        self.line.SetPosition2(0,0)
        self.clear()

    def clear(self):
        self.p1 = None
        self.p2 = None
        self.annotationFunc = None
        self.lastMovePos = [0, 0]
        self.renderer.RemoveActor2D(self.line)

    def onMouseMove(self, displayPoint, modifiers=None):
        self.lastMovePos = displayPoint

    def onMousePress(self, displayPoint, modifiers=None):

        if self.p1 is None:
            self.p1 = list(self.lastMovePos)
            if self.p1 is not None:
                self.renderer.AddActor2D(self.line)
        else:
            self.p2 = self.lastMovePos
            self.finish()

    def finish(self):

        self.enabled = False
        self.renderer.RemoveActor2D(self.line)
        if self.annotationFunc is not None:
            self.annotationFunc(self.p1, self.p2)


    def handleRelease(self, displayPoint):
        pass

    def tick(self):

        if not self.enabled:
            return

        if self.p1:
            self.line.SetPosition(self.p1)
            self.line.SetPosition2(self.lastMovePos)
            self.view.render()

viewPickers = []

def addViewPicker(picker):
    global viewPickers
    viewPickers.append(picker)

def removeViewPicker(picker):
    global viewPickers
    viewPickers.remove(picker)


def distanceToLine(x0, x1, x2):
    numerator = np.sqrt(np.sum(np.cross((x0 - x1), (x0-x2))**2))
    denom = np.linalg.norm(x2-x1)
    return numerator / denom


def labelDistanceToLine(polyData, linePoint1, linePoint2, resultArrayName='distance_to_line'):

    x0 = vtkNumpy.getNumpyFromVtk(polyData, 'Points')
    x1 = linePoint1
    x2 = linePoint2

    numerator = np.sqrt(np.sum(np.cross((x0 - x1), (x0-x2))**2, axis=1))
    denom = np.linalg.norm(x2-x1)

    dists = numerator / denom

    polyData = shallowCopy(polyData)
    vtkNumpy.addNumpyToVtk(polyData, dists, resultArrayName)
    return polyData


def labelDistanceToPoint(polyData, point, resultArrayName='distance_to_point'):
    points = vtkNumpy.getNumpyFromVtk(polyData, 'Points')
    points = points - point
    dists = np.sqrt(np.sum(points**2, axis=1))
    polyData = shallowCopy(polyData)
    vtkNumpy.addNumpyToVtk(polyData, dists, resultArrayName)
    return polyData


def getRayFromDisplayPoint(view, displayPoint):

    worldPt1 = [0,0,0,0]
    worldPt2 = [0,0,0,0]
    renderer = view.renderer()

    vtk.vtkInteractorObserver.ComputeDisplayToWorld(renderer, displayPoint[0], displayPoint[1], 0, worldPt1)
    vtk.vtkInteractorObserver.ComputeDisplayToWorld(renderer, displayPoint[0], displayPoint[1], 1, worldPt2)

    worldPt1 = np.array(worldPt1[:3])
    worldPt2 = np.array(worldPt2[:3])
    return worldPt1, worldPt2


def getPlaneEquationFromPolyData(polyData, expectedNormal):

    _, origin, normal  = applyPlaneFit(polyData, expectedNormal=expectedNormal, returnOrigin=True)
    return origin, normal, np.hstack((normal, [np.dot(origin, normal)]))




def computeEdge(polyData, edgeAxis, perpAxis, binWidth=0.03):

    polyData = pointCloudUtils.labelPointDistanceAlongAxis(polyData, edgeAxis, resultArrayName='dist_along_edge')
    polyData = pointCloudUtils.labelPointDistanceAlongAxis(polyData, perpAxis, resultArrayName='dist_perp_to_edge')


    polyData, bins = binByScalar(polyData, 'dist_along_edge', binWidth)
    points = vtkNumpy.getNumpyFromVtk(polyData, 'Points')
    binLabels = vtkNumpy.getNumpyFromVtk(polyData, 'bin_labels')
    distToEdge = vtkNumpy.getNumpyFromVtk(polyData, 'dist_perp_to_edge')

    numberOfBins = len(bins) - 1
    edgePoints = []
    for i in xrange(numberOfBins):
        binPoints = points[binLabels == i]
        binDists = distToEdge[binLabels == i]
        if len(binDists):
            edgePoints.append(binPoints[binDists.argmax()])

    return np.array(edgePoints)


def computeCentroids(polyData, axis, binWidth=0.025):

    polyData = pointCloudUtils.labelPointDistanceAlongAxis(polyData, axis, resultArrayName='dist_along_axis')

    polyData, bins = binByScalar(polyData, 'dist_along_axis', binWidth)
    points = vtkNumpy.getNumpyFromVtk(polyData, 'Points')
    binLabels = vtkNumpy.getNumpyFromVtk(polyData, 'bin_labels')

    numberOfBins = len(bins) - 1
    centroids = []
    for i in xrange(numberOfBins):
        binPoints = points[binLabels == i]

        if len(binPoints):
            centroids.append(np.average(binPoints, axis=0))

    return np.array(centroids)



def binByScalar(lidarData, scalarArrayName, binWidth, binLabelsArrayName='bin_labels'):
    '''
    Gets the array with name scalarArrayName from lidarData.
    Computes bins by dividing the scalar array into bins of size binWidth.
    Adds a new label array to the lidar points identifying which bin the point belongs to,
    where the first bin is labeled with 0.
    Returns the new, labeled lidar data and the bins.
    The bins are an array where each value represents a bin edge.
    '''

    scalars = vtkNumpy.getNumpyFromVtk(lidarData, scalarArrayName)
    bins = np.arange(scalars.min(), scalars.max()+binWidth, binWidth)
    binLabels = np.digitize(scalars, bins) - 1
    assert(len(binLabels) == len(scalars))
    newData = shallowCopy(lidarData)
    vtkNumpy.addNumpyToVtk(newData, binLabels, binLabelsArrayName)
    return newData, bins


def showObbs(polyData):

    f = pcl.vtkAnnotateOBBs()
    f.SetInputArrayToProcess(0,0,0, vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS, 'cluster_labels')
    f.SetInput(polyData)
    f.Update()


def segmentBlockByAnnotation(blockDimensions, p1, p2, p3):

    segmentationObj = om.findObjectByName('pointcloud snapshot')
    segmentationObj.mapper.ScalarVisibilityOff()
    segmentationObj.setProperty('Point Size', 2)
    segmentationObj.setProperty('Alpha', 0.8)

    # constraint z to lie in plane
    #p1[2] = p2[2] = p3[2] = max(p1[2], p2[2], p3[2])

    zedge = p2 - p1
    zaxis = zedge / np.linalg.norm(zedge)

    #xwidth = distanceToLine(p3, p1, p2)

    # expected dimensions
    xwidth, ywidth = blockDimensions

    zwidth = np.linalg.norm(zedge)

    yaxis = np.cross(p2 - p1, p3 - p1)
    yaxis = yaxis / np.linalg.norm(yaxis)

    xaxis = np.cross(yaxis, zaxis)

    # reorient axes
    if np.dot(yaxis, _spindleAxis) < 0:
        yaxis *= -1

    if np.dot(xaxis, p3 - p1) < 0:
        xaxis *= -1

    # make right handed
    zaxis = np.cross(xaxis, yaxis)

    origin = ((p1 + p2) / 2.0) + xaxis*xwidth/2.0 + yaxis*ywidth/2.0

    d = DebugData()
    d.addSphere(origin, radius=0.01)
    d.addLine(origin - xaxis*xwidth/2.0, origin + xaxis*xwidth/2.0)
    d.addLine(origin - yaxis*ywidth/2.0, origin + yaxis*ywidth/2.0)
    d.addLine(origin - zaxis*zwidth/2.0, origin + zaxis*zwidth/2.0)
    obj = updatePolyData(d.getPolyData(), 'block axes')
    obj.setProperty('Color', QtGui.QColor(255, 255, 0))
    obj.setProperty('Visible', False)
    om.findObjectByName('annotation').setProperty('Visible', False)

    cube = vtk.vtkCubeSource()
    cube.SetXLength(xwidth)
    cube.SetYLength(ywidth)
    cube.SetZLength(zwidth)
    cube.Update()
    cube = shallowCopy(cube.GetOutput())

    t = getTransformFromAxes(xaxis, yaxis, zaxis)
    t.PostMultiply()
    t.Translate(origin)

    obj = updatePolyData(cube, 'block affordance', cls=BlockAffordanceItem, parent='affordances')
    obj.actor.SetUserTransform(t)

    obj.addToView(app.getDRCView())

    params = dict(origin=origin, xwidth=xwidth, ywidth=ywidth, zwidth=zwidth, xaxis=xaxis, yaxis=yaxis, zaxis=zaxis)
    obj.setAffordanceParams(params)
    obj.updateParamsFromActorTransform()


def projectPointToPlane(point, origin, normal):
    projectedPoint = np.zeros(3)
    vtk.vtkPlane.ProjectPoint(point, origin, normal, projectedPoint)
    return projectedPoint


def segmentBlockByTopPlane(polyData, blockDimensions, expectedNormal=None, expectedXAxis=None, edgeSign=1, name='block affordance'):

    if expectedNormal is None:
        expectedNormal = _spindleAxis

    if expectedXAxis is None:
        expectedXAxis = _spindleAxis

    polyData, planeOrigin, normal  = applyPlaneFit(polyData, distanceThreshold=0.05, expectedNormal=expectedNormal, returnOrigin=True)

    _, lineDirection, _ = applyLineFit(polyData)

    zaxis = lineDirection
    yaxis = normal
    xaxis = np.cross(yaxis, zaxis)

    if np.dot(xaxis, expectedXAxis) < 0:
        xaxis *= -1

    # make right handed
    zaxis = np.cross(xaxis, yaxis)


    edgePoints = computeEdge(polyData, zaxis, xaxis*edgeSign)
    edgePoints = vtkNumpy.getVtkPolyDataFromNumpyPoints(edgePoints)

    d = DebugData()
    obj = updatePolyData(edgePoints, 'edge points', parent=getDebugFolder(), visible=False)

    linePoint, lineDirection, _ = applyLineFit(edgePoints)
    zaxis = lineDirection
    xaxis = np.cross(yaxis, zaxis)


    if np.dot(xaxis, expectedXAxis) < 0:
        xaxis *= -1

    # make right handed
    zaxis = np.cross(xaxis, yaxis)

    polyData = pointCloudUtils.labelPointDistanceAlongAxis(polyData, xaxis, resultArrayName='dist_along_line')
    pts = vtkNumpy.getNumpyFromVtk(polyData, 'Points')

    dists = np.dot(pts-linePoint, zaxis)

    p1 = linePoint + zaxis*np.min(dists)
    p2 = linePoint + zaxis*np.max(dists)

    p1 = projectPointToPlane(p1, planeOrigin, normal)
    p2 = projectPointToPlane(p2, planeOrigin, normal)

    xwidth, ywidth = blockDimensions
    zwidth = np.linalg.norm(p2 - p1)

    origin = p1 - edgeSign*xaxis*xwidth/2.0 + yaxis*ywidth/2.0 + zaxis*zwidth/2.0 

    d = DebugData()

    #d.addSphere(linePoint, radius=0.02)
    #d.addLine(linePoint, linePoint + yaxis*ywidth)
    #d.addLine(linePoint, linePoint + xaxis*xwidth)
    #d.addLine(linePoint, linePoint + zaxis*zwidth)


    d.addSphere(p1, radius=0.01)
    d.addSphere(p2, radius=0.01)
    d.addLine(p1, p2)

    d.addSphere(origin, radius=0.01)
    #d.addLine(origin - xaxis*xwidth/2.0, origin + xaxis*xwidth/2.0)
    #d.addLine(origin - yaxis*ywidth/2.0, origin + yaxis*ywidth/2.0)
    #d.addLine(origin - zaxis*zwidth/2.0, origin + zaxis*zwidth/2.0)

    d.addLine(origin, origin + xaxis*xwidth/2.0)
    d.addLine(origin, origin + yaxis*ywidth/2.0)
    d.addLine(origin, origin + zaxis*zwidth/2.0)


    #obj = updatePolyData(d.getPolyData(), 'block axes')
    #obj.setProperty('Color', QtGui.QColor(255, 255, 0))
    #obj.setProperty('Visible', False)

    cube = vtk.vtkCubeSource()
    cube.SetXLength(xwidth)
    cube.SetYLength(ywidth)
    cube.SetZLength(zwidth)
    cube.Update()
    cube = shallowCopy(cube.GetOutput())

    t = getTransformFromAxes(xaxis, yaxis, zaxis)
    t.PostMultiply()
    t.Translate(origin)

    obj = showPolyData(cube, name, cls=BlockAffordanceItem, parent='affordances')
    obj.actor.SetUserTransform(t)
    obj.addToView(app.getDRCView())

    params = dict(origin=origin, xwidth=xwidth, ywidth=ywidth, zwidth=zwidth, xaxis=xaxis, yaxis=yaxis, zaxis=zaxis, friendly_name=name)
    obj.setAffordanceParams(params)
    obj.updateParamsFromActorTransform()

    if len(_icpTransforms):
        objTrack = showPolyData(cube, name, cls=BlockAffordanceItem, parent=obj, color=[0.8, 1, 0.8])
        objTrack.actor.SetUserTransform(t)
        objTrack.baseTransform = vtk.vtkTransform()
        objTrack.baseTransform.SetMatrix(t.GetMatrix())
        objTrack.icpTransformInitial = _icpTransforms[-1]
        objTrack.addToView(app.getDRCView())

        print 'setting base transform:', objTrack.baseTransform.GetPosition()
        print 'setting initial icp:', objTrack.icpTransformInitial.GetPosition()

        _icpCallbacks.append(objTrack.updateICPTransform)


    frameObj = showFrame(obj.actor.GetUserTransform(), name + ' frame', parent=obj, visible=False)


def segmentBlockByPlanes(blockDimensions):

    planes = om.getObjectChildren(om.findObjectByName('selected planes'))[:2]

    origin1, normal1, plane1 = getPlaneEquationFromPolyData(planes[0].polyData, expectedNormal=_spindleAxis)
    origin2, normal2, plane2 = getPlaneEquationFromPolyData(planes[1].polyData, expectedNormal=_spindleAxis)

    xaxis = normal2
    yaxis = normal1
    zaxis = np.cross(xaxis, yaxis)
    xaxis = np.cross(yaxis, zaxis)

    pts1 = vtkNumpy.getNumpyFromVtk(planes[0].polyData, 'Points')
    pts2 = vtkNumpy.getNumpyFromVtk(planes[1].polyData, 'Points')

    linePoint = np.zeros(3)
    centroid2 = np.sum(pts2, axis=0)/len(pts2)
    vtk.vtkPlane.ProjectPoint(centroid2, origin1, normal1, linePoint)

    dists = np.dot(pts1-linePoint, zaxis)

    p1 = linePoint + zaxis*np.min(dists)
    p2 = linePoint + zaxis*np.max(dists)

    xwidth, ywidth = blockDimensions
    zwidth = np.linalg.norm(p2 - p1)

    origin = p1 + xaxis*xwidth/2.0 + yaxis*ywidth/2.0 + zaxis*zwidth/2.0 


    '''

    # reorient axes
    if np.dot(yaxis, _spindleAxis) < 0:
        yaxis *= -1

    if np.dot(xaxis, p3 - p1) < 0:
        xaxis *= -1

    # make right handed
    zaxis = np.cross(xaxis, yaxis)

    '''

    d = DebugData()

    d.addSphere(linePoint, radius=0.02)
    d.addSphere(p1, radius=0.01)
    d.addSphere(p2, radius=0.01)
    d.addLine(p1, p2)

    d.addSphere(origin, radius=0.01)
    d.addLine(origin - xaxis*xwidth/2.0, origin + xaxis*xwidth/2.0)
    d.addLine(origin - yaxis*ywidth/2.0, origin + yaxis*ywidth/2.0)
    d.addLine(origin - zaxis*zwidth/2.0, origin + zaxis*zwidth/2.0)
    obj = updatePolyData(d.getPolyData(), 'block axes')
    obj.setProperty('Color', QtGui.QColor(255, 255, 0))
    obj.setProperty('Visible', False)

    cube = vtk.vtkCubeSource()
    cube.SetXLength(xwidth)
    cube.SetYLength(ywidth)
    cube.SetZLength(zwidth)
    cube.Update()
    cube = shallowCopy(cube.GetOutput())

    t = getTransformFromAxes(xaxis, yaxis, zaxis)
    t.PostMultiply()
    t.Translate(origin)

    obj = updatePolyData(cube, 'block affordance', cls=BlockAffordanceItem, parent='affordances')
    obj.actor.SetUserTransform(t)
    obj.addToView(app.getDRCView())

    params = dict(origin=origin, xwidth=xwidth, ywidth=ywidth, zwidth=zwidth, xaxis=xaxis, yaxis=yaxis, zaxis=zaxis)
    obj.setAffordanceParams(params)
    obj.updateParamsFromActorTransform()


def startBlockSegmentation(dimensions):

    if om.findObjectByName('selected planes'):
        polyData = om.getObjectChildren(om.findObjectByName('selected planes'))[0].polyData
        segmentBlockByTopPlane(polyData, dimensions)
        return

    picker = PointPicker()
    addViewPicker(picker)
    picker.enabled = True
    picker.start()
    picker.annotationFunc = functools.partial(segmentBlockByAnnotation, dimensions)
    om.removeFromObjectModel(om.findObjectByName('block affordance'))
    om.removeFromObjectModel(om.findObjectByName('block axes'))
    om.removeFromObjectModel(om.findObjectByName('annotation'))


def startBoundedPlaneSegmentation():

    picker = PointPicker(numberOfPoints=2)
    addViewPicker(picker)
    picker.enabled = True
    picker.start()
    picker.annotationFunc = functools.partial(segmentBoundedPlaneByAnnotation)


def startValveSegmentationByWallPlane(expectedValveRadius):

    picker = PointPicker(numberOfPoints=2)
    addViewPicker(picker)
    picker.enabled = True
    picker.start()
    picker.annotationFunc = functools.partial(segmentValveByWallPlane, expectedValveRadius)


def startWyeSegmentation():

    picker = PointPicker(numberOfPoints=2)
    addViewPicker(picker)
    picker.enabled = True
    picker.drawLines = False
    picker.start()
    picker.annotationFunc = functools.partial(segmentWye)


def startDrillSegmentation():

    picker = PointPicker(numberOfPoints=3)
    addViewPicker(picker)
    picker.enabled = True
    picker.drawLines = False
    picker.start()
    picker.annotationFunc = functools.partial(segmentDrill)


def startDrillAutoSegmentation():

    picker = PointPicker(numberOfPoints=1)
    addViewPicker(picker)
    picker.enabled = True
    picker.drawLines = False
    picker.start()
    picker.annotationFunc = functools.partial(segmentDrillAuto)


def startDrillWallSegmentation():

    picker = PointPicker(numberOfPoints=3)
    addViewPicker(picker)
    picker.enabled = True
    picker.drawLines = True
    picker.start()
    picker.annotationFunc = functools.partial(segmentDrillWall)

def startDrillInHandSegmentation():

    picker = PointPicker(numberOfPoints=2)
    addViewPicker(picker)
    picker.enabled = True
    picker.drawLines = True
    picker.start()
    picker.annotationFunc = functools.partial(segmentDrillInHand)


def segmentBoundedPlaneByAnnotation(point1, point2):

    inputObj = om.findObjectByName('pointcloud snapshot')
    polyData = shallowCopy(inputObj.polyData)


    viewPlaneNormal = np.array(getSegmentationView().camera().GetViewPlaneNormal())

    polyData, origin, normal = applyPlaneFit(polyData, distanceThreshold=0.015, expectedNormal=viewPlaneNormal, perpendicularAxis=viewPlaneNormal,
                                             searchOrigin=point1, searchRadius=0.3, angleEpsilon=0.7, returnOrigin=True)


    planePoints = thresholdPoints(polyData, 'dist_to_plane', [-0.015, 0.015])
    updatePolyData(planePoints, 'unbounded plane points', parent=getDebugFolder(), visible=False)


    planePoints = applyVoxelGrid(planePoints, leafSize=0.03)
    planePoints = labelOutliers(planePoints, searchRadius=0.06, neighborsInSearchRadius=12)

    updatePolyData(planePoints, 'voxel plane points', parent=getDebugFolder(), colorByName='is_outlier', visible=False)

    planePoints = thresholdPoints(planePoints, 'is_outlier', [0, 0])

    planePoints = labelDistanceToPoint(planePoints, point1)
    clusters = extractClusters(planePoints, clusterTolerance=0.10)
    clusters.sort(key=lambda x: vtkNumpy.getNumpyFromVtk(x, 'distance_to_point').min())

    planePoints = clusters[0]
    updatePolyData(planePoints, 'plane points', parent=getDebugFolder(), visible=False)


    perpAxis = point2 - point1
    perpAxis /= np.linalg.norm(perpAxis)
    edgeAxis = np.cross(normal, perpAxis)

    edgePoints = computeEdge(planePoints, edgeAxis, perpAxis)
    edgePoints = vtkNumpy.getVtkPolyDataFromNumpyPoints(edgePoints)
    updatePolyData(edgePoints, 'edge points', parent=getDebugFolder(), visible=False)


    linePoint, lineDirection, _ = applyLineFit(edgePoints)

    zaxis = normal
    yaxis = lineDirection
    xaxis = np.cross(yaxis, zaxis)

    if np.dot(xaxis, perpAxis) < 0:
        xaxis *= -1

    # make right handed
    yaxis = np.cross(zaxis, xaxis)

    pts = vtkNumpy.getNumpyFromVtk(planePoints, 'Points')

    dists = np.dot(pts-linePoint, yaxis)

    p1 = linePoint + yaxis*np.min(dists)
    p2 = linePoint + yaxis*np.max(dists)

    p1 = projectPointToPlane(p1, origin, normal)
    p2 = projectPointToPlane(p2, origin, normal)

    d = DebugData()
    d.addSphere(p1, radius=0.01)
    d.addSphere(p2, radius=0.01)
    d.addLine(p1, p2)
    updatePolyData(d.getPolyData(), 'plane edge', parent=getDebugFolder(), visible=False)

    t = getTransformFromAxes(xaxis, yaxis, zaxis)
    t.PostMultiply()
    t.Translate((p1 + p2)/ 2.0)

    updateFrame(t, 'plane edge frame', parent=getDebugFolder(), visible=False)



savedCameraParams = None

def perspective():

    global savedCameraParams
    if savedCameraParams is None:
        return

    aff = getDefaultAffordanceObject()
    if aff:
        aff.setProperty('Alpha', 1.0)

    obj = om.findObjectByName('pointcloud snapshot')
    if obj is not None:
        obj.actor.SetPickable(1)

    view = getSegmentationView()
    c = view.camera()
    c.ParallelProjectionOff()
    c.SetPosition(savedCameraParams['Position'])
    c.SetFocalPoint(savedCameraParams['FocalPoint'])
    c.SetViewUp(savedCameraParams['ViewUp'])
    view.setCameraManipulationStyle()
    view.render()


def saveCameraParams(overwrite=False):

    global savedCameraParams
    if overwrite or (savedCameraParams is None):

        view = getSegmentationView()
        c = view.camera()
        savedCameraParams = dict(Position=c.GetPosition(), FocalPoint=c.GetFocalPoint(), ViewUp=c.GetViewUp())



def getDefaultAffordanceObject():

    obj = om.getActiveObject()
    if isinstance(obj, om.AffordanceItem):
        return obj

    for obj in om.objects.values():
        if isinstance(obj, om.AffordanceItem):
            return obj


def orthoX():

    aff = getDefaultAffordanceObject()
    if not aff:
        return

    saveCameraParams()

    aff.updateParamsFromActorTransform()
    aff.setProperty('Alpha', 0.3)
    om.findObjectByName('pointcloud snapshot').actor.SetPickable(0)

    view = getSegmentationView()
    c = view.camera()
    c.ParallelProjectionOn()

    origin = aff.params['origin']
    viewDirection = aff.params['xaxis']
    viewUp = -aff.params['yaxis']
    viewDistance = aff.params['xwidth']*3
    scale = aff.params['zwidth']

    c.SetFocalPoint(origin)
    c.SetPosition(origin - viewDirection*viewDistance)
    c.SetViewUp(viewUp)
    c.SetParallelScale(scale)

    view.setActorManipulationStyle()
    view.render()


def orthoY():

    aff = getDefaultAffordanceObject()
    if not aff:
        return

    saveCameraParams()

    aff.updateParamsFromActorTransform()
    aff.setProperty('Alpha', 0.3)
    om.findObjectByName('pointcloud snapshot').actor.SetPickable(0)

    view = getSegmentationView()
    c = view.camera()
    c.ParallelProjectionOn()

    origin = aff.params['origin']
    viewDirection = aff.params['yaxis']
    viewUp = -aff.params['xaxis']
    viewDistance = aff.params['ywidth']*4
    scale = aff.params['zwidth']

    c.SetFocalPoint(origin)
    c.SetPosition(origin - viewDirection*viewDistance)
    c.SetViewUp(viewUp)
    c.SetParallelScale(scale)

    view.setActorManipulationStyle()
    view.render()


def orthoZ():

    aff = getDefaultAffordanceObject()
    if not aff:
        return

    saveCameraParams()

    aff.updateParamsFromActorTransform()
    aff.setProperty('Alpha', 0.3)
    om.findObjectByName('pointcloud snapshot').actor.SetPickable(0)

    view = getSegmentationView()
    c = view.camera()
    c.ParallelProjectionOn()

    origin = aff.params['origin']
    viewDirection = aff.params['zaxis']
    viewUp = -aff.params['yaxis']
    viewDistance = aff.params['zwidth']
    scale = aff.params['ywidth']*6

    c.SetFocalPoint(origin)
    c.SetPosition(origin - viewDirection*viewDistance)
    c.SetViewUp(viewUp)
    c.SetParallelScale(scale)

    view.setActorManipulationStyle()
    view.render()


def getObjectByDataSet(polyData):

    for item, obj in om.objects.iteritems():
        if isinstance(obj, om.PolyDataItem) and obj.polyData == polyData:
            return obj


def selectActor(displayPoint):

    polyData, pickedPoint = pickDataSet(displayPoint)
    obj = getObjectByDataSet(polyData)
    if not obj:
        return

    name = obj.getProperty('Name')
    om.getOrCreateContainer('selected planes', om.findObjectByName('segmentation'))
    obj2 = showPolyData(shallowCopy(polyData), name, parent='selected planes')
    obj2.setProperty('Color', obj.getProperty('Color'))
    obj2.setProperty('Point Size', 4)
    obj.setProperty('Visible', False)


def zoomToDisplayPoint(displayPoint, boundsRadius=0.5, view=None):

    pickedPoint = pickPoint('pointcloud snapshot', displayPoint)
    if pickedPoint is None:
        return

    view = view or app.getCurrentRenderView()

    worldPt1, worldPt2 = getRayFromDisplayPoint(getSegmentationView(), displayPoint)
    global _spindleAxis
    _spindleAxis = worldPt2 - worldPt1

    diagonal = np.array([boundsRadius, boundsRadius, boundsRadius])
    bounds = np.hstack([pickedPoint - diagonal, pickedPoint + diagonal])
    bounds = [bounds[0], bounds[3], bounds[1], bounds[4], bounds[2], bounds[5]]
    view.renderer().ResetCamera(bounds)
    view.camera().SetFocalPoint(pickedPoint)
    view.render()


def extractPointsAlongClickRay(displayPoint, distanceToLineThreshold=0.3, addDebugRay=False):

    worldPt1, worldPt2 = getRayFromDisplayPoint(getSegmentationView(), displayPoint)

    if showDebugRay:
        d = DebugData()
        d.addLine(worldPt1, worldPt2)
        showPolyData(d.getPolyData(), 'mouse click ray', visible=False)


    segmentationObj = om.findObjectByName('pointcloud snapshot')
    polyData = segmentationObj.polyData

    polyData = labelDistanceToLine(polyData, worldPt1, worldPt2)

    # extract points near line
    polyData = thresholdPoints(polyData, 'distance_to_line', [0.0, distanceToLineThreshold])
    showPolyData(polyData, 'selected cluster', colorByName='distance_to_line', visible=False)
    return polyData


def onSegmentationViewDoubleClicked(displayPoint):


    action = 'zoom_to'


    if om.findObjectByName('major planes'):
        action = 'select_actor'


    if action == 'zoom_to':

        zoomToDisplayPoint(displayPoint)

    elif action == 'select_with_ray':

        extractPointsAlongClickRay(displayPoint)

    elif action == 'select_actor':
        selectActor(displayPoint)


def cleanup():

    om.removeFromObjectModel(om.findObjectByName('segmentation'))


def segmentationViewEventFilter(obj, event):

    eventFilter = eventFilters[obj]

    if event.type() == QtCore.QEvent.MouseButtonDblClick:
        eventFilter.setEventHandlerResult(True)
        onSegmentationViewDoubleClicked(mapMousePosition(obj, event))

    else:

        for picker in viewPickers:
            if not picker.enabled:
                continue

            if event.type() == QtCore.QEvent.MouseMove:
                picker.onMouseMove(mapMousePosition(obj, event), event.modifiers())
                eventFilter.setEventHandlerResult(True)
            elif event.type() == QtCore.QEvent.MouseButtonPress:
                picker.onMousePress(mapMousePosition(obj, event), event.modifiers())
                eventFilter.setEventHandlerResult(True)



def drcViewEventFilter(obj, event):

    eventFilter = eventFilters[obj]
    if event.type() == QtCore.QEvent.MouseButtonDblClick:
        eventFilter.setEventHandlerResult(True)
        activateSegmentationMode()


def installEventFilter(view, func):

    global eventFilters
    eventFilter = PythonQt.dd.ddPythonEventFilter()

    qvtkwidget = view.vtkWidget()
    qvtkwidget.installEventFilter(eventFilter)
    eventFilters[qvtkwidget] = eventFilter

    eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonDblClick)
    eventFilter.addFilteredEventType(QtCore.QEvent.MouseMove)
    eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonPress)
    eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonRelease)
    eventFilter.connect('handleEvent(QObject*, QEvent*)', func)


def init():

    installEventFilter(app.getViewManager().findView('DRC View'), drcViewEventFilter)

    #activateSegmentationMode(debug=True)

