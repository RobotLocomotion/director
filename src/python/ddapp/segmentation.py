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
from ddapp.transformUtils import getTransformFromAxes
from ddapp.timercallback import TimerCallback

import numpy as np
import vtkNumpy
from debugVis import DebugData
from shallowCopy import shallowCopy
import affordance
import ioUtils
import pointCloudUtils

import vtkPCLFiltersPython as pcl

import drc as lcmdrc


eventFilters = {}

_spindleAxis = None


class BlockAffordanceItem(om.AffordanceItem):

    def setAffordanceParams(self, params):
        self.params = params

    def updateParamsFromActorTransform(self):

        t = self.actor.GetUserTransform()

        xaxis = np.array(t.TransformVector([1,0,0]))
        yaxis = np.array(t.TransformVector([0,1,0]))
        zaxis = np.array(t.TransformVector([0,0,1]))
        self.params['xaxis'] = xaxis
        self.params['yaxis'] = yaxis
        self.params['zaxis'] = zaxis
        self.params['origin'] = t.GetPosition()


    def publish(self):
        self.updateParamsFromActorTransform()
        aff = affordance.createBoxAffordance(self.params)
        #aff.otdf_type = 'cinderblockstep'
        #aff.nparams = 0
        #aff.params = []
        #aff.param_names = []
        affordance.publishAffordance(aff)


class CylinderAffordanceItem(om.AffordanceItem):

    def setAffordanceParams(self, params):
        self.params = params

    def updateParamsFromActorTransform(self):

        t = self.actor.GetUserTransform()

        xaxis = np.array(t.TransformVector([1,0,0]))
        yaxis = np.array(t.TransformVector([0,1,0]))
        zaxis = np.array(t.TransformVector([0,0,1]))
        self.params['axis'] = zaxis
        self.params['origin'] = t.GetPosition()


    def publish(self):
        self.updateParamsFromActorTransform()
        aff = affordance.createValveAffordance(self.params)
        affordance.publishAffordance(aff)


class FrameItem(om.PolyDataItem):

    def __init__(self, name, transform, view):

        scale = 1.0
        self.transform = transform
        polyData = self._createAxes(scale)

        om.PolyDataItem.__init__(self, name, polyData, view)

        self.colorBy('Axes')
        lut = self.mapper.GetLookupTable()
        lut.SetHueRange(0, 0.667)

        self.actor.SetUserTransform(transform)

        self.widget = perception.drc.vtkFrameWidget()
        self.widget.CreateDefaultRepresentation()
        self.widget.EnabledOff()
        self.rep = self.widget.GetRepresentation()
        self.rep.SetWorldSize(scale)
        self.rep.SetTransform(transform)

        self.addProperty('Scale', scale)
        self.addProperty('Edit', False)


    def _createAxes(self, scale):
        axes = vtk.vtkAxes()
        axes.SetComputeNormals(0)
        axes.SetScaleFactor(scale)
        axes.Update()

        #t = vtk.vtkTransformPolyDataFilter()
        #t.SetTransform(transform)
        #t.AddInputConnection(self.axes.GetOutputPort())
        #t.Update()

        return shallowCopy(axes.GetOutput())

    def addToView(self, view):
        om.PolyDataItem.addToView(self, view)

    def _onPropertyChanged(self, propertyName):

        if propertyName == 'Scale':
            scale = self.getProperty(propertyName)
            self.rep.SetWorldSize(scale)
            self.setPolyData(self._createAxes(scale))
        elif propertyName == 'Edit':
            view = app.getCurrentRenderView()
            if view not in self.views:
                view = self.views[0]
            self.widget.SetInteractor(view.renderWindow().GetInteractor())
            self.widget.SetEnabled(self.getProperty(propertyName))

        om.PolyDataItem._onPropertyChanged(self, propertyName)

    def getPropertyAttributes(self, propertyName):

        if propertyName == 'Scale':
            return om.PropertyAttributes(decimals=2, minimum=0.01, maximum=100, singleStep=0.1, hidden=False)
        else:
            return om.PolyDataItem.getPropertyAttributes(self, propertyName)

    def onRemoveFromObjectModel(self):
        om.PolyDataItem.onRemoveFromObjectModel(self)

        for view in self.views:
            view.renderer().RemoveActor(self.actor)
            view.render()


def getSegmentationView():
    return app.getViewManager().findView('Segmentation View')


def getDRCView():
    return app.getViewManager().findView('DRC View')


def switchToView(viewName):
    app.getViewManager().switchToView(viewName)


def getCurrentView():
    return app.getViewManager().currentView()


def getDebugFolder():
    return om.getOrCreateContainer('debug', om.findObjectByName('segmentation'))


def thresholdPoints(polyData, arrayName, thresholdRange):
    assert(polyData.GetPointData().GetArray(arrayName))
    f = vtk.vtkThresholdPoints()
    f.SetInput(polyData)
    f.ThresholdBetween(thresholdRange[0], thresholdRange[1])
    f.SetInputArrayToProcess(0,0,0, vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS, arrayName)
    f.Update()
    return shallowCopy(f.GetOutput())



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


def applyPlaneFit(dataObj, distanceThreshold=0.02, expectedNormal=None, perpendicularAxis=None, angleEpsilon=0.2, returnOrigin=False):

    expectedNormal = expectedNormal if expectedNormal is not None else [-1,0,0]

    # perform plane segmentation
    f = pcl.vtkPCLSACSegmentationPlane()
    f.SetInput(dataObj)
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

    polyData = shallowCopy(f.GetOutput())
    points = vtkNumpy.getNumpyFromVtk(dataObj, 'Points')
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
    polyData = pointCloudUtils.labelPointDistanceAlongAxis(polyData, bodyX, origin=bodyOrigin, resultArrayName='distance_along_robot_x')
    polyData = pointCloudUtils.labelPointDistanceAlongAxis(polyData, bodyY, origin=bodyOrigin, resultArrayName='distance_along_robot_y')

    return polyData


def getDebugRevolutionData():
    dataDir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../drc-data'))
    #filename = os.path.join(dataDir, 'valve_wall.vtp')
    #filename = os.path.join(dataDir, 'bungie_valve.vtp')
    #filename = os.path.join(dataDir, 'cinder-blocks.vtp')
    #filename = os.path.join(dataDir, 'cylinder_table.vtp')
    #filename = os.path.join(dataDir, 'debris.vtp')
    filename = os.path.join(dataDir, 'rev1.vtp')
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

        self.taskSelection.connect('taskSelected(int)', self.onTaskSelected)

        l = QtGui.QVBoxLayout(self.panel)
        l.addWidget(self.taskSelection)
        l.addWidget(self.debrisWizard)
        l.addWidget(self.terrainWizard)
        self.debrisWizard.hide()
        self.terrainWizard.hide()

    def _makeDebrisWizard(self):
        debrisWizard = QtGui.QWidget()
        lumberSelection = PythonQt.dd.ddLumberSelection()
        lumberSelection.connect('lumberSelected(int)', self.onDebrisLumberSelected)
        l = QtGui.QVBoxLayout(debrisWizard)
        l.addWidget(lumberSelection)
        l.addStretch()
        return debrisWizard

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
        l.addWidget(buttons)
        l.addStretch()
        return terrainWizard

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

    def cancelCurrentTask(self):
        self.debrisWizard.hide()
        self.taskSelection.show()

    def onTaskSelected(self, taskId):

        taskFunctions = {
                         2:self.startTerrainTask,
                         4:self.startDebrisTask,
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

    #cleanup()
    segmentationView = getOrCreateSegmentationView()

    perspective()

    thresholdWorkspace = False
    doRemoveGround = True

    if thresholdWorkspace:
        polyData = thresholdPoints(polyData, 'distance_along_robot_x', [0.3, 2.0])
        polyData = thresholdPoints(polyData, 'distance_along_robot_y', [-1.0, 1.0])

    if doRemoveGround:
        groundPoints, polyData = removeGround(polyData)
        segmentationObj = updatePolyData(groundPoints, 'ground', alpha=0.3, visible=False)

    segmentationObj = updatePolyData(polyData, 'pointcloud snapshot', alpha=0.3)
    segmentationObj.headAxis = perception._multisenseItem.model.getAxis('head', [1,0,0])

    segmentationView.camera().DeepCopy(app.getDRCView().camera())

    #app.resetCamera(perception._multisenseItem.model.getSpindleAxis())
    #segmentationView.camera().Dolly(3.0)
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

def updatePolyData(polyData, name, **kwargs):

    obj = om.findObjectByName(name)
    obj = obj or showPolyData(polyData, name, **kwargs)
    obj.setPolyData(polyData)
    return obj


def showPolyData(polyData, name, color=None, colorByName=None, colorByRange=None, alpha=1.0, visible=True, view=None, parent='segmentation', cls=None):

    view = view or app.getCurrentRenderView()
    if view is None:
        return

    cls = cls or om.PolyDataItem
    item = cls(name, polyData, view)

    if isinstance(parent, str):
        parentObj = om.getOrCreateContainer(parent)
    else:
        parentObj = parent

    om.addToObjectModel(item, parentObj)
    item.setProperty('Visible', visible)
    item.setProperty('Alpha', alpha)
    if color:
        color = [component * 255 for component in color]
        item.setProperty('Color', QtGui.QColor(*color))
        item.colorBy(None)
    else:
        item.colorBy(colorByName, colorByRange)
    return item


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


def showFrame(frame, name, parent='segmentation', scale=0.5, visible=True):
    if isinstance(parent, str):
        parentObj = om.getOrCreateContainer(parent)
    else:
        parentObj = parent

    item = FrameItem(name, frame, app.getCurrentRenderView())
    om.addToObjectModel(item, parentObj)
    item.setProperty('Visible', visible)
    item.setProperty('Scale', scale)
    return item


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


def startValveSegmentation():


    segmentValveByWallPlane()
    return


    if om.findObjectByName('selected planes'):
        segmentValveByWallPlane()
        return

    picker = PointPicker()
    addViewPicker(picker)
    picker.enabled = True
    picker.start()
    picker.annotationFunc = segmentValveByAnnotation
    om.removeFromObjectModel(om.findObjectByName('valve affordance'))
    om.removeFromObjectModel(om.findObjectByName('valve axes'))
    om.removeFromObjectModel(om.findObjectByName('annotation'))


def segmentValveByWallPlane():

    # find wall plane

    inputObj = om.findObjectByName('pointcloud snapshot')
    polyData = inputObj.polyData


    bodyX = perception._multisenseItem.model.getAxis('body', [1.0, 0.0, 0.0])

    polyData, origin, normal  = applyPlaneFit(polyData, expectedNormal=-bodyX, returnOrigin=True)

    wallPoints = thresholdPoints(polyData, 'dist_to_plane', [-0.01, 0.01])
    updatePolyData(wallPoints, 'valve wall')

    searchRegion = thresholdPoints(polyData, 'dist_to_plane', [0.05, 0.4])

    updatePolyData(searchRegion, 'valve search region')


    searchRegion, origin, _  = applyPlaneFit(searchRegion, expectedNormal=normal, perpendicularAxis=normal, returnOrigin=True)
    searchRegion = thresholdPoints(searchRegion, 'dist_to_plane', [-0.01, 0.01])

    updatePolyData(searchRegion, 'valve search region 2')


    largestCluster = extractLargestCluster(searchRegion)

    updatePolyData(largestCluster, 'valve cluster')


    polyData, circleFit = extractCircle(largestCluster, distanceThreshold=0.01, radiusLimit=[0.19, 0.21])
    updatePolyData(polyData, 'circle fit', visible=False)


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

    d = DebugData()
    d.addLine(origin - normal*radius, origin + normal*radius)
    d.addCircle(origin, circleNormal, radius)
    updatePolyData(d.getPolyData(), 'valve axes', visible=False)


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
        self.clear()

    def clear(self):
        self.points = [None for i in xrange(self.numberOfPoints)]
        self.hoverPos = None
        self.annotationFunc = None
        self.lastMovePos = [0, 0]

    def onMouseMove(self, displayPoint):
        self.lastMovePos = displayPoint

    def onMousePress(self, displayPoint):

        for i in xrange(self.numberOfPoints):
            if self.points[i] is None:
                self.points[i] = self.hoverPos
                break

        if self.points[-1] is not None:
            self.finish()

    def finish(self):

        self.enabled = False
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

        # draw lines
        for a, b in zip(points, points[1:]):
            if b is not None:
                d.addLine(a, b)

        # connect end points
        if points[-1] is not None:
            d.addLine(points[0], points[-1])


        obj = updatePolyData(d.getPolyData(), 'annotation')
        obj.setProperty('Color', QtGui.QColor(0, 255, 0))


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

    def onMouseMove(self, displayPoint):
        self.lastMovePos = displayPoint

    def onMousePress(self, displayPoint):

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


def startBoundedPlaneSegmentation(expectedNormal):

    picker = PointPicker(numberOfPoints=1)
    addViewPicker(picker)
    picker.enabled = True
    picker.start()
    picker.annotationFunc = functools.partial(segmentBoundedPlaneByAnnotation, expectedNormal)


def segmentBoundedPlaneByAnnotation(expectedNormal, point):


    inputObj = om.findObjectByName('pointcloud snapshot')
    inputObj.setProperty('Visible', False)
    polyData = shallowCopy(inputObj.polyData)

    searchRegion = cropToSphere(polyData, point, radius=0.5)


    updatePolyData(searchRegion, 'search region', parent=getDebugFolder(), visible=False)

    _, origin, normal = applyPlaneFit(searchRegion, distanceThreshold=0.02, expectedNormal=expectedNormal, perpendicularAxis=expectedNormal, returnOrigin=True)

    points = vtkNumpy.getNumpyFromVtk(searchRegion, 'Points')
    dist = np.dot(points - origin, normal)
    vtkNumpy.addNumpyToVtk(searchRegion, dist, 'dist_to_plane')


    planePoints = thresholdPoints(searchRegion, 'dist_to_plane', [-0.01, 0.01])
    scenePoints = thresholdPoints(searchRegion, 'dist_to_plane', [0.025, 10])

    updatePolyData(planePoints, 'plane points', color=[1,0,0], alpha=0.3)
    updatePolyData(scenePoints, 'scene points', color=[0,1,0], alpha=0.3)

    #scenePoints = applyEuclideanClustering(scenePoints, clusterTolerance=0.10, minClusterSize=20, maxClusterSize=1e6)
    #updatePolyData(scenePoints, 'scene points', colorByName='cluster_labels')



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
                picker.onMouseMove(mapMousePosition(obj, event))
                eventFilter.setEventHandlerResult(True)
            elif event.type() == QtCore.QEvent.MouseButtonPress:
                picker.onMousePress(mapMousePosition(obj, event))
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

