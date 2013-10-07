import os
import sys
import vtk
import colorsys
import PythonQt
from PythonQt import QtCore, QtGui
import ddapp.applogic as app
from ddapp import objectmodel as om
from ddapp import perception
from ddapp.timercallback import TimerCallback

import numpy as np
from vtkPointCloudUtils import vtkNumpy
from vtkPointCloudUtils.debugVis import DebugData
from vtkPointCloudUtils.shallowCopy import shallowCopy
from vtkPointCloudUtils import affordance
from vtkPointCloudUtils import io
from vtkPointCloudUtils import pointCloudUtils


import vtkPCLFiltersPython as pcl


eventFilters = {}

_spindleAxis = None
_spindleAxis = np.array([ 18.63368125,   4.80462354, -20.86224685])


class CubeAffordanceItem(om.AffordanceItem):

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
        affordance.publishAffordance(aff)


def getSegmentationView():
    return app.getViewManager().findView('Segmentation View')

def getDRCView():
    return app.getViewManager().findView('DRC View')

def switchToView(viewName):
    app.getViewManager().switchToView(viewName)

def getCurrentView():
    return app.getViewManager().currentView()


def colorBy(obj, arrayName, scalarRange=None):

    polyData = obj.polyData
    mapper = obj.mapper

    polyData.GetPointData().SetScalars(polyData.GetPointData().GetArray(arrayName))
    lut = vtk.vtkLookupTable()
    lut.SetNumberOfColors(256)
    lut.SetHueRange(0.667, 0)
    lut.Build()
    mapper.SetLookupTable(lut)
    mapper.ScalarVisibilityOn()
    mapper.SetScalarRange(scalarRange or polyData.GetPointData().GetArray(arrayName).GetRange())
    mapper.InterpolateScalarsBeforeMappingOff()


def thresholdPoints(polyData, arrayName, thresholdRange):
    assert(polyData.GetPointData().GetArray(arrayName))
    f = vtk.vtkThresholdPoints()
    f.SetInputData(polyData)
    f.ThresholdBetween(thresholdRange[0], thresholdRange[1])
    f.SetInputArrayToProcess(0,0,0, vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS, arrayName);
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
t.SetInputData(points.VTKObject)
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
        #showPolyData(inliers, 'inliers %d' % i, color=getRandomColor(), parentName='major planes')
        #showPolyData(outliers, 'outliers %d' % i, color=getRandomColor(), parentName='major planes')
        #showPolyData(largestCluster, 'cluster %d' % i, color=getRandomColor(), parentName='major planes')

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
        obj = showPolyData(polyData, 'plane %d' % i, color=getRandomColor(), visible=True, parentName='major planes')
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


def applyEuclideanClustering(dataObj, clusterTolerance=0.05, minClusterSize=100, maxClusterSize=1e6):

    f = pcl.vtkPCLEuclideanClusterExtraction()
    f.SetInputData(dataObj)
    f.SetClusterTolerance(clusterTolerance)
    f.SetMinClusterSize(int(minClusterSize))
    f.SetMaxClusterSize(int(maxClusterSize))
    f.Update()
    return shallowCopy(f.GetOutput())


def applyPlaneFit(dataObj, distanceThreshold=0.02, expectedNormal=None, returnOrigin=False):

    expectedNormal = expectedNormal if expectedNormal is not None else [-1,0,0]

    # perform plane segmentation
    f = pcl.vtkPCLSACSegmentationPlane()
    f.SetInputData(dataObj)
    f.SetDistanceThreshold(distanceThreshold)
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
    #filename = os.path.join(os.getcwd(), 'valve_wall.vtp')
    #filename = os.path.join(os.getcwd(), 'bungie_valve.vtp')
    filename = os.path.join(os.getcwd(), 'cinder-blocks.vtp')
    #filename = os.path.join(os.getcwd(), 'cylinder_table.vtp')
    #filename = os.path.join(os.getcwd(), 'two-by-fours.vtp')

    return addCoordArraysToPolyData(io.readPolyData(filename))


def getCurrentRevolutionData():
    revPolyData = perception._multisenseItem.model.revPolyData
    if not revPolyData or not revPolyData.GetNumberOfPoints():
        return None

    if useVoxelGrid:
        revPolyData = applyVoxelGrid(revPolyData, leafSize=0.015)

    return addCoordArraysToPolyData(revPolyData)


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
    v.SetInputData(polyData)
    v.Update()
    return shallowCopy(v.GetOutput())


def activateSegmentationMode(debug=False):

    if debug:
        polyData = getDebugRevolutionData()
    else:
        polyData = getCurrentRevolutionData()

    if not polyData:
        return

    global _spindleAxis
    _spindleAxis = perception._multisenseItem.model.getSpindleAxis()

    cleanup()
    segmentationView = getOrCreateSegmentationView()

    perspective()

    thresholdWorkspace = False

    if thresholdWorkspace:
        polyData = thresholdPoints(polyData, 'distance_along_robot_x', [0.3, 2.0])
        polyData = thresholdPoints(polyData, 'distance_along_robot_y', [-1.0, 1.0])
        segmentationObj = showPolyData(polyData, 'pointcloud snapshot', colorByName='distance_along_robot_x')

    else:
        segmentationObj = showPolyData(polyData, 'pointcloud snapshot', alpha=0.3)

    app.resetCamera(perception._multisenseItem.model.getSpindleAxis())
    #segmentationView.camera().Dolly(3.0)
    segmentationView.render()


def updatePolyData(polyData, name, **kwargs):

    obj = om.findObjectByName(name)
    obj = obj or showPolyData(polyData, name, **kwargs)
    obj.setPolyData(polyData)
    return obj


def showPolyData(polyData, name, color=None, colorByName=None, colorByRange=None, alpha=1.0, visible=True, view=None, parentName='segmentation', cls=None):

    view = view or app.getCurrentRenderView()
    if view is None:
        return

    cls = cls or om.PolyDataItem
    item = cls(name, polyData, view)

    parentObj = om.getOrCreateContainer(parentName) if parentName else None

    om.addToObjectModel(item, parentObj)
    item.setProperty('Visible', visible)
    item.setProperty('Alpha', alpha)
    if color:
        color = [component * 255 for component in color]
        item.setProperty('Color', QtGui.QColor(*color))
    if colorByName:
        colorBy(item, colorByName, colorByRange)
    return item


def extractCircle(polyData, distanceThreshold=0.04):

    circleFit = pcl.vtkPCLSACSegmentationCircle()
    circleFit.SetDistanceThreshold(distanceThreshold)
    circleFit.SetInputData(polyData)
    circleFit.Update()

    polyData = thresholdPoints(circleFit.GetOutput(), 'ransac_labels', [1.0, 1.0])
    return polyData, circleFit


def removeMajorPlane(polyData, distanceThreshold=0.02):

    # perform plane segmentation
    f = pcl.vtkPCLSACSegmentationPlane()
    f.SetInputData(polyData)
    f.SetDistanceThreshold(distanceThreshold)
    f.Update()

    polyData = thresholdPoints(f.GetOutput(), 'ransac_labels', [0.0, 0.0])
    return polyData, f


def segmentValve(polyData, planeNormal=None):


    polyData, circleFit = extractCircle(polyData, distanceThreshold=0.04)
    showPolyData(polyData, 'circle fit (initial)', colorByName='z', visible=False)


    polyData, circleFit = extractCircle(polyData, distanceThreshold=0.01)
    showPolyData(polyData, 'circle fit', colorByName='z')


    radius = circleFit.GetCircleRadius()
    origin = np.array(circleFit.GetCircleOrigin())
    normal = np.array(circleFit.GetCircleNormal())


    normal = planeNormal if planeNormal is not None else normal
    normal = normal/np.linalg.norm(normal)


    p1 = origin - normal*radius
    p2 = origin + normal*radius

    d = DebugData()
    d.addLine(p1, p2)
    d.addLine(origin - normal*0.015, origin + normal*0.015, radius=radius)
    showPolyData(d.getPolyData(), 'circle model')
    showPolyData(d.getPolyData(), 'valve', view=getDRCView(), parentName='affordances')

    getDRCView().renderer().ResetCamera(d.getPolyData().GetBounds())
    getSegmentationView().renderer().ResetCamera(d.getPolyData().GetBounds())

    global params
    params = {}
    params['axis'] = normal
    params['radius'] = radius
    params['origin'] = origin
    params['length'] = 0.03
    return params


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

    def __init__(self):
        TimerCallback.__init__(self)
        self.targetFps = 30
        self.enabled = False
        self.clear()

    def clear(self):
        self.p1 = None
        self.p2 = None
        self.p3 = None
        self.hoverPos = None
        self.lastMovePos = [0, 0]

    def onMouseMove(self, displayPoint):
        self.lastMovePos = displayPoint

    def onMousePress(self, displayPoint):

        if self.p1 is None:
            self.p1 = self.hoverPos
        elif self.p2 is None:
            self.p2 = self.hoverPos
        elif self.p3 is None:
            self.p3 = self.hoverPos

            if self.p3 is not None:
                self.finish()


    def finish(self):

        self.enabled = False

        segmentationObj = om.findObjectByName('pointcloud snapshot')
        segmentationObj.mapper.ScalarVisibilityOff()
        segmentationObj.setProperty('Point Size', 2)
        segmentationObj.setProperty('Alpha', 0.8)

        p1 = self.p1.copy()
        p2 = self.p2.copy()
        p3 = self.p3.copy()

        # constraint z to lie in plane
        p1[2] = p2[2] = p3[2] = max(p1[2], p2[2], p3[2])

        zedge = p2 - p1
        zaxis = zedge / np.linalg.norm(zedge)

        #xwidth = distanceToLine(p3, p1, p2)

        # expected dimensions
        xwidth, ywidth = self.blockDimensions

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

        obj = updatePolyData(cube, 'block affordance', cls=CubeAffordanceItem, parentName='affordances')
        obj.actor.SetUserTransform(t)
        obj.addToView(app.getDRCView())

        params = dict(origin=origin, xwidth=xwidth, ywidth=ywidth, zwidth=zwidth, xaxis=xaxis, yaxis=yaxis, zaxis=zaxis)
        obj.setAffordanceParams(params)
        obj.updateParamsFromActorTransform()


    def handleRelease(self, displayPoint):
        pass


    def draw(self):

        d = DebugData()

        p1 = self.p1 if self.p1 is not None else self.hoverPos
        p2 = self.p2 if self.p2 is not None else self.hoverPos
        p3 = self.p3 if self.p3 is not None else self.hoverPos


        for p in (p1, p2, p3):
            if p is not None:
                d.addSphere(p, radius=0.01)


        if p1 is not None:
            if p2 is not None:
                d.addLine(p1, p2)
            if p3 is not None:
                d.addLine(p1, p3)
                d.addLine(p2, p3)


        obj = updatePolyData(d.getPolyData(), 'annotation')
        obj.setProperty('Color', QtGui.QColor(0, 255, 0))


    def tick(self):

        if not self.enabled:
            return

        self.hoverPos = pickPoint('pointcloud snapshot', self.lastMovePos)
        self.draw()



pointPicker = PointPicker()
pointPicker.start()


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

    obj = updatePolyData(cube, 'block affordance', cls=CubeAffordanceItem, parentName='affordances')
    obj.actor.SetUserTransform(t)
    obj.addToView(app.getDRCView())

    params = dict(origin=origin, xwidth=xwidth, ywidth=ywidth, zwidth=zwidth, xaxis=xaxis, yaxis=yaxis, zaxis=zaxis)
    obj.setAffordanceParams(params)
    obj.updateParamsFromActorTransform()


def startBlockSegmentation(dimensions):


    if om.findObjectByName('selected planes'):
        segmentBlockByPlanes(dimensions)
        return

    pointPicker.clear()
    pointPicker.enabled = True
    pointPicker.blockDimensions = dimensions
    om.removeFromObjectModel(om.findObjectByName('block affordance'))
    om.removeFromObjectModel(om.findObjectByName('block axes'))
    om.removeFromObjectModel(om.findObjectByName('annotation'))


savedCameraParams = None

def perspective():

    global savedCameraParams
    if savedCameraParams is None:
        return

    aff = om.findObjectByName('block affordance')
    if aff is not None:
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


def orthoX():

    aff = om.findObjectByName('block affordance')
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

    aff = om.findObjectByName('block affordance')
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

    aff = om.findObjectByName('block affordance')
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
    obj2 = showPolyData(shallowCopy(polyData), name, parentName='selected planes')
    obj2.setProperty('Color', obj.getProperty('Color'))
    obj2.setProperty('Point Size', 4)
    obj.setProperty('Visible', False)


def zoomToDisplayPoint(displayPoint, boundsRadius=0.2):

    pickedPoint = pickPoint('pointcloud snapshot', displayPoint)
    if pickedPoint is None:
        return


    worldPt1, worldPt2 = getRayFromDisplayPoint(getSegmentationView(), displayPoint)
    global _spindleAxis
    _spindleAxis = worldPt2 - worldPt1


    diagonal = np.array([boundsRadius, boundsRadius, boundsRadius])
    bounds = np.hstack([pickedPoint - diagonal, pickedPoint + diagonal])
    bounds = [bounds[0], bounds[3], bounds[1], bounds[4], bounds[2], bounds[5]]
    getSegmentationView().renderer().ResetCamera(bounds)

    segmentationObj = om.findObjectByName('pointcloud snapshot')
    segmentationObj.setProperty('Point Size', 5)
    #colorBy(segmentationObj, 'z', [pickedPoint[2]-0.1, pickedPoint[2]+0.1])
    getSegmentationView().render()


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


    '''
    segmentationObj.mapper.ScalarVisibilityOff()
    segmentationObj.setProperty('Alpha', 0.3)

    # plane fit
    polyData, normal = applyPlaneFit(polyData)
    polyData = thresholdPoints(polyData, 'dist_to_plane', [-0.02, 0.02])
    showPolyData(polyData, 'plane fit', colorByName='z')

    params = segmentValve(polyData)

    #affordance.publishValve(params)

    getSegmentationView().render()
    '''


def cleanup():

    obj = om.findObjectByName('segmentation')
    if not obj:
        return

    item = om.getItemForObject(obj)
    childItems = item.takeChildren()

    for childItem in childItems:
        obj = om.getObjectForItem(childItem)
        if isinstance(obj, om.PolyDataItem):
            obj.view.renderer().RemoveActor(obj.actor)
        obj.view.render()
        del om.objects[childItem]


def segmentationViewEventFilter(obj, event):

    eventFilter = eventFilters[obj]

    if event.type() == QtCore.QEvent.MouseButtonDblClick:
        eventFilter.setEventHandlerResult(True)
        onSegmentationViewDoubleClicked(mapMousePosition(obj, event))

    elif pointPicker.enabled:

        if event.type() == QtCore.QEvent.MouseMove:
            pointPicker.onMouseMove(mapMousePosition(obj, event))
            eventFilter.setEventHandlerResult(True)
        elif event.type() == QtCore.QEvent.MouseButtonPress:
            pointPicker.onMousePress(mapMousePosition(obj, event))
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

    activateSegmentationMode(debug=True)

