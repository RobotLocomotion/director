import os
import sys
import vtk
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



def applyPlaneFit(dataObj, distanceThreshold=0.02, expectedNormal=None):

    expectedNormal = expectedNormal or [-1,0,0]

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
    #filename = os.path.join(os.getcwd(), 'cinder-blocks.vtp')
    #filename = os.path.join(os.getcwd(), 'cylinder_table.vtp')
    filename = os.path.join(os.getcwd(), 'two-by-fours.vtp')

    return addCoordArraysToPolyData(io.readPolyData(filename))


def getCurrentRevolutionData():
    revPolyData = perception._multisenseItem.model.revPolyData
    if not revPolyData or not revPolyData.GetNumberOfPoints():
        return None
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


def activateSegmentationMode():

    polyData = getDebugRevolutionData()
    #polyData = getCurrentRevolutionData()

    if not polyData:
        return

    cleanup()
    segmentationView = getOrCreateSegmentationView()

    #polyData = thresholdPoints(polyData, 'distance_along_robot_x', [0.3, 2.0])
    #polyData = thresholdPoints(polyData, 'distance_along_robot_y', [-1, 1])

    segmentationObj = showPolyData(polyData, 'pointcloud snapshot', colorByName='x')

    app.resetCamera(perception._multisenseItem.model.getSpindleAxis())
    #segmentationView.camera().Dolly(3.0)
    segmentationView.render()


def getOrCreateContainer(containerName):

    folder = om.findObjectByName(containerName)
    if not folder:
        folder = om.addContainer(containerName)
    return folder


def updatePolyData(polyData, name):

    obj = om.findObjectByName(name)
    obj = obj or showPolyData(polyData, name)
    obj.setPolyData(polyData)
    return obj


def showPolyData(polyData, name, colorByName=None, colorByRange=None, alpha=1.0, visible=True, view=None, parentName='segmentation'):

    view = view or getCurrentView()
    item = om.PolyDataItem(name, polyData, view)

    parentObj = getOrCreateContainer(parentName) if parentName else None

    om.addToObjectModel(item, parentObj)
    item.setProperty('Visible', visible)
    item.setProperty('Alpha', alpha)
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


_removeMajorPlane = False


def onSegmentationMouseMove(widget, mousePosition):

    global lastMovePos
    lastMovePos = mousePosition.x(), widget.height - mousePosition.y()

def onSegmentationMousePress(widget, mousePosition):

    displayPoint = mousePosition.x(), widget.height - mousePosition.y()
    pointPicker.handleClick(displayPoint)

def onSegmentationMouseRelease(widget, mousePosition):
    pass


def pickPoint(objName, displayPoint, tolerance=0.01):

    view = app.getCurrentRenderView()
    obj = om.findObjectByName(objName)
    assert obj and view

    picker = vtk.vtkPointPicker()
    picker.AddPickList(obj.actor)
    picker.PickFromListOn()
    picker.SetTolerance(tolerance)
    picker.Pick(displayPoint[0], displayPoint[1], 0, view.renderer())
    pickPoints = picker.GetPickedPositions()
    return np.array(pickPoints.GetPoint(0)) if pickPoints.GetNumberOfPoints() else None


class PointPicker(TimerCallback):

    def __init__(self):
        TimerCallback.__init__(self)
        self.targetFps = 30
        self.clear()

    def clear(self):
        self.p1 = None
        self.p2 = None
        self.p3 = None
        self.hoverPos = None

    def handleClick(self, displayPoint):

        if self.p1 is None:
            self.p1 = self.hoverPos
            print 'set p1:', self.p1
        elif self.p2 is None:
            self.p2 = self.hoverPos
            print 'set p2:', self.p2
        elif self.p3 is None:
            self.p3 = self.hoverPos
            print 'set p3:', self.p3

            if self.p3 is not None:
                print 'stopping'
                self.finish()


    def finish(self):

        stopSegment()

        p1 = self.p1.copy()
        p2 = self.p2.copy()
        p3 = self.p3.copy()

        # constraint z to lie in plane
        #p1[2] = p2[2] = p3[2] = max(p1[2], p2[2], p3[2])

        zedge = p2 - p1
        zaxis = zedge / np.linalg.norm(zedge)

        xwidth = distanceToLine(p3, p1, p2)
        print 'x width:', xwidth

        # expected dimensions for 2x4 board
        xwidth = 0.089
        ywidth = 0.038

        zwidth = np.linalg.norm(zedge)

        yaxis = np.cross(p2 - p1, p3 - p1)
        yaxis = yaxis / np.linalg.norm(yaxis)

        xaxis = np.cross(yaxis, zaxis)

        # flip axes
        if np.dot(yaxis, [0,0,-1]) < 0:
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
        obj = updatePolyData(d.getPolyData(), 'board axes')
        obj.setProperty('Color', QtGui.QColor(255, 255, 0))

        cube = vtk.vtkCubeSource()
        cube.SetXLength(xwidth)
        cube.SetYLength(ywidth)
        cube.SetZLength(zwidth)
        cube.Update()

        t = getTransformFromAxes(xaxis, yaxis, zaxis)
        t.PostMultiply()
        t.Translate(origin)

        f = vtk.vtkTransformPolyDataFilter()
        f.AddInputConnection(cube.GetOutputPort())
        f.SetTransform(t)
        f.Update()
        obj = updatePolyData(shallowCopy(f.GetOutput()), 'board model')

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

        if not segmentationMode:
            return

        self.hoverPos = pickPoint('pointcloud snapshot', lastMovePos)
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


segmentationMode = False

def startSegment():

    global segmentationMode
    segmentationMode = True
    pointPicker.clear()


def stopSegment():

    global segmentationMode
    segmentationMode = False


def onSegmentationViewDoubleClicked(widget, mousePosition):


    displayPoint = mousePosition.x(), widget.height - mousePosition.y()

    pickedPoint = pickPoint('pointcloud snapshot', displayPoint)
    if pickedPoint is not None:


        #getSegmentationView().camera().SetFocalPoint(pickedPoint)
        #getSegmentationView().camera().SetPosition(pickedPoint - np.array([1.0, 0.0, 0.0]))

        boundsRadius = 0.2
        diagonal = np.array([boundsRadius, boundsRadius, boundsRadius])
        bounds = np.hstack([pickedPoint - diagonal, pickedPoint + diagonal])

        bounds = [bounds[0], bounds[3], bounds[1], bounds[4], bounds[2], bounds[5]]
        getSegmentationView().renderer().ResetCamera(bounds)

        segmentationObj = om.findObjectByName('pointcloud snapshot')
        colorBy(segmentationObj, 'x', [bounds[0], bounds[1]])

        getSegmentationView().render()

    return

    worldPt1, worldPt2 = getRayFromDisplayPoint(getSegmentationView(), displayPoint)

    #d = DebugData()
    #d.addLine(worldPt1, worldPt2)
    #showPolyData(d.getPolyData(), 'mouse click ray', visible=False)


    segmentationObj = om.findObjectByName('pointcloud snapshot')
    polyData = segmentationObj.polyData

    polyData = labelDistanceToLine(polyData, worldPt1, worldPt2)

    # extract points near line
    distanceToLineThreshold = 0.3
    polyData = thresholdPoints(polyData, 'distance_to_line', [0.0, distanceToLineThreshold])

    if _removeMajorPlane:
        polyData, planeFit = removeMajorPlane(polyData, distanceThreshold=0.03)

    showPolyData(polyData, 'selected cluster', colorByName='distance_to_line', visible=False)

    return


    segmentationObj.mapper.ScalarVisibilityOff()
    segmentationObj.setProperty('Alpha', 0.3)

    # plane fit
    polyData, normal = applyPlaneFit(polyData)
    polyData = thresholdPoints(polyData, 'dist_to_plane', [-0.02, 0.02])
    showPolyData(polyData, 'plane fit', colorByName='z')

    params = segmentValve(polyData)

    #affordance.publishValve(params)

    getSegmentationView().render()


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
        onSegmentationViewDoubleClicked(obj, event.pos())
    elif event.type() == QtCore.QEvent.MouseMove:
        eventFilter.setEventHandlerResult(segmentationMode)
        onSegmentationMouseMove(obj, event.pos())
    elif event.type() == QtCore.QEvent.MouseButtonPress:
        eventFilter.setEventHandlerResult(segmentationMode)
        onSegmentationMousePress(obj, event.pos())
    elif event.type() == QtCore.QEvent.MouseButtonRelease:
        eventFilter.setEventHandlerResult(segmentationMode)
        onSegmentationMouseRelease(obj, event.pos())
    else:
        eventFilter.setEventHandlerResult(False)


def drcViewEventFilter(obj, event):

    eventFilter = eventFilters[obj]
    if event.type() == QtCore.QEvent.MouseButtonDblClick:
        eventFilter.setEventHandlerResult(True)
        activateSegmentationMode()
    else:
        eventFilter.setEventHandlerResult(False)


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

    activateSegmentationMode()

