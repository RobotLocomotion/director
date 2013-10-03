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


def colorBy(polyData, mapper, arrayName, scalarRange=None):

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
    filename = os.path.join(os.getcwd(), 'bungie_valve.vtp')
    return io.readPolyData(filename)


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


def activateSegmentationMode():

    polyData = getDebugRevolutionData()
    #polyData = getCurrentRevolutionData()

    if not polyData:
        return

    cleanup()
    segmentationView = getOrCreateSegmentationView()

    #polyData = thresholdPoints(polyData, 'distance_along_robot_x', [0.3, 1.5])
    #polyData = thresholdPoints(polyData, 'distance_along_robot_y', [-1, 1])

    segmentationObj = showPolyData(polyData, 'pointcloud snapshot', colorByName='distance_along_robot_x')

    app.resetCamera(perception._multisenseItem.model.getSpindleAxis())
    segmentationView.camera().Dolly(3.0)
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
        colorBy(polyData, item.mapper, colorByName, colorByRange)
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


_removeMajorPlane = True


def onSegmentationMouseMove(widget, mousePosition):

    global lastMovePos
    lastMovePos = mousePosition.x(), widget.height - mousePosition.y()



class PointPicker(TimerCallback):

    def __init__(self):
        TimerCallback.__init__(self)
        self.targetFps = 30
        pass

    def tick(self):

        updatePolyData(vtk.vtkPolyData(), 'pick point')


        picker = vtk.vtkPointPicker()
        picker.SetTolerance(0.01)
        picker.Pick(lastMovePos[0], lastMovePos[1], 0, app.getCurrentView().renderer())

        pickPoints = picker.GetPickedPositions()


        if pickPoints.GetNumberOfPoints() and picker.GetDataSet() == om.findObjectByName('pointcloud snapshot').polyData:

            #print pickPoints.GetNumberOfPoints()

            pickedPoint = picker.GetPickedPositions().GetPoint(0)

            d = DebugData()
            d.addSphere(pickedPoint, radius=0.0075)
            obj = updatePolyData(d.getPolyData(), 'pick point')
            obj.setProperty('Color', QtGui.QColor(0, 255, 0))

        else:
            updatePolyData(vtk.vtkPolyData(), 'pick point')
            #print 'no picked points'



pointPicker = None


def onSegmentationViewDoubleClicked(widget, mousePosition):


    global pointPicker
    if not pointPicker:
        pointPicker = PointPicker()
        pointPicker.start()
    else:
        pointPicker.stop()
        pointPicker = None
    return


    displayPoint = mousePosition.x(), widget.height - mousePosition.y()

    global lastClickPos
    lastClickPos = displayPoint

    worldPt1 = [0,0,0,0]
    worldPt2 = [0,0,0,0]

    renderer = getSegmentationView().renderer()
    vtk.vtkInteractorObserver.ComputeDisplayToWorld(renderer, displayPoint[0], displayPoint[1], 0, worldPt1)
    vtk.vtkInteractorObserver.ComputeDisplayToWorld(renderer, displayPoint[0], displayPoint[1], 1, worldPt2)


    worldPt1 = np.array(worldPt1[:3])
    worldPt2 = np.array(worldPt2[:3])

    d = DebugData()
    d.addLine(worldPt1, worldPt2)
    showPolyData(d.getPolyData(), 'mouse click ray', visible=False)


    segmentationObj = om.findObjectByName('pointcloud snapshot')
    polyData = segmentationObj.polyData
    points = vtkNumpy.getNumpyFromVtk(polyData, 'Points')

    x1 = worldPt1
    x2 = worldPt2
    x0 = points

    numerator = np.sqrt(np.sum(np.cross((x0 - x1), (x0-x2))**2, axis=1))
    denom = np.linalg.norm(x2-x1)

    dists = numerator / denom

    vtkNumpy.addNumpyToVtk(polyData, dists, 'dist_to_line')
    colorBy(polyData, segmentationObj.mapper, 'dist_to_line', [0.0, 0.2])


    # extract cluster
    polyData = thresholdPoints(polyData, 'dist_to_line', [0.0, 0.5])
    polyData = thresholdPoints(polyData, 'distance', [0.3, 1.5])

    if _removeMajorPlane:
        polyData, planeFit = removeMajorPlane(polyData, distanceThreshold=0.03)

    showPolyData(polyData, 'selected cluster', colorByName='dist_to_line', visible=False)


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
        eventFilter.setEventHandlerResult(False)
        onSegmentationMouseMove(obj, event.pos())
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
    eventFilter.connect('handleEvent(QObject*, QEvent*)', func)


def init():

    installEventFilter(app.getViewManager().findView('DRC View'), drcViewEventFilter)

    activateSegmentationMode()

