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


def switchToSegmentationView():

    if not perception._multisenseItem.model.revPolyData or not perception._multisenseItem.model.revPolyData.GetNumberOfPoints():
        return

    viewManager = app.getViewManager()
    segmentationView = viewManager.findView('Segmentation View')
    if not segmentationView:
        segmentationView = viewManager.createView('Segmentation View')
        installEventFilter(segmentationView, segmentationViewEventFilter)

    viewManager.switchToView('Segmentation View')

    polyData = vtk.vtkPolyData()
    polyData.ShallowCopy(perception._multisenseItem.model.revPolyData)

    segmentationObj = om.findObjectByName('pointcloud snapshot')
    if not segmentationObj:
        folder = om.addContainer('segmentation')
        segmentationObj = om.PolyDataItem('pointcloud snapshot', polyData, segmentationView)
        om.addToObjectModel(segmentationObj, folder)
    else:
        segmentationObj.setPolyData(polyData)


    points = vtkNumpy.getNumpyFromVtk(polyData, 'Points')
    vtkNumpy.addNumpyToVtk(polyData, points[:,0].copy(), 'x')
    vtkNumpy.addNumpyToVtk(polyData, points[:,1].copy(), 'y')
    vtkNumpy.addNumpyToVtk(polyData, points[:,2].copy(), 'z')

    colorBy(polyData, segmentationObj.mapper, 'x')


    app.resetCamera(perception._multisenseItem.model.getSpindleAxis())
    segmentationView.render()



def showPolyData(polyData, name, colorByName=None, colorByRange=None, alpha=0.5, visible=True, view=None):

    view = view or getCurrentView()
    item = om.PolyDataItem(name, polyData, view)
    om.addToObjectModel(item, om.findObjectByName('segmentation'))
    item.setProperty('Visible', visible)
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


def segmentValve(polyData, planeNormal=None):


    polyData, circleFit = extractCircle(polyData, distanceThreshold=0.04)
    showPolyData(polyData, 'circle fit (initial)', colorByName='z', alpha=0.5, visible=False)


    polyData, circleFit = extractCircle(polyData, distanceThreshold=0.01)
    showPolyData(polyData, 'circle fit', colorByName='z', alpha=0.5)


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
    showPolyData(d.getPolyData(), 'circle model', alpha=0.5, view=getDRCView())

    getDRCView().renderer().ResetCamera(d.getPolyData().GetBounds())

    global params
    params = {}
    params['axis'] = normal
    params['radius'] = radius
    params['origin'] = origin
    params['length'] = 0.03
    return params



def onSegmentationViewDoubleClicked(widget, mousePosition):

    displayPoint = mousePosition.x(), widget.height - mousePosition.y()

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
    polyData = thresholdPoints(polyData, 'dist_to_line', [0.0, 0.25])
    polyData = thresholdPoints(polyData, 'distance', [0.3, 1.5])

    showPolyData(polyData, 'selected cluster', colorByName='dist_to_line', visible=False)


    segmentationObj.mapper.ScalarVisibilityOff()
    segmentationObj.setProperty('Alpha', 0.3)

    # plane fit
    polyData, normal = applyPlaneFit(polyData)
    polyData = thresholdPoints(polyData, 'dist_to_plane', [-0.02, 0.02])
    showPolyData(polyData, 'plane fit', colorByName='z')

    params = segmentValve(polyData)

    affordance.publishValve(params)

    getSegmentationView().render()

    switchToView('DRC View')


def segmentationViewEventFilter(obj, event):

    eventFilter = eventFilters[obj]
    if event.type() == QtCore.QEvent.MouseButtonDblClick:
        eventFilter.setEventHandlerResult(True)
        onSegmentationViewDoubleClicked(obj, event.pos())
    else:
        eventFilter.setEventHandlerResult(False)


def drcViewEventFilter(obj, event):

    eventFilter = eventFilters[obj]
    if event.type() == QtCore.QEvent.MouseButtonDblClick:
        eventFilter.setEventHandlerResult(True)
        switchToSegmentationView()
    else:
        eventFilter.setEventHandlerResult(False)


def installEventFilter(view, func):

    global eventFilters
    eventFilter = PythonQt.dd.ddPythonEventFilter()

    qvtkwidget = view.vtkWidget()
    qvtkwidget.installEventFilter(eventFilter)
    eventFilters[qvtkwidget] = eventFilter

    eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonDblClick)
    eventFilter.connect('handleEvent(QObject*, QEvent*)', func)


def init():

    installEventFilter(app.getViewManager().findView('DRC View'), drcViewEventFilter)
