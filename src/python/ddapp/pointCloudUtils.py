import vtkAll as vtk
import vtkNumpy
import numpy as np
from shallowCopy import shallowCopy

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


def appendPolyData(polyDataList):
    assert(len(polyDataList))
    append = vtk.vtkAppendPolyData()
    for polyData in polyDataList:
        append.AddInputData(polyData)
    append.Update()
    return shallowCopy(append.GetOutput())


def thresholdPoints(dataObj, arrayName, minValue, maxValue):
    if not dataObj.GetPointData().GetArray(arrayName):
        raise Exception("The data object passed to thresholdPoints has no array named: '%s'" % arrayName)
    thresh = vtk.vtkThresholdPoints()
    thresh.SetInputData(dataObj)
    thresh.ThresholdBetween(minValue, maxValue)
    thresh.SetInputArrayToProcess(0,0,0, vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS, arrayName);
    thresh.Update()
    return shallowCopy(thresh.GetOutput())
