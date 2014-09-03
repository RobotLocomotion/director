''' Convienence methods on VTK routines only '''

import ddapp.vtkAll as vtk
import ddapp.vtkNumpy as vnp
from ddapp.shallowCopy import shallowCopy
import numpy as np


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


def computeDelaunay3D(polyData):
    f = vtk.vtkDelaunay3D()
    f.SetInput(polyData)
    f.SetOffset(100.0)
    f.Update()

    surface = vtk.vtkGeometryFilter()
    surface.SetInput(f.GetOutput())
    surface.Update()

    clean = vtk.vtkCleanPolyData()
    clean.SetInput(surface.GetOutput())
    clean.Update()

    return shallowCopy(clean.GetOutput())


def computeDelaunay2D(polyData):
    f = vtk.vtkDelaunay2D()
    f.SetInput(polyData)
    f.Update()
    return shallowCopy(f.GetOutput())


def appendPolyData(polyDataList):
    assert len(polyDataList)
    append = vtk.vtkAppendPolyData()
    for polyData in polyDataList:
        append.AddInput(polyData)
    append.Update()
    return shallowCopy(append.GetOutput())


def computeNormals(polyData, featureAngle=45):
    normals = vtk.vtkPolyDataNormals()
    normals.SetFeatureAngle(featureAngle)
    normals.SetInput(polyData)
    normals.Update()
    return shallowCopy(normals.GetOutput())


def cleanPolyData(polyData):
    clean = vtk.vtkCleanPolyData()
    clean.SetInput(polyData)
    clean.Update()
    return shallowCopy(clean.GetOutput())


def hasNanPoints(polyData):
    pts = vnp.getNumpyFromVtk(polyData, 'Points')
    return np.isnan(pts).any()


def addNanLabels(polyData):
    '''
    adds is_nan label to polyData
    '''
    pts = vnp.getNumpyFromVtk(polyData, 'Points')
    labels = np.isnan(pts).any(axis=1)
    vnp.addNumpyToVtk(polyData, np.array(labels, dtype=np.int32), 'is_nan')


def removeNanPoints(polyData):
    polyData = shallowCopy(polyData)
    addNanLabels(polyData)
    return thresholdPoints(polyData, 'is_nan', [0, 0])
