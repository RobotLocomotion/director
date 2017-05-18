''' Convienence methods on VTK routines only '''

import director.vtkAll as vtk
import director.vtkNumpy as vnp
from director.shallowCopy import shallowCopy
import numpy as np


def thresholdPoints(polyData, arrayName, thresholdRange):
    assert(polyData.GetPointData().GetArray(arrayName))
    f = vtk.vtkThresholdPoints()
    f.SetInputData(polyData)
    f.ThresholdBetween(thresholdRange[0], thresholdRange[1])
    f.SetInputArrayToProcess(0,0,0, vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS, arrayName)
    f.Update()
    return shallowCopy(f.GetOutput())

def thresholdCells(polyData, arrayName, thresholdRange, arrayType='cells'):

    assert arrayType in ('points', 'cells')

    f = vtk.vtkThreshold()
    f.SetInputData(polyData)
    f.ThresholdBetween(thresholdRange[0], thresholdRange[1])

    if arrayType == 'cells':
        assert(polyData.GetCellData().GetArray(arrayName))
        f.SetInputArrayToProcess(0,0,0, vtk.vtkDataObject.FIELD_ASSOCIATION_CELLS, arrayName)
    else:
        assert(polyData.GetPointData().GetArray(arrayName))
        f.SetInputArrayToProcess(0,0,0, vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS, arrayName)

    f.Update()
    g = vtk.vtkGeometryFilter()
    g.SetInputConnection(f.GetOutputPort())
    g.Update()
    return shallowCopy(g.GetOutput())


def transformPolyData(polyData, transform):

    t = vtk.vtkTransformPolyDataFilter()
    t.SetTransform(transform)
    t.SetInputData(shallowCopy(polyData))
    t.Update()
    return shallowCopy(t.GetOutput())


def computeDelaunay3D(polyData):
    f = vtk.vtkDelaunay3D()
    f.SetInputData(polyData)
    f.SetOffset(100.0)
    f.Update()

    surface = vtk.vtkGeometryFilter()
    surface.SetInputData(f.GetOutput())
    surface.Update()

    clean = vtk.vtkCleanPolyData()
    clean.SetInputData(surface.GetOutput())
    clean.Update()

    return shallowCopy(clean.GetOutput())


def computeDelaunay2D(polyData):
    f = vtk.vtkDelaunay2D()
    f.SetInputData(polyData)
    f.Update()
    return shallowCopy(f.GetOutput())


def computeCentroid(polyData):
    return np.average(vnp.getNumpyFromVtk(polyData, 'Points'), axis=0)


def appendPolyData(polyDataList):
    append = vtk.vtkAppendPolyData()
    if polyDataList:
        for polyData in polyDataList:
            append.AddInputData(polyData)
        append.Update()
    return shallowCopy(append.GetOutput())


def computeNormals(polyData, featureAngle=45):
    normals = vtk.vtkPolyDataNormals()
    normals.SetFeatureAngle(featureAngle)
    normals.SetInputData(polyData)
    normals.Update()
    return shallowCopy(normals.GetOutput())


def cleanPolyData(polyData):
    clean = vtk.vtkCleanPolyData()
    clean.SetInputData(polyData)
    clean.Update()
    return shallowCopy(clean.GetOutput())


def triangulatePolyData(polyData):
    f = vtk.vtkTriangleFilter()
    f.SetInputData(polyData)
    f.Update()
    return shallowCopy(f.GetOutput())


def decimateMesh(polyData, targetReduction=0.1):
    '''
    Reduce the number of  triangles in the input mesh by targetReduction.
    0.1 = 10% reduction (if there was 100 triangles, now there will be 90)
    '''
    f = vtk.vtkDecimatePro()
    f.SetInputData(polyData)
    f.SetTargetReduction(targetReduction)
    f.Update()
    return shallowCopy(f.GetOutput())


def hasNonFinitePoints(polyData, arrayName='Points'):
    pts = vnp.getNumpyFromVtk(polyData, arrayName)
    return np.isfinite(pts).any()


def labelNonFinitePoints(polyData, arrayName='Points'):
    '''
    adds is_nonfinite label to polyData.  non finite includes nan and +/- inf.
    '''
    pts = vnp.getNumpyFromVtk(polyData, arrayName)
    labels = np.logical_not(np.isfinite(pts)).any(axis=1)
    vnp.addNumpyToVtk(polyData, np.array(labels, dtype=np.int32), 'is_nonfinite')


def removeNonFinitePoints(polyData, arrayName='Points'):
    polyData = shallowCopy(polyData)
    labelNonFinitePoints(polyData, arrayName)
    return thresholdPoints(polyData, 'is_nonfinite', [0, 0])


def flipImage(image, flipAxis=1):
    '''
    Flip a vtkImageData using the vtkImageFlip filter.
    The flipAxis can be 0 or 1 to flip horizontally or vertically.
    '''
    assert flipAxis in (0, 1)
    f = vtk.vtkImageFlip()
    f.SetFilteredAxis(flipAxis)
    f.SetInputData(image)
    f.Update()
    return shallowCopy(f.GetOutput())


def rotateImage180(image):
    '''
    rotates an image by 180 degrees
    '''
    r1 = vtk.vtkImageFlip()
    r1.SetInputData(image)
    r1.SetFilteredAxis(0)
    r1.Update()
    r2 = vtk.vtkImageFlip()
    r2.SetInputData(r1.GetOutput())
    r2.SetFilteredAxis(1)
    r2.Update()
    return shallowCopy(r2.GetOutput())
