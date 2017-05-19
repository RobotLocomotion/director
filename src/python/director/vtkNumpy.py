from director.shallowCopy import shallowCopy
import director.vtkAll as vtk
from vtk.util import numpy_support
import numpy as np


def numpyToPolyData(pts, pointData=None, createVertexCells=False):

    pd = vtk.vtkPolyData()
    pd.SetPoints(getVtkPointsFromNumpy(pts.copy()))

    if pointData is not None:
        for key, value in pointData.iteritems():
            addNumpyToVtk(pd, value.copy(), key)

    if createVertexCells:
        f = vtk.vtkVertexGlyphFilter()
        f.SetInput(pd)
        f.Update()
        pd = shallowCopy(f.GetOutput())

    return pd


def getNumpyFromVtk(dataObj, arrayName='Points'):
    if arrayName == 'Points':
        vtkArray = dataObj.GetPoints().GetData()
    else:
        vtkArray = dataObj.GetPointData().GetArray(arrayName)

    if not vtkArray:
        raise KeyError('Array not found')

    return numpy_support.vtk_to_numpy(vtkArray)


def getVtkPointsFromNumpy(numpyArray):

    points = vtk.vtkPoints()
    points.SetData(getVtkFromNumpy(numpyArray))
    return points


def getVtkPolyDataFromNumpyPoints(points):
    '''
    Given an Nx3 array of xyz points
    Return a new vtkPolyData containing points and vertex cells.
    If the input points is not float64 it will be converted first.
    '''

    if points.dtype != np.float64:
        points = points.astype(np.float64)

    polyData = vtk.vtkPolyData()
    polyData.SetPoints(getVtkPointsFromNumpy(points))
    vtk.vtkPCLConversions.AddVertexCells(polyData)
    return polyData


def getVtkFromNumpy(numpyArray):

    def MakeCallback(numpyArray):
        def Closure(caller, event):
            closureArray = numpyArray
        return Closure

    vtkArray = numpy_support.numpy_to_vtk(numpyArray)
    vtkArray.AddObserver('DeleteEvent', MakeCallback(numpyArray))
    return vtkArray


def addNumpyToVtk(dataObj, numpyArray, arrayName):
    assert dataObj.GetNumberOfPoints() == numpyArray.shape[0]

    vtkArray = getVtkFromNumpy(numpyArray)
    vtkArray.SetName(arrayName)
    dataObj.GetPointData().AddArray(vtkArray)
