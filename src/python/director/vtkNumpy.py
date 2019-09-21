from director.shallowCopy import shallowCopy
import director.vtkAll as vtk
from vtk.util import numpy_support
import numpy as np


def numpyToPolyData(pts, pointData=None, createVertexCells=True):

    pd = vtk.vtkPolyData()
    pd.SetPoints(getVtkPointsFromNumpy(pts.copy()))

    if pointData is not None:
        for key, value in list(pointData.items()):
            addNumpyToVtk(pd, value.copy(), key)

    if createVertexCells:
        f = vtk.vtkVertexGlyphFilter()
        f.SetInputData(pd)
        f.Update()
        pd = shallowCopy(f.GetOutput())

    return pd


def numpyToImageData(img, flip=True, vtktype=None):
    if flip:
        img = np.flipud(img)
    assert len(img.shape) in (2, 3)
    height, width = img.shape[:2]
    numChannels = 1 if len(img.shape) == 2 else img.shape[2]
    image = vtk.vtkImageData()
    image.SetDimensions(width, height, 1)
    if vtktype is None:
        vtktype = numpy_support.get_vtk_array_type(img.dtype)
    image.AllocateScalars(vtktype, numChannels)
    scalars = getNumpyFromVtk(image, 'ImageScalars')
    if numChannels > 1:
        scalars[:] = img.reshape(width*height, numChannels)[:]
    else:
        scalars[:] = img.reshape(width*height)[:]
    return image


def getNumpyImageFromVtk(vtkimg, flip=True):
    img = numpy_support.vtk_to_numpy(vtkimg.GetPointData().GetScalars())
    w, h, _ = vtkimg.GetDimensions()
    img.shape = (h, w, -1)
    if img.shape[2] == 1:
        img.shape = img.shape[:2]
    if flip:
        img = np.flipud(img)
    return img


def getNumpyFromVtk(dataObj, arrayName='Points', arrayType='points'):
    assert arrayType in ('points', 'cells')

    if arrayName == 'Points':
        vtkArray = dataObj.GetPoints().GetData()
    elif arrayType == 'points':
        vtkArray = dataObj.GetPointData().GetArray(arrayName)
    else:
        vtkArray = dataObj.GetCellData().GetArray(arrayName)

    if not vtkArray:
        raise KeyError('Array not found')

    return numpy_support.vtk_to_numpy(vtkArray)


def getVtkPointsFromNumpy(numpyArray):

    points = vtk.vtkPoints()
    points.SetData(getVtkFromNumpy(numpyArray))
    return points


def getVtkPolyDataFromNumpyPoints(points):
    return numpyToPolyData(points)


def getVtkFromNumpy(numpyArray):

    def MakeCallback(numpyArray):
        def Closure(caller, event):
            closureArray = numpyArray
        return Closure

    vtkArray = numpy_support.numpy_to_vtk(numpyArray)
    vtkArray.AddObserver('DeleteEvent', MakeCallback(numpyArray))
    return vtkArray


def addNumpyToVtk(dataObj, numpyArray, arrayName, arrayType='points'):
    assert arrayType in ('points', 'cells')
    vtkArray = getVtkFromNumpy(numpyArray)
    vtkArray.SetName(arrayName)
    if arrayType == 'points':
        assert dataObj.GetNumberOfPoints() == numpyArray.shape[0]
        dataObj.GetPointData().AddArray(vtkArray)
    else:
        assert dataObj.GetNumberOfCells() == numpyArray.shape[0]
        dataObj.GetCellData().AddArray(vtkArray)
