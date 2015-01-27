import os
import vtkAll as vtk
from shallowCopy import shallowCopy

def readPolyData(filename, computeNormals=False):

    ext = os.path.splitext(filename)[1].lower()

    readers = {
            '.vtp' : vtk.vtkXMLPolyDataReader,
            '.vtk' : vtk.vtkPolyDataReader,
            '.ply' : vtk.vtkPLYReader,
            '.obj' : vtk.vtkOBJReader,
            '.stl' : vtk.vtkSTLReader,
              }

    try:
        readers['.pcd'] = vtk.vtkPCDReader
    except AttributeError:
        pass

    if ext not in readers:
        raise Exception('Unknown file extension in readPolyData: %s' % filename)

    reader = readers[ext]()
    reader.SetFileName(filename)
    reader.Update()
    polyData = shallowCopy(reader.GetOutput())

    if computeNormals:
        return _computeNormals(polyData)
    else:
        return polyData


def readImage(filename):

    ext = os.path.splitext(filename)[1].lower()

    readers = {
            '.png' : vtk.vtkPNGReader,
            '.jpg' : vtk.vtkJPEGReader,
              }

    if ext not in readers:
        raise Exception('Unknown file extension in readImage: %s' % filename)

    reader = readers[ext]()
    reader.SetFileName(filename)
    reader.Update()
    image = shallowCopy(reader.GetOutput())
    return image


def writePolyData(polyData, filename):

    ext = os.path.splitext(filename)[1].lower()

    writers = {
            '.vtp' : vtk.vtkXMLPolyDataWriter,
            '.ply' : vtk.vtkPLYWriter,
            '.stl' : vtk.vtkSTLWriter,
              }

    if ext not in writers:
        raise Exception('Unknown file extension in writePolyData: %s' % filename)

    writer = writers[ext]()

    if ext in ('.ply', '.stl'):
        polyData = _triangulate(polyData)
        writer.SetFileTypeToASCII()

    writer.SetFileName(filename)
    writer.SetInput(polyData)
    writer.Update()

def writeImage(image, filename):

    ext = os.path.splitext(filename)[1].lower()

    writers = {
            '.png' : vtk.vtkPNGWriter,
            '.jpg' : vtk.vtkJPEGWriter,
            '.pnm' : vtk.vtkPNMWriter,
            '.tiff' : vtk.vtkTIFFWriter,
            '.bmp' : vtk.vtkBMPWriter,
            '.vti' : vtk.vtkXMLImageDataWriter,
              }

    if ext not in writers:
        raise Exception('Unknown file extension in writePolyData: %s' % filename)

    writer = writers[ext]()
    writer.SetFileName(filename)
    writer.SetInput(image)
    writer.Write()


def _computeNormals(polyData):
    normals = vtk.vtkPolyDataNormals()
    normals.SetFeatureAngle(45)
    normals.SetInput(polyData)
    normals.Update()
    return shallowCopy(normals.GetOutput())

def _triangulate(polyData):
    normals = vtk.vtkTriangleFilter()
    normals.SetInput(polyData)
    normals.Update()
    return shallowCopy(normals.GetOutput())
