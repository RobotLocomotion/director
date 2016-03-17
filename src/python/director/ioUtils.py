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


def readMultiBlock(filename):
    '''Reads a .vtm file and returns a list of vtkPolyData objects'''

    reader = vtk.vtkXMLMultiBlockDataReader()
    reader.SetFileName(filename)
    reader.Update()

    polyDataList = []
    mb = reader.GetOutput()
    for i in xrange(mb.GetNumberOfBlocks()):
        polyData = vtk.vtkPolyData.SafeDownCast(mb.GetBlock(i))
        if polyData and polyData.GetNumberOfPoints():
            polyDataList.append(shallowCopy(polyData))

    return polyDataList


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


def readVrml(filename):
    '''
    Returns list of vtkPolyData meshes and a list of colors as 3-tuples
    '''
    l = vtk.vtkVRMLImporter()
    l.SetFileName(filename)
    l.Read()
    w = l.GetRenderWindow()
    ren = w.GetRenderers().GetItemAsObject(0)
    actors = ren.GetActors()
    actors = [actors.GetItemAsObject(i) for i in xrange(actors.GetNumberOfItems())]
    meshes = [a.GetMapper().GetInput() for a in actors]
    colors = [ac.GetProperty().GetColor() for ac in actors]
    return meshes, colors


def writePolyData(polyData, filename):

    ext = os.path.splitext(filename)[1].lower()

    writers = {
            '.vtp' : vtk.vtkXMLPolyDataWriter,
            '.vtk' : vtk.vtkPolyDataWriter,
            '.ply' : vtk.vtkPLYWriter,
            '.stl' : vtk.vtkSTLWriter,
              }

    if ext not in writers:
        raise Exception('Unknown file extension in writePolyData: %s' % filename)

    writer = writers[ext]()

    if ext in ('.ply', '.stl'):
        polyData = _triangulate(polyData)
        writer.SetFileTypeToASCII()

    if ext in ('.ply'):
        if polyData.GetPointData().GetArray('RGB255'):
            writer.SetArrayName('RGB255')

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
