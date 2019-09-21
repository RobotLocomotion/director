import os
import director.vtkAll as vtk
from director.shallowCopy import shallowCopy
import shelve
import os.path

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

    if polyData.GetNumberOfPoints() and not polyData.GetNumberOfCells():
        f = vtk.vtkVertexGlyphFilter()
        f.SetInputData(polyData)
        f.Update()
        polyData = shallowCopy(f.GetOutput())

    if computeNormals:
        return _computeNormals(polyData)
    else:
        return polyData

def readPlyFile(filename):
    """
    Usese plyfile python pacakage to read a ply file.
    Gets around issues with pcl having a bad ply writer for pointclouds
    :param filename:
    :type filename: str
    :return: vtkPolyData
    :rtype:
    """

    from plyfile import PlyData

    plydata = PlyData.read(filename)
    vertex_data = plydata['vertex'].data # numpy array with fields ['x', 'y', 'z']
    pts = np.zeros([vertex_data.size, 3])
    pts[:, 0] = vertex_data['x']
    pts[:, 1] = vertex_data['y']
    pts[:, 2] = vertex_data['z']

    return vnp.numpyToPolyData(pts)


def readMultiBlock(filename):
    '''Reads a .vtm file and returns a list of vtkPolyData objects'''

    reader = vtk.vtkXMLMultiBlockDataReader()
    reader.SetFileName(filename)
    reader.Update()

    polyDataList = []
    mb = reader.GetOutput()
    for i in range(mb.GetNumberOfBlocks()):
        polyData = vtk.vtkPolyData.SafeDownCast(mb.GetBlock(i))
        if polyData and polyData.GetNumberOfPoints():
            polyDataList.append(shallowCopy(polyData))

    return polyDataList


def readImage(filename):

    ext = os.path.splitext(filename)[1].lower()

    readers = {
            '.png' : vtk.vtkPNGReader,
            '.jpg' : vtk.vtkJPEGReader,
            '.vti' : vtk.vtkXMLImageDataReader,
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
    actors = [actors.GetItemAsObject(i) for i in range(actors.GetNumberOfItems())]
    meshes = [a.GetMapper().GetInput() for a in actors]
    colors = [ac.GetProperty().GetColor() for ac in actors]
    return meshes, colors


def readObjMtl(filename):
    '''
    Read an obj file and return a list of vtkPolyData objects.
    If the obj file has an associated material file, this function returns
    (polyDataList, actors).  If there is not a material file, this function
    returns (polyDataList, None).
    '''

    def getMtlFilename(filename, maxLines=1000):
        with open(filename) as f:
            for i, l in enumerate(f):
                if l.startswith('mtllib'):
                    tokens = l.split()
                    if len(tokens) < 2:
                        raise Exception('Error parsing mtllib line in file: %s\n%s' % (filename, l))
                    return os.path.join(os.path.dirname(filename), tokens[1])

    mtlFilename = getMtlFilename(filename)

    l = vtk.vtkOBJImporter()
    l.SetFileName(filename)
    if mtlFilename:
        l.SetFileNameMTL(mtlFilename)
    l.SetTexturePath(os.path.dirname(filename))
    l.Read()
    w = l.GetRenderWindow()
    ren = w.GetRenderers().GetItemAsObject(0)
    actors = ren.GetActors()
    actors = [actors.GetItemAsObject(i) for i in range(actors.GetNumberOfItems())]
    meshes = [a.GetMapper().GetInput() for a in actors]

    if mtlFilename:
        return (meshes, actors)
    else:
        return (meshes, None)


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
    writer.SetInputData(polyData)
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
    writer.SetInputData(image)
    writer.Write()

def _computeNormals(polyData):
    normals = vtk.vtkPolyDataNormals()
    normals.SetFeatureAngle(45)
    normals.SetInputData(polyData)
    normals.Update()
    return shallowCopy(normals.GetOutput())

def _triangulate(polyData):
    normals = vtk.vtkTriangleFilter()
    normals.SetInputData(polyData)
    normals.Update()
    return shallowCopy(normals.GetOutput())

def saveDataToFile(filename, dataDict, overwrite=False):
    if overwrite is False and os.path.isfile(filename):
        raise ValueError("file already exists, overwrite option was False")

    myShelf = shelve.open(filename,'n')
    myShelf['dataDict'] = dataDict
    myShelf.close()

def readDataFromFile(filename):
    myShelf = shelve.open(filename)
    dataDict = myShelf['dataDict']
    return dataDict
