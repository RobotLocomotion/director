import os
import sys
import director.vtkAll as vtk
import director.vtkNumpy as vnp
from director import ioUtils
from director import filterUtils
from director import transformUtils
from director import objectmodel as om
from director.shallowCopy import shallowCopy
from collections import OrderedDict
import pyassimp
import numpy as np


class AssimpScene(object):

    def __init__(self, scene, baseDir=''):
        self.scene = scene
        self.baseDir = baseDir
        self.meshToPolyData = {}
        self.textureCache = {}

    def getPolyDataForMesh(self, mesh):
        meshIndex = self.scene.meshes.index(mesh)
        if meshIndex not in self.meshToPolyData:
            self.meshToPolyData[meshIndex] = assimpMeshToPolyData(mesh)
        return self.meshToPolyData[meshIndex]

    def getTextureForMaterial(self, material):

        materialDict = dict(material.properties.items())

        if 'file' not in materialDict:
            return None

        textureFile = materialDict['file']

        if textureFile in self.textureCache:
            return self.textureCache[textureFile]

        if not os.path.isabs(textureFile):
            imageFile = os.path.join(self.baseDir, textureFile)
        else:
            imageFile = textureFile

        if os.path.isfile(imageFile):
            image = ioUtils.readImage(imageFile)

            if image:
                texture = vtk.vtkTexture()
                texture.SetInput(image)
                texture.EdgeClampOn()
                texture.RepeatOn()
            else:
                print 'failed to load image file:', imageFile
                texture = None
        else:
            print 'cannot find texture file:', textureFile
            texture = None

        self.textureCache[textureFile] = texture
        return texture

    def addToView(self, view):

        rootT = transformUtils.getTransformFromNumpy(self.scene.rootnode.transformation)

        for node in self.scene.rootnode.children:

            t = transformUtils.getTransformFromNumpy(node.transformation)
            folder = om.addContainer(node.name)
            for i, mesh in enumerate(node.meshes):

                pd = self.getPolyDataForMesh(mesh)
                obj = vis.showPolyData(pd, '%s (mesh %d)' % (node.name, i), parent=folder, view=view)
                obj.actor.SetTexture(self.getTextureForMaterial(mesh.material))
                obj.actor.SetUserTransform(t)


def addMaterialMetaData(polyData, material):

    materialDict = dict(material.properties.items())
    textureFile = materialDict['file']

    textureFileAbsPath = os.path.join(meshDir, textureFile)
    if not os.path.isfile(textureFileAbsPath):
        print 'warning, cannot find texture file:', os.path.abspath(textureFileAbsPath)

    s = vtk.vtkStringArray()
    s.InsertNextValue(textureFile)
    s.SetName('texture_filename')
    polyData.GetFieldData().AddArray(s)


def assimpTextureCoordsToArray(tcoords):

    if tcoords.shape[1] >= 3 and not tcoords[:,2].any():
        tcoords = tcoords[:,:2].copy()

    tcoordArray = vnp.getVtkFromNumpy(tcoords)
    tcoordArray.SetName('tcoords')
    return tcoordArray


def assimpMeshToPolyData(mesh):

    verts = mesh.vertices
    faces = mesh.faces

    nfaces = faces.shape[0]
    nverts = verts.shape[0]

    assert verts.shape[1] == 3
    assert faces.shape[1] == 3

    points = vnp.getVtkPointsFromNumpy(verts)

    cells = vtk.vtkCellArray()

    for i in xrange(nfaces):
        face = faces[i]
        tri = vtk.vtkTriangle()
        tri.GetPointIds().SetId(0, face[0])
        tri.GetPointIds().SetId(1, face[1])
        tri.GetPointIds().SetId(2, face[2])
        cells.InsertNextCell(tri)

    polyData = vtk.vtkPolyData()
    polyData.SetPoints(points)
    polyData.SetPolys(cells)


    if mesh.normals.shape[0] > 0:
        assert mesh.normals.shape[0] == nverts
        normals = vnp.getVtkFromNumpy(mesh.normals)
        normals.SetName('normals')
        polyData.GetPointData().AddArray(normals)
        polyData.GetPointData().SetNormals(normals)


    for i, tcoords in enumerate(mesh.texturecoords):
        tcoordArray = assimpTextureCoordsToArray(tcoords)
        tcoordArray.SetName('tcoords_%d' % i)
        polyData.GetPointData().AddArray(tcoordArray)
        if i == 0:
            polyData.GetPointData().SetTCoords(tcoordArray)

    return polyData


def assimpTransformToVtk(node):

    assert node.matrix.shape == (4,4)

    m = vtk.vtkMatrix4x4()

    for r in xrange(4):
        for c in xrange(4):
            m.SetElement(r, c, node.matrix[r][c])

    t = vtk.vtkTransform()
    t.SetMatrix(m)
    return t


def assimpMeshesToPolyData(meshes):
    return [assimpMeshToPolyData(mesh) for mesh in meshes]


def assimpSceneToPolyData(scene):
    return assimpMeshesToPolyData(scene.meshes)


def loadAssimpSceneFromFile(inFile):
    baseDir = os.path.dirname(inFile)
    scene = pyassimp.load(inFile, pyassimp.postprocess.aiProcess_Triangulate)
    return AssimpScene(scene, baseDir)


def writeMultiBlockPolyData(polyDataList, outFile):

    mb = vtk.vtkMultiBlockDataSet()
    mb.SetNumberOfBlocks(len(polyDataList))

    for i, polyData in enumerate(polyDataList):
        mb.SetBlock(i, polyData)

    writer = vtk.vtkXMLMultiBlockDataWriter()
    writer.SetFileName(outFile)
    writer.SetInput(mb)
    writer.Write()


if __name__ == '__main__':

    if len(sys.argv) < 2:
        print 'Usage: %s <mesh file>' % sys.argv[0]
        sys.exit(1)


    scene = loadAssimpSceneFromFile(sys.argv[1])

    from director import mainwindowapp
    import director.visualization as vis

    app = mainwindowapp.MainWindowAppFactory().construct()
    scene.addToView(app.view)
    app.app.start()
