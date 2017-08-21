import director.visualization as vis
from director import filterUtils
import director.vtkAll as vtk
import director.vtkNumpy as vnp
from director.shallowCopy import shallowCopy
from director import ioUtils
import numpy as np


def createTexturedPlane():
    source = vtk.vtkPlaneSource()
    textureMap = vtk.vtkTextureMapToPlane()
    textureMap.SetInput(source.GetOutput())

    textureMap.Update()
    return shallowCopy(textureMap.GetOutput())


def getSkyboxSides():
    return ['top', 'bottom', 'front', 'back', 'left', 'right']


def createSkyboxPlane(side):

    pd = createTexturedPlane()

    t = vtk.vtkTransform()
    t.PostMultiply()

    if side == 'top':
        t.Translate(0,0,0.5)
        t.RotateZ(180)

    elif side == 'bottom':
        t.RotateX(180)
        t.RotateY(180)
        t.RotateZ(-270)
        t.Translate(0,0,-0.5)

    elif side == 'front':
        t.RotateY(90)
        t.RotateX(90)
        t.RotateZ(180)
        t.Translate(0.5,0.0,0.0)

    elif side == 'back':
        t.RotateY(90)
        t.RotateX(90)
        t.RotateZ(0)
        t.Translate(-0.5,0.0,0.0)

    elif side == 'left':
        t.RotateY(90)
        t.RotateX(90)
        t.RotateZ(-90)
        t.Translate(0.0,0.5,0.0)

    elif side == 'right':
        t.RotateY(90)
        t.RotateX(90)
        t.RotateZ(90)
        t.Translate(0.0,-0.5,0.0)

    pd = filterUtils.transformPolyData(pd, t)
    return pd


def createSkyboxPlanes():
    planes = {}
    for side in getSkyboxSides():
        planes[side] = createSkyboxPlane(side)
    return planes


def createTexture(imageFilename):
    image = ioUtils.readImage(imageFilename)
    tex = vtk.vtkTexture()
    tex.SetInput(image)
    tex.EdgeClampOn()
    tex.RepeatOff()
    return tex


def createSkybox(imageMap, view):

    objs = {}
    planes = createSkyboxPlanes()

    for side, imageFilename in imageMap.items():
        texture = createTexture(imageFilename)
        obj = vis.PolyDataItem('skybox %s' % side, planes[side], view=None)
        obj.actor.SetTexture(texture)
        obj.actor.GetProperty().LightingOff()
        view.backgroundRenderer().AddActor(obj.actor)
        objs[side] = obj

    return objs


def getSkyboxImages(baseDir):
    imageMap =  dict(
        top    = baseDir + '/topmars1.jpg',
        bottom = baseDir + '/botmars1.jpg',
        front  = baseDir + '/frontmars1.jpg',
        back   = baseDir + '/backmars1.jpg',
        left   = baseDir + '/leftmars1.jpg',
        right  = baseDir + '/rightmars1.jpg')

    return imageMap


def createTextureGround(imageFilename, view):

    pd = createTexturedPlane()
    texture = createTexture(imageFilename)
    texture.RepeatOn()

    tcoords = vnp.getNumpyFromVtk(pd, 'Texture Coordinates')
    tcoords *= 60

    t = vtk.vtkTransform()
    t.PostMultiply()
    t.Scale(200,200,200)
    t.Translate(0,0,-0.005)

    pd = filterUtils.transformPolyData(pd, t)

    obj = vis.showPolyData(pd, 'ground', view=view, alpha=1.0, parent='skybox')
    obj.actor.SetTexture(texture)
    obj.actor.GetProperty().LightingOff()


def connectSkyboxCamera(view, debug=False):

    baseRen = view.backgroundRenderer()

    def updateSkyboxCamera(o, e):
        c = baseRen.GetActiveCamera()
        c2 = view.camera()

        viewDirection = np.array(c2.GetFocalPoint()) - np.array(c2.GetPosition())
        viewDirection /= np.linalg.norm(viewDirection)

        if debug:
            c.SetPosition(c2.GetPosition())
            c.SetFocalPoint(c2.GetFocalPoint())

        else:
            c.SetPosition(0,0,0)
            c.SetFocalPoint(viewDirection)

        c.SetViewUp(c2.GetViewUp())
        c.SetViewAngle(c2.GetViewAngle())


    view.renderWindow().AddObserver('StartEvent', updateSkyboxCamera)

