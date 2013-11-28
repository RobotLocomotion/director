import ddapp.applogic as app

from ddapp import lcmUtils
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp.shallowCopy import shallowCopy
from ddapp.timercallback import TimerCallback
from ddapp import vtkNumpy
import ddapp.vtkAll as vtk

import PythonQt
import bot_core as lcmbotcore

from ddapp.simpletimer import SimpleTimer

from ddapp import ioUtils
import sys


def clipRange(dataObj, arrayName, thresholdRange):
    if not dataObj.GetPointData().GetArray(arrayName):
        raise Exception('clipRange: could not locate array: %s' % arrayName)

    dataObj.GetPointData().SetScalars(dataObj.GetPointData().GetArray(arrayName))

    f = vtk.vtkClipPolyData()
    f.SetInput(dataObj)
    f.SetValue(thresholdRange[0])
    f.SetInputArrayToProcess(0, 0, 0, vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS, arrayName)

    f2 = vtk.vtkClipPolyData()
    f2.AddInputConnection(f.GetOutputPort())
    f2.SetValue(thresholdRange[1])
    f2.InsideOutOn()
    f2.SetInputArrayToProcess(0, 0, 0, vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS, arrayName)

    f2.Update()
    return shallowCopy(f2.GetOutput())



def makeSphere(radius, resolution):

    s = vtk.vtkSphereSource()
    s.SetThetaResolution(resolution)
    s.SetPhiResolution(resolution)
    s.SetRadius(radius)
    s.SetEndPhi(85)
    s.Update()
    return shallowCopy(s.GetOutput())


def colorizeLidar(polyData):
    cameraView.queue.colorizeLidar(polyData)


def testColorize():

    radius = 10
    resolution = 400
    s = makeSphere(radius, resolution)
    cameraView.queue.colorizeLidar(s)
    showPolyData(p, 'sphere', colorByName='rgb')



class CameraView(TimerCallback):

    def __init__(self):
        TimerCallback.__init__(self)
        self.view = app.getViewManager().createView('Image View', 'VTK View')

        app.getViewManager().switchToView('Image View')


        self.view.camera().SetViewAngle(90)
        self.view.camera().SetPosition(-7.5, 0.0, 5.0)
        self.view.camera().SetFocalPoint(0.0, 0.0, 0.0)
        self.view.camera().SetViewUp(0.0, 0.0, 1.0)

        app.toggleCameraTerrainMode()



        self.queue = PythonQt.dd.ddBotImageQueue(lcmUtils.getGlobalLCMThread())
        self.queue.init(lcmUtils.getGlobalLCMThread())

        self.targetFps = 30

        self.images = {
            'CAMERA_LEFT' : vtk.vtkImageData(),
            'CAMERACHEST_LEFT' : vtk.vtkImageData(),
            'CAMERACHEST_RIGHT' : vtk.vtkImageData(),
          }

        self.transforms = {
            'CAMERA_LEFT' : vtk.vtkTransform(),
            'CAMERACHEST_LEFT' : vtk.vtkTransform(),
            'CAMERACHEST_RIGHT' : vtk.vtkTransform(),
          }

        self.chestSphere = None
        self.headSphere = None


        '''
        self.imageActor = vtk.vtkImageActor()
        self.image = vtk.vtkImageData()
        self.imageActor.SetInput(self.image)
        self.view.renderer().AddActor(self.imageActor)
        self.view.camera().SetFocalPoint(0,0,0)
        self.view.camera().SetPosition(0,0,-1)
        self.view.camera().SetViewUp(0,-1, 0)
        '''


    def initGeometry(self):

        if self.headSphere:
            return True

        for image in self.images.values():
            if not image.GetDimensions()[0]:
                return False

        self.chestSphereL = makeSphere(10, 50)
        self.chestSphereR = makeSphere(9.9, 50)
        self.headSphere = makeSphere(9.8, 50)
        self.queue.computeTextureCoords(self.headSphere)
        self.queue.computeTextureCoords(self.chestSphereL)
        self.queue.computeTextureCoords(self.chestSphereR)


        vtkNumpy.addNumpyToVtk(self.headSphere, vtkNumpy.getNumpyFromVtk(self.headSphere, 'tcoords_CAMERA_LEFT')[:,0].copy(), 'tcoords_U')
        vtkNumpy.addNumpyToVtk(self.headSphere, vtkNumpy.getNumpyFromVtk(self.headSphere, 'tcoords_CAMERA_LEFT')[:,1].copy(), 'tcoords_V')


        vtkNumpy.addNumpyToVtk(self.chestSphereL, vtkNumpy.getNumpyFromVtk(self.chestSphereL, 'tcoords_CAMERACHEST_LEFT')[:,0].copy(), 'tcoords_U')
        vtkNumpy.addNumpyToVtk(self.chestSphereL, vtkNumpy.getNumpyFromVtk(self.chestSphereL, 'tcoords_CAMERACHEST_LEFT')[:,1].copy(), 'tcoords_V')


        vtkNumpy.addNumpyToVtk(self.chestSphereR, vtkNumpy.getNumpyFromVtk(self.chestSphereR, 'tcoords_CAMERACHEST_RIGHT')[:,0].copy(), 'tcoords_U')
        vtkNumpy.addNumpyToVtk(self.chestSphereR, vtkNumpy.getNumpyFromVtk(self.chestSphereR, 'tcoords_CAMERACHEST_RIGHT')[:,1].copy(), 'tcoords_V')


        self.headSphere = clipRange(self.headSphere, 'tcoords_U', [0.0, 1.0])
        self.headSphere = clipRange(self.headSphere, 'tcoords_V', [0.0, 1.0])

        chestRange = [0.0, 1.0]

        self.chestSphereL = clipRange(self.chestSphereL, 'tcoords_U', chestRange)
        self.chestSphereL = clipRange(self.chestSphereL, 'tcoords_V', chestRange)

        self.chestSphereR = clipRange(self.chestSphereR, 'tcoords_U', chestRange)
        self.chestSphereR = clipRange(self.chestSphereR, 'tcoords_V', chestRange)

        self.headSphere.GetPointData().SetTCoords(self.headSphere.GetPointData().GetArray('tcoords_CAMERA_LEFT'))
        self.headObj = vis.showPolyData(self.headSphere, 'camera - HEAD')

        self.chestSphereL.GetPointData().SetTCoords(self.chestSphereL.GetPointData().GetArray('tcoords_CAMERACHEST_LEFT'))
        self.chestObjL = vis.showPolyData(self.chestSphereL, 'camera - CHESTLEFT')

        self.chestSphereR.GetPointData().SetTCoords(self.chestSphereR.GetPointData().GetArray('tcoords_CAMERACHEST_RIGHT'))
        self.chestObjR = vis.showPolyData(self.chestSphereR, 'camera - CHESTRIGHT')


        #self.chestObj = vis.showPolyData(self.chestSphere, 'camera - SA')

        self.textures = {}
        for name, image in self.images.iteritems():
            tex = vtk.vtkTexture()
            self.textures[name] = tex
            tex.SetInput(image)
            tex.EdgeClampOn()
            tex.RepeatOff()

        renwin = self.view.renderWindow()
        hardware = renwin.GetHardwareSupport()

        #self.textures['CAMERA_LEFT'].SetBlendingMode(vtk.vtkTexture.VTK_TEXTURE_BLENDING_MODE_REPLACE)
        #self.textures['CAMERACHEST_LEFT'].SetBlendingMode(vtk.vtkTexture.VTK_TEXTURE_BLENDING_MODE_REPLACE)
        #self.textures['CAMERACHEST_RIGHT'].SetBlendingMode(vtk.vtkTexture.VTK_TEXTURE_BLENDING_MODE_ADD)

        #self.headObj.mapper.MapDataArrayToMultiTextureAttribute(vtk.vtkProperty.VTK_TEXTURE_UNIT_0, 'tcoords_CAMERA_LEFT', vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS)
        #self.chestObj.mapper.MapDataArrayToMultiTextureAttribute(vtk.vtkProperty.VTK_TEXTURE_UNIT_1, 'tcoords_CAMERACHEST_LEFT', vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS)
        #self.chestObj.mapper.MapDataArrayToMultiTextureAttribute(vtk.vtkProperty.VTK_TEXTURE_UNIT_2, 'tcoords_CAMERACHEST_RIGHT', vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS)

        self.headObj.actor.SetTexture(self.textures['CAMERA_LEFT'])
        self.chestObjL.actor.SetTexture(self.textures['CAMERACHEST_LEFT'])
        self.chestObjR.actor.SetTexture(self.textures['CAMERACHEST_RIGHT'])
        #self.headObj.actor.GetProperty().SetTexture(vtk.vtkProperty.VTK_TEXTURE_UNIT_0, self.textures['CAMERA_LEFT'])
        #self.chestObj.actor.GetProperty().SetTexture(vtk.vtkProperty.VTK_TEXTURE_UNIT_1, self.textures['CAMERACHEST_LEFT'])
        #self.chestObj.actor.GetProperty().SetTexture(vtk.vtkProperty.VTK_TEXTURE_UNIT_2, self.textures['CAMERACHEST_RIGHT'])


        self.headObj.actor.GetProperty().LightingOff()
        self.chestObjL.actor.GetProperty().LightingOff()
        self.chestObjR.actor.GetProperty().LightingOff()

        #ioUtils.writePolyData(self.chestSphereL, 'CAMERACHEST_LEFT.vtp')
        #ioUtils.writePolyData(self.chestSphereR, 'CAMERACHEST_RIGHT.vtp')
        #ioUtils.writePolyData(self.headSphere, 'CAMERA_LEFT.vtp')
        #sys.exit(1)

        return True


    def tick(self):


        for imageName, image in self.images.iteritems():
            self.queue.getImage(imageName, image)
            #writer = vtk.vtkPNGWriter()
            #writer.SetInput(image)
            #writer.SetFileName(imageName + '.png')
            #writer.Write()


        if not self.initGeometry():
            return

        for imageName, transform in self.transforms.iteritems():
            self.queue.getBodyToCameraTransform(imageName, transform)
            if imageName == 'CAMERA_LEFT':
                self.headObj.actor.SetUserTransform(transform.GetLinearInverse())
            if imageName == 'CAMERACHEST_LEFT':
                self.chestObjL.actor.SetUserTransform(transform.GetLinearInverse())
            if imageName == 'CAMERACHEST_RIGHT':
                self.chestObjR.actor.SetUserTransform(transform.GetLinearInverse())

        self.view.render()


def init():
    global cameraView
    cameraView = CameraView()
    cameraView.start()

