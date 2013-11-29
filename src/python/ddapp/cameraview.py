import ddapp.applogic as app

from ddapp import lcmUtils
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp.shallowCopy import shallowCopy
from ddapp.timercallback import TimerCallback
from ddapp import vtkNumpy
import ddapp.vtkAll as vtk

import PythonQt
from PythonQt import QtCore, QtGui
import bot_core as lcmbotcore
import numpy as np
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


def colorizePoints(polyData, cameraName='CAMERA_LEFT'):
    cameraView.queue.colorizePoints(cameraName, polyData)


def testColorize():

    radius = 10
    resolution = 400
    s = makeSphere(radius, resolution)
    cameraView.queue.colorizePoints(s)
    showPolyData(p, 'sphere', colorByName='rgb')


class CameraView(object):

    def __init__(self):

        self.initImageQueue()

        self.initView()
        #self.initView2()

        self.initEventFilter()

    def initImageQueue(self):

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

        self.imageUtimes = {}

        self.textures = {}
        for name, image in self.images.iteritems():
            tex = vtk.vtkTexture()
            tex.SetInput(image)
            tex.EdgeClampOn()
            tex.RepeatOff()
            self.textures[name] = tex

        self.queue = PythonQt.dd.ddBotImageQueue(lcmUtils.getGlobalLCMThread())
        self.queue.init(lcmUtils.getGlobalLCMThread())



    def onViewDoubleClicked(self, displayPoint):

        obj, pickedPoint = vis.findPickedObject(displayPoint, view=self.view)

        if pickedPoint is None:
            return

        imageName = obj.getProperty('Name')
        imageUtime = self.imageUtimes[imageName]

        cameraToLocal = vtk.vtkTransform()
        self.queue.getTransform(imageName, 'local', imageUtime, cameraToLocal)

        utorsoToLocal = vtk.vtkTransform()
        self.queue.getTransform('utorso', 'local', imageUtime, utorsoToLocal)

        drcView = app.getViewManager().findView('DRC View')

        p = range(3)
        utorsoToLocal.TransformPoint(pickedPoint, p)
        d = vis.DebugData()
        d.addLine(cameraToLocal.GetPosition(), p)
        vis.updatePolyData(d.getPolyData(), 'camera ray', view=drcView)


    def filterEvent(self, obj, event):
        if event.type() == QtCore.QEvent.MouseButtonDblClick:
            self.eventFilter.setEventHandlerResult(True)
            self.onViewDoubleClicked(vis.mapMousePosition(obj, event))


    def initView(self):
        self.view = app.getViewManager().createView('Image View', 'VTK View')
        app.getViewManager().switchToView('Image View')

        self.view.camera().SetViewAngle(90)
        self.view.camera().SetPosition(-7.5, 0.0, 5.0)
        self.view.camera().SetFocalPoint(0.0, 0.0, 0.0)
        self.view.camera().SetViewUp(0.0, 0.0, 1.0)

        self.sphereObjects = None

        app.toggleCameraTerrainMode()

        self.timerCallback = TimerCallback()
        self.timerCallback.targetFps = 30
        self.timerCallback.callback = self.updateImageView
        self.timerCallback.start()


    def initView2(self):
        self.view = app.getViewManager().createView('Image View', 'VTK View')

        self.imageActor = vtk.vtkImageActor()
        self.imageActor.SetInput(self.images['CAMERA_LEFT'])
        self.view.renderer().AddActor(self.imageActor)
        self.view.camera().SetFocalPoint(0,0,0)
        self.view.camera().SetPosition(0,0,-1)
        self.view.camera().SetViewUp(0,-1, 0)

        app.getViewManager().switchToView('Image View')

        self.timerCallback = TimerCallback()
        self.timerCallback.targetFps = 30
        self.timerCallback.callback = self.updateImageView2
        self.timerCallback.start()


    def initEventFilter(self):
        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        qvtkwidget = self.view.vtkWidget()
        qvtkwidget.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonDblClick)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.filterEvent)


    def initSphereGeometry(self):

        if self.sphereObjects:
            return True

        for image in self.images.values():
            if not image.GetDimensions()[0]:
                return False

        sphereResolution = 50
        sphereRadii = [9.95, 9.95, 10]
        imageNames = ['CAMERA_LEFT', 'CAMERACHEST_LEFT', 'CAMERACHEST_RIGHT']
        self.sphereObjects = []

        for imageName, sphereRadius in zip(imageNames, sphereRadii):

            geometry = makeSphere(sphereRadius, sphereResolution)
            self.queue.computeTextureCoords(imageName, geometry)

            tcoordsArrayName = 'tcoords_%s' % imageName
            vtkNumpy.addNumpyToVtk(geometry, vtkNumpy.getNumpyFromVtk(geometry, tcoordsArrayName)[:,0].copy(), 'tcoords_U')
            vtkNumpy.addNumpyToVtk(geometry, vtkNumpy.getNumpyFromVtk(geometry, tcoordsArrayName)[:,1].copy(), 'tcoords_V')
            geometry = clipRange(geometry, 'tcoords_U', [0.0, 1.0])
            geometry = clipRange(geometry, 'tcoords_V', [0.0, 1.0])
            geometry.GetPointData().SetTCoords(geometry.GetPointData().GetArray(tcoordsArrayName))

            sphereObj = vis.showPolyData(geometry, imageName, view=self.view)
            sphereObj.actor.SetTexture(self.textures[imageName])
            sphereObj.actor.GetProperty().LightingOff()

            self.sphereObjects.append(sphereObj)


        # not used: example code for multitexturing
        #renwin = self.view.renderWindow()
        #hardware = renwin.GetHardwareSupport()
        #self.textures['CAMERA_LEFT'].SetBlendingMode(vtk.vtkTexture.VTK_TEXTURE_BLENDING_MODE_REPLACE)
        #self.textures['CAMERACHEST_LEFT'].SetBlendingMode(vtk.vtkTexture.VTK_TEXTURE_BLENDING_MODE_ADD)
        #self.textures['CAMERACHEST_RIGHT'].SetBlendingMode(vtk.vtkTexture.VTK_TEXTURE_BLENDING_MODE_ADD)
        #obj.mapper.MapDataArrayToMultiTextureAttribute(vtk.vtkProperty.VTK_TEXTURE_UNIT_0, 'tcoords_CAMERA_LEFT', vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS)
        #obj.mapper.MapDataArrayToMultiTextureAttribute(vtk.vtkProperty.VTK_TEXTURE_UNIT_1, 'tcoords_CAMERACHEST_LEFT', vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS)
        #obj.mapper.MapDataArrayToMultiTextureAttribute(vtk.vtkProperty.VTK_TEXTURE_UNIT_2, 'tcoords_CAMERACHEST_RIGHT', vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS)
        #obj.actor.GetProperty().SetTexture(vtk.vtkProperty.VTK_TEXTURE_UNIT_0, self.textures['CAMERA_LEFT'])
        #obj.actor.GetProperty().SetTexture(vtk.vtkProperty.VTK_TEXTURE_UNIT_1, self.textures['CAMERACHEST_LEFT'])
        #obj.actor.GetProperty().SetTexture(vtk.vtkProperty.VTK_TEXTURE_UNIT_2, self.textures['CAMERACHEST_RIGHT'])

        return True


    def writeImage(self, imageName, outFile):
        writer = vtk.vtkPNGWriter()
        writer.SetInput(self.images[imageName])
        writer.SetFileName(outFile)
        writer.Write()


    def updateImages(self):
        for imageName, image in self.images.iteritems():
            imageUtime = self.queue.getCurrentImageTime(imageName)
            if imageUtime != self.imageUtimes.get(imageName, 0):
                self.imageUtimes[imageName] = self.queue.getImage(imageName, image)


    def updateImageView(self):

        self.updateImages()

        if not self.initSphereGeometry():
            return

        imageNames = ['CAMERA_LEFT', 'CAMERACHEST_LEFT', 'CAMERACHEST_RIGHT']

        for sphereObj, imageName in zip(self.sphereObjects, imageNames):
            transform = self.transforms[imageName]
            self.queue.getBodyToCameraTransform(imageName, transform)
            sphereObj.actor.SetUserTransform(transform.GetLinearInverse())

        self.view.render()


    def updateImageView2(self):

        self.updateImages()
        self.view.render()


def init():
    global cameraView
    cameraView = CameraView()

