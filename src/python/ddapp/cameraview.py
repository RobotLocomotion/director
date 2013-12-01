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
        self.initEventFilter()


    def initImageQueue(self):

        imageNames = [
            'CAMERA_LEFT',
            'CAMERACHEST_LEFT',
            'CAMERACHEST_RIGHT',
            ]

        self.images = {}
        self.transforms = {}
        self.imageUtimes = {}
        self.textures = {}

        for name in imageNames:

            image = vtk.vtkImageData()
            tex = vtk.vtkTexture()
            tex.SetInput(image)
            tex.EdgeClampOn()
            tex.RepeatOff()

            self.imageUtimes[name] = 0
            self.images[name] = image
            self.textures[name] = tex
            self.transforms[name] = vtk.vtkTransform()

        self.queue = PythonQt.dd.ddBotImageQueue(lcmUtils.getGlobalLCMThread())
        self.queue.init(lcmUtils.getGlobalLCMThread())


    def onViewDoubleClicked(self, displayPoint):

        obj, pickedPoint = vis.findPickedObject(displayPoint, view=self.view)

        if pickedPoint is None or not obj:
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

    def initEventFilter(self):
        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        qvtkwidget = self.view.vtkWidget()
        qvtkwidget.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonDblClick)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.filterEvent)

    def initView(self):
        self.view = app.getViewManager().createView('Camera View', 'VTK View')

        self.view.camera().SetViewAngle(90)
        self.view.camera().SetPosition(-7.5, 0.0, 5.0)
        self.view.camera().SetFocalPoint(0.0, 0.0, 0.0)
        self.view.camera().SetViewUp(0.0, 0.0, 1.0)

        self.sphereObjects = {}

        app.toggleCameraTerrainMode(self.view)

        self.timerCallback = TimerCallback()
        self.timerCallback.targetFps = 60
        self.timerCallback.callback = self.updateView
        self.timerCallback.start()


    def getSphereGeometry(self, imageName):

        sphereObj = self.sphereObjects.get(imageName)
        if sphereObj:
            return sphereObj

        if not self.images[imageName].GetDimensions()[0]:
            return None

        sphereResolution = 50
        sphereRadii = {
                'CAMERA_LEFT' : 9.90,
                'CAMERACHEST_LEFT' : 9.95,
                'CAMERACHEST_RIGHT' : 10
                }

        geometry = makeSphere(sphereRadii[imageName], sphereResolution)
        self.queue.computeTextureCoords(imageName, geometry)

        tcoordsArrayName = 'tcoords_%s' % imageName
        vtkNumpy.addNumpyToVtk(geometry, vtkNumpy.getNumpyFromVtk(geometry, tcoordsArrayName)[:,0].copy(), 'tcoords_U')
        vtkNumpy.addNumpyToVtk(geometry, vtkNumpy.getNumpyFromVtk(geometry, tcoordsArrayName)[:,1].copy(), 'tcoords_V')
        geometry = clipRange(geometry, 'tcoords_U', [0.0, 1.0])
        geometry = clipRange(geometry, 'tcoords_V', [0.0, 1.0])
        geometry.GetPointData().SetTCoords(geometry.GetPointData().GetArray(tcoordsArrayName))

        sphereObj = vis.showPolyData(geometry, imageName, view=self.view, parent='cameras')
        sphereObj.actor.SetTexture(self.textures[imageName])
        sphereObj.actor.GetProperty().LightingOff()

        self.sphereObjects[imageName] = sphereObj
        return sphereObj


    def updateSphereGeometry(self):

        for imageName, transform in self.transforms.iteritems():
            sphereObj = self.getSphereGeometry(imageName)
            if not sphereObj:
                continue

            self.queue.getBodyToCameraTransform(imageName, transform)
            sphereObj.actor.SetUserTransform(transform.GetLinearInverse())

        return True


    def writeImage(self, imageName, outFile):
        writer = vtk.vtkPNGWriter()
        writer.SetInput(self.images[imageName])
        writer.SetFileName(outFile)
        writer.Write()


    def updateImages(self):
        updated = False
        for imageName, image in self.images.iteritems():
            imageUtime = self.queue.getCurrentImageTime(imageName)
            if imageUtime != self.imageUtimes.get(imageName, 0):
                self.imageUtimes[imageName] = self.queue.getImage(imageName, image)
                updated = True
        return updated


    def updateView(self):

        if not self.updateImages():
            return

        if not self.view.isVisible():
            return

        self.updateSphereGeometry()
        self.view.render()



class CameraImageView(object):

    def __init__(self, imageManager, imageName='CAMERA_LEFT', viewName='Head camera'):

        self.imageManager = imageManager
        self.viewName = viewName
        self.imageName = imageName
        self.imageInitialized = False
        self.updateUtime = 0
        self.initView()
        self.initEventFilter()


    def onViewDoubleClicked(self, displayPoint):

        obj, pickedPoint = vis.findPickedObject(displayPoint, view=self.view)

        if pickedPoint is None or not obj:
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
        self.view = app.getViewManager().createView(self.viewName, 'VTK View')

        self.imageActor = vtk.vtkImageActor()
        self.imageActor.SetInput(self.imageManager.images[self.imageName])
        self.view.renderer().AddActor(self.imageActor)

        self.timerCallback = TimerCallback()
        self.timerCallback.targetFps = 60
        self.timerCallback.callback = self.updateView
        self.timerCallback.start()


    def initEventFilter(self):
        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        qvtkwidget = self.view.vtkWidget()
        qvtkwidget.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonDblClick)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.filterEvent)

    def resetCamera(self):
        camera = self.view.camera()
        camera.ParallelProjectionOn()
        camera.SetFocalPoint(0,0,0)
        camera.SetPosition(0,0,-1)
        camera.SetViewUp(0,-1, 0)
        self.view.resetCamera()

    def updateView(self):

        if not self.view.isVisible():
            return

        currentUtime = self.imageManager.imageUtimes[self.imageName]
        if currentUtime != self.updateUtime:
            self.view.render()
            self.updateUtime = currentUtime

            if not self.imageInitialized and self.imageActor.GetInput().GetDimensions()[0]:
                self.resetCamera()
                self.imageInitialized = True


def init():
    global cameraView
    cameraView = CameraView()

    global headView, chestLeft, chestRight
    headView = CameraImageView(cameraView)
    chestLeft = CameraImageView(cameraView, 'CAMERACHEST_LEFT', 'Chest left')
    chestRight = CameraImageView(cameraView, 'CAMERACHEST_RIGHT', 'Chest right')

