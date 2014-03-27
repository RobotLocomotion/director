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
import drc as lcmdrc


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



def sendFOVRequest(channel, imagePoints):

    channelToImageType = {
        'CAMERA_LEFT' : lcmdrc.data_request_t.CAMERA_IMAGE_HEAD_LEFT,
        'CAMERACHEST_LEFT' : lcmdrc.data_request_t.CAMERA_IMAGE_LCHEST,
        'CAMERACHEST_RIGHT' : lcmdrc.data_request_t.CAMERA_IMAGE_RCHEST,
                         }

    dataRequest = lcmdrc.data_request_t()
    dataRequest.type = channelToImageType[channel]

    message = lcmdrc.subimage_request_t()
    message.data_request = dataRequest

    imagePoints = np.array([[pt[0], pt[1]] for pt in imagePoints])
    minX, maxX = imagePoints[:,0].min(), imagePoints[:,0].max()
    minY, maxY = imagePoints[:,1].min(), imagePoints[:,1].max()
    
    message.x = minX
    message.y = minY
    message.w = maxX - minX
    message.h = maxY - minY

    print message.x, message.y, message.w, message.h

    requestChannel = 'SUBIMAGE_REQUEST'
    lcmUtils.publish(requestChannel, message)


def testColorize():

    radius = 10
    resolution = 400
    s = makeSphere(radius, resolution)
    cameraView.queue.colorizePoints(s)
    showPolyData(p, 'sphere', colorByName='rgb')


def rayDebug(position, ray):
    d = vis.DebugData()
    d.addLine(position, position+ray*5.0)
    drcView = app.getViewManager().findView('DRC View')
    vis.updatePolyData(d.getPolyData(), 'camera ray', view=drcView)


class CameraView(object):

    def __init__(self):

        self.initImageQueue()

        if app.getMainWindow():
            self.initView()
            self.initEventFilter()
            self.rayCallback = rayDebug
        else:
            self.view = None

        self.timerCallback = TimerCallback()
        self.timerCallback.targetFps = 60
        self.timerCallback.callback = self.updateView
        self.timerCallback.start()


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
            self.addImage(name)

        self.queue = PythonQt.dd.ddBotImageQueue(lcmUtils.getGlobalLCMThread())
        self.queue.init(lcmUtils.getGlobalLCMThread())

    def addImage(self, name):
        image = vtk.vtkImageData()
        tex = vtk.vtkTexture()
        tex.SetInput(image)
        tex.EdgeClampOn()
        tex.RepeatOff()

        self.imageUtimes[name] = 0
        self.images[name] = image
        self.textures[name] = tex
        self.transforms[name] = vtk.vtkTransform()

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

        p = range(3)
        utorsoToLocal.TransformPoint(pickedPoint, p)

        ray = np.array(p) - np.array(cameraToLocal.GetPosition())
        ray /= np.linalg.norm(ray)

        if self.rayCallback:
            self.rayCallback(np.array(cameraToLocal.GetPosition()), ray)


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


    def getSphereGeometry(self, imageName):

        sphereObj = self.sphereObjects.get(imageName)
        if sphereObj:
            return sphereObj

        if not self.images[imageName].GetDimensions()[0]:
            return None

        sphereResolution = 50
        sphereRadii = {
                'CAMERA_LEFT' : 9.85,
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

        if not self.view:
            return


        self.updateSphereGeometry()

        if not self.view.isVisible():
            return
        self.view.render()



class CameraImageView(object):

    def __init__(self, imageManager, imageName, viewName=None, view=None):

        self.imageManager = imageManager
        self.viewName = viewName or imageName
        self.imageName = imageName
        self.imageInitialized = False
        self.updateUtime = 0
        self.initView(view)
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
        if self.eventFilterEnabled and event.type() == QtCore.QEvent.MouseButtonDblClick:
            self.eventFilter.setEventHandlerResult(True)
            self.onViewDoubleClicked(vis.mapMousePosition(obj, event))

    def onRubberBandPick(self, obj, event):
        displayPoints = self.interactorStyle.GetStartPosition(), self.interactorStyle.GetEndPosition()
        imagePoints = [vis.pickImage(point, view=self.view)[1] for point in displayPoints]
        sendFOVRequest(self.imageName, imagePoints)

    def initView(self, view):
        self.view = view or app.getViewManager().createView(self.viewName, 'VTK View')
        self.view.installImageInteractor()
        self.interactorStyle = self.view.renderWindow().GetInteractor().GetInteractorStyle()
        self.interactorStyle.AddObserver('SelectionChangedEvent', self.onRubberBandPick)

        self.view.renderWindow().GetInteractor().AddObserver('KeyPressEvent', self.onKeyPress)

        self.imageActor = vtk.vtkImageActor()
        self.imageActor.SetInput(self.imageManager.images[self.imageName])
        self.imageActor.SetVisibility(False)
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
        self.eventFilterEnabled = True

    def resetCamera(self):
        camera = self.view.camera()
        camera.ParallelProjectionOn()
        camera.SetFocalPoint(0,0,0)
        camera.SetPosition(0,0,-1)
        camera.SetViewUp(0,-1, 0)
        self.view.resetCamera()
        self.fitImageToView()


    def onKeyPress(self, obj, event):
        if obj.GetKeyCode() == 'p':
            return
        if obj.GetKeyCode() == 'r':
            self.fitImageToView()


    def fitImageToView(self):

        camera = self.view.camera()
        image = self.imageManager.images[self.imageName]
        imageWidth, imageHeight, _ = image.GetDimensions()

        viewWidth, viewHeight = self.view.renderWindow().GetSize()
        aspectRatio = float(viewWidth)/viewHeight
        parallelScale = max(imageWidth/aspectRatio, imageHeight) / 2.0
        camera.SetParallelScale(parallelScale)


    def setImageName(self, imageName):
        if imageName == self.imageName:
            return

        assert imageName in self.imageManager.images

        self.imageName = imageName
        self.imageInitialized = False
        self.updateUtime = 0
        self.imageActor.SetInput(self.imageManager.images[self.imageName])
        self.imageActor.SetVisibility(False)
        self.view.render()

    def updateView(self):

        if not self.view.isVisible():
            return

        currentUtime = self.imageManager.imageUtimes[self.imageName]
        if currentUtime != self.updateUtime:
            self.view.render()
            self.updateUtime = currentUtime

            if not self.imageInitialized and self.imageActor.GetInput().GetDimensions()[0]:
                self.imageActor.SetVisibility(True)
                self.resetCamera()
                self.imageInitialized = True

views = {}

def addCameraView(channel, name):

    cameraView.queue.addCameraStream(channel)
    cameraView.addImage(channel)
    view = CameraImageView(cameraView, channel, name)
    global views
    views[channel] = view
    return view


def init():
    global cameraView
    cameraView = CameraView()

    global headView, chestLeft, chestRight
    headView = CameraImageView(cameraView, 'CAMERA_LEFT', 'Head camera')
    chestLeft = CameraImageView(cameraView, 'CAMERACHEST_LEFT', 'Chest left')
    chestRight = CameraImageView(cameraView, 'CAMERACHEST_RIGHT', 'Chest right')

    #addCameraView('AFFORDANCE_OVERLAY', 'Affordance overlay')

    useAnaglyph = False
    if useAnaglyph:
        addCameraView('CAMERA_ANAGLYPH', 'Head Anaglyph')

