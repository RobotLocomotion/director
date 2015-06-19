import ddapp.applogic as app
from ddapp import lcmUtils
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp import filterUtils
from ddapp import drcargs
from ddapp.shallowCopy import shallowCopy
from ddapp.timercallback import TimerCallback
from ddapp import vtkNumpy
from ddapp import objectmodel as om
import ddapp.vtkAll as vtk
from ddapp.debugVis import DebugData

import PythonQt
from PythonQt import QtCore, QtGui
import bot_core as lcmbotcore
import numpy as np
from ddapp.simpletimer import SimpleTimer
from ddapp import ioUtils
import sys
import drc as lcmdrc
import multisense as lcmmultisense


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
    imageManager.queue.colorizePoints(cameraName, polyData)



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

    #print message.x, message.y, message.w, message.h

    requestChannel = 'SUBIMAGE_REQUEST'
    lcmUtils.publish(requestChannel, message)


def testColorize():

    radius = 10
    resolution = 400
    s = makeSphere(radius, resolution)
    cameraView.queue.colorizePoints(s)
    showPolyData(p, 'sphere', colorByName='rgb')


def rayDebug(position, ray):
    d = DebugData()
    d.addLine(position, position+ray*5.0)
    drcView = app.getViewManager().findView('DRC View')
    obj = vis.updatePolyData(d.getPolyData(), 'camera ray', view=drcView, color=[0,1,0])
    obj.actor.GetProperty().SetLineWidth(2)


class ImageManager(object):

    def __init__(self):

        self.images = {}
        self.imageUtimes = {}
        self.textures = {}

        self.queue = PythonQt.dd.ddBotImageQueue(lcmUtils.getGlobalLCMThread())
        self.queue.init(lcmUtils.getGlobalLCMThread(), drcargs.args().config_file)


    def addImage(self, name):

        if name in self.images:
            return

        image = vtk.vtkImageData()
        tex = vtk.vtkTexture()
        tex.SetInput(image)
        tex.EdgeClampOn()
        tex.RepeatOff()

        self.imageUtimes[name] = 0
        self.images[name] = image
        self.textures[name] = tex

    def writeImage(self, imageName, outFile):
        writer = vtk.vtkPNGWriter()
        writer.SetInput(self.images[imageName])
        writer.SetFileName(outFile)
        writer.Write()

    def updateImage(self, imageName):
        imageUtime = self.queue.getCurrentImageTime(imageName)
        if imageUtime != self.imageUtimes[imageName]:
            image = self.images[imageName]
            self.imageUtimes[imageName] = self.queue.getImage(imageName, image)
        return imageUtime

    def updateImages(self):
        for imageName in self.images.keys():
            self.updateImage(imageName)

    def hasImage(self, imageName):
        return imageName in self.images

    def getImage(self, imageName):
        return self.images[imageName]

    def getUtime(self, imageName):
        return self.imageUtimes[imageName]

    def getTexture(self, imageName):
        return self.textures[imageName]


def disableCameraTexture(obj):
    obj.actor.SetTexture(None)
    obj.actor.GetProperty().LightingOn()
    obj.actor.GetProperty().SetColor(obj.getProperty('Color'))

def applyCameraTexture(obj, imageManager, imageName='CAMERA_LEFT'):

    imageUtime = imageManager.getUtime(imageName)
    if not imageUtime:
        return

    cameraToLocal = vtk.vtkTransform()
    imageManager.queue.getTransform(imageName, 'local', imageUtime, cameraToLocal)

    pd = filterUtils.transformPolyData(obj.polyData, obj.actor.GetUserTransform())
    pd = filterUtils.transformPolyData(pd, cameraToLocal.GetLinearInverse())

    imageManager.queue.computeTextureCoords(imageName, pd)

    tcoordsArrayName = 'tcoords_%s' % imageName
    tcoords = pd.GetPointData().GetArray(tcoordsArrayName)
    assert tcoords

    obj.polyData.GetPointData().SetTCoords(None)
    obj.polyData.GetPointData().SetTCoords(tcoords)
    obj._updateColorByProperty()

    obj.actor.SetTexture(imageManager.getTexture(imageName))
    obj.actor.GetProperty().LightingOff()
    obj.actor.GetProperty().SetColor([1,1,1])


class CameraView(object):

    def __init__(self, imageManager, view=None):

        self.imageManager = imageManager
        self.updateUtimes = {}
        self.sphereObjects = {}
        self.sphereImages = [
                'CAMERA_LEFT',
                'CAMERACHEST_RIGHT',
                'CAMERACHEST_LEFT']

        for name in self.sphereImages:
            imageManager.addImage(name)
            self.updateUtimes[name] = 0

        self.initView(view)
        self.initEventFilter()
        self.rayCallback = rayDebug

        self.timerCallback = TimerCallback()
        self.timerCallback.targetFps = 60
        self.timerCallback.callback = self.updateView
        self.timerCallback.start()

    def onViewDoubleClicked(self, displayPoint):

        obj, pickedPoint = vis.findPickedObject(displayPoint, self.view)

        if pickedPoint is None or not obj:
            return

        imageName = obj.getProperty('Name')
        imageUtime = self.imageManager.getUtime(imageName)

        cameraToLocal = vtk.vtkTransform()
        self.imageManager.queue.getTransform(imageName, 'local', imageUtime, cameraToLocal)

        utorsoToLocal = vtk.vtkTransform()
        self.imageManager.queue.getTransform('utorso', 'local', imageUtime, utorsoToLocal)

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
        elif event.type() == QtCore.QEvent.KeyPress:
            if str(event.text()).lower() == 'p':
                self.eventFilter.setEventHandlerResult(True)
            elif str(event.text()).lower() == 'r':
                self.eventFilter.setEventHandlerResult(True)
                self.resetCamera()

    def initEventFilter(self):
        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        qvtkwidget = self.view.vtkWidget()
        qvtkwidget.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonDblClick)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.KeyPress)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.filterEvent)

    def initView(self, view):

        self.view = view or app.getViewManager().createView('Camera View', 'VTK View')

        self.renderers = [self.view.renderer()]
        renWin = self.view.renderWindow()
        renWin.SetNumberOfLayers(3)
        for i in [1, 2]:
            ren = vtk.vtkRenderer()
            ren.SetLayer(2)
            ren.SetActiveCamera(self.view.camera())
            renWin.AddRenderer(ren)
            self.renderers.append(ren)

        def applyCustomBounds():
            self.view.addCustomBounds([-100, 100, -100, 100, -100, 100])
        self.view.connect('computeBoundsRequest(ddQVTKWidgetView*)', applyCustomBounds)

        app.setCameraTerrainModeEnabled(self.view, True)
        self.resetCamera()

    def resetCamera(self):
        self.view.camera().SetViewAngle(90)
        self.view.camera().SetPosition(-7.5, 0.0, 5.0)
        self.view.camera().SetFocalPoint(0.0, 0.0, 0.0)
        self.view.camera().SetViewUp(0.0, 0.0, 1.0)
        self.view.render()

    def getSphereGeometry(self, imageName):

        sphereObj = self.sphereObjects.get(imageName)
        if sphereObj:
            return sphereObj

        if not self.imageManager.getImage(imageName).GetDimensions()[0]:
            return None

        sphereResolution = 50
        sphereRadii = {
                'CAMERA_LEFT' : 20,
                'CAMERACHEST_LEFT' : 20,
                'CAMERACHEST_RIGHT' : 20
                }

        geometry = makeSphere(sphereRadii[imageName], sphereResolution)
        self.imageManager.queue.computeTextureCoords(imageName, geometry)

        tcoordsArrayName = 'tcoords_%s' % imageName
        vtkNumpy.addNumpyToVtk(geometry, vtkNumpy.getNumpyFromVtk(geometry, tcoordsArrayName)[:,0].copy(), 'tcoords_U')
        vtkNumpy.addNumpyToVtk(geometry, vtkNumpy.getNumpyFromVtk(geometry, tcoordsArrayName)[:,1].copy(), 'tcoords_V')
        geometry = clipRange(geometry, 'tcoords_U', [0.0, 1.0])
        geometry = clipRange(geometry, 'tcoords_V', [0.0, 1.0])
        geometry.GetPointData().SetTCoords(geometry.GetPointData().GetArray(tcoordsArrayName))

        sphereObj = vis.showPolyData(geometry, imageName, view=self.view, parent='cameras')
        sphereObj.actor.SetTexture(self.imageManager.getTexture(imageName))
        sphereObj.actor.GetProperty().LightingOff()

        self.view.renderer().RemoveActor(sphereObj.actor)
        rendererId = 2 - self.sphereImages.index(imageName)
        self.renderers[rendererId].AddActor(sphereObj.actor)

        self.sphereObjects[imageName] = sphereObj
        return sphereObj

    def updateSphereGeometry(self):

        for imageName in self.sphereImages:
            sphereObj = self.getSphereGeometry(imageName)
            if not sphereObj:
                continue

            transform = vtk.vtkTransform()
            self.imageManager.queue.getBodyToCameraTransform(imageName, transform)
            sphereObj.actor.SetUserTransform(transform.GetLinearInverse())

    def updateImages(self):

        updated = False
        for imageName, lastUtime in self.updateUtimes.iteritems():
            currentUtime = self.imageManager.updateImage(imageName)
            if currentUtime != lastUtime:
                self.updateUtimes[imageName] = currentUtime
                updated = True

        return updated

    def updateView(self):

        if not self.view.isVisible():
            return

        if not self.updateImages():
            return

        self.updateSphereGeometry()
        self.view.render()


class ImageWidget(object):

    def __init__(self, imageManager, imageName, view):
        self.view = view
        self.imageManager = imageManager
        self.imageName = imageName

        self.updateUtime = 0
        self.initialized = False

        self.imageWidget = vtk.vtkLogoWidget()
        rep = self.imageWidget.GetRepresentation()
        rep.GetImageProperty().SetOpacity(1.0)
        self.imageWidget.SetInteractor(self.view.renderWindow().GetInteractor())

        self.flip = vtk.vtkImageFlip()
        self.flip.SetFilteredAxis(1)
        self.flip.SetInput(imageManager.getImage(imageName))
        rep.SetImage(self.flip.GetOutput())

        self.timerCallback = TimerCallback()
        self.timerCallback.targetFps = 60
        self.timerCallback.callback = self.updateView
        self.timerCallback.start()


    def hide(self):
        self.imageWidget.Off()

    def show(self):
        self.imageWidget.On()

    def updateView(self):

        if not self.view.isVisible():
            return

        currentUtime = self.imageManager.updateImage(self.imageName)
        if currentUtime != self.updateUtime:
            if not self.initialized:
                self.show()
                self.initialized = True

            self.updateUtime = currentUtime
            self.flip.Update()
            self.view.render()


class CameraImageView(object):

    def __init__(self, imageManager, imageName, viewName=None, view=None):

        imageManager.addImage(imageName)

        self.imageManager = imageManager
        self.viewName = viewName or imageName
        self.imageName = imageName
        self.imageInitialized = False
        self.updateUtime = 0
        self.initView(view)
        self.initEventFilter()


    def getImagePixel(self, displayPoint, restrictToImageDimensions=True):

        worldPoint = [0.0, 0.0, 0.0, 0.0]
        vtk.vtkInteractorObserver.ComputeDisplayToWorld(self.view.renderer(), displayPoint[0], displayPoint[1], 0, worldPoint)

        imageDimensions = self.getImage().GetDimensions()

        if 0.0 <= worldPoint[0] <= imageDimensions[0] and 0.0 <= worldPoint[1] <= imageDimensions[1] or not restrictToImageDimensions:
            return [worldPoint[0], worldPoint[1], 0.0]
        else:
            return None


    def getWorldPositionAndRay(self, imagePixel, imageUtime=None):
        '''
        Given an XY image pixel, computes an equivalent ray in the world
        coordinate system using the camera to local transform at the given
        imageUtime.  If imageUtime is None, then the utime of the most recent
        image is used.

        Returns the camera xyz position in world, and a ray unit vector.
        '''
        if imageUtime is None:
            imageUtime = self.imageManager.getUtime(self.imageName)

        # input is pixel u,v, output is unit x,y,z in camera coordinates
        cameraPoint = self.imageManager.queue.unprojectPixel(self.imageName, imagePixel[0], imagePixel[1])

        cameraToLocal = vtk.vtkTransform()
        self.imageManager.queue.getTransform(self.imageName, 'local', imageUtime, cameraToLocal)

        p = np.array(cameraToLocal.TransformPoint(cameraPoint))
        cameraPosition = np.array(cameraToLocal.GetPosition())
        ray = p - cameraPosition
        ray /= np.linalg.norm(ray)

        return cameraPosition, ray


    def filterEvent(self, obj, event):
        if self.eventFilterEnabled and event.type() == QtCore.QEvent.MouseButtonDblClick:
            self.eventFilter.setEventHandlerResult(True)

        elif event.type() == QtCore.QEvent.KeyPress:
            if str(event.text()).lower() == 'p':
                self.eventFilter.setEventHandlerResult(True)
            elif str(event.text()).lower() == 'r':
                self.eventFilter.setEventHandlerResult(True)
                self.resetCamera()

    def onRubberBandPick(self, obj, event):
        displayPoints = self.interactorStyle.GetStartPosition(), self.interactorStyle.GetEndPosition()
        imagePoints = [vis.pickImage(point, self.view)[1] for point in displayPoints]
        sendFOVRequest(self.imageName, imagePoints)

    def getImage(self):
        return self.imageManager.getImage(self.imageName)

    def initView(self, view):
        self.view = view or app.getViewManager().createView(self.viewName, 'VTK View')
        self.view.installImageInteractor()
        self.interactorStyle = self.view.renderWindow().GetInteractor().GetInteractorStyle()
        self.interactorStyle.AddObserver('SelectionChangedEvent', self.onRubberBandPick)

        self.imageActor = vtk.vtkImageActor()
        self.imageActor.SetInput(self.getImage())
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
        self.eventFilter.addFilteredEventType(QtCore.QEvent.KeyPress)
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
        self.view.render()

    def fitImageToView(self):

        camera = self.view.camera()
        image = self.getImage()
        imageWidth, imageHeight, _ = image.GetDimensions()

        viewWidth, viewHeight = self.view.renderWindow().GetSize()
        aspectRatio = float(viewWidth)/viewHeight
        parallelScale = max(imageWidth/aspectRatio, imageHeight) / 2.0
        camera.SetParallelScale(parallelScale)

    def setImageName(self, imageName):
        if imageName == self.imageName:
            return

        assert self.imageManager.hasImage(imageName)

        self.imageName = imageName
        self.imageInitialized = False
        self.updateUtime = 0
        self.imageActor.SetInput(self.imageManager.getImage(self.imageName))
        self.imageActor.SetVisibility(False)
        self.view.render()

    def updateView(self):

        if not self.view.isVisible():
            return

        currentUtime = self.imageManager.updateImage(self.imageName)
        if currentUtime != self.updateUtime:
            self.updateUtime = currentUtime
            self.view.render()

            if not self.imageInitialized and self.imageActor.GetInput().GetDimensions()[0]:
                self.imageActor.SetVisibility(True)
                self.resetCamera()
                self.imageInitialized = True


class CameraFrustumVisualizer(object):

    def __init__(self, robotModel, imageManager, cameraName):
        self.robotModel = robotModel
        self.cameraName = cameraName
        self.imageManager = imageManager
        self.rayLength = 2.0
        self.headLink = drcargs.getDirectorConfig()['headLink']
        robotModel.connectModelChanged(self.update)
        self.update(robotModel)



    def getCameraToLocal(self):
        '''
        Returns cameraToLocal.  cameraToHead is pulled from bot frames while
        headToLocal is pulled from the robot model forward kinematics.
        '''
        headToLocal = self.robotModel.getLinkFrame( self.headLink )
        cameraToHead = vtk.vtkTransform()
        # TODO: 'head' should match self.headLink
        self.imageManager.queue.getTransform(self.cameraName, 'head', 0, cameraToHead)
        return transformUtils.concatenateTransforms([cameraToHead, headToLocal])

    def getCameraFrustumRays(self):
        '''
        Returns (cameraPositions, rays)
        cameraPosition is in world frame.
        rays are four unit length vectors in world frame that point in the
        direction of the camera frustum edges
        '''

        cameraToLocal = self.getCameraToLocal()
        cameraPos = np.array(cameraToLocal.GetPosition())

        camRays = []
        rays = np.array(self.imageManager.queue.getCameraFrustumBounds(self.cameraName))
        for i in xrange(4):
            ray = np.array(cameraToLocal.TransformVector(rays[i*3:i*3+3]))
            ray /= np.linalg.norm(ray)
            camRays.append(ray)

        return cameraPos, camRays

    def getCameraFrustumGeometry(self, rayLength):

        camPos, rays = self.getCameraFrustumRays()

        rays = [rayLength*r for r in rays]

        d = DebugData()
        d.addLine(camPos, camPos+rays[0])
        d.addLine(camPos, camPos+rays[1])
        d.addLine(camPos, camPos+rays[2])
        d.addLine(camPos, camPos+rays[3])
        d.addLine(camPos+rays[0], camPos+rays[1])
        d.addLine(camPos+rays[1], camPos+rays[2])
        d.addLine(camPos+rays[2], camPos+rays[3])
        d.addLine(camPos+rays[3], camPos+rays[0])
        return d.getPolyData()

    def update(self, robotModel):
        name = 'camera frustum %s' % self.robotModel.getProperty('Name')
        obj = om.findObjectByName(name)

        if obj and not obj.getProperty('Visible'):
            return

        vis.updatePolyData(self.getCameraFrustumGeometry(self.rayLength), name, parent=self.robotModel, visible=False)



views = {}


def addCameraView(channel, viewName=None, cameraName=None, imageType=-1):

    cameraName = cameraName or channel
    imageManager.queue.addCameraStream(channel, cameraName, imageType)
    imageManager.addImage(cameraName)
    view = CameraImageView(imageManager, cameraName, viewName)
    global views
    views[channel] = view
    return view


def getStereoPointCloud(decimation=4, imagesChannel='CAMERA', cameraName='CAMERA_LEFT', removeSize=0):

    q = imageManager.queue

    utime = q.getCurrentImageTime(cameraName)
    if utime == 0:
        return None

    p = vtk.vtkPolyData()
    cameraToLocal = vtk.vtkTransform()

    q.getPointCloudFromImages(imagesChannel, p, decimation, removeSize)
    q.getTransform('CAMERA_LEFT', 'local', utime, cameraToLocal)
    p = filterUtils.transformPolyData(p, cameraToLocal)

    return p



class KintinuousMapping(object):

    def __init__(self):

        self.lastUtime = 0
        self.lastCameraToLocal = vtk.vtkTransform()

        self.cameraToLocalFusedTransforms = []
        self.cameraToLocalTransforms = []
        self.pointClouds = []

    def getStereoPointCloudElapsed(self,decimation=4, imagesChannel='CAMERA', cameraName='CAMERA_LEFT', removeSize=0):
        q = imageManager.queue

        utime = q.getCurrentImageTime(cameraName)
        if utime == 0:
            return None, None, None

        if (utime - self.lastUtime < 1E6):
            return None, None, None

        p = vtk.vtkPolyData()
        cameraToLocalFused = vtk.vtkTransform()
        q.getTransform('CAMERA_LEFT_ALT', 'local', utime, cameraToLocalFused)
        cameraToLocal = vtk.vtkTransform()
        q.getTransform('CAMERA_LEFT', 'local', utime, cameraToLocal)
        prevToCurrentCameraTransform = vtk.vtkTransform()
        prevToCurrentCameraTransform.PostMultiply()
        prevToCurrentCameraTransform.Concatenate( cameraToLocal )
        prevToCurrentCameraTransform.Concatenate( self.lastCameraToLocal.GetLinearInverse() )
        distTravelled = np.linalg.norm( prevToCurrentCameraTransform.GetPosition() )

        # 0.2 heavy overlap
        # 0.5 quite a bit of overlap
        # 1.0 is good
        if (distTravelled  < 0.2 ):
            return None, None, None

        q.getPointCloudFromImages(imagesChannel, p, decimation, removeSize)

        self.lastCameraToLocal = cameraToLocal
        self.lastUtime = utime
        return p, cameraToLocalFused, cameraToLocal


    def showFusedMaps(self):
        om.removeFromObjectModel(om.findObjectByName('stereo'))
        om.getOrCreateContainer('stereo')

        q = imageManager.queue
        cameraToLocalNow = vtk.vtkTransform()
        utime = q.getCurrentImageTime('CAMERA_TSDF')

        q.getTransform('CAMERA_LEFT','local', utime,cameraToLocalNow)
        cameraToLocalFusedNow = vtk.vtkTransform()
        q.getTransform('CAMERA_LEFT_ALT','local', utime,cameraToLocalFusedNow)

        for i in range(len(self.pointClouds)):

            fusedNowToLocalNow = vtk.vtkTransform()
            fusedNowToLocalNow.PreMultiply()
            fusedNowToLocalNow.Concatenate( cameraToLocalNow)
            fusedNowToLocalNow.Concatenate( cameraToLocalFusedNow.GetLinearInverse() )


            fusedTransform = vtk.vtkTransform()
            fusedTransform.PreMultiply()
            fusedTransform.Concatenate( fusedNowToLocalNow)
            fusedTransform.Concatenate( self.cameraToLocalFusedTransforms[i] )

            pd = filterUtils.transformPolyData(self.pointClouds[i], fusedTransform)
            vis.showFrame(fusedTransform, ('cloud frame ' + str(i)), visible=True, scale=0.2, parent='stereo')
            vis.showPolyData(pd, ('stereo ' + str(i)), parent='stereo', colorByName='rgb_colors')

            # Without compensation for fusion motion estimation:
            #pd = filterUtils.transformPolyData(self.pointClouds[i], self.cameraToLocalTransforms[i])
            #vis.showFrame(self.cameraToLocalTransforms[i], ('cloud frame ' + str(i)), visible=True, scale=0.2)
            #vis.showPolyData(pd, ('stereo ' + str(i)) )

            # in fusion coordinate frame:
            #pd = filterUtils.transformPolyData(self.pointClouds[i], self.cameraToLocalFusedTransforms[i])
            #vis.showFrame(self.cameraToLocalFusedTransforms[i], ('cloud frame ' + str(i)), visible=True, scale=0.2)
            #vis.showPolyData(pd, ('stereo ' + str(i)) )

    def cameraFusedCallback(self):
        #pd = cameraview.getStereoPointCloud(2,"CAMERA_FUSED")
        pd, cameraToLocalFused, cameraToLocal = self.getStereoPointCloudElapsed(2,"CAMERA_FUSED")
        #vis.updateFrame(cameraToLocal, 'cloud frame now', visible=True, scale=0.2)

        if (pd is None):
            return

        self.pointClouds.append(pd)
        self.cameraToLocalFusedTransforms.append( cameraToLocalFused )
        self.cameraToLocalTransforms.append( cameraToLocal )

        #pdCopy = vtk.vtkPolyData()
        #pdCopy.DeepCopy(pd)
        #cameraToLocalCopy = transformUtils.copyFrame(cameraToLocalFused)
        #pdCopy = filterUtils.transformPolyData(pdCopy, cameraToLocalCopy)
        #vis.showFrame(cameraToLocalCopy, 'cloud frame', visible=True, scale=0.2)
        #vis.showPolyData(pdCopy,'stereo')

        self.showFusedMaps()


def init():

    global imageManager
    imageManager = ImageManager()

    global cameraView
    cameraView = CameraView(imageManager)

    addCameraView('CAMERA_LEFT', 'Head camera')
    #addCameraView('CAMERA', 'Head camera right', 'CAMERA_RIGHT', lcmmultisense.images_t.RIGHT)
    #addCameraView('CAMERA', 'Head camera depth', 'CAMERA_DISPARITY', lcmmultisense.images_t.DISPARITY_ZIPPED)
    addCameraView('CAMERACHEST_LEFT', 'Chest left')
    addCameraView('CAMERACHEST_RIGHT', 'Chest right')
    addCameraView('CAMERALHAND')
    addCameraView('CAMERARHAND')
    addCameraView('KINECT_RGB', 'Kinect RGB')
