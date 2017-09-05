import PythonQt
from PythonQt import QtGui, QtCore
from director import pointpicker
from director import vtkNumpy as vnp
from director import vtkAll as vtk
import numpy as np


class ImageView(object):

    def __init__(self):
        self.autoResetCamera = False
        self.view = PythonQt.dd.ddQVTKWidgetView()
        self.view.setWindowTitle('Image View')
        self.imageActor = vtk.vtkImageActor()
        self.setImage(vtk.vtkImageData())
        self.view.renderer().AddActor(self.imageActor)
        self.view.orientationMarkerWidget().Off()
        self.setBackgroundColor([0,0,0])
        self.initInteractor()
        self.installEventFilter()
        self.resetCamera()

    def installEventFilter(self):
        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        qvtkwidget = self.view.vtkWidget()
        qvtkwidget.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonDblClick)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.KeyPress)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.Resize)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.filterEvent)

    def initInteractor(self):
        self.view.installImageInteractor()
        self.interactorStyle = self.view.renderWindow().GetInteractor().GetInteractorStyle()
        self.interactorStyle.AddObserver('SelectionChangedEvent', self.onRubberBandPick)

    def initPointPicker(self):
        self.pointPicker = pointpicker.ImagePointPicker(self, callback=self.onPickedPoints)
        self.pointPicker.start()

    def onPickedPoints(self, *points):
        self.pickedPoints = points

    def onRubberBandPick(self, obj, event):
        displayPoints = self.interactorStyle.GetStartPosition(), self.interactorStyle.GetEndPosition()
        self.rubberBandPickPoints = [self.getImagePixel(p) for p in displayPoints]

    def setBackgroundColor(self, color):
        self.view.renderer().SetBackground(color)
        self.view.renderer().SetBackground2(color)

    def getImagePixel(self, displayPoint, restrictToImageDimensions=True):
        worldPoint = [0.0, 0.0, 0.0, 0.0]
        vtk.vtkInteractorObserver.ComputeDisplayToWorld(self.view.renderer(), displayPoint[0], displayPoint[1], 0, worldPoint)
        imageDimensions = self.getImage().GetDimensions()
        if 0.0 <= worldPoint[0] <= imageDimensions[0] and 0.0 <= worldPoint[1] <= imageDimensions[1] or not restrictToImageDimensions:
            return [worldPoint[0], worldPoint[1], 0.0]
        else:
            return None

    def resizeView(self, scale=1.0):
        image = self.getImage()
        assert image
        width, height, _ = image.GetDimensions()
        assert width > 0 and height > 0
        self.view.resize(width * scale, height * scale)
        self.resetCamera()

    def show(self):
        if not self.view.isVisible() and self.getImage():
            self.resizeView()
        self.view.show()

    def filterEvent(self, obj, event):

        if event.type() == QtCore.QEvent.MouseButtonDblClick:
            self.eventFilter.setEventHandlerResult(True)

        elif event.type() == QtCore.QEvent.Resize:
            if self.autoResetCamera:
                self.resetCamera()

        elif event.type() == QtCore.QEvent.KeyPress:
            if str(event.text()).lower() == 'p':
                self.eventFilter.setEventHandlerResult(True)
            elif str(event.text()).lower() == 'r':
                self.eventFilter.setEventHandlerResult(True)
                self.resetCamera()

    def setImage(self, image):
        if image != self.getImage():
            self.imageActor.SetInputData(image)
            self.resetCamera()

    def getImage(self):
        return self.imageActor.GetInput()

    def resetCamera(self):
        camera = self.view.camera()
        camera.ParallelProjectionOn()
        camera.SetFocalPoint(0,0,0)
        camera.SetPosition(0,0,1)
        camera.SetViewUp(0,1,0)

        self.view.resetCamera()
        self.fitImageToView()
        self.view.render()

    def fitImageToView(self):
        viewWidth, viewHeight = self.view.renderWindow().GetSize()
        if viewHeight == 0:
            return

        camera = self.view.camera()
        image = self.getImage()
        imageWidth, imageHeight, _ = image.GetDimensions()
        aspectRatio = float(viewWidth)/viewHeight
        parallelScale = max(imageWidth/aspectRatio, imageHeight) / 2.0
        camera.SetParallelScale(parallelScale)

    def showNumpyImage(self, img, flip=True):

        image = self.getImage()
        if not image:
            image = vtk.vtkImageData()
            self.setImage(image)

        if flip:
          img = np.flipud(img)

        height, width, numChannels = img.shape
        dims = image.GetDimensions()
        if dims[0] != width or dims[1] != height or image.GetNumberOfScalarComponents() != numChannels:
          image.SetDimensions(width, height, 1)
          image.AllocateScalars(vtk.VTK_UNSIGNED_CHAR, numChannels)

        scalars = vnp.getNumpyFromVtk(image, 'ImageScalars')
        if numChannels > 1:
            scalars[:] = img.reshape(width*height, numChannels)[:]
        else:
            scalars[:] = img.reshape(width*height)[:]
        image.Modified()
        self.view.render()
