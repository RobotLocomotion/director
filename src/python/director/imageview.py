from director import vtkAll as vtk
import PythonQt

class ImageView(object):

    def __init__(self):
        self.view = PythonQt.dd.ddQVTKWidgetView()
        self.view.setWindowTitle('Image View')
        self.imageActor = vtk.vtkImageActor()
        self.setImage(vtk.vtkImageData())
        self.view.renderer().AddActor(self.imageActor)
        self.view.orientationMarkerWidget().Off()
        self.view.renderer().SetBackground([0,0,0])
        self.view.renderer().SetBackground2([0,0,0])
        self.view.installImageInteractor()

    def setImage(self, image):
        self.imageActor.SetInput(image)

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
        camera = self.view.camera()
        image = self.getImage()
        imageWidth, imageHeight, _ = image.GetDimensions()

        viewWidth, viewHeight = self.view.renderWindow().GetSize()
        aspectRatio = float(viewWidth)/viewHeight
        parallelScale = max(imageWidth/aspectRatio, imageHeight) / 2.0
        camera.SetParallelScale(parallelScale)
