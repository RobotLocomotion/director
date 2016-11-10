from director import consoleapp
from director import imageview
from director import vtkAll as vtk
from director import transformUtils
from director import visualization as vis
from director import vtkNumpy as vnp
from director.debugVis import DebugData
from director.timercallback import TimerCallback
from director import ioUtils
from director import filterUtils
import PythonQt
import numpy as np


def computeDepthImageAndPointCloud(depthBuffer, colorBuffer, camera):
    '''
    Input args are an OpenGL depth buffer and color buffer as vtkImageData objects,
    and the vtkCamera instance that was used to render the scene.  The function returns
    returns a depth image and a point cloud as vtkImageData and vtkPolyData.
    '''
    depthImage = vtk.vtkImageData()
    pts = vtk.vtkPoints()
    ptColors = vtk.vtkUnsignedCharArray()
    vtk.vtkDepthImageUtils.DepthBufferToDepthImage(depthBuffer, colorBuffer, camera, depthImage, pts, ptColors)

    pts = vnp.numpy_support.vtk_to_numpy(pts.GetData())
    polyData = vnp.numpyToPolyData(pts, createVertexCells=True)
    ptColors.SetName('rgb')
    polyData.GetPointData().AddArray(ptColors)

    return depthImage, polyData


class DepthScanner(object):

    def __init__(self, view):
        self.view = view

        self.depthImage = None
        self.pointCloudObj = None
        self.renderObserver = None

        self.windowToDepthBuffer = vtk.vtkWindowToImageFilter()
        self.windowToDepthBuffer.SetInput(self.view.renderWindow())
        self.windowToDepthBuffer.SetInputBufferTypeToZBuffer()
        self.windowToDepthBuffer.ShouldRerenderOff()

        self.windowToColorBuffer = vtk.vtkWindowToImageFilter()
        self.windowToColorBuffer.SetInput(self.view.renderWindow())
        self.windowToColorBuffer.SetInputBufferTypeToRGB()
        self.windowToColorBuffer.ShouldRerenderOff()

        useBackBuffer = False
        if useBackBuffer:
            self.windowToDepthBuffer.ReadFrontBufferOff()
            self.windowToColorBuffer.ReadFrontBufferOff()

        self.initDepthImageView()
        self.initPointCloudView()

        self._block = False
        self.singleShotTimer = TimerCallback()
        self.singleShotTimer.callback = self.update

    def getDepthBufferImage(self):
        return self.windowToDepthBuffer.GetOutput()

    def getColorBufferImage(self):
        return self.windowToColorBuffer.GetOutput()

    def updateBufferImages(self):
        for f in [self.windowToDepthBuffer, self.windowToColorBuffer]:
            f.Modified()
            f.Update()

    def initDepthImageView(self):

        self.depthImageColorByRange = [0.0, 4.0]

        lut = vtk.vtkLookupTable()
        lut.SetNumberOfColors(256)
        lut.SetHueRange(0, 0.667) # red to blue
        lut.SetRange(self.depthImageColorByRange) # map red (near) to blue (far)
        lut.SetRampToLinear()
        lut.Build()

        im = vtk.vtkImageMapToColors()
        im.SetLookupTable(lut)

        self.depthImageLookupTable = lut
        self.imageMapToColors = im

        self.imageView = imageview.ImageView()
        self.imageView.view.setWindowTitle('Depth image')
        self.imageView.setImage(self.imageMapToColors.GetOutput())

    def initPointCloudView(self):
        self.pointCloudView = PythonQt.dd.ddQVTKWidgetView()
        self.pointCloudView.setWindowTitle('Pointcloud')

    def update(self):

        if not self.renderObserver:
            def onEndRender(obj, event):
                if self._block:
                    return
                if not self.singleShotTimer.singleShotTimer.isActive():
                    self.singleShotTimer.singleShot(0)
            self.renderObserver = self.view.renderWindow().AddObserver('EndEvent', onEndRender)

        self._block = True
        self.view.forceRender()
        self.updateBufferImages()
        self._block = False

        depthImage, polyData = computeDepthImageAndPointCloud(self.getDepthBufferImage(), self.getColorBufferImage(), self.view.camera())

        self.imageMapToColors.SetInput(depthImage)
        self.imageMapToColors.Update()
        self.imageView.resetCamera()
        #self.imageView.view.render()

        if not self.pointCloudObj:
            self.pointCloudObj = vis.showPolyData(polyData, 'point cloud', colorByName='rgb', view=self.pointCloudView)
        else:
            self.pointCloudObj.setPolyData(polyData)
        self.pointCloudView.render()



def main(globalsDict=None):

    app = consoleapp.ConsoleApp()
    view = app.createView()
    app.gridObj.setProperty('Visible', False)

    d = DebugData()
    d.addArrow((0,0,0), (0,0,1), color=[1,0,0])
    d.addArrow((0,0,1), (0,.5,1), color=[0,1,0])

    vis.showPolyData(d.getPolyData(), 'debug data', colorByName='RGB255')
    view.orientationMarkerWidget().Off()
    view.show()
    view.resize(600,600)
    view.move(0,0)
    view.camera().SetPosition(3,3,0)
    view.camera().SetFocalPoint(0,0,0)
    view.camera().SetViewUp(0,0,1)
    view.resetCamera()
    view.forceRender()

    depthScanner = DepthScanner(view)
    depthScanner.update()

    pcView = depthScanner.pointCloudView
    pcView.show()
    pcView.resize(300,300)
    pcView.move(600,0)

    imageView = depthScanner.imageView
    imageView.view.show()
    imageView.view.resize(300,300)
    imageView.view.move(600,300)
    imageView.resetCamera()

    # xvfb command
    # /usr/bin/Xvfb  :99 -ac -screen 0 1280x1024x16

    if globalsDict is not None:
        globalsDict.update(dict(app=app, view=view, depthScanner=depthScanner))

    app.start()


if __name__ == '__main__':
    main(globals())
