from director import applogic
from director import imageview
from director import vtkAll as vtk
from director import filterUtils
from director import transformUtils
from director import visualization as vis
from director import viewbehaviors
from director import vtkNumpy as vnp
from director.debugVis import DebugData
from director.timercallback import TimerCallback
from director.fieldcontainer import FieldContainer
import PythonQt
from PythonQt import QtCore
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
        self.parentFolder = 'depth scanner'

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
        self._updateFunc = None

    def getDepthBufferImage(self):
        return self.windowToDepthBuffer.GetOutput()

    def getDepthImage(self):
        return self.depthScaleFilter.GetOutput()

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

        self.depthScaleFilter = vtk.vtkImageShiftScale()
        self.depthScaleFilter.SetScale(1.0)
        self.depthScaleFilter.SetOutputScalarTypeToDouble()

        self.depthImageLookupTable = lut
        self.imageMapToColors = vtk.vtkImageMapToColors()
        self.imageMapToColors.SetLookupTable(self.depthImageLookupTable)
        self.imageMapToColors.SetInputConnection(self.depthScaleFilter.GetOutputPort())

        self.imageView = imageview.ImageView()
        self.imageView.view.setWindowTitle('Depth image')
        self.imageView.setImage(self.imageMapToColors.GetOutput())

    def initPointCloudView(self):
        self.pointCloudView = PythonQt.dd.ddQVTKWidgetView()
        self.pointCloudView.setWindowTitle('Pointcloud')
        self.pointCloudViewBehaviors = viewbehaviors.ViewBehaviors(self.pointCloudView)

    def update(self):

        if not self.renderObserver:
            def onEndRender(obj, event):
                if self._block:
                    return
                if not self.singleShotTimer.singleShotTimer.isActive():
                    self.singleShotTimer.singleShot(0)
            self.renderObserver = self.view.renderWindow().AddObserver('EndEvent', onEndRender)

        if not self.pointCloudView.visible and not self.imageView.view.visible:
            return

        self._block = True
        self.view.forceRender()
        self.updateBufferImages()
        self._block = False

        depthImage, polyData = computeDepthImageAndPointCloud(self.getDepthBufferImage(), self.getColorBufferImage(), self.view.camera())

        self.depthScaleFilter.SetInputData(depthImage)
        self.depthScaleFilter.Update()

        self.depthImageLookupTable.SetRange(self.depthScaleFilter.GetOutput().GetScalarRange())
        self.imageMapToColors.Update()
        self.imageView.resetCamera()
        #self.imageView.view.render()

        if not self.pointCloudObj:
            self.pointCloudObj = vis.showPolyData(polyData, 'point cloud', colorByName='rgb', view=self.pointCloudView, parent=self.parentFolder)
        else:
            self.pointCloudObj.setPolyData(polyData)
        self.pointCloudView.render()
        if self._updateFunc:
            self._updateFunc()

    def addViewsToDock(self, app):
        for view in [self.imageView.view, self.pointCloudView]:
            dock = app.addWidgetToDock(view, QtCore.Qt.RightDockWidgetArea)
            dock.setMinimumWidth(300)
            dock.setMinimumHeight(300)


def getCameraFrustumMesh(view, rayLength=1.0):

    origin = np.array(view.camera().GetPosition())

    def getCameraRay(displayPoint):
        _, ray = vis.getRayFromDisplayPoint(view, displayPoint)
        ray = ray-origin
        ray /= np.linalg.norm(ray)
        return ray

    viewWidth, viewHeight = view.renderWindow().GetSize()

    rays = [getCameraRay(x) for x in [[0.0, 0.0], [viewWidth, 0.0], [viewWidth, viewHeight], [0.0, viewHeight]]]

    rays = [rayLength*r for r in rays]
    camPos = origin
    lineRadius = 0.0
    color = [1.0, 1.0, 1.0]

    d = DebugData()
    d.addLine(camPos, camPos+rays[0], radius=lineRadius, color=color)
    d.addLine(camPos, camPos+rays[1], radius=lineRadius, color=color)
    d.addLine(camPos, camPos+rays[2], radius=lineRadius, color=color)
    d.addLine(camPos, camPos+rays[3], radius=lineRadius, color=color)
    d.addLine(camPos+rays[0], camPos+rays[1], radius=lineRadius, color=color)
    d.addLine(camPos+rays[1], camPos+rays[2], radius=lineRadius, color=color)
    d.addLine(camPos+rays[2], camPos+rays[3], radius=lineRadius, color=color)
    d.addLine(camPos+rays[3], camPos+rays[0], radius=lineRadius, color=color)
    pd = d.getPolyData()
    return pd


def getCameraTransform(camera):
    return transformUtils.getLookAtTransform(
              camera.GetFocalPoint(),
              camera.GetPosition(),
              camera.GetViewUp())


def initCameraFrustumVisualizer(depthScanner):

    cameraObj = vis.showPolyData(vtk.vtkPolyData(), 'camera', parent=depthScanner.parentFolder, view=depthScanner.pointCloudView)
    cameraFrame = vis.addChildFrame(cameraObj)
    cameraFrame.setProperty('Scale', 50)
    cameraObj.setProperty('Visible', False)
    pointCloudToCamera = transformUtils.frameFromPositionAndRPY((0,0,0,), (-90, 90, 0)).GetLinearInverse()

    def updateCameraMesh():
        scale = cameraObj.getChildFrame().getProperty('Scale') * 10.0
        rayLength = scale
        d = DebugData()
        d.addCube(dimensions=[0.04, 0.08, 0.06], center=[-0.02, 0.0, 0.0], color=[1,0.5,0])
        d.addLine([0.0, 0.0, 0.0], [0.01, 0.0, 0.0], radius=0.023, color=[1,0.5,0])
        cameraModelMesh = d.getPolyData()

        t = vtk.vtkTransform()
        t.Scale(scale, scale, scale)
        cameraModelMesh = filterUtils.transformPolyData(cameraModelMesh, t)

        cameraMesh = getCameraFrustumMesh(depthScanner.view, rayLength)
        cameraMesh = filterUtils.transformPolyData(cameraMesh, getCameraTransform(depthScanner.view.camera()).GetLinearInverse())
        cameraMesh = filterUtils.appendPolyData([cameraMesh, cameraModelMesh])
        cameraObj.setPolyData(cameraMesh)

    def onCameraModified():
        cameraToWorld = getCameraTransform(depthScanner.view.camera())
        depthScanner.pointCloudObj.actor.SetUserTransform(transformUtils.concatenateTransforms([pointCloudToCamera, cameraToWorld]))
        cameraFrame.copyFrame(cameraToWorld)

    def enableFrustum():
        updateCameraMesh()
        cameraObj.setProperty('Visible', True)
        onCameraModified()
        depthScanner._updateFunc = onCameraModified
        applogic.setCameraTerrainModeEnabled(depthScanner.pointCloudView, True)
        applogic.resetCamera(viewDirection=[1, 1, -0.4], view=depthScanner.pointCloudView)
        depthScanner.pointCloudView.camera().SetViewUp([0, 0, 1])

    def disableFrustum():
        cameraObj.setProperty('Visible', False)
        depthScanner.pointCloudObj.actor.SetUserTransform(None)
        depthScanner._updateFunc = None
        applogic.setCameraTerrainModeEnabled(depthScanner.pointCloudView, False)
        applogic.resetCamera(viewDirection=[0, 0, -1], view=depthScanner.pointCloudView)
        depthScanner.pointCloudView.camera().SetViewUp([0, 1, 0])

    return FieldContainer(
        updateCameraMesh=updateCameraMesh,
        onCameraModified=onCameraModified,
        enableFrustum=enableFrustum,
        disableFrustum=disableFrustum
        )


def main(globalsDict=None):

    from director import mainwindowapp
    from PythonQt import QtCore, QtGui

    app = mainwindowapp.construct()
    app.gridObj.setProperty('Visible', True)
    app.viewOptions.setProperty('Orientation widget', False)
    app.viewOptions.setProperty('View angle', 30)
    app.sceneBrowserDock.setVisible(False)
    app.propertiesDock.setVisible(False)
    app.mainWindow.setWindowTitle('Depth Scanner')
    app.mainWindow.show()
    app.mainWindow.resize(920,600)
    app.mainWindow.move(0,0)

    view = app.view
    view.setParent(None)
    mdiArea = QtGui.QMdiArea()
    app.mainWindow.setCentralWidget(mdiArea)
    subWindow = mdiArea.addSubWindow(view)
    subWindow.setMinimumSize(300,300)
    subWindow.setWindowTitle('Camera image')
    subWindow.resize(640, 480)
    mdiArea.tileSubWindows()

    #affordanceManager = affordancemanager.AffordanceObjectModelManager(view)

    depthScanner = DepthScanner(view)
    depthScanner.update()
    depthScanner.addViewsToDock(app.app)


    # add some test data
    def addTestData():
        d = DebugData()
        d.addArrow((0,0,0), (0,0,1), color=[1,0,0])
        d.addArrow((0,0,1), (0,.5,1), color=[0,1,0])
        vis.showPolyData(d.getPolyData(), 'debug data', colorByName='RGB255')
        view.resetCamera()

    addTestData()

    # xvfb command
    # /usr/bin/Xvfb  :99 -ac -screen 0 1280x1024x16

    if globalsDict is not None:
        globalsDict.update(dict(app=app, view=view, depthScanner=depthScanner))

    app.app.start(restoreWindow=False)



if __name__ == '__main__':
    main(globals())
