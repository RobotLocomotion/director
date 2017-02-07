import argparse
from director import consoleapp
from director import cameraview
from director import applogic
from director import viewbehaviors
from director import objectmodel as om
from director import vtkAll as vtk

import PythonQt
from PythonQt import QtGui

import bot_core as lcmbotcore


class ImageViewApp(object):

    def __init__(self):
        self.setup()

    def addShortcuts(self, widget):
        applogic.addShortcut(widget, 'Ctrl+Q', consoleapp.ConsoleApp.quit)
        applogic.addShortcut(widget, 'F8', consoleapp.ConsoleApp.showPythonConsole)

    def parseArgs(self, defaultChannel='MULTISENSE_CAMERA_LEFT'):

        parser = argparse.ArgumentParser()
        parser.add_argument('--channel', type=str, help='image channel', default=defaultChannel)
        parser.add_argument('--pointcloud', action='store_true', help='display pointcloud view for RGB-D messages')
        parser.add_argument('--disparity', action='store_true', help='receive disparity images for --rgbd flag')
        imageType = parser.add_mutually_exclusive_group(required=False)
        imageType.add_argument('--rgb', action='store_const', const='rgb', help='receive RGB image messages', dest='imageType')
        imageType.add_argument('--rgbd', action='store_const', const='rgbd', help='receive RGB-D images messages', dest='imageType')
        imageType.set_defaults(imageType='rgb')

        args, unknown = parser.parse_known_args()
        return args

    def setup(self):

        args = self.parseArgs()
        imageManager = cameraview.ImageManager()
        self.imageManager = imageManager

        channel = args.channel
        imageType = args.imageType

        self.app = consoleapp.ConsoleApp()
        self.views = []

        if imageType == 'rgb':

            imageName = channel
            imageManager.queue.addCameraStream(channel, imageName, -1)
            imageManager.addImage(imageName)
            cameraView = cameraview.CameraImageView(imageManager, imageName, view=PythonQt.dd.ddQVTKWidgetView())
            cameraView.eventFilterEnabled = False
            cameraView.view.renderWindow().GetInteractor().SetInteractorStyle(vtk.vtkInteractorStyleImage())
            cameraView.view.resize(640, 480)
            self.views.append(cameraView.view)
            self.cameraView = cameraView

        elif imageType == 'rgbd':

            imageName = channel + '_LEFT'
            imageManager.queue.addCameraStream(channel, imageName, 0)
            imageManager.addImage(imageName)
            cameraView = cameraview.CameraImageView(imageManager, imageName, view=PythonQt.dd.ddQVTKWidgetView())
            cameraView.eventFilterEnabled = False
            cameraView.view.renderWindow().GetInteractor().SetInteractorStyle(vtk.vtkInteractorStyleImage())
            self.views.append(cameraView.view)

            imageName2 = channel + '_D'
            if args.disparity:
                imageManager.queue.addCameraStream(channel, imageName2, lcmbotcore.images_t.DISPARITY_ZIPPED)
            else:
                imageManager.queue.addCameraStream(channel, imageName2, lcmbotcore.images_t.DEPTH_MM_ZIPPED)
            imageManager.addImage(imageName2)
            cameraView2 = cameraview.CameraImageView(imageManager, imageName2, view=PythonQt.dd.ddQVTKWidgetView())
            cameraView2.eventFilterEnabled = False
            cameraView2.useImageColorMap = True
            cameraView2.view.renderWindow().GetInteractor().SetInteractorStyle(vtk.vtkInteractorStyleImage())
            self.views.append(cameraView2.view)

            if args.pointcloud:
                from director import segmentation
                cameraview.imageManager = imageManager
                pointCloudObj = segmentation.DisparityPointCloudItem('Point cloud', channel, imageName, imageManager)
                view = PythonQt.dd.ddQVTKWidgetView()
                pointCloudObj.addToView(view)
                om.addToObjectModel(pointCloudObj)
                pointCloudObj.setProperty('Visible', True)
                pointCloudObj.setProperty('Target FPS', 30)
                pointCloudObj.setProperty('Max Range', 30)
                pointCloudObj.setProperty('Remove Size', 0)
                viewBehaviors = viewbehaviors.ViewBehaviors(view)
                view.camera().SetPosition([0, 0, 0])
                view.camera().SetFocalPoint([0,0,1])
                view.camera().SetViewUp([0,-1,0])
                view.camera().SetViewAngle(45)
                self.views.append(view)

            self.cameraView = cameraView
            self.cameraView2 = cameraView2

        w = QtGui.QWidget()
        l = QtGui.QHBoxLayout(w)
        for view in self.views:
            l.addWidget(view)
        l.setContentsMargins(0, 0, 0, 0)
        w.resize(640*len(self.views), 480)
        w.show()
        self.addShortcuts(w)
        self.widget = w

    def start(self):
        self.app.start()


def main():
    ImageViewApp().start()


if __name__ == '__main__':
    main()
