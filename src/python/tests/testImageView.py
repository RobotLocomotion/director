import argparse
from ddapp.consoleapp import ConsoleApp
from ddapp import cameraview
from ddapp import vtkAll as vtk


def parseChannelArgument(defaultChannel='CAMERA_LEFT'):

    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--channel', type=str, help='image channel', default=defaultChannel)
    args, unknown = parser.parse_known_args()
    return args.channel


def main():

    app = ConsoleApp()

    view = app.createView(useGrid=False)
    view.orientationMarkerWidget().Off()
    view.backgroundRenderer().SetBackground([0,0,0])
    view.backgroundRenderer().SetBackground2([0,0,0])


    cameraChannel = parseChannelArgument()
    imageManager = cameraview.ImageManager()
    imageManager.queue.addCameraStream(cameraChannel)
    imageManager.addImage(cameraChannel)


    cameraView = cameraview.CameraImageView(imageManager, cameraChannel, view=view)
    cameraView.eventFilterEnabled = False
    view.renderWindow().GetInteractor().SetInteractorStyle(vtk.vtkInteractorStyleImage())


    view.show()
    app.start()


if __name__ == '__main__':
    main()
