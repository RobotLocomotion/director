from ddapp.consoleapp import ConsoleApp
from ddapp import cameraview


def main():

    app = ConsoleApp()

    view = app.createView(useGrid=False)
    imageManager = cameraview.ImageManager()
    cameraView = cameraview.CameraView(imageManager, view)

    view.show()
    app.start()


if __name__ == '__main__':
    main()
