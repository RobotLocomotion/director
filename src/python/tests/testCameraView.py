from director.consoleapp import ConsoleApp
from director import cameraview


def main():

    app = ConsoleApp()

    view = app.createView(useGrid=False)
    imageManager = cameraview.ImageManager()
    cameraView = cameraview.CameraView(imageManager, view)

    view.show()
    app.start()


if __name__ == '__main__':
    main()
