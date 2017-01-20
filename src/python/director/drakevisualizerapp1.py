from director import drakevisualizer
from director.drakevisualizerapp import DrakeVisualizerApp


def main(globalsDict=None):
    app = DrakeVisualizerApp(drakevisualizer.DrakeVisualizer)
    app.start(globalsDict)


if __name__ == '__main__':
    main(globals())
