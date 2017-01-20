from director import drakevisualizer2
from director.drakevisualizerapp import DrakeVisualizerApp


def main(globalsDict=None):
    app = DrakeVisualizerApp(drakevisualizer2.DrakeVisualizer)
    app.start(globalsDict)


if __name__ == '__main__':
    main(globals())
