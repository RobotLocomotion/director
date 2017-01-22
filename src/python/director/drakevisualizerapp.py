import sys
from director import drcargs
from director import mainwindowapp


def main(globalsDict=None):

    appName = 'Drake Visualizer'
    app = mainwindowapp.MainWindowAppFactory().construct(globalsDict=globalsDict, windowTitle=appName, applicationName=appName)

    fact = mainwindowapp.MainWindowPanelFactory()
    options = fact.getDefaultOptions()
    options.useLCMGLRenderer = True
    fact.setDependentOptions(options, useTreeViewer=True)
    fact.setDependentOptions(options, useDrakeVisualizer=True)
    fact.construct(options, app=app.app, view=app.view)

    if globalsDict is not None:
        globalsDict.update(**dict(app))

    app.app.start()


if __name__ == '__main__':
    main(globals())
