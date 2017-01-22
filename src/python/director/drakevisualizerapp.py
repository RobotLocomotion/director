import sys
from director import drcargs
from director import mainwindowapp

# todo:
# this check is required because openhumanoids
# does not yet have robotlocomotion/lcmtypes
try:
    import robotlocomotion as lcmrl
    HAVE_LCMRL = True
except ImportError:
    HAVE_LCMRL = False


def main(globalsDict=None):

    appName = 'Drake Visualizer'
    app = mainwindowapp.MainWindowAppFactory().construct(globalsDict=globalsDict, windowTitle=appName, applicationName=appName)

    fact = mainwindowapp.MainWindowPanelFactory()

    options = fact.getDefaultOptions()
    fact.setDependentOptions(options,
        useTreeViewer=HAVE_LCMRL,
        useDrakeVisualizer=True,
        useLCMGLRenderer=True)

    fact.construct(options, app=app.app, view=app.view)

    if globalsDict is not None:
        globalsDict.update(**dict(app))

    app.app.start()


if __name__ == '__main__':
    main(globals())
