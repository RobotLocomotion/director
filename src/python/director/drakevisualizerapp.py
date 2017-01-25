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

    if '--testing' not in sys.argv:
        drcargs.requireStrict()

    appName = 'Drake Visualizer'
    app = mainwindowapp.MainWindowAppFactory().construct(globalsDict=globalsDict, windowTitle=appName, applicationName=appName)

    fact = mainwindowapp.MainWindowPanelFactory()

    options = fact.getDefaultOptions()
    fact.setDependentOptions(options,
        useTreeViewer=HAVE_LCMRL,
        useDrakeVisualizer=True,
        useLCMGLRenderer=True)

    fields = fact.construct(options, app=app.app, view=app.view)

    if globalsDict is not None:
        for d in [app, fields]:
            globalsDict.update(**dict(d))

    app.app.start()


if __name__ == '__main__':
    main(globals())
