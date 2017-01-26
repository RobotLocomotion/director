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

    fact = mainwindowapp.ComponentFactory()
    fact.register(mainwindowapp.MainWindowAppFactory)
    fact.register(mainwindowapp.MainWindowPanelFactory)

    options = fact.getDefaultOptions()
    fact.setDependentOptions(options,
        useTreeViewer=HAVE_LCMRL,
        useDrakeVisualizer=True,
        useLCMGLRenderer=True)

    fields = fact.construct(
        options=options,
        globalsDict=globalsDict,
        windowTitle=appName,
        applicationName=appName)

    if globalsDict is not None:
        globalsDict.update(**dict(fields))

    fields.app.start()


if __name__ == '__main__':
    main(globals())
