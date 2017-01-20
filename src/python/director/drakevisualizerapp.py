import sys
from director import drcargs
from director import mainwindowapp


def main(globalsDict=None):

    if '--testing' not in sys.argv:
        drcargs.requireStrict()

    parser = drcargs.getGlobalArgParser().getParser()
    parser.add_argument('--protocol', dest='visualizer_protocol', default='drake', type=str, help='Visualizer protocol (drake or json)')
    args = drcargs.args()

    knownProtocols = ('drake', 'json')
    if args.visualizer_protocol not in knownProtocols:
        print
        print 'Unrecognized visualizer protocol:', args.visualizer_protocol
        print 'Available protocols:', ', '.join(knownProtocols)
        print
        sys.exit(1)

    appName = 'Drake Visualizer'
    app = mainwindowapp.MainWindowAppFactory().construct(globalsDict=globalsDict, windowTitle=appName, applicationName=appName)

    fact = mainwindowapp.MainWindowPanelFactory()
    options = fact.getDefaultOptions()
    options.useLCMGLRenderer = True

    if args.visualizer_protocol == 'json':
        fact.setDependentOptions(options, useLCMVisualizer=True)
    elif args.visualizer_protocol == 'drake':
        fact.setDependentOptions(options, useDrakeVisualizer=True)

    fact.construct(options, app=app.app, view=app.view)

    if globalsDict is not None:
        globalsDict.update(**dict(app))

    app.app.start()


if __name__ == '__main__':
    main(globals())
