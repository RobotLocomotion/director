import os
import sys
import traceback
import argparse

from director import applogic
from director import drcargs
from director import objectmodel as om
from director import viewbehaviors
from director import visualization as vis
from director.timercallback import TimerCallback

import PythonQt
from PythonQt import QtCore, QtGui


def _consoleAppExceptionHook(exc_type, exc_value, exc_traceback):
    msg =  ''.join(traceback.format_exception(exc_type, exc_value, exc_traceback))
    sys.stderr.write(msg)
    ConsoleApp.exit(1)


class ConsoleApp(object):

    _startupCallbacks = {}
    _exitCode = 0
    _quitTimer = None

    def __init__(self):
        om.init()
        self.objectModelWidget = None

    @staticmethod
    def start(enableAutomaticQuit=True):
        '''
        In testing mode, the application will quit automatically after starting
        unless enableAutomaticQuit is set to False.  Tests that need to perform
        work after the QApplication has started can set this flag to False and
        call quit or exit themselves.

        In testing mode, this function will register an exception hook so that
        tests will return on error code if an unhandled exception is raised.
        '''
        if enableAutomaticQuit:
            ConsoleApp.startTestingModeQuitTimer()

        if ConsoleApp.getTestingEnabled() and not ConsoleApp.getTestingInteractiveEnabled():
            sys.excepthook = _consoleAppExceptionHook

        def onStartup():
            callbacks = []
            for priority in sorted(ConsoleApp._startupCallbacks.keys()):
                callbacks.extend(ConsoleApp._startupCallbacks[priority])
            for func in callbacks:
                try:
                    func()
                except:
                    if ConsoleApp.getTestingEnabled():
                        raise
                    else:
                        print(traceback.format_exc())

        startTimer = TimerCallback(callback=onStartup)
        startTimer.singleShot(0)

        result = ConsoleApp.applicationInstance().exec_()

        if ConsoleApp.getTestingEnabled() and not ConsoleApp.getTestingInteractiveEnabled():
            print('TESTING PROGRAM RETURNING EXIT CODE:', result)
            sys.exit(result)

        return result


    @staticmethod
    def startTestingModeQuitTimer(timeoutInSeconds=0.1):
        if ConsoleApp.getTestingEnabled() and not ConsoleApp.getTestingInteractiveEnabled():
            ConsoleApp.startQuitTimer(timeoutInSeconds)

    @staticmethod
    def startQuitTimer(timeoutInSeconds):
        quitTimer = TimerCallback()
        quitTimer.callback = ConsoleApp.quit
        quitTimer.singleShot(timeoutInSeconds)
        ConsoleApp._quitTimer = quitTimer

    @staticmethod
    def getQuitTimer():
        return ConsoleApp._quitTimer

    @staticmethod
    def quit():
        ConsoleApp.exit(ConsoleApp._exitCode)

    @staticmethod
    def exit(exitCode=0):
        ConsoleApp._exitCode = exitCode
        ConsoleApp.applicationInstance().exit(exitCode)

    @staticmethod
    def applicationInstance():
        return QtCore.QCoreApplication.instance()

    @staticmethod
    def processEvents():
        QtCore.QCoreApplication.instance().processEvents()

    @staticmethod
    def registerStartupCallback(func, priority=1):
        ConsoleApp._startupCallbacks.setdefault(priority, []).append(func)

    def showObjectModel(self):

        if not self.objectModelWidget:
            w = QtGui.QSplitter(QtCore.Qt.Vertical)
            model = om.getDefaultObjectModel()
            w.addWidget(model.getTreeWidget())
            sw = QtGui.QScrollArea()
            sw.setWidget(model.getPropertiesPanel())
            sw.setWidgetResizable(True)
            w.addWidget(sw)
            applogic.addShortcut(w, 'Ctrl+Q', self.quit)
            self.objectModelWidget = w
            self.objectModelWidget.resize(350, 700)
            w.setSizes([350, 350])

        self.objectModelWidget.show()
        self.objectModelWidget.raise_()
        self.objectModelWidget.activateWindow()
        return self.objectModelWidget

    def createView(self, useGrid=True):
        view = PythonQt.dd.ddQVTKWidgetView()
        applogic._defaultRenderView = view
        view.resize(600, 400)

        applogic.setCameraTerrainModeEnabled(view, True)
        if useGrid:
            self.gridObj = vis.showGrid(view, parent='scene')

        self.viewOptions = vis.ViewOptionsItem(view)
        om.addToObjectModel(self.viewOptions, parentObj=om.findObjectByName('scene'))

        applogic.resetCamera(viewDirection=[-1,-1,-0.3], view=view)
        self.viewBehaviors = viewbehaviors.ViewBehaviors(view)

        applogic.addShortcut(view, 'Ctrl+Q', self.quit)
        applogic.addShortcut(view, 'F8', self.showPythonConsole)
        applogic.addShortcut(view, 'F1', self.showObjectModel)

        view.setWindowIcon(om.Icons.getIcon(om.Icons.Robot))
        view.setWindowTitle('View')

        return view

    @staticmethod
    def showPythonConsole():
        applogic.showPythonConsole()

    def setupGlobals(self, globalsDict):

        quit = ConsoleApp.quit
        exit = ConsoleApp.exit

        globalsDict.update(locals())
        for arg in ['globalsDict', 'self']:
            del globalsDict[arg]

    @staticmethod
    def getTestingArgs(dataDirRequired=False, outputDirRequired=False):

      parser = drcargs.DRCArgParser().getParser()
      parser.add_argument('--testing', action='store_true', help='enable testing mode')
      parser.add_argument('--data-dir', type=str, help='testing data directory', required=dataDirRequired)
      parser.add_argument('--output-dir', type=str, help='output directory for writing test output', required=outputDirRequired)
      parser.add_argument('--interactive', action='store_true', help='enable interactive testing mode')

      args, unknown = parser.parse_known_args()
      return args

    @staticmethod
    def getTestingDataDirectory():
        path = ConsoleApp.getTestingArgs(dataDirRequired=True).data_dir
        if not os.path.isdir(path):
            raise Exception('Testing data directory does not exist: %s' % path)
        return path

    @staticmethod
    def getTestingOutputDirectory(outputDirRequired=True):
        path = ConsoleApp.getTestingArgs().output_dir
        if not os.path.isdir(path):
            raise Exception('Testing output directory does not exist: %s' % path)
        return path

    @staticmethod
    def getTestingInteractiveEnabled():
        return ConsoleApp.getTestingArgs().interactive

    @staticmethod
    def getTestingEnabled():
        return ConsoleApp.getTestingArgs().testing



def main(globalsDict=None):

    app = ConsoleApp()
    app.showPythonConsole()
    view = app.createView()
    view.show()
    view.raise_()
    view.activateWindow()

    if globalsDict is not None:
        app.setupGlobals(globalsDict)
        globalsDict.update(dict(view=view, app=app))

    app.start()


if __name__ == '__main__':
    main(globals())
