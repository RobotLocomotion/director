import os
import sys
import argparse
import PythonQt
from PythonQt import QtCore, QtGui

import ddapp
from ddapp import applogic
from ddapp import objectmodel as om
from ddapp import vtkAll as vtk
from ddapp import vtkNumpy as vnp
from ddapp import viewbehaviors
from ddapp import visualization as vis
from ddapp.timercallback import TimerCallback

import numpy as np

import sys
import traceback


def _consoleAppExceptionHook(exc_type, exc_value, exc_traceback):
    msg =  ''.join(traceback.format_exception(exc_type, exc_value, exc_traceback))
    sys.stderr.write(msg)
    ConsoleApp.exit(1)


class ConsoleApp(object):

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

        result = ConsoleApp.applicationInstance().exec_()

        if ConsoleApp.getTestingEnabled() and not ConsoleApp.getTestingInteractiveEnabled():
            print 'TESTING PROGRAM RETURNING EXIT CODE:', result
            sys.exit(result)


    @staticmethod
    def startTestingModeQuitTimer(timeoutInSeconds=0.1):
        if ConsoleApp.getTestingEnabled() and not ConsoleApp.getTestingInteractiveEnabled():
            ConsoleApp.startQuitTimer(timeoutInSeconds)

    @staticmethod
    def startQuitTimer(timeoutInSeconds):
        quitTimer = TimerCallback()
        quitTimer.callback = ConsoleApp.quit
        quitTimer.singleShot(timeoutInSeconds)

    @staticmethod
    def quit():
        ConsoleApp.applicationInstance().quit()

    @staticmethod
    def exit(exitCode=0):
        ConsoleApp.applicationInstance().exit(exitCode)

    @staticmethod
    def applicationInstance():
        return QtCore.QCoreApplication.instance()

    def showObjectModel(self):

        if not self.objectModelWidget:
            w = QtGui.QWidget()
            l = QtGui.QVBoxLayout(w)
            model = om.getDefaultObjectModel()
            l.addWidget(model.getTreeWidget())
            l.addWidget(model.getPropertiesPanel())
            applogic.addShortcut(w, 'Ctrl+Q', self.quit)
            self.objectModelWidget = w
            self.objectModelWidget.resize(350, 700)

        self.objectModelWidget.show()
        self.objectModelWidget.raise_()
        self.objectModelWidget.activateWindow()
        return self.objectModelWidget

    def createView(self, useGrid=True):
        view = PythonQt.dd.ddQVTKWidgetView()
        view.resize(600, 400)

        applogic.setCameraTerrainModeEnabled(view, True)
        if useGrid:
            self.gridObj = vis.showGrid(view, parent='scene')

        self.viewOptions = vis.ViewOptionsItem(view)
        om.addToObjectModel(self.viewOptions, parentObj=om.findObjectByName('scene'))

        applogic.resetCamera(viewDirection=[-1,-1,-0.3], view=view)
        viewBehaviors = viewbehaviors.ViewBehaviors(view)
        applogic._defaultRenderView = view

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

      parser = argparse.ArgumentParser()
      parser.add_argument('--testing', action='store_true', help='enable testing mode')
      parser.add_argument('-d', '--data-dir', type=str, help='testing data directory', required=dataDirRequired)
      parser.add_argument('-o', '--output-dir', type=str, help='output directory for writing test output', required=outputDirRequired)
      parser.add_argument('-i', '--interactive', action='store_true', help='enable interactive testing mode')

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



def main():

    global app, view
    app = ConsoleApp()
    app.setupGlobals(globals())
    app.showPythonConsole()
    view = app.createView()
    view.show()
    view.raise_()
    view.activateWindow()
    app.start()


if __name__ == '__main__':
    main()
