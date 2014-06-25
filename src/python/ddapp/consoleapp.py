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


class ConsoleApp(object):

    def __init__(self):
        om.init()
        self.objectModelWidget = None

    @staticmethod
    def start():

        if ConsoleApp.getTestingEnabled() and not ConsoleApp.getTestingInteractiveEnabled():
            quitTimer = TimerCallback()
            quitTimer.callback = ConsoleApp.quit
            quitTimer.singleShot(0.1)

        ConsoleApp.applicationInstance().exec_()

    @staticmethod
    def quit():
        ConsoleApp.applicationInstance().quit()

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
            w.show()
            self.objectModelWidget = w
            self.objectModelWidget.resize(350, 700)
        self.objectModelWidget.show()

    @staticmethod
    def showPythonConsole():
        applogic.showPythonConsole()

    @staticmethod
    def createView():
        view = PythonQt.dd.ddQVTKWidgetView()
        view.resize(600, 400)

        applogic.setCameraTerrainModeEnabled(view, True)
        vis.showGrid(view)

        applogic.resetCamera(viewDirection=[-1,-1,-0.3], view=view)
        viewBehaviors = viewbehaviors.ViewBehaviors(view)
        applogic._defaultRenderView = view

        applogic.addShortcut(view, 'Ctrl+Q', ConsoleApp.quit)
        applogic.addShortcut(view, 'F8', ConsoleApp.showPythonConsole)

        view.setWindowIcon(om.Icons.Robot)
        view.setWindowTitle('View')

        return view

    def setupGlobals(self, globalsDict):

        quit = ConsoleApp.quit
        exit = quit

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

      return parser.parse_args()

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
