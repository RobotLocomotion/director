import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
import ddapp
from ddapp import lcmUtils
from ddapp import applogic as app
from ddapp.utime import getUtime
from ddapp.timercallback import TimerCallback
from ddapp.simpletimer import SimpleTimer
import numpy as np
import math
from time import sleep
import subprocess
import os
import vtkAll as vtk

from time import time
from copy import deepcopy
from ddapp import transformUtils
import ddapp.visualization as vis
import ddapp.objectmodel as om
from ddapp import robotstate
from ddapp import botpy


from ddapp import ioUtils as io
from ddapp import vtkNumpy as vnp
from ddapp import segmentation


# lcmtypes:
import drc as lcmdrc
from mav.indexed_measurement_t import indexed_measurement_t



def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)

class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


class MappingPanel(object):

    def __init__(self, jointController, footstepDriver):

        self.jointController = jointController
        self.footstepDriver = footstepDriver

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddMapping.ui')
        assert uifile.open(uifile.ReadOnly)
        self.widget = loader.load(uifile)
        self.ui = WidgetDict(self.widget.children())

        self.ui.showMapButton.connect("clicked()", self.onShowMapButton)
        self.ui.hideMapButton.connect("clicked()", self.onHideMapButton)

        # Data Variables:
        self.octomap_cloud = None

        self.manager = lcmUtils.LCMLoggerManager()
        #self.statusBar = statusBar

        self.lastActiveLogFile = None
        self.numProcesses = 0
        self.numLogFiles = 0
        self.userTag = ''

        #self.button = QtGui.QPushButton('')
        self.ui.logButton.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        #self.button.connect('customContextMenuRequested(const QPoint&)', self.showContextMenu)
        self.ui.logButton.connect('customContextMenuRequested(const QPoint&)', self.showContextMenu)
        self.ui.logButton.connect("clicked()", self.onClick)
        #self.button.connect('clicked()', self.onClick)


        self.timer = TimerCallback(targetFps=0.25)
        self.timer.callback = self.updateState
        self.timer.start()


    def updateState(self):

        t = SimpleTimer()
        self.manager.updateExistingLoggerProcesses()

        activeLogFiles = self.manager.getActiveLogFilenames()
        self.numProcesses = len(self.manager.getActiveLoggerPids())
        self.numLogFiles = len(activeLogFiles)

        if self.numLogFiles == 1:
            self.lastActiveLogFile = activeLogFiles[0]

        if self.numProcesses == 0:
            self.ui.logButton.text = 'xxstart logger'
        elif self.numProcesses == 1:
            self.ui.logButton.text = 'xxstop logger'
        elif self.numProcesses > 1:
            self.ui.logButton.text = 'xxstop all loggers'

        statusDescription = 'active' if self.numProcesses else 'last'
        logFileDescription = self.lastActiveLogFile or '<unknown>'
        self.ui.logButton.setToolTip('%s log file: %s' % (statusDescription, logFileDescription))


    def onClick(self):
        if self.numProcesses == 0:
            self.manager.startNewLogger(tag=self.userTag)
            self.updateState()
            self.showStatusMessage('start logging: ' + self.lastActiveLogFile)
        else:
            self.manager.killAllLoggingProcesses()
            self.showStatusMessage('stopped logging')
            self.updateState()

    def showStatusMessage(self, msg, timeout=2000):
        x = 0
        #if self.statusBar:
        #    self.statusBar.showMessage(msg, timeout)

    def showContextMenu(self, clickPosition):

        globalPos = self.ui.logButton.mapToGlobal(clickPosition)

        menu = QtGui.QMenu()

        action = menu.addAction('Stop logger')
        action.enabled = (self.numProcesses > 0)

        action = menu.addAction('Stop and delete log file')
        action.enabled = (self.numProcesses > 0 and self.lastActiveLogFile)

        action = menu.addAction('Set logger tag')
        action.enabled = (self.numProcesses == 0)

        action = menu.addAction('Copy log filename')
        action.enabled = (self.lastActiveLogFile is not None)

        action = menu.addAction('Review log')
        action.enabled = (self.lastActiveLogFile is not None)


        selectedAction = menu.exec_(globalPos)
        if selectedAction is None:
            return

        if selectedAction.text == 'Copy log filename':
            clipboard = QtGui.QApplication.instance().clipboard()
            clipboard.setText(self.lastActiveLogFile)
            self.showStatusMessage('copy to clipboard: ' + self.lastActiveLogFile)

        elif selectedAction.text == 'Stop logger':
            self.manager.killAllLoggingProcesses()
            self.showStatusMessage('stopped logger')
            self.updateState()

        elif selectedAction.text == 'Stop and delete log file':
            logFileToRemove = self.lastActiveLogFile
            self.manager.killAllLoggingProcesses()
            self.updateState()
            os.remove(logFileToRemove)
            self.showStatusMessage('deleted: ' + logFileToRemove)

        elif selectedAction.text == 'Set logger tag':
            inputDialog = QtGui.QInputDialog()
            inputDialog.setInputMode(inputDialog.TextInput)
            inputDialog.setLabelText('Log file tag:')
            inputDialog.setWindowTitle('Enter tag')
            inputDialog.setTextValue(self.userTag)
            result = inputDialog.exec_()

            if result:
                tag = inputDialog.textValue()
                self.userTag = tag
                self.showStatusMessage('Set lcm logger tag: ' + self.userTag)

        elif selectedAction.text == 'Review log':
            newEnv = dict(os.environ)
            newEnv['LCM_DEFAULT_URL'] = newEnv['LCM_REVIEW_DEFAULT_URL']
            devnull = open(os.devnull, 'w')
            print 'process ', self.lastActiveLogFile
            subprocess.Popen(['Kintinuous','-a', '-m', '-f', '-s', '6', '-l' ,self.lastActiveLogFile ], stdout=devnull, stderr=devnull, env=newEnv)


    ###############################
    def onShowMapButton(self):
        # reloads the map each time - in case its changed on disk:
        #if (self.octomap_cloud is None):
        filename = self.lastActiveLogFile + ".pcd"
        #filename = ddapp.getDRCBaseDir() + "/software/build/data/octomap.pcd"
        self.octomap_cloud = io.readPolyData(filename) # c++ object called vtkPolyData

        assert (self.octomap_cloud.GetNumberOfPoints() !=0 )

        # clip point cloud to height - doesnt work very well yet. need to know robot's height
        #self.octomap_cloud = segmentation.cropToLineSegment(self.octomap_cloud, np.array([0,0,-10]), np.array([0,0,3]) )

        # access to z values
        #points= vnp.getNumpyFromVtk(self.octomap_cloud, 'Points')
        #zvalues = points[:,2]

        # remove previous map:
        folder = om.getOrCreateContainer("segmentation")
        om.removeFromObjectModel(folder)
        vis.showPolyData(self.octomap_cloud, 'prior map', alpha=1.0, color=[0,0,0.4])


    def onHideMapButton(self):
        folder = om.getOrCreateContainer("segmentation")
        om.removeFromObjectModel(folder)


def _getAction():
    return app.getToolBarActions()['ActionMappingPanel']


def init(jointController, footstepDriver):

    global panel
    global dock

    panel = MappingPanel(jointController, footstepDriver)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel
