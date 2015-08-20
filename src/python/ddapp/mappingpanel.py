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
from ddapp import drcargs


from ddapp import ioUtils as io
from ddapp import vtkNumpy as vnp
from ddapp import segmentation

import ddapp.filterUtils as filterUtils

from kinect.map_command_t import map_command_t


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

        self.ui.startMappingButton.connect("clicked()", self.onStartMappingButton)
        self.ui.stopMappingButton.connect("clicked()", self.onStopMappingButton)

        self.ui.showMapButton.connect("clicked()", self.onShowMapButton)
        self.ui.hideMapButton.connect("clicked()", self.onHideMapButton)

        self.queue = PythonQt.dd.ddBotImageQueue(lcmUtils.getGlobalLCMThread())
        self.queue.init(lcmUtils.getGlobalLCMThread(), drcargs.args().config_file)


    def onStartMappingButton(self):
        msg = map_command_t()
        msg.timestamp = getUtime()
        msg.command = 0
        lcmUtils.publish('KINECT_MAP_COMMAND', msg)

        utime = self.queue.getCurrentImageTime('KINECT_RGB')
        self.cameraToLocalInit = vtk.vtkTransform()
        self.queue.getTransform('KINECT_RGB', 'local', utime, self.cameraToLocalInit)
        vis.updateFrame(self.cameraToLocalInit, 'initial cam' )
        print "starting mapping", utime
        print self.cameraToLocalInit.GetPosition()
        print self.cameraToLocalInit.GetOrientation()

    def onStopMappingButton(self):
        msg = map_command_t()
        msg.timestamp = getUtime()
        msg.command = 1
        lcmUtils.publish('KINECT_MAP_COMMAND', msg)


    def onShowMapButton(self):
        vis.updateFrame(self.cameraToLocalInit, 'initial cam' )
        filename = os.path.expanduser('~/Kinect_Logs/Kintinuous.pcd')
        print filename
        pd = io.readPolyData(filename)
        pd = filterUtils.transformPolyData(pd, self.cameraToLocalInit )

        #t = transformUtils.frameFromPositionAndRPY([0,0,0],[-90,0,-90])
        #pd = filterUtils.transformPolyData(pd , t)

        pdi = vis.updatePolyData(pd,'map')
        pdi.setProperty('Color By', 'rgb_colors')

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
