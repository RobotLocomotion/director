import os
import sys
import vtk
import math
import PythonQt
from PythonQt import QtCore, QtGui
import ddapp.objectmodel as om
from ddapp import robotstate
from ddapp.timercallback import TimerCallback
from ddapp.utime import getUtime
import vtkDRCFiltersPython as drc
from ddapp.debugVis import DebugData
import numpy as np

import drc as lcmdrc
import multisense as lcmmultisense
import lcmUtils

_view = None
def getRenderView():
    return _view


_debugItem = None

def updateDebugItem(polyData):
    global _debugItem
    if not _debugItem:
        _debugItem = om.PolyDataItem('spindle axis', polyData, getRenderView())
        _debugItem.setProperty('Color', QtGui.QColor(0, 255, 0))
        _debugItem.setProperty('Visible', False)
        om.addToObjectModel(_debugItem, om.findObjectByName('sensors'))
    else:
        _debugItem.setPolyData(polyData)


class MultisenseItem(om.ObjectModelItem):

    def __init__(self, model):

        om.ObjectModelItem.__init__(self, 'Multisense', om.Icons.Laser)

        self.model = model
        self.addProperty('Updates Enabled', True)
        self.addProperty('Min Range', model.reader.GetDistanceRange()[0])
        self.addProperty('Max Range', model.reader.GetDistanceRange()[1])
        self.addProperty('Edge Filter Distance', model.reader.GetEdgeDistanceThreshold())
        self.addProperty('Number of Scan Lines', model.numberOfScanLines)
        self.addProperty('Visible', model.visible)
        self.addProperty('Point Size', model.pointSize)
        self.addProperty('Alpha', model.alpha)

        #self.addProperty('Color', QtGui.QColor(255,255,255))
        #self.addProperty('Scanline Color', QtGui.QColor(255,0,0))

    def _onPropertyChanged(self, propertyName):
        om.ObjectModelItem._onPropertyChanged(self, propertyName)

        if propertyName == 'Updates Enabled':
            if self.getProperty('Updates Enabled'):
                self.model.start()
            else:
                self.model.stop()

        elif propertyName == 'Edge Filter Distance':
            self.model.reader.SetEdgeDistanceThreshold(self.getProperty('Edge Filter Distance'))
            self.model.showRevolution(self.model.displayedRevolution)

        elif propertyName == 'Alpha':
            self.model.setAlpha(self.getProperty(propertyName))

        elif propertyName == 'Visible':
            self.model.setVisible(self.getProperty(propertyName))

        elif propertyName == 'Point Size':
            self.model.setPointSize(self.getProperty(propertyName))

        elif propertyName == 'Number of Scan Lines':
            self.model.numberOfScanLines = self.getProperty(propertyName)
            self.model.initScanLines()

        elif propertyName in ('Min Range', 'Max Range'):
            self.model.reader.SetDistanceRange(self.getProperty('Min Range'), self.getProperty('Max Range'))
            self.model.showRevolution(self.model.displayedRevolution)

        _view.render()


    def getPropertyAttributes(self, propertyName):

        if propertyName == 'Point Size':
            return om.PropertyAttributes(decimals=0, minimum=1, maximum=20, singleStep=1, hidden=False)

        elif propertyName in ('Min Range', 'Max Range'):
            return om.PropertyAttributes(decimals=2, minimum=0.0, maximum=100.0, singleStep=0.25, hidden=False)
        elif propertyName == 'Edge Filter Distance':
            return om.PropertyAttributes(decimals=3, minimum=0.0, maximum=10.0, singleStep=0.01, hidden=False)
        elif propertyName == 'Number of Scan Lines':
            return om.PropertyAttributes(decimals=0, minimum=0, maximum=100, singleStep=1, hidden=False)
        else:
            return om.ObjectModelItem.getPropertyAttributes(self, propertyName)


class MultiSenseSource(TimerCallback):

    def __init__(self, view):
        TimerCallback.__init__(self)
        self.view = view
        self.reader = None
        self.displayedRevolution = -1
        self.lastScanLine = 0
        self.numberOfScanLines = 1
        self.nextScanLineId = 0
        self.scanLines = []
        self.pointSize = 1
        self.alpha = 0.5
        self.visible = True
        self.initScanLines()

        self.revPolyData = vtk.vtkPolyData()
        self.polyDataObj = om.PolyDataItem('Multisense Scan', self.revPolyData, view)
        self.polyDataObj.actor.SetPickable(1)


        self.setPointSize(self.pointSize)
        self.setAlpha(self.alpha)
        self.targetFps = 60
        self.showRevolutionCallback = None
        self.colorizeCallback = None

    def initScanLines(self):

        for scanLine in self.scanLines:
            scanLine.removeFromAllViews()

        self.scanLines = []
        self.nextScanLineId = 0
        self.lastScanLine = max(self.lastScanLine - self.numberOfScanLines, 0)

        for i in xrange(self.numberOfScanLines):
            polyData = vtk.vtkPolyData()
            scanLine = om.PolyDataItem('scan line %d' % i, polyData, self.view)
            scanLine.actor.SetPickable(0)
            scanLine.setSolidColor((1,0,0))
            self.scanLines.append(scanLine)

    def getScanToLocal(self):
        return None

    def showRevolution(self, revId):

        colorByArray = self.polyDataObj.getColorByArrayName()
        self.reader.GetDataForRevolution(revId, self.revPolyData)

        self.displayedRevolution = revId

        if self.showRevolutionCallback:
            self.showRevolutionCallback()
        if self.colorizeCallback:
            self.colorizeCallback()

        self.polyDataObj.colorBy(colorByArray, lut=self.polyDataObj.mapper.GetLookupTable())
        self.view.render()


    def setPointSize(self, pointSize):
        for scanLine in self.scanLines:
            scanLine.setProperty('Point Size', pointSize + 2)
        self.polyDataObj.setProperty('Point Size', pointSize)

    def setAlpha(self, alpha):
        self.alpha = alpha
        for scanLine in self.scanLines:
            scanLine.setProperty('Alpha', alpha)
        self.polyDataObj.setProperty('Alpha', alpha)

    def setVisible(self, visible):
        self.visible = visible
        for scanLine in self.scanLines:
            scanLine.setProperty('Visible', visible)
        self.polyDataObj.setProperty('Visible', visible)

    def start(self):
        if self.reader is None:
            self.reader = drc.vtkMultisenseSource()
            self.reader.SetDistanceRange(0.25, 4.0)
            self.reader.Start()

        TimerCallback.start(self)

    def updateScanLines(self):

        if not self.numberOfScanLines:
            return

        currentScanLine = self.reader.GetCurrentScanLine() - 1
        scanLinesToUpdate = currentScanLine - self.lastScanLine
        scanLinesToUpdate = min(scanLinesToUpdate, self.numberOfScanLines)

        if not scanLinesToUpdate:
            return

        #print 'current scanline:', currentScanLine
        #print 'scan lines to update:', scanLinesToUpdate
        #print 'updating actors:', self.nextScanLineId, (self.nextScanLineId + (scanLinesToUpdate-1)) % self.numberOfActors
        #print 'updating scan lines:', self.lastScanLine + 1, self.lastScanLine + 1 + (scanLinesToUpdate-1)

        for i in xrange(scanLinesToUpdate):
            scanLine = self.scanLines[(self.nextScanLineId + i) % self.numberOfScanLines]
            self.reader.GetDataForScanLine(self.lastScanLine + i + 1, scanLine.polyData)

        self.lastScanLine = currentScanLine
        self.nextScanLineId = (self.nextScanLineId + scanLinesToUpdate) % self.numberOfScanLines

        self.view.render()


    def updateRevolution(self):
        currentRevolution = self.reader.GetCurrentRevolution() - 1
        if currentRevolution == self.displayedRevolution:
            return

        self.showRevolution(currentRevolution)

    def getSpindleAxis(self):
        return self.getAxis('SCAN', [1.0, 0.0, 0.0])

    def getAxis(self, frameName, axisVector):
        t = self.getFrame(frameName)
        return np.array(t.TransformVector(axisVector))

    def getFrame(self, name, relativeTo='local'):
        t = vtk.vtkTransform()
        self.reader.GetTransform(name, relativeTo, self.reader.GetCurrentScanTime(), t)
        return t


    def updateDebugItems(self):

        t = self.getFrame('SCAN')

        p1 = [0.0, 0.0, 0.0]
        p2 = [2.0, 0.0, 0.0]

        p1 = t.TransformPoint(p1)
        p2 = t.TransformPoint(p2)

        d = DebugData()
        d.addSphere(p1, radius=0.01, color=[0,1,0])
        d.addLine(p1, p2, color=[0,1,0])
        updateDebugItem(d.getPolyData())


    def tick(self):

        self.updateDebugItems()
        self.updateRevolution()
        self.updateScanLines()


    @staticmethod
    def setLidarRevolutionTime(secondsPerRevolution):

        assert secondsPerRevolution >= 1.0

        m = lcmmultisense.command_t()
        m.utime = getUtime()
        m.fps = -1
        m.gain = -1
        m.agc = -1
        m.rpm = 60.0 / (secondsPerRevolution)

        lcmUtils.publish('MULTISENSE_COMMAND', m)

    @staticmethod
    def setLidarRpm(rpm):

        m = lcmmultisense.command_t()
        m.utime = getUtime()
        m.fps = -1
        m.gain = -1
        m.agc = -1
        m.rpm = rpm

        lcmUtils.publish('MULTISENSE_COMMAND', m)

    @staticmethod
    def setMultisenseCommand(fps, gain, agc, rpm, led_flash, led_duty):

        m = lcmmultisense.command_t()
        m.utime = getUtime()
        m.fps = fps
        m.gain = gain
        m.agc = agc
        m.rpm = rpm
        m.leds_flash = led_flash
        m.leds_duty_cycle = led_duty

        lcmUtils.publish('MULTISENSE_COMMAND', m)

    @staticmethod
    def setNeckPitch(neckPitchDegrees):

        assert neckPitchDegrees <= 90 and neckPitchDegrees >= -90
        m = lcmdrc.neck_pitch_t()
        m.utime = 0
        m.pitch = math.radians(neckPitchDegrees)
        lcmUtils.publish('DESIRED_NECK_PITCH', m)



class MapServerSource(TimerCallback):

    def __init__(self, polyData=None, callbackFunc=None):
        TimerCallback.__init__(self)
        self.reader = None
        self.displayedMapId = -1
        self.displayedViewId = lcmdrc.data_request_t.DEPTH_MAP_WORKSPACE
        self.targetFps = 10
        self.polyData = polyData or vtk.vtkPolyData()
        self.callbackFunc = callbackFunc
        self.colorizeCallback = None

    def showMap(self, mapId):
        polyData = vtk.vtkPolyData()
        self.reader.GetDataForMapId(self.displayedViewId, mapId, polyData)
        self.polyData.ShallowCopy(polyData)
        if self.colorizeCallback:
            self.colorizeCallback()

        self.displayedMapId = mapId
        if self.callbackFunc:
            self.callbackFunc()

    def start(self):
        if self.reader is None:
            self.reader = drc.vtkMapServerSource()
            self.reader.Start()

        TimerCallback.start(self)

    def updateMap(self):
        mapId = self.reader.GetCurrentMapId(self.displayedViewId)
        if mapId != self.displayedMapId:
            self.showMap(mapId)

    def tick(self):
        self.updateMap()


def init(view):
    global _multisenseItem
    global _view
    global multisenseDriver

    _view = view

    m = MultiSenseSource(view)
    m.start()
    multisenseDriver = m

    sensorsFolder = om.getOrCreateContainer('sensors')

    _multisenseItem = MultisenseItem(m)
    om.addToObjectModel(_multisenseItem, sensorsFolder)

    useMapServer = hasattr(drc, 'vtkMapServerSource')
    if useMapServer:
        mapServerSource = MapServerSource(callbackFunc=view.render)
        mapServerObj = om.PolyDataItem('Map Server', mapServerSource.polyData, view)
        mapServerObj.source = mapServerSource
        mapServerSource.start()
        om.addToObjectModel(mapServerObj, sensorsFolder)

    return _multisenseItem


