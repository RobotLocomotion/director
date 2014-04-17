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
import ddapp.visualization as vis
from ddapp import vtkNumpy as vnp
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
        self.scalarBarWidget = None
        self.addProperty('Color By', 0, attributes=om.PropertyAttributes(enumNames=['Solid Color', 'Intensity', 'Z Coordinate', 'Range', 'Spindle Angle', 'Azimuth', 'Camera RGB', 'Scan Delta']))
        self.addProperty('Show Scalar Bar', False)
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

        elif propertyName == 'Color By':
            self._updateColorBy()

        elif propertyName == 'Show Scalar Bar':
            self._updateScalarBar()

        _view.render()


    def _updateColorBy(self):

        arrayMap = {
          0 : None,
          1 : 'intensity',
          2 : 'z',
          3 : 'distance',
          4 : 'spindle_angle',
          5 : 'azimuth',
          6 : 'rgb',
          7 : 'scan_delta'
          }

        rangeMap = {
            1 : (400, 4000),
            2 : (0.0, 2.0),
            3 : (0.5, 4.0),
            4 : (0, 360),
            5 : (-2.5, 2.5),
            7 : (0.0, 0.3)
        }

        colorBy = self.getProperty('Color By')
        scalarRange = rangeMap.get(colorBy)
        arrayName = arrayMap.get(colorBy)

        if arrayName:
            if arrayName == 'rgb' and arrayName not in self.model.polyDataObj.getArrayNames():
                self.model.colorizeCallback()
            self.model.polyDataObj.colorBy(arrayName, scalarRange=scalarRange)
        else:
            self.model.polyDataObj.colorBy(None)

        self._updateScalarBar()

    def hasDataSet(self, dataSet):
        return self.model.polyDataObj.hasDataSet(dataSet)

    def _updateScalarBar(self):
        barEnabled = self.getProperty('Show Scalar Bar')
        colorBy = self.getProperty('Color By')
        if barEnabled and colorBy not in (0, 6):
            self._showScalarBar()
        else:
            self._hideScalarBar()

    def _hideScalarBar(self):
        if self.scalarBarWidget:
            self.scalarBarWidget.Off()
            self.scalarBarWidget.SetInteractor(None)
            self.scalarBarWidget = None
            self.model.polyDataObj._renderAllViews()

    def _showScalarBar(self):
        title = self.getPropertyAttributes('Color By').enumNames[self.getProperty('Color By')]
        view = self.model.polyDataObj.views[0]
        lut = self.model.polyDataObj.mapper.GetLookupTable()
        self.scalarBarWidget = vis.createScalarBarWidget(view, lut, title)
        self.model.polyDataObj._renderAllViews()

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
        self.polyDataObj = om.PolyDataItem('Multisense Sweep', self.revPolyData, view)
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
        if self.colorizeCallback and colorByArray == 'rgb':
            self.colorizeCallback()

        self.polyDataObj.colorBy(colorByArray, lut=self.polyDataObj.mapper.GetLookupTable())

        if self.polyDataObj.getProperty('Visible'):
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

        if self.scanLines[0].getProperty('Visible'):
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

    def __init__(self, view, callbackFunc=None):
        TimerCallback.__init__(self)
        self.reader = None
        self.view = view
        self.displayedMapIds = {}
        self.polyDataObjects = {}
        self.targetFps = 10
        self.callbackFunc = callbackFunc
        self.colorizeCallback = None

    def getNameForViewId(self, viewId):

        for typeName, typeValue in lcmdrc.data_request_t.__dict__.iteritems():
            if typeValue == viewId:
                return typeName

        return 'Map View ' + str(viewId)

    def updatePolyData(self, viewId, polyData):

        obj = self.polyDataObjects.get(viewId)
        if obj not in om.objects.values():
            obj = None
        if not obj:
            obj = om.PolyDataItem(self.getNameForViewId(viewId), polyData, self.view)
            obj.setProperty('Color', QtGui.QColor(0, 175, 255))
            folder = om.findObjectByName('Map Server')
            om.addToObjectModel(obj, folder)
            om.expand(folder)
            self.polyDataObjects[viewId] = obj
        else:
            obj.setPolyData(polyData)

    def showMap(self, viewId, mapId):
        polyData = vtk.vtkPolyData()
        self.reader.GetDataForMapId(viewId, mapId, polyData)
        self.updatePolyData(viewId, polyData)
        self.displayedMapIds[viewId] = mapId

        #if self.colorizeCallback:
        #    self.colorizeCallback()

        if self.callbackFunc:
            self.callbackFunc()

    def start(self):
        if self.reader is None:
            self.reader = drc.vtkMapServerSource()
            self.reader.Start()

        TimerCallback.start(self)

    def updateMap(self):
        viewIds = self.reader.GetViewIds()
        viewIds = vnp.numpy_support.vtk_to_numpy(viewIds) if viewIds.GetNumberOfTuples() else []
        for viewId in viewIds:
            mapId = self.reader.GetCurrentMapId(viewId)
            if viewId not in self.displayedMapIds or mapId != self.displayedMapIds[viewId]:
                self.showMap(viewId, mapId)

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
        mapServerSource = MapServerSource(view, callbackFunc=view.render)
        mapsServerContainer = om.ObjectModelItem('Map Server', icon=om.Icons.Robot)
        mapsServerContainer.source = mapServerSource
        om.addToObjectModel(mapsServerContainer, parentObj=sensorsFolder)
        mapServerSource.start()

    return _multisenseItem


