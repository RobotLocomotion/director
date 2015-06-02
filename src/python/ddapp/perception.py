import os
import sys
import vtk
import math
import PythonQt
from PythonQt import QtCore, QtGui
import ddapp.objectmodel as om
from ddapp import drcargs
from ddapp import robotstate
from ddapp.timercallback import TimerCallback
from ddapp.utime import getUtime
from ddapp.simpletimer import MovingAverageComputer
import vtkDRCFiltersPython as drc
from ddapp.debugVis import DebugData
import ddapp.visualization as vis
from ddapp import vtkNumpy as vnp
import numpy as np

import drc as lcmdrc
import multisense as lcmmultisense
import lcmUtils


class MultisenseItem(om.ObjectModelItem):

    def __init__(self, model):

        om.ObjectModelItem.__init__(self, 'Multisense', om.Icons.Laser)

        self.model = model
        self.scalarBarWidget = None
        self.addProperty('Color By', 0,
                         attributes=om.PropertyAttributes(enumNames=['Solid Color', 'Intensity', 'Z Coordinate', 'Range', 'Spindle Angle', 'Azimuth', 'Camera RGB', 'Scan Delta']))
        self.addProperty('Show Scalar Bar', False)
        self.addProperty('Updates Enabled', True)
        self.addProperty('Min Range', model.reader.GetDistanceRange()[0],
                         attributes=om.PropertyAttributes(decimals=2, minimum=0.0, maximum=100.0, singleStep=0.25, hidden=False))
        self.addProperty('Max Range', model.reader.GetDistanceRange()[1],
                         attributes=om.PropertyAttributes(decimals=2, minimum=0.0, maximum=100.0, singleStep=0.25, hidden=False))
        self.addProperty('Edge Filter Angle', model.reader.GetEdgeAngleThreshold(),
                         attributes=om.PropertyAttributes(decimals=0, minimum=0.0, maximum=60.0, singleStep=1, hidden=False))
        self.addProperty('Number of Scan Lines', model.numberOfScanLines,
                         attributes=om.PropertyAttributes(decimals=0, minimum=0, maximum=100, singleStep=1, hidden=False))
        self.addProperty('Visible', model.visible)
        self.addProperty('Point Size', model.pointSize,
                         attributes=om.PropertyAttributes(decimals=0, minimum=1, maximum=20, singleStep=1, hidden=False))
        self.addProperty('Alpha', model.alpha,
                         attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=1.0, singleStep=0.1, hidden=False))

        #self.addProperty('Color', QtGui.QColor(255,255,255))
        #self.addProperty('Scanline Color', QtGui.QColor(255,0,0))

    def _onPropertyChanged(self, propertySet, propertyName):
        om.ObjectModelItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Updates Enabled':
            if self.getProperty('Updates Enabled'):
                self.model.start()
            else:
                self.model.stop()

        elif propertyName == 'Edge Filter Angle':
            self.model.reader.SetEdgeAngleThreshold(self.getProperty('Edge Filter Angle'))
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

        self.model.polyDataObj._renderAllViews()


    def _updateColorBy(self):

        arrayMap = {
          0 : 'Solid Color',
          1 : 'intensity',
          2 : 'z',
          3 : 'distance',
          4 : 'spindle_angle',
          5 : 'azimuth',
          6 : 'rgb',
          7 : 'scan_delta'
          }

        colorBy = self.getProperty('Color By')
        arrayName = arrayMap.get(colorBy)

        if arrayName == 'rgb' and arrayName not in self.model.polyDataObj.getArrayNames():
            self.model.colorizeCallback()
            self.model.polyDataObj._updateColorByProperty()
        self.model.polyDataObj.setProperty('Color By', arrayName)
        self._updateScalarBar()

    def hasDataSet(self, dataSet):
        return self.model.polyDataObj.hasDataSet(dataSet)

    def _updateScalarBar(self):
        self.model.polyDataObj.setProperty('Show Scalar Bar', self.getProperty('Show Scalar Bar'))



class SpindleAxisDebug(vis.PolyDataItem):

    def __init__(self, frameProvider):
        vis.PolyDataItem.__init__(self, 'spindle axis', vtk.vtkPolyData(), view=None)
        self.frameProvider = frameProvider
        self.timer = TimerCallback()
        self.timer.callback = self.update
        self.setProperty('Color', QtGui.QColor(0, 255, 0))
        self.setProperty('Visible', False)

    def _onPropertyChanged(self, propertySet, propertyName):
        vis.PolyDataItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Visible':
            if self.getProperty(propertyName):
                self.timer.start()
            else:
                self.timer.stop()

    def onRemoveFromObjectModel(self):
        vis.PolyDataItem.onRemoveFromObjectModel(self)
        self.timer.stop()

    def update(self):

        t = self.frameProvider.getFrame('SCAN')

        p1 = [0.0, 0.0, 0.0]
        p2 = [2.0, 0.0, 0.0]

        p1 = t.TransformPoint(p1)
        p2 = t.TransformPoint(p2)

        d = DebugData()
        d.addSphere(p1, radius=0.01, color=[0,1,0])
        d.addLine(p1, p2, color=[0,1,0])
        self.setPolyData(d.getPolyData())



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
        self.polyDataObj = vis.PolyDataItem('Multisense Sweep', self.revPolyData, view)
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
            scanLine = vis.PolyDataItem('scan line %d' % i, polyData, self.view)
            scanLine.actor.SetPickable(0)
            scanLine.setSolidColor((1,0,0))
            self.scanLines.append(scanLine)

    def getScanToLocal(self):
        return None

    def showRevolution(self, revId):

        self.reader.GetDataForRevolution(revId, self.revPolyData)
        self.displayedRevolution = revId

        if self.showRevolutionCallback:
            self.showRevolutionCallback()
        if self.colorizeCallback and self.polyDataObj.getPropertyEnumValue('Color By') == 'rgb':
            self.colorizeCallback()

        self.polyDataObj._updateColorByProperty()
        self.polyDataObj._updateColorBy()

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
            self.reader.InitBotConfig(drcargs.args().config_file)
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

    def tick(self):
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
    def sendMultisenseCommand(fps, gain, exposure, agc, rpm, led_flash, led_duty):

        m = lcmmultisense.command_t()
        m.utime = getUtime()
        m.fps = fps
        m.gain = gain
        m.exposure_us = exposure
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


class NeckDriver(object):

    def __init__(self, view, getNeckPitchFunction):
        self.active = False
        self.view = view
        self.delta = 0
        self.textObj = None
        self._getNeckPitchFunction = getNeckPitchFunction
        self.presetMap = {0:0, 1:60, 2:45, 3:30}

    def showText(self, neckPitch):
        if not self.view:
            return
        self.textObj = vis.updateText('Neck pitch: %d' % neckPitch, 'Neck Pitch Annotation', parent='spindle axis')

    def cleanupText(self):
        if self.textObj:
            om.removeFromObjectModel(self.textObj)
            self.textObj = None

    def activateNeckControl(self):
        if not self.active:
            self.active = True
            self.delta = 0

    def getNeckPitchDegrees(self):
        return math.degrees(self._getNeckPitchFunction())

    def applyNeckPitchPreset(self, presetId):
        if not self.active:
            return False
        self.setNeckPitch(self.presetMap[presetId])
        return True


    def applyNeckPitchDelta(self, delta):
        if delta == 0:
            return

        neckPitch = self.getNeckPitchDegrees() + delta
        neckPitch = np.clip(neckPitch, -90, 90)
        self.setNeckPitch(neckPitch)

    def setNeckPitch(self, neckPitch):
        MultiSenseSource.setNeckPitch(neckPitch)

    def deactivateNeckControl(self):
        if self.active:
            self.active = False
            self.applyNeckPitchDelta(self.delta)
            self.cleanupText()
            self.delta = 0

    def onWheelDelta(self, delta):
        if not self.active:
            return False

        self.delta += np.clip(delta, -1, 1)
        neckPitch = self.getNeckPitchDegrees() + self.delta
        self.showText(neckPitch)
        return True


class SpindleMonitor(object):
    def __init__(self, getSpindleAngleFunction):
        self.lastSpindleAngle = 0
        self.lastStateTime = 0
        self.spindleSpinRateAverager = MovingAverageComputer()
        self.spindleSpinRateAverager.timeWindow = 0.5
        self._getSpindleAngleFunction = getSpindleAngleFunction

    def onRobotStateChanged(self, newState):
        t, newAngle = self._getSpindleAngleFunction()
        elapsed = t - self.lastStateTime
        if (elapsed > 0.001 and elapsed < 100):
            # unwrap
            diff = newAngle - self.lastSpindleAngle
            if (abs(diff - 2*math.pi) < abs(diff)):
                diff = diff - 2*math.pi
            if (abs(diff + 2*math.pi) < abs(diff)):
                diff = diff + 2*math.pi
            velocity = diff / elapsed
            self.spindleSpinRateAverager.update(velocity)
            # if avg veloicty is bad panic
        self.lastStateTime = t
        self.lastSpindleAngle = newAngle

    def getAverageSpindleVelocity(self):
        return self.spindleSpinRateAverager.getAverage()


class MapServerSource(TimerCallback):

    def __init__(self, view, callbackFunc=None):
        TimerCallback.__init__(self)
        self.reader = None
        self.folder = None
        self.view = view
        self.displayedMapIds = {}
        self.polyDataObjects = {}
        self.targetFps = 10
        self.callbackFunc = callbackFunc
        self.colorizeCallback = None
        self.useMeshes = True

    def getNameForViewId(self, viewId):

        for typeName, typeValue in lcmdrc.data_request_t.__dict__.iteritems():
            if typeValue == viewId:
                return typeName

        return 'Map View ' + str(viewId)

    def updatePolyData(self, viewId, polyData):

        obj = self.polyDataObjects.get(viewId)
        if obj not in om.getObjects():
            obj = None
        if not obj:
            hiddenMapIds = [9999]
            visibleDefault = False if viewId in hiddenMapIds else True
            obj = vis.PolyDataItem(self.getNameForViewId(viewId), polyData, self.view)

            obj.setProperty('Visible', visibleDefault)
            if obj._isPointCloud():
                obj.setProperty('Color', [1, 1, 1])
                obj.setProperty('Alpha', 0.5)
            else:
                obj.setProperty('Color', [0, 0.68, 1])

            if viewId == lcmdrc.data_request_t.HEIGHT_MAP_SCENE:
                obj.setProperty('Surface Mode', 'Wireframe')

            folder = om.findObjectByName('Map Server')
            folder.addProperty('Min Range', self.reader.GetDistanceRange()[0],
                             attributes=om.PropertyAttributes(decimals=2, minimum=0.0, maximum=100.0, singleStep=0.25, hidden=False))
            folder.addProperty('Max Range', self.reader.GetDistanceRange()[1],
                             attributes=om.PropertyAttributes(decimals=2, minimum=0.0, maximum=100.0, singleStep=0.25, hidden=False))
            folder.addProperty('Edge Filter Angle', self.reader.GetEdgeAngleThreshold(),
                         attributes=om.PropertyAttributes(decimals=0, minimum=0.0, maximum=60.0, singleStep=1, hidden=False))
            om.addToObjectModel(obj, folder)
            om.expand(folder)
            self.folder = folder
            self.polyDataObjects[viewId] = obj
        else:
            obj.setPolyData(polyData)

        if self.colorizeCallback:
            self.colorizeCallback(obj)

    def showMap(self, viewId, mapId):
        polyData = vtk.vtkPolyData()

        if self.useMeshes:
            self.reader.GetMeshForMapId(viewId, mapId, polyData)
        else:
            self.reader.GetDataForMapId(viewId, mapId, polyData)

        self.updatePolyData(viewId, polyData)
        self.displayedMapIds[viewId] = mapId

        if self.callbackFunc:
            self.callbackFunc()

    def getSceneHeightData(self):
        return self.getDepthMapData(lcmdrc.data_request_t.HEIGHT_MAP_SCENE)

    def getDepthMapData(self, viewId):

        mapId = self.reader.GetCurrentMapId(viewId)
        if mapId < 0:
            return None, None

        depthImage = vtk.vtkImageData()
        transform = vtk.vtkTransform()
        self.reader.GetDataForMapId(viewId, mapId, depthImage, transform)

        dims = depthImage.GetDimensions()
        d = vnp.getNumpyFromVtk(depthImage, 'ImageScalars')
        d = d.reshape(dims[1], dims[0])
        t = np.array([[transform.GetMatrix().GetElement(r, c) for c in xrange(4)] for r in xrange(4)])

        return d, t

    def start(self):
        if self.reader is None:
            self.reader = drc.vtkMapServerSource()
            self.reader.Start()

        TimerCallback.start(self)

    def updateMap(self):
        if (self.folder):
            self.reader.SetDistanceRange(self.folder.getProperty('Min Range'), self.folder.getProperty('Max Range'))
            self.reader.SetEdgeAngleThreshold(self.folder.getProperty('Edge Filter Angle'))
            
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
    global multisenseDriver


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
    else:
        mapServerSource = None


    spindleDebug = SpindleAxisDebug(multisenseDriver)
    spindleDebug.addToView(view)
    om.addToObjectModel(spindleDebug, sensorsFolder)


    return multisenseDriver, mapServerSource


