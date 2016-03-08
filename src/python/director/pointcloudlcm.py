import director.applogic as app
from director import lcmUtils
from director import transformUtils
from director import visualization as vis
from director import filterUtils
from director import drcargs
from director.shallowCopy import shallowCopy
from director.timercallback import TimerCallback
from director import vtkNumpy
from director import objectmodel as om
import director.vtkAll as vtk
from director.debugVis import DebugData

import PythonQt
from PythonQt import QtCore, QtGui
import bot_core as lcmbotcore
import numpy as np
from director.simpletimer import SimpleTimer
from director import ioUtils
import sys
import drc as lcmdrc
from director.consoleapp import ConsoleApp


class PointCloudItem(om.ObjectModelItem):

    def __init__(self, model):

        om.ObjectModelItem.__init__(self, 'PointCloud', om.Icons.Eye)

        self.model = model
        self.scalarBarWidget = None
        self.addProperty('Color By', 0,
                         attributes=om.PropertyAttributes(enumNames=['Solid Color']))
        self.addProperty('Updates Enabled', True)
        self.addProperty('Framerate', model.targetFps,
                         attributes=om.PropertyAttributes(decimals=0, minimum=1.0, maximum=30.0, singleStep=1, hidden=False))
        self.addProperty('Visible', model.visible)
        
    def _onPropertyChanged(self, propertySet, propertyName):
        om.ObjectModelItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Updates Enabled':
            if self.getProperty('Updates Enabled'):
                self.model.start()
            else:
                self.model.stop()

        elif propertyName == 'Visible':
            self.model.setVisible(self.getProperty(propertyName))

        elif propertyName == 'Framerate':
            self.model.setFPS(self.getProperty('Framerate'))

        elif propertyName == 'Color By':
            self._updateColorBy()

        self.model.polyDataObj._renderAllViews()


    def _updateColorBy(self):

        arrayMap = {
          0 : 'Solid Color'
          }

        colorBy = self.getProperty('Color By')
        arrayName = arrayMap.get(colorBy)

        self.model.polyDataObj.setProperty('Color By', arrayName)



class PointCloudSource(TimerCallback):

    def __init__(self, view, _PointCloudQueue):
        self.view = view
        self.PointCloudQueue = _PointCloudQueue

        self.visible = True
        
        self.p = vtk.vtkPolyData()
        utime = PointCloudQueue.getPointCloudFromPointCloud(self.p)
        self.polyDataObj = vis.PolyDataItem('pointcloud source', shallowCopy(self.p), view)
        self.polyDataObj.actor.SetPickable(1)
        self.polyDataObj.initialized = False

        om.addToObjectModel(self.polyDataObj)

        self.queue = PythonQt.dd.ddBotImageQueue(lcmUtils.getGlobalLCMThread())
        self.queue.init(lcmUtils.getGlobalLCMThread(), drcargs.args().config_file)

        self.targetFps = 30
        self.timerCallback = TimerCallback(targetFps=self.targetFps)
        self.timerCallback.callback = self._updateSource
        #self.timerCallback.start()
        
    def start(self):
        self.timerCallback.start()

    def stop(self):
        self.timerCallback.stop()

    def setFPS(self, framerate):
        self.targetFps = framerate
        self.timerCallback.stop()
        self.timerCallback.targetFps = framerate
        self.timerCallback.start()

    def setVisible(self, visible):
        self.polyDataObj.setProperty('Visible', visible)

    def _updateSource(self):
        p = vtk.vtkPolyData()
        utime = self.PointCloudQueue.getPointCloudFromPointCloud(p)

        if not p.GetNumberOfPoints():
            return

        sensorToLocalFused = vtk.vtkTransform()
        self.queue.getTransform('local', 'local', utime, sensorToLocalFused)
        p = filterUtils.transformPolyData(p,sensorToLocalFused)
        self.polyDataObj.setPolyData(p)

        if not self.polyDataObj.initialized:
            self.polyDataObj.initialized = True


def init(view):
    global PointCloudQueue, _pointcloudItem, _pointcloudSource
    PointCloudQueue = PythonQt.dd.ddPointCloudLCM(lcmUtils.getGlobalLCMThread())
    PointCloudQueue.init(lcmUtils.getGlobalLCMThread(), drcargs.args().config_file)
    
    _pointcloudSource = PointCloudSource(view, PointCloudQueue)
    _pointcloudSource.start()

    sensorsFolder = om.getOrCreateContainer('sensors')

    _pointcloudItem = PointCloudItem(_pointcloudSource)
    om.addToObjectModel(_pointcloudItem, sensorsFolder)
    

def startButton():
    view = app.getCurrentRenderView()
    init(view)
    _pointcloudSource.start()

try:
    app.addToolbarMacro('start live pointcloud', startButton)
except AttributeError:
    pass
