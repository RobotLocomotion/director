import ddapp.applogic as app
from ddapp import lcmUtils
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp import filterUtils
from ddapp import drcargs
from ddapp.shallowCopy import shallowCopy
from ddapp.timercallback import TimerCallback
from ddapp import vtkNumpy
from ddapp import objectmodel as om
import ddapp.vtkAll as vtk
from ddapp.debugVis import DebugData

import PythonQt
from PythonQt import QtCore, QtGui
import bot_core as lcmbotcore
import numpy as np
from ddapp.simpletimer import SimpleTimer
from ddapp import ioUtils
import sys
import drc as lcmdrc
import multisense as lcmmultisense
from ddapp.consoleapp import ConsoleApp


class KinectItem(om.ObjectModelItem):

    def __init__(self, model):

        om.ObjectModelItem.__init__(self, 'Kinect', om.Icons.Eye)

        self.model = model
        self.scalarBarWidget = None
        self.addProperty('Color By', 1,
                         attributes=om.PropertyAttributes(enumNames=['Solid Color', 'rgb_colors']))
        self.addProperty('Updates Enabled', True)
        self.addProperty('Framerate', model.targetFps,
                         attributes=om.PropertyAttributes(decimals=0, minimum=1.0, maximum=30.0, singleStep=1, hidden=False))
        self.addProperty('Visible', model.visible)
        #self.addProperty('Point Size', model.pointSize,
        #                 attributes=om.PropertyAttributes(decimals=0, minimum=1, maximum=20, singleStep=1, hidden=False))
        #self.addProperty('Alpha', model.alpha,
        #                 attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=1.0, singleStep=0.1, hidden=False))

        #self.addProperty('Color', QtGui.QColor(255,255,255))
        
    def _onPropertyChanged(self, propertySet, propertyName):
        om.ObjectModelItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Updates Enabled':
            if self.getProperty('Updates Enabled'):
                self.model.start()
            else:
                self.model.stop()

        #elif propertyName == 'Alpha':
        #    self.model.setAlpha(self.getProperty(propertyName))

        elif propertyName == 'Visible':
            self.model.setVisible(self.getProperty(propertyName))

        #elif propertyName == 'Point Size':
        #    self.model.setPointSize(self.getProperty(propertyName))

        elif propertyName == 'Framerate':
            self.model.setFPS(self.getProperty('Framerate'))

        elif propertyName == 'Color By':
            self._updateColorBy()

        self.model.polyDataObj._renderAllViews()


    def _updateColorBy(self):

        arrayMap = {
          0 : 'Solid Color',
          1 : 'rgb_colors'
          }

        colorBy = self.getProperty('Color By')
        arrayName = arrayMap.get(colorBy)

        self.model.polyDataObj.setProperty('Color By', arrayName)



class KinectSource(TimerCallback):

    def __init__(self, view, _KinectQueue):
        self.view = view
        self.KinectQueue = _KinectQueue

        self.visible = True
        
        self.p = vtk.vtkPolyData()
        utime = KinectQueue.getPointCloudFromKinect(self.p)
        self.polyDataObj = vis.PolyDataItem('kinect source', shallowCopy(self.p), view)
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
        utime = self.KinectQueue.getPointCloudFromKinect(p)

        if not p.GetNumberOfPoints():
            return

        cameraToLocalFused = vtk.vtkTransform()
        self.queue.getTransform('KINECT_RGB', 'local', utime, cameraToLocalFused)
        p = filterUtils.transformPolyData(p,cameraToLocalFused)
        self.polyDataObj.setPolyData(p)

        if not self.polyDataObj.initialized:
            self.polyDataObj.setProperty('Color By', 'rgb_colors')
            self.polyDataObj.initialized = True


def init(view):
    global KinectQueue, _kinectItem, _kinectSource
    KinectQueue = PythonQt.dd.ddKinectLCM(lcmUtils.getGlobalLCMThread())
    KinectQueue.init(lcmUtils.getGlobalLCMThread(), drcargs.args().config_file)
    
    _kinectSource = KinectSource(view, KinectQueue)
    _kinectSource.start()

    sensorsFolder = om.getOrCreateContainer('sensors')

    _kinectItem = KinectItem(_kinectSource)
    om.addToObjectModel(_kinectItem, sensorsFolder)
    

# Hasn't been used - currently deactivated
#def renderLastKinectPointCloud():
#    # view = view or app.getCurrentRenderView()
#    # if view is None:
#    #     return
#    p = vtk.vtkPolyData()
#    print("will grab the last point cloud in python \n")
#    KinectQueue.getPointCloudFromKinect(p)
#    print("grabbed the last point cloud in python, will #render now \n")
#    obj = vis.showPolyData (p, 'kinect cloud')
#    print("director rendered last point cloud \n")


def startButton():
    view = app.getCurrentRenderView()
    init(view)
    _kinectSource.start()

app.addToolbarMacro('start live kinect', startButton)
