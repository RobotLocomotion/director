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






def init():
    global KinectQueue
    KinectQueue = PythonQt.dd.ddKinectLCM(lcmUtils.getGlobalLCMThread())
    KinectQueue.init(lcmUtils.getGlobalLCMThread(), drcargs.args().config_file)
    # global app, view
    # app = ConsoleApp()
    # app.setupGlobals(globals())
    # app.showPythonConsole()
    # view = app.createView()
    # view.show()
    # app.start()



def renderLastKinectPointCloud():
    # view = view or app.getCurrentRenderView()
    # if view is None:
    #     return
    p = vtk.vtkPolyData()
    print("will grab the last point cloud in python \n")
    KinectQueue.getPointCloudFromKinect(p)
    print("grabbed the last point cloud in python, will render now \n")
    obj = vis.showPolyData (p, 'kinect cloud')
    print("director rendered last point cloud \n")





def startKinectLCM():

    global source, obj, t
    p = vtk.vtkPolyData()

    KinectQueue.getPointCloudFromKinect(p)
    obj = vis.showPolyData(shallowCopy(p), 'kinect source')



    print obj.getArrayNames()
    obj.initialized = False

    def updateSource():

        p = vtk.vtkPolyData()
        KinectQueue.getPointCloudFromKinect(p)

        if not p.GetNumberOfPoints():
            return

        p = filterUtils.transformPolyData(p,pointCloudTransform)
        obj.setPolyData(p)

        if not obj.initialized:
            obj.setProperty('Color By', 'rgb_colors')
            obj.initialized = True


    global timerCallback
    timerCallback = TimerCallback(targetFps=30)
    timerCallback.callback = updateSource
    timerCallback.start()


def startButton():
    init()
    startKinectLCM()


pointCloudTransform = transformUtils.transformFromPose([-0.17790587, -1.24393384, 0.22335996], [0.71466764, -0.64979598,  0.17682733, -0.18906994])

f = vis.showFrame(pointCloudTransform, 'point cloud transform', scale=0.2, visible=False)

w = vis.showFrame(vtk.vtkTransform(), 'widget', scale=0.7)
frameSync = vis.FrameSync()
frameSync.addFrame(f)
frameSync.addFrame(w)

def zeroWidget():
    frameSync.removeFrame(w)
    w.copyFrame(transformUtils.frameFromPositionAndRPY(w.transform.GetPosition(), [0.0, 0.0, 0.0]))
    frameSync.addFrame(w)

app.addToolbarMacro('zero widget', zeroWidget)      
app.addToolbarMacro('start live kinect', startButton)




