import os
import sys

import time
import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools

from director.timercallback import TimerCallback
from director.consoleapp import ConsoleApp
from director import applogic
from director import transformUtils
from director import filterUtils
from director import visualization as vis
from director import ioUtils
from director import vtkAll as vtk
from director import objectmodel as om
from director import cameracontrolpanel
from director.debugVis import DebugData

import numpy as np



def makeRobotPolyData():
    d = DebugData()
    d.addCone(origin=(0,0,0), normal=(1,0,0), radius=0.2, height=0.3)
    return d.getPolyData()


class AnimateRobot(object):

    def __init__(self, obj, view):
        self.obj = obj
        self.view = view
        self.timer = TimerCallback(targetFps=60)
        self.timer.callback = self.tick

    def start(self):
        self.startTime = time.time()
        self.timer.start()

    def stop(self):
        self.timer.stop()

    def tick(self):
        tNow = time.time()
        elapsed = tNow - self.startTime

        x, y = np.sin(elapsed), np.cos(elapsed)
        angle = -elapsed

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.RotateZ(np.degrees(angle))
        t.Translate(x, y, 0)
        self.obj.getChildFrame().copyFrame(t)
        self.view.render()




app = ConsoleApp()

app.setupGlobals(globals())


view = app.createView()


obj = vis.showPolyData(makeRobotPolyData(), 'robot', color=[0.7, 0.7, 0.8])
vis.addChildFrame(obj)


animator = AnimateRobot(om.findObjectByName('robot'), view)
animator.start()

panel = cameracontrolpanel.CameraControlPanel(view)
panel.widget.show()




def addWidgetToDock(widget, window, dockArea):
    dock = QtGui.QDockWidget()
    dock.setWidget(widget)
    dock.setWindowTitle(widget.windowTitle)
    window.addDockWidget(dockArea, dock)
    return dock


w = QtGui.QMainWindow()
w.setCentralWidget(view)
w.setWindowTitle('Test Camera Control')
addWidgetToDock(panel.widget, w, QtCore.Qt.RightDockWidgetArea)


w.show()
w.raise_()
w.activateWindow()
w.resize(1024, 600)
app.start()

