import os
import sys
import PythonQt
from PythonQt import QtCore, QtGui

import ddapp.applogic as app
from ddapp import jointcontrol
from ddapp import lcmUtils
from ddapp import robotstate
from ddapp import drakevisualizer
from ddapp import lcmgl
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp.timercallback import TimerCallback
import functools
import drc as lcmdrc



class ViewerApp(object):

    def __init__(self):

        app.setupPackagePaths()

        self.setupUi()

        app.toggleCameraTerrainMode(self.view)
        self.resetCamera()

        self.drakeVis = drakevisualizer.DrakeVisualizer(self.view)

        vis.showGrid(self.view, color=[0,0,0], parent=None)

        self.timer = TimerCallback()
        self.timer.callback = self.update
        self.timer.targetFps = 60
        #self.timer.start()

    def setupUi(self):

        self.objectTree = QtGui.QTreeWidget()
        self.propertiesPanel = PythonQt.dd.ddPropertiesPanel()

        sidePanel = QtGui.QSplitter(QtCore.Qt.Vertical)
        sidePanel.addWidget(self.objectTree)
        sidePanel.addWidget(self.propertiesPanel)
        sidePanel.setSizes([6, 4])

        om.init(self.objectTree, self.propertiesPanel)

        self.view = PythonQt.dd.ddQVTKWidgetView()
        lcmgl.init(self.view)

        self.view.backgroundRenderer().SetBackground(1,1,1)
        self.view.backgroundRenderer().SetBackground2(1,1,1)

        def setTextProperty(caption):
          prop = caption.GetCaptionTextProperty()
          prop.ShadowOff()
          prop.BoldOn()
          prop.ItalicOff()
          prop.SetColor(0,0,0)

        axesActor = self.view.orientationMarkerWidget().GetOrientationMarker()
        setTextProperty(axesActor.GetXAxisCaptionActor2D())
        setTextProperty(axesActor.GetYAxisCaptionActor2D())
        setTextProperty(axesActor.GetZAxisCaptionActor2D())

        w = QtGui.QWidget()
        l = QtGui.QHBoxLayout(w)
        self.slider = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.clearButton = QtGui.QPushButton('clear')
        self.zeroButton = QtGui.QPushButton('zero')

        l.addWidget(self.clearButton)
        l.addWidget(self.zeroButton)
        l.addWidget(self.slider)

        self.slider.connect(self.slider, 'valueChanged(int)', self.onSlider)
        self.slider.connect(self.zeroButton, 'clicked()', self.onZeroButtonClicked)
        self.slider.connect(self.clearButton, 'clicked()', self.onClearButtonClicked)

        ww = QtGui.QWidget()
        ll = QtGui.QVBoxLayout(ww)
        ll.addWidget(self.view)
        #ll.addWidget(w)
        ll.setMargin(0)

        splitter = QtGui.QSplitter()
        splitter.addWidget(sidePanel)
        splitter.addWidget(ww)
        splitter.setSizes([0, 1])

        splitter.show()
        splitter.resize(800, 600)
        self.widget = splitter
        self.widget.setWindowTitle('Drake Viewer')


    def shutdown(self):
        self.widget.delete()
        pass

    def onZeroButtonClicked(self):
        self.resetCamera()
        self.view.render()

    def onSlider(self):
        value = self.slider.value / 100.0

    def resetCamera(self):
        camera = self.view.camera()
        camera.SetPosition(3,0,0.8)
        camera.SetFocalPoint(0,0,0.5)
        camera.SetViewUp(0,0,1)
        self.view.resetCamera()


    def onClearButtonClicked(self):
        pass


    def update(self):
        self.view.render()





def startup():

    viewApp = ViewerApp()
    QtCore.QCoreApplication.instance().exec_()
    viewApp.shutdown()


startup()
