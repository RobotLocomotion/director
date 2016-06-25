from director import robotsystem
from director.consoleapp import ConsoleApp
from director import visualization as vis
from director.debugVis import DebugData
from director import objectmodel as om
import numpy as np

import PythonQt
from PythonQt import QtGui, QtCore


class LinkWidget(object):

    def __init__(self, view, robotModel):
        self.view = view
        self.robotModel = robotModel
        self.linkName = None
        self.pickedPoint = None

    def start(self):
        self.installEventFilter()

    def stop(self):
        self.removeEventFilter()

    def installEventFilter(self):

        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        self.view.vtkWidget().installEventFilter(self.eventFilter)

        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseMove)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonPress)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.onEvent)

    def removeEventFilter(self):
        self.view.vtkWidget().removeEventFilter(self.eventFilter)

    def onEvent(self, obj, event):
        if event.type() == QtCore.QEvent.MouseMove:
            self.onMouseMove(vis.mapMousePosition(obj, event), event.modifiers())
        elif event.type() == QtCore.QEvent.MouseButtonPress:
            self.onMousePress(vis.mapMousePosition(obj, event), event.modifiers())

    def getSelection(self, displayPoint):

        pickedPoint, pickedProp, pickedDataset, normal = vis.pickPoint(displayPoint, self.view, pickType='cells', tolerance=0.0, returnNormal=True)

        if not pickedDataset:
            return None

        linkName = self.robotModel.model.getLinkNameForMesh(pickedDataset)
        if not linkName:
            return None

        return pickedPoint, linkName, normal


    def onMouseMove(self, displayPoint, modifiers=None):

        om.removeFromObjectModel(om.findObjectByName('link selection'))
        self.linkName = None
        self.pickedPoint = None


        selection = self.getSelection(displayPoint)
        if selection is None:
            return

        pickedPoint, linkName, normal = selection

        d = DebugData()
        d.addSphere(pickedPoint, radius=0.01)
        d.addLine(pickedPoint, np.array(pickedPoint) + 0.1 * np.array(normal), radius=0.005)
        obj = vis.updatePolyData(d.getPolyData(), 'link selection', color=[0,1,0])
        obj.actor.SetPickable(False)

        self.linkName = linkName
        self.pickedPoint = pickedPoint


    def onMousePress(self, displayPoint, modifiers=None):
        print self.linkName, self.pickedPoint


class ForcesPanel(object):

    def __init__(self):
        self.forces = ['pelvis']
        self.widget = QtGui.QWidget()
        self.layout = QtGui.QVBoxLayout(self.widget)

        self.rebuild()
        self.layout.addStretch()

    def rebuild(self):
        for force in self.forces:
            self.layout.addWidget(self.makeForcesWidget(force))

    def makeForcesWidget(self, force):
        w = QtGui.QWidget()
        h = QtGui.QHBoxLayout(w)
        h.addWidget(QtGui.QLabel(force))
        slider = QtGui.QSlider(QtCore.Qt.Horizontal)
        h.addWidget(slider)

        def onSliderChanged(value):
            print value, force

        slider.connect('valueChanged(int)', onSliderChanged)
        return w




###########################

app = ConsoleApp()
app.setupGlobals(globals())

view = app.createView()
view.show()
view.resize(1080, 768)
robotSystem = robotsystem.create(view)

widget = LinkWidget(view, robotSystem.robotStateModel)
widget.start()

app.viewOptions.setProperty('Gradient background', False)
app.viewOptions.setProperty('Background color', [1,1,1])
app.viewOptions.setProperty('Orientation widget', False)
app.gridObj.setProperty('Color', [0,0,0])
app.gridObj.setProperty('Surface Mode', 'Surface with edges')


w = ForcesPanel()

m = QtGui.QMainWindow()
m.setCentralWidget(view)
dock = QtGui.QDockWidget()
dock.setWindowTitle('Forces panel')
dock.setWidget(w.widget)

m.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock)
m.resize(1080, 768)
m.show()



app.start()
