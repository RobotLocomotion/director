from ddapp import robotsystem
from ddapp.consoleapp import ConsoleApp
from ddapp import visualization as vis
from ddapp.debugVis import DebugData
from ddapp import objectmodel as om
import numpy as np

import PythonQt
from PythonQt import QtGui, QtCore


class LinkWidget(object):

    def __init__(self, view, robotModel, externalForce):
        self.view = view
        self.robotModel = robotModel
        self.linkName = None
        self.pickedPoint = None
        self.normal = None
        self.externalForce = externalForce

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
        elif event.type() == QtCore.QEvent.MouseButtonPress and event.button() == QtCore.Qt.LeftButton:
            self.onLeftMousePress(vis.mapMousePosition(obj, event), event.modifiers())

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
        self.normal = normal


    def onLeftMousePress(self, displayPoint, modifiers=None):
        selection = self.getSelection(displayPoint)
        if selection is None:
            return
        pickedPoint, linkName, normal = selection
        

        # add this as an external force with magnitude zero
        if self.linkName is not None:
            print ""
            print "added external force"
            print linkName, pickedPoint, normal
            print ""
            forceDirection = -np.array(normal)
            forceLocation = np.array(pickedPoint)
            self.externalForce.addForce(linkName, forceDirection=forceDirection, forceLocation=forceLocation, forceMagnitude=0.0, inWorldFrame=True)