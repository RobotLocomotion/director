import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from ddapp import applogic as app
from ddapp import visualization as vis
from ddapp.debugVis import DebugData
from ddapp import transformUtils
from ddapp.timercallback import TimerCallback
from ddapp.simpletimer import FPSCounter
from ddapp import objectmodel as om
from ddapp import affordanceitems
from ddapp import affordanceurdf
from ddapp.uuidutil import newUUID

import ddapp.vtkAll as vtk
import numpy as np

def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


class AffordancePanel(object):

    def __init__(self, view, affordanceManager, ikServer, jointController, raycastDriver):

        self.view = view
        self.affordanceManager = affordanceManager
        self.ikServer = ikServer
        self.jointController = jointController
        self.raycastDriver = raycastDriver

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddAffordancePanel.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)
        self.ui = WidgetDict(self.widget.children())

        self.ui.affordanceListWidget.hide()

        self.ui.spawnBoxButton.connect('clicked()', self.onSpawnBox)
        self.ui.spawnSphereButton.connect('clicked()', self.onSpawnSphere)
        self.ui.spawnCylinderButton.connect('clicked()', self.onSpawnCylinder)
        self.ui.spawnCapsuleButton.connect('clicked()', self.onSpawnCapsule)
        self.ui.spawnRingButton.connect('clicked()', self.onSpawnRing)
        self.ui.spawnMeshButton.connect('clicked()', self.onSpawnMesh)
        self.ui.getRaycastTerrainButton.connect('clicked()', self.onGetRaycastTerrain)

        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        self.ui.scrollArea.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.Resize)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.onEvent)

        self.updateTimer = TimerCallback(targetFps=30)
        self.updateTimer.callback = self.updatePanel
        self.updateTimer.start()

    def onEvent(self, obj, event):
        minSize = self.ui.scrollArea.widget().minimumSizeHint.width() + self.ui.scrollArea.verticalScrollBar().width
        self.ui.scrollArea.setMinimumWidth(minSize)

    def updatePanel(self):
        if not self.widget.isVisible():
            return

    def getSpawnFrame(self):
        pos = self.jointController.q[:3]
        rpy = np.degrees(self.jointController.q[3:6])
        frame = transformUtils.frameFromPositionAndRPY(pos, rpy)
        frame.PreMultiply()
        frame.Translate(0.5, 0.0, 0.3)
        return frame


    def onGetRaycastTerrain(self):
        affs = self.affordanceManager.getCollisionAffordances()
        xy = self.jointController.q[:2]
        self.raycastDriver.requestRaycast(affs, xy-2, xy+2)

    def affordanceFromDescription(self, desc):
        self.affordanceManager.collection.updateDescription(desc)
        return self.affordanceManager.getAffordanceById(desc['uuid'])

    def onSpawnBox(self):
        pose = transformUtils.poseFromTransform(self.getSpawnFrame())
        desc = dict(classname='BoxAffordanceItem', Name='box', uuid=newUUID(), pose=pose)
        return self.affordanceFromDescription(desc)

    def onSpawnSphere(self):
        pose = transformUtils.poseFromTransform(self.getSpawnFrame())
        desc = dict(classname='SphereAffordanceItem', Name='sphere', uuid=newUUID(), pose=pose)
        return self.affordanceFromDescription(desc)

    def onSpawnCylinder(self):
        pose = transformUtils.poseFromTransform(self.getSpawnFrame())
        desc = dict(classname='CylinderAffordanceItem', Name='cylinder', uuid=newUUID(), pose=pose)
        return self.affordanceFromDescription(desc)

    def onSpawnCapsule(self):
        pose = transformUtils.poseFromTransform(self.getSpawnFrame())
        desc = dict(classname='CapsuleAffordanceItem', Name='capsule', uuid=newUUID(), pose=pose)
        return self.affordanceFromDescription(desc)

    def onSpawnRing(self):
        pose = transformUtils.poseFromTransform(self.getSpawnFrame())
        desc = dict(classname='CapsuleRingAffordanceItem', Name='ring', uuid=newUUID(), pose=pose)
        return self.affordanceFromDescription(desc)

    def onSpawnMesh(self):

        d = DebugData()
        d.addArrow((0,0,0), (0,0,0.3))
        pd = d.getPolyData()
        meshId = affordanceitems.MeshAffordanceItem.getMeshManager().add(pd)

        pose = transformUtils.poseFromTransform(self.getSpawnFrame())
        desc = dict(classname='MeshAffordanceItem', Name='mesh', Filename=meshId, uuid=newUUID(), pose=pose)
        return self.affordanceFromDescription(desc)


def _getAction():
    return None

def init(view, affordanceManager, ikServer, jointController, raycastDriver):

    global panel
    global dock

    panel = AffordancePanel(view, affordanceManager, ikServer, jointController, raycastDriver)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel
