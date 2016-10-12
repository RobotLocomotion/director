from director import applogic as app
from director.debugVis import DebugData
from director import transformUtils
from director import objectmodel as om
from director import affordanceitems
from director import affordanceurdf
from director.uuidutil import newUUID
import director.vtkAll as vtk
import numpy as np

import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools



def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


class AffordancePanel(object):

    def __init__(self, view, affordanceManager, jointController=None, raycastDriver=None):

        self.view = view
        self.affordanceManager = affordanceManager
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

        if not self.raycastDriver:
            self.ui.getRaycastTerrainButton.hide()

        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        self.ui.scrollArea.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.Resize)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.onEvent)

    def onEvent(self, obj, event):
        minSize = self.ui.scrollArea.widget().minimumSizeHint.width() + self.ui.scrollArea.verticalScrollBar().width
        self.ui.scrollArea.setMinimumWidth(minSize)

    def getSpawnFrame(self):

        if self.jointController:
            # get spawn frame in front of robot
            pos = self.jointController.q[:3]
            rpy = np.degrees(self.jointController.q[3:6])
            frame = transformUtils.frameFromPositionAndRPY(pos, rpy)
            frame.PreMultiply()
            frame.Translate(0.5, 0.0, 0.3)
        else:
            frame = vtk.vtkTransform()

        return frame

    def onGetRaycastTerrain(self):
        affs = self.affordanceManager.getCollisionAffordances()
        xy = self.jointController.q[:2]
        self.raycastDriver.requestRaycast(affs, xy-2, xy+2)

    def onSpawnBox(self):
        pose = transformUtils.poseFromTransform(self.getSpawnFrame())
        desc = dict(classname='BoxAffordanceItem', Name='box', uuid=newUUID(), pose=pose)
        return self.affordanceManager.newAffordanceFromDescription(desc)

    def onSpawnSphere(self):
        pose = transformUtils.poseFromTransform(self.getSpawnFrame())
        desc = dict(classname='SphereAffordanceItem', Name='sphere', uuid=newUUID(), pose=pose)
        return self.affordanceManager.newAffordanceFromDescription(desc)

    def onSpawnCylinder(self):
        pose = transformUtils.poseFromTransform(self.getSpawnFrame())
        desc = dict(classname='CylinderAffordanceItem', Name='cylinder', uuid=newUUID(), pose=pose)
        return self.affordanceManager.newAffordanceFromDescription(desc)

    def onSpawnCapsule(self):
        pose = transformUtils.poseFromTransform(self.getSpawnFrame())
        desc = dict(classname='CapsuleAffordanceItem', Name='capsule', uuid=newUUID(), pose=pose)
        return self.affordanceManager.newAffordanceFromDescription(desc)

    def onSpawnRing(self):
        pose = transformUtils.poseFromTransform(self.getSpawnFrame())
        desc = dict(classname='CapsuleRingAffordanceItem', Name='ring', uuid=newUUID(), pose=pose)
        return self.affordanceManager.newAffordanceFromDescription(desc)

    def onSpawnMesh(self):

        d = DebugData()
        d.addArrow((0,0,0), (0,0,0.3))
        pd = d.getPolyData()
        meshId = affordanceitems.MeshAffordanceItem.getMeshManager().add(pd)

        pose = transformUtils.poseFromTransform(self.getSpawnFrame())
        desc = dict(classname='MeshAffordanceItem', Name='mesh', Filename=meshId, uuid=newUUID(), pose=pose)
        return self.affordanceManager.newAffordanceFromDescription(desc)


def _getAction():
    return None


def init(view, affordanceManager, jointController, raycastDriver):

    global panel
    global dock

    panel = AffordancePanel(view, affordanceManager, jointController, raycastDriver)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel
