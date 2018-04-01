from PythonQt import QtCore, QtGui

from director import uipanel
from director import pointpicker
from director import applogic
from director import objectmodel as om
from director import vieweventfilter
from director import visualization as vis
from director import vtkAll as vtk
from director.debugVis import DebugData
import numpy as np

class MyEventFilter(vieweventfilter.ViewEventFilter):

    def __init__(self, view, panel):
        vieweventfilter.ViewEventFilter.__init__(self, view)
        self.panel = panel

    def onMouseMove(self, event):
        displayPoint = self.getMousePositionInView(event)
        self.panel.onMouseMove(displayPoint)

    def onLeftClick(self, event):
        displayPoint = self.getMousePositionInView(event)

        if event.modifiers() == QtCore.Qt.ShiftModifier:
            self.panel.onShiftMouseClick(displayPoint)


def computeViewScale(view, position):
    camera = view.camera()
    if camera.GetParallelProjection():
        worldHeight = 2*camera.GetParallelScale()
    else:
        mat = camera.GetViewTransformMatrix()
        cvz = np.zeros(3)
        cvz[0] = mat.GetElement(2, 0)
        cvz[1] = mat.GetElement(2, 1)
        cvz[2] = mat.GetElement(2, 2)

        cameraPosition = np.array(camera.GetPosition())
        v = cameraPosition - np.array(position)
        worldHeight = 2*(np.dot(v, cvz) * np.tan(0.5*camera.GetViewAngle()/57.296))
    windowHeight = view.renderer().GetSize()[1]
    return worldHeight / windowHeight


class MeasurementPanel(uipanel.UiPanel):

    def __init__(self, app, view):
        uipanel.UiPanel.__init__(self, 'ddMeasurementPanel.ui')

        self.view = view
        self.ui.enabledCheck.connect('toggled(bool)', self.onEnabledCheckBox)
        self.ui.clearButton.connect('clicked()', self.onClear)

        #self.snapshotTextShortcut = applogic.addShortcut(app.mainWindow, ' ', self.snapshotText)
        #self.snapshotGeometryShortcut = applogic.addShortcut(app.mainWindow, 'Shift+ ', self.snapshotGeometry)
        self.eventFilter = MyEventFilter(view, self)
        self.annotation = vis.PolyDataItem('annotation', self.makeSphere((0,0,0)), view)
        self.annotation.setProperty('Color', [0,1,0])
        self.annotation.actor.SetPickable(False)
        self.annotation.actor.SetUserTransform(vtk.vtkTransform())
        self.pickPoints = []
        self.setEnabled(False)


    def onEnabledCheckBox(self):
        self.setEnabled(self.isEnabled())

    def isEnabled(self):
        return bool(self.ui.enabledCheck.checked)

    def setEnabled(self, enabled):
        self.ui.enabledCheck.checked = enabled

        self.annotation.setProperty('Visible', False)

        folder = self.getRootFolder(create=False)
        if folder:
            for obj in folder.children():
                obj.actor.SetPickable(not enabled)

        #self.snapshotTextShortcut.enabled = self.isEnabled()
        #self.snapshotGeometryShortcut.enabled = self.isEnabled()
        if self.isEnabled():
            self.eventFilter.installEventFilter()
        else:
            self.eventFilter.removeEventFilter()

        self.ui.panelContents.setEnabled(enabled)

    def onClear(self):
        self.ui.textEdit.clear()
        om.removeFromObjectModel(self.getRootFolder())
        self.pickPoints = []

    def pickIsValid(self):
        return self.ui.objName.text != 'none'

    def getRootFolder(self, create=True):
        name = 'measurements'
        if create:
            return om.getOrCreateContainer(name)
        else:
            return om.findObjectByName(name)

    def makeSphere(self, position, radius=1.0):
        d = DebugData()
        d.addSphere(position, radius=radius)
        return d.getPolyData()

    def snapshotGeometry(self):
        if not self.pickIsValid():
            return

        p = np.array([float(x) for x in self.ui.pickPt.text.split(', ')])
        self.pickPoints.append(p)
        polyData = self.makeSphere((0,0,0))
        folder = self.getRootFolder()
        i = len(folder.children())
        obj = vis.showPolyData(polyData, 'point %d' % i, color=[1,0,0], parent=folder)
        obj.actor.SetPickable(False)

        scale = computeViewScale(self.view, p)
        scale = scale * 10
        t = vtk.vtkTransform()
        t.Translate(p)
        t.Scale(scale, scale, scale)
        obj.actor.SetUserTransform(t)

    def snapshotText(self):
        if not self.pickIsValid():
            return

        if len(self.pickPoints) > 1:
            dist = np.linalg.norm(self.pickPoints[-1] - self.pickPoints[-2])
        else:
            dist = 0.0

        s = 'pick_point ' + self.ui.pickPt.text + '\n'
        s += 'pick_normal ' + self.ui.pickNormal.text + '\n'
        s += 'dist_to_previous_point ' + '%f' % dist + '\n'
        s += '\n'

        self.ui.textEdit.append(s.replace('\n','<br/>'))

    def onShiftMouseClick(self, displayPoint):
        self.updatePick(displayPoint)
        self.snapshotGeometry()
        self.snapshotText()
        self.annotation.setProperty('Visible', False)

    def onMouseMove(self, displayPoint):
        self.updatePick(displayPoint)

    def updatePick(self, displayPoint):

        pickType = str(self.ui.pickTypeCombo.currentText)
        if 'render' in pickType:
            pickType = 'render'
        elif 'vertex' in pickType:
            pickType = 'points'
        elif 'surface' in pickType:
            pickType = 'cells'
        else:
            raise Exception('unknown pick type')


        tolerance = self.ui.toleranceSpinBox.value
        pickPointFields = vis.pickPoint(
            displayPoint,
            self.view,
            pickType=pickType,
            tolerance=tolerance)
        worldPoint = pickPointFields.pickedPoint
        prop = pickPointFields.pickedProp
        dataset = pickPointFields.pickedDataset
        normal = pickPointFields.pickedNormal


        if not prop:
            worldPoint = np.zeros(3)
            normal = np.zeros(3)

        obj = vis.getObjectByProp(prop)

        self.ui.displayPt.text = '%d, %d' % tuple(displayPoint)
        self.ui.worldPt.text = '%.5f, %.5f, %.5f' % tuple(worldPoint)
        self.ui.pickPt.text = '%.5f, %.5f, %.5f' % tuple(worldPoint)

        if normal is not None:
          self.ui.pickNormal.text = '%.5f, %.5f, %.5f' % tuple(normal)
        else:
          self.ui.pickNormal.text = 'not available'

        scale = computeViewScale(self.view, worldPoint)
        scale = scale * 10

        self.annotation.setProperty('Visible', prop is not None)
        t = vtk.vtkTransform()
        t.Translate(worldPoint)
        t.Scale(scale, scale, scale)
        self.annotation.actor.SetUserTransform(t)
        self.annotation._renderAllViews()

        if obj:
            self.ui.objName.text = obj.getProperty('Name')
        else:
            self.ui.objName.text = 'none'

        if dataset:
            self.ui.numPts.text = dataset.GetNumberOfPoints()
            self.ui.numCells.text = dataset.GetNumberOfCells()
        else:
            self.ui.numPts.text = '0'
            self.ui.numCells.text = '0'
