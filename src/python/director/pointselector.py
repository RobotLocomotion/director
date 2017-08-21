from director import filterUtils
from director import vieweventfilter
from director import visualization as vis
from director import vtkAll as vtk
from director import vtkNumpy as vnp
from director.shallowCopy import shallowCopy
from PythonQt import QtCore
import numpy as np


class PointSelector(object):
    '''
    Usage:

      selector = PointSelector(view, polyData)

      # hold shift and left click and drag to select points
      # hold shift+control and left click and drag to deselect points

      # disable the selector
      selector.stop()

      # get the selected points
      selection = selector.getSelectedPoints()

      # get the selected point ids of the input points
      pointIds = vnp.getNumpyFromVtk(selection, 'point_ids')
    '''

    class EventFilter(vieweventfilter.ViewEventFilter):
        def onLeftMousePress(self, event):
            modifiers = event.modifiers()
            self.selector.modifiers = modifiers

            isShift = modifiers == QtCore.Qt.ShiftModifier
            isShiftAndControl = (
                modifiers == QtCore.Qt.ShiftModifier | QtCore.Qt.ControlModifier
                or modifiers == QtCore.Qt.ShiftModifier | QtCore.Qt.MetaModifier)

            if isShift or isShiftAndControl:
                self.selector.iren.SetInteractorStyle(self.selector.rubberBandStyle)
                self.selector.selectMode = 1 if isShift else 0

    def __init__(self, view, polyData):
        self.view = view
        self.polyData = shallowCopy(polyData)
        self.selectionObj = None
        self.selectionColor = [1, 0, 0]
        self.selectionPointSize = 3
        self.selectMode = 1
        self.iren = view.renderWindow().GetInteractor()
        self.prevStyle = self.iren.GetInteractorStyle()
        self.rubberBandStyle = vtk.vtkInteractorStyleRubberBand3D()
        self.rubberBandStyle.AddObserver('SelectionChangedEvent', self.onRubberBandPickEvent)
        self.eventFilter = PointSelector.EventFilter(view)
        self.eventFilter.selector = self

        vnp.addNumpyToVtk(self.polyData, np.arange(self.polyData.GetNumberOfPoints(), dtype=int), 'point_ids')
        vnp.addNumpyToVtk(self.polyData, np.zeros(self.polyData.GetNumberOfPoints(), dtype=int), 'is_selected')

    def stop(self):
        self.eventFilter.removeEventFilter()

    def start(self):
        self.eventFilter.installEventFilter()

    def getSelectedPoints(self):
        return filterUtils.thresholdPoints(self.polyData, 'is_selected', [1, 1])

    def getNonSelectedPoints(self):
        return filterUtils.thresholdPoints(self.polyData, 'is_selected', [0, 0])

    def onRubberBandPickEvent(self, obj, event):
        self.pickArea(self.rubberBandStyle.GetStartPosition(), self.rubberBandStyle.GetEndPosition())

    def pickArea(self, startPos, endPos):

        self.iren.SetInteractorStyle(self.prevStyle)

        picker = vtk.vtkAreaPicker()
        picker.AreaPick(min(startPos[0], endPos[0]),
                        min(startPos[1], endPos[1]),
                        max(startPos[0], endPos[0]),
                        max(startPos[1], endPos[1]),
                        self.view.renderer())
        frustum = picker.GetFrustum()

        extractGeometry = vtk.vtkExtractPolyDataGeometry()
        extractGeometry.SetImplicitFunction(frustum)
        extractGeometry.SetInputData(self.polyData)
        extractGeometry.ExtractBoundaryCellsOn()
        extractGeometry.Update()
        selected = filterUtils.cleanPolyData(extractGeometry.GetOutput())

        if not selected.GetNumberOfPoints():
            return

        pickedIds = vnp.getNumpyFromVtk(selected, 'point_ids')
        vnp.getNumpyFromVtk(self.polyData, 'is_selected')[pickedIds] = self.selectMode
        selection = self.getSelectedPoints()

        if not self.selectionObj:
            self.selectionObj = vis.showPolyData(selection, 'selected points', color=self.selectionColor, parent='selection')
            self.selectionObj.setProperty('Point Size', self.selectionPointSize)
        else:
            self.selectionObj.setPolyData(selection)
