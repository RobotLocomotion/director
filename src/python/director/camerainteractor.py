from director import vieweventfilter
from director import visualization as vis
from director import vtkAll as vtk
from director.debugVis import DebugData
from PythonQt import QtCore

import numpy as np


class CameraInteractor(vieweventfilter.ViewEventFilter):

    def __init__(self, view):
        vieweventfilter.ViewEventFilter.__init__(self, view)
        d = DebugData()
        d.addSphere((0, 0, 0), radius=1.0)
        self.cameraCenterObj = vis.PolyDataItem('mouse point', d.getPolyData(), view)
        self.cameraCenterObj.setProperty('Visible', False)
        self.cameraCenterObj.setProperty('Color', [1, 1, 0])
        self.cameraCenterObj.actor.SetUserTransform(vtk.vtkTransform())
        self.cameraCenterObj.actor.SetPickable(False)
        self.renderWindowobserver = view.renderWindow().AddObserver('StartEvent', self.onStartRender)
        self.renderWindowobserver = view.renderWindow().AddObserver('EndEvent', self.onEndRender)
        self.style = vtk.vtkPickCenteredInteractorStyle()
        self.style.SetMotionFactor(3)
        self.showOnMove = False
        self.showCenter = False
        self.lastHitPoint = None
        self._enabled = False
        self.setEnabled(True)

    def getViewInteractorStyle(self):
        return self.view.renderWindow().GetInteractor().GetInteractorStyle()

    def setEnabled(self, enabled):
        if enabled:
            self.installEventFilter()
            self._previousStyle = self.getViewInteractorStyle()
            self.view.renderWindow().GetInteractor().SetInteractorStyle(self.style)
        else:
            self.removeEventFilter()
            self.view.renderWindow().GetInteractor().SetInteractorStyle(self._previousStyle)        
        self._enabled = enabled

    def getEnabled(self):
        return self._enabled

    def filterEvent(self, obj, event):
        if event.type() == QtCore.QEvent.MouseButtonRelease:
            self.hideCameraCenter()
        vieweventfilter.ViewEventFilter.filterEvent(self, obj, event)

    def onRightMousePress(self, event):
        self.updateMouseHitPoint(event)

    def onLeftMousePress(self, event):
        self.updateMouseHitPoint(event)

    def onMiddleMousePress(self, event):
        self.updateMouseHitPoint(event)

    def onMouseMove(self, event):
        if self.showCenter and self.showOnMove:
            self.cameraCenterObj.setProperty('Visible', True)
        self.showOnMove = False

    def hideCameraCenter(self):
        self.cameraCenterObj.setProperty('Visible', False)
        self.showOnMove = False

    def updateMouseHitPoint(self, event):

        displayPoint = self.getMousePositionInView(event)
        camera = self.view.camera()

        actors = self.view.renderer().GetActors()
        actors = [actors.GetItemAsObject(i) for i in range(actors.GetNumberOfItems())]
        pickableSettings = [actor.GetPickable() for actor in actors]
        for actor in actors:
            if actor != self.cameraCenterObj.actor:
                actor.SetPickable(True)

        #res = vis.pickPoint(displayPoint, self.view, pickType='render')
        #if res.pickedProp:
        #    print('got pick with hardware')
        #if not res.pickedProp:
        res = vis.pickPoint(displayPoint, self.view, pickType='points', tolerance=0.0005)
        if not res.pickedProp:
            res = vis.pickPoint(displayPoint, self.view, pickType='points', tolerance=0.001)
        if not res.pickedProp:
            res = vis.pickPoint(displayPoint, self.view, pickType='cells')

        for actor, pickable in zip(actors, pickableSettings):
            actor.SetPickable(pickable)

        if res.pickedProp is not None:
            worldPoint = res.pickedPoint
        else:
            pt1, pt2 = vis.getRayFromDisplayPoint(self.view, displayPoint)
            if self.lastHitPoint is None:
                worldPoint = projectPointToLine(pt1, pt2, np.array(camera.GetFocalPoint()))
            else:
                projectedWorldPoint, pcoord = projectPointToLineParam(pt1, pt2, self.lastHitPoint)
                if pcoord >= 0.1:
                    worldPoint = projectedWorldPoint
                else:
                    worldPoint = projectPointToLine(pt1, pt2, np.array(camera.GetFocalPoint()))

        # don't use last hit point
        # self.lastHitPoint = np.array(worldPoint)

        t = vtk.vtkTransform()
        t.Translate(worldPoint[:3])
        self.cameraCenterObj.actor.SetUserTransform(t)
        self.showOnMove = True
        self.style.SetCustomCenterOfRotation(worldPoint[:3])

    def onEndRender(self, renderWindow, event):
        if not self._enabled:
            return

    def onStartRender(self, renderWindow, event):
        if not self._enabled:
            return

        if not self.cameraCenterObj.getProperty('Visible'):
            return

        t = self.cameraCenterObj.actor.GetUserTransform()
        scale = self.style.ComputeScale(t.GetPosition(), self.view.renderer())
        scale = scale * 10

        tt = vtk.vtkTransform()
        tt.Translate(t.GetPosition())
        tt.Scale(scale, scale, scale)
        self.cameraCenterObj.actor.SetUserTransform(tt)


def projectPointToLineParam(linePoint1, linePoint2, pt):
    lineVector = linePoint2 - linePoint1
    pcoord =  np.dot(pt - linePoint1, lineVector) / np.dot(lineVector, lineVector)
    result = linePoint1 + pcoord * lineVector
    return result, pcoord


def projectPointToLine(linePoint1, linePoint2, pt):
    return projectPointToLineParam(linePoint1, linePoint2, pt)[0]
