import PythonQt
from PythonQt import QtCore, QtGui
import ddapp.visualization as vis
import ddapp.objectmodel as om
from ddapp.debugVis import DebugData
import ddapp.vtkAll as vtk
from ddapp import callbacks
import numpy as np
from ddapp.affordanceitems import AffordanceItem


class PointPicker(object):

    def __init__(self, view, obj=None, callback=None, numberOfPoints=2, drawLines=True, abortCallback=None):

        self.view = view
        self.obj = obj
        self.pickType = 'points'
        self.tolerance = 0.01
        self.numberOfPoints = numberOfPoints
        self.drawLines = drawLines
        self.annotationObj = None
        self.annotationFunc = callback
        self.abortFunc = abortCallback
        self.annotationName = 'annotation'
        self.annotationFolder = 'segmentation'
        self.clear()

    def start(self):
        self.installEventFilter()
        self.clear()

    def stop(self):
        self.removeEventFilter()

    def installEventFilter(self):

        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        self.view.vtkWidget().installEventFilter(self.eventFilter)

        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseMove)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonPress)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.KeyPress)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.KeyRelease)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.onEvent)

    def removeEventFilter(self):
        self.view.vtkWidget().removeEventFilter(self.eventFilter)

    def onEvent(self, obj, event):

        if event.type() == QtCore.QEvent.KeyPress and event.key() == QtCore.Qt.Key_Escape:
            self.stop()
            self.clear()
            if self.abortFunc is not None:
                self.abortFunc()
            return

        if event.modifiers() != QtCore.Qt.ShiftModifier:
            if self.annotationObj:
                self.annotationObj.setProperty('Visible', False)
            return

        if self.annotationObj:
            self.annotationObj.setProperty('Visible', True)

        if event.type() == QtCore.QEvent.MouseMove:
            self.onMouseMove(vis.mapMousePosition(obj, event), event.modifiers())
        elif event.type() == QtCore.QEvent.MouseButtonPress:
            self.onMousePress(vis.mapMousePosition(obj, event), event.modifiers())

    def clear(self):
        self.annotationObj = None
        self.points = [None for i in xrange(self.numberOfPoints)]
        self.hoverPos = None
        self.lastMovePos = [0, 0]

    def onMouseMove(self, displayPoint, modifiers=None):
        self.lastMovePos = displayPoint
        self.tick()

    def onMousePress(self, displayPoint, modifiers=None):

        for i in xrange(self.numberOfPoints):
            if self.points[i] is None:
                self.points[i] = self.hoverPos
                break

        if self.points[-1] is not None:
            self.finish()

    def finish(self):

        points = [p.copy() for p in self.points]
        if self.annotationFunc is not None:
            self.annotationFunc(*points)

        self.clear()


    def draw(self):

        d = DebugData()

        points = [p if p is not None else self.hoverPos for p in self.points]

        # draw points
        for p in points:
            if p is not None:
                d.addSphere(p, radius=0.008)

        if self.drawLines:
            # draw lines
            for a, b in zip(points, points[1:]):
                if b is not None:
                    d.addLine(a, b)

            # connect end points
            if points[-1] is not None:
                d.addLine(points[0], points[-1])

        self.annotationObj = vis.updatePolyData(d.getPolyData(), self.annotationName, parent=self.annotationFolder)
        self.annotationObj.setProperty('Color', [1,0,0])
        self.annotationObj.actor.SetPickable(False)


    def tick(self):

        if self.obj is None:
            self.hoverPos, prop, _ = vis.pickPoint(self.lastMovePos, self.view, pickType=self.pickType, tolerance=self.tolerance)
            if prop is None:
                self.hoverPos = None
        else:
            self.hoverPos = vis.pickPoint(self.lastMovePos, self.view, obj=self.obj, pickType=self.pickType, tolerance=self.tolerance)

        self.draw()



class ImagePointPicker(object):


    DOUBLE_CLICK_EVENT = 'DOUBLE_CLICK_EVENT'


    def __init__(self, imageView, obj=None, callback=None, numberOfPoints=1, drawLines=True):

        self.imageView = imageView
        self.view = imageView.view
        self.obj = obj
        self.drawLines = drawLines
        self.annotationObj = None
        self.annotationFunc = callback
        self.numberOfPoints = numberOfPoints
        self.showCursor = False
        self.cursorObj = None
        self.callbacks = callbacks.CallbackRegistry([self.DOUBLE_CLICK_EVENT])
        self.clear()

    def start(self):
        self.installEventFilter()
        self.clear()

    def stop(self):
        self.removeEventFilter()

    def installEventFilter(self):

        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        self.view.vtkWidget().installEventFilter(self.eventFilter)

        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseMove)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonPress)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonDblClick)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.KeyPress)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.KeyRelease)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.onEvent)

    def removeEventFilter(self):
        self.view.vtkWidget().removeEventFilter(self.eventFilter)

    def connectDoubleClickEvent(self, func):
        return self.callbacks.connect(self.DOUBLE_CLICK_EVENT, func)

    def disconnectDoubleClickEvent(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def onEvent(self, obj, event):

        if event.type() == QtCore.QEvent.MouseButtonDblClick and event.button() == QtCore.Qt.LeftButton:

            self.callbacks.process(self.DOUBLE_CLICK_EVENT, vis.mapMousePosition(obj, event), event.modifiers(), self.imageView)

        if event.type() in (QtCore.QEvent.MouseMove, QtCore.QEvent.MouseButtonPress, QtCore.QEvent.Wheel):
            if self.showCursor:
                self.updateCursor(vis.mapMousePosition(obj, event))
        elif event.type() == QtCore.QEvent.KeyPress:
            if event.key() == QtCore.Qt.Key_Shift:
                self.showCursor = True

        elif event.type() == QtCore.QEvent.KeyRelease:
            if event.key() == QtCore.Qt.Key_Shift:
                self.showCursor = False
                self.hideCursor()

        if event.modifiers() != QtCore.Qt.ShiftModifier:
            if self.annotationObj:
                self.hoverPos = None
                self.draw()
                self.annotationObj.setProperty('Color', [1, 0, 0])
                self.clear()
            return

        if self.annotationObj:
            self.annotationObj.setProperty('Visible', True)

        if event.type() == QtCore.QEvent.MouseMove:
            self.onMouseMove(vis.mapMousePosition(obj, event), event.modifiers())

        elif event.type() == QtCore.QEvent.MouseButtonPress:
            self.onMousePress(vis.mapMousePosition(obj, event), event.modifiers())


    def clear(self):
        if self.annotationObj:
            self.annotationObj.setProperty('Visible', False)
        self.annotationObj = None
        self.points = []
        self.hoverPos = None
        self.lastMovePos = [0, 0]

    def onMouseMove(self, displayPoint, modifiers=None):
        self.lastMovePos = displayPoint
        self.hoverPos = self.displayPointToImagePoint(self.lastMovePos)
        self.draw()

    def onMousePress(self, displayPoint, modifiers=None):

        point = self.displayPointToImagePoint(displayPoint)
        if point is None:
            return

        self.points.append(point)

        if len(self.points) == self.numberOfPoints:
            self.finish()


    def finish(self):
        points = [np.array(p) for p in self.points]
        self.clear()
        if self.annotationFunc is not None:
            self.annotationFunc(*points)

    def draw(self):

        d = DebugData()

        points = list(self.points)
        if self.hoverPos is not None:
            points.append(self.hoverPos)

        # draw points
        for p in points:
            d.addSphere(p, radius=5)

        if self.drawLines and len(points) > 1:
            for a, b in zip(points, points[1:]):
                d.addLine(a, b)

            # connect end points
            # d.addLine(points[0], points[-1])

        if self.annotationObj:
            self.annotationObj.setPolyData(d.getPolyData())
        else:
            self.annotationObj = vis.updatePolyData(d.getPolyData(), 'annotation', parent='segmentation', color=[1,0,0], view=self.view)
            self.annotationObj.addToView(self.view)
            self.annotationObj.actor.SetPickable(False)
            self.annotationObj.actor.GetProperty().SetLineWidth(2)

    def hideCursor(self):
        if self.cursorObj:
            om.removeFromObjectModel(self.cursorObj)

    def updateCursor(self, displayPoint):

        center = self.displayPointToImagePoint(displayPoint, restrictToImageDimensions=False)
        center = np.array(center)

        d = DebugData()
        d.addLine(center + [0, -3000, 0], center + [0, 3000, 0])
        d.addLine(center + [-3000, 0, 0], center + [3000, 0, 0])
        self.cursorObj = vis.updatePolyData(d.getPolyData(), 'cursor', alpha=0.5, view=self.view)
        self.cursorObj.addToView(self.view)
        self.cursorObj.actor.SetUseBounds(False)
        self.cursorObj.actor.SetPickable(False)
        self.view.render()

    def displayPointToImagePoint(self, displayPoint, restrictToImageDimensions=True):
        point = self.imageView.getImagePixel(displayPoint, restrictToImageDimensions)
        if point is not None:
            point[2] = -1.0
            return point


class PlacerWidget(object):

    def __init__(self, view, handle, points):

        assert handle.actor
        assert handle.actor.GetUserTransform()

        self.view = view
        self.handle = handle
        self.points = points
        self.moving = False

    def start(self):
        self.installEventFilter()

    def stop(self):
        self.removeEventFilter()

    def installEventFilter(self):

        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        self.view.vtkWidget().installEventFilter(self.eventFilter)

        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseMove)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonPress)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonRelease)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.onEvent)

    def removeEventFilter(self):
        self.view.vtkWidget().removeEventFilter(self.eventFilter)

    def onEvent(self, obj, event):

        if event.type() == QtCore.QEvent.MouseMove:
            self.onMouseMove(vis.mapMousePosition(obj, event), event.modifiers())
        elif event.type() == QtCore.QEvent.MouseButtonPress and event.button() == QtCore.Qt.LeftButton:
            self.onMousePress(vis.mapMousePosition(obj, event), event.modifiers())
        elif event.type() == QtCore.QEvent.MouseButtonRelease and event.button() == QtCore.Qt.LeftButton:
            self.onMouseRelease(vis.mapMousePosition(obj, event), event.modifiers())

    def onMouseMove(self, displayPoint, modifiers=None):

        self.updateHighlight(displayPoint)

        if not self.moving:
            return


        self.eventFilter.setEventHandlerResult(True)

        pickPoint = self.getPointPick(displayPoint)
        # print displayPoint, pickPoint

        if pickPoint is not None:
            t = self.handle.actor.GetUserTransform()
            assert t
            currentPos = np.array(t.GetPosition())
            t.Translate(np.array(pickPoint - currentPos))
            t.Modified()
            self.handle._renderAllViews()

    def onMousePress(self, displayPoint, modifiers=None):

        picked = self.getHandlePick(displayPoint)
        if picked is not None:
            self.moving = True
            self.eventFilter.setEventHandlerResult(True)

    def onMouseRelease(self, displayPoint, modifiers=None):

        if self.moving:
            self.eventFilter.setEventHandlerResult(True)
            self.moving = False

    def updateHighlight(self, displayPoint):
        if self.getHandlePick(displayPoint) is not None:
            self.handle.actor.GetProperty().SetAmbient(0.5)
            self.handle._renderAllViews()
        else:
            self.handle.actor.GetProperty().SetAmbient(0.0)
            self.handle._renderAllViews()

    def getHandlePick(self, displayPoint):
        return vis.pickPoint(displayPoint, self.view, obj=self.handle, pickType='cells', tolerance=0.01)

    def getPointPick(self, displayPoint):
        return vis.pickPoint(displayPoint, self.view, obj=self.points, pickType='cells', tolerance=0.01)






class AffordancePicker(object):

    def __init__(self, view, manager, callback=None, numberOfObjects=1, drawLines=True, abortCallback=None, filterFunc=None, hoverColor=[1.0, 0.8, 0.8, 1.0]):

        self.view = view
        self.affordanceManager = manager
        self.tolerance = 0.01
        self.numberOfObjects = numberOfObjects
        self.callbackFunc = callback
        self.abortFunc = abortCallback
        self.filterFunc = filterFunc
        self.hoverColor = hoverColor[0:3]
        self.hoverAlpha = hoverColor[3]
        self.pickedObj = None
        self.clear()

    def start(self):
        self.installEventFilter()
        self.clear()

    def stop(self):
        self.removeEventFilter()

    def installEventFilter(self):

        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        self.view.vtkWidget().installEventFilter(self.eventFilter)

        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseMove)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonPress)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.KeyPress)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.KeyRelease)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.onEvent)

    def removeEventFilter(self):
        self.view.vtkWidget().removeEventFilter(self.eventFilter)

    def onEvent(self, obj, event):

        if event.type() == QtCore.QEvent.KeyPress and event.key() == QtCore.Qt.Key_Escape:
            self.stop()
            self.clear()
            if self.abortFunc is not None:
                self.abortFunc()
            return

        if event.type() == QtCore.QEvent.MouseMove:
            self.onMouseMove(vis.mapMousePosition(obj, event), event.modifiers())
        elif event.type() == QtCore.QEvent.MouseButtonPress:
            self.onMousePress(vis.mapMousePosition(obj, event), event.modifiers())

    def clear(self):
        self.objects = [None for i in xrange(self.numberOfObjects)]
        self.hoverPos = None
        self.lastMovePos = [0, 0]
        if self.pickedObj is not None:
            self.pickedObj.setProperty('Color', self.storedProps['Color'])
            self.pickedObj.setProperty('Alpha', self.storedProps['Alpha'])
        self.pickedObj = None
        self.storedProps = {'Color': [0,0,0], 'Alpha':1.0}

    def onMouseMove(self, displayPoint, modifiers=None):
        self.lastMovePos = displayPoint
        self.tick()

    def onMousePress(self, displayPoint, modifiers=None):
        for i in xrange(self.numberOfObjects):
            if self.objects[i] is None:
                self.objects[i] = self.pickedObj
                break

        if self.objects[-1] is not None:
            self.finish()

    def finish(self):
        if self.callbackFunc is not None:
            self.callbackFunc(self.objects)
        self.clear()
        self.stop()


    def draw(self):
        pass
        # TODO

    def tick(self):

        # get affordances
        affs = self.affordanceManager.getAffordances()
        affs = [a for a in affs if a.getProperty('Visible')]
        if self.filterFunc is not None:
            affs = [a for a in affs if self.filterFunc(a)]

        # get picked affordance
        self.hoverPos, prop, _ = vis.pickPoint(self.lastMovePos, self.view, pickType='cells', tolerance=self.tolerance, obj=affs)
        prevPickedObj = self.pickedObj
        curPickedObj = vis.getObjectByProp(prop)
        if curPickedObj is not prevPickedObj:
            if prevPickedObj is not None:
                prevPickedObj.setProperty('Color', self.storedProps['Color'])
                prevPickedObj.setProperty('Alpha', self.storedProps['Alpha'])
            if curPickedObj is not None:
                self.storedProps['Color'] = curPickedObj.getProperty('Color')
                self.storedProps['Alpha'] = curPickedObj.getProperty('Alpha')
                curPickedObj.setProperty('Color', self.hoverColor)
                curPickedObj.setProperty('Alpha', self.hoverAlpha)
            self.pickedObj = curPickedObj

        self.draw()
