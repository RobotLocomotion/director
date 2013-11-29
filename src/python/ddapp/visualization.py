import ddapp.objectmodel as om
import ddapp.applogic as app
from ddapp import affordance
import ddapp.affordanceupdater as affup
from shallowCopy import shallowCopy
import ddapp.vtkAll as vtk
from ddapp.debugVis import DebugData
import numpy as np
from PythonQt import QtCore, QtGui

from ddapp.affordancelistener import listener as affListener

def computeAToB(a,b):

    t = vtk.vtkTransform()
    t.PostMultiply()
    t.Concatenate(b)
    t.Concatenate(a.GetLinearInverse())
    tt = vtk.vtkTransform()
    tt.SetMatrix(t.GetMatrix())
    return tt


class AffordanceItem(om.PolyDataItem):

    def __init__(self, name, polyData, view):
        om.PolyDataItem.__init__(self, name, polyData, view)
        self.baseTransform = None
        self.params = {}
        affup.updater.addCallback(self.onGroundTransform)
        affListener.registerAffordance(self)


    def publish(self):
        pass

    def getActionNames(self):
        actions = ['Publish affordance']
        return om.PolyDataItem.getActionNames(self) + actions

    def onAction(self, action):
        if action == 'Publish affordance':
            self.publish()
        else:
            om.PolyDataItem.onAction(self, action)

    def onServerAffordanceUpdate(self, serverAff):
        if not self.params.get('uid'):
            print 'assigning uid from server:', serverAff.uid
            self.params['uid'] = serverAff.uid

    def computeBaseTransform(self):
        self.baseTransform = computeAToB(self.groundTransform, self.actor.GetUserTransform())

    def onGroundTransform(self, newTransform, resetTime):

        if not self.baseTransform or resetTime > self.groundTransformResetTime:
            self.groundTransform = newTransform
            self.groundTransformResetTime = resetTime
            self.computeBaseTransform()
            return

        newUserTransform = vtk.vtkTransform()
        newUserTransform.PostMultiply()
        newUserTransform.Identity()
        newUserTransform.Concatenate(self.baseTransform)
        newUserTransform.Concatenate(newTransform)

        self.actor.GetUserTransform().SetMatrix(newUserTransform.GetMatrix())
        self._renderAllViews()

    def onRemoveFromObjectModel(self):
        om.PolyDataItem.onRemoveFromObjectModel(self)
        affListener.unregisterAffordance(self)



class BlockAffordanceItem(AffordanceItem):

    def setAffordanceParams(self, params):
        self.params = params

    def updateParamsFromActorTransform(self):

        t = self.actor.GetUserTransform()

        xaxis = np.array(t.TransformVector([1,0,0]))
        yaxis = np.array(t.TransformVector([0,1,0]))
        zaxis = np.array(t.TransformVector([0,0,1]))
        self.params['xaxis'] = xaxis
        self.params['yaxis'] = yaxis
        self.params['zaxis'] = zaxis
        self.params['origin'] = t.GetPosition()


    def publish(self):
        self.updateParamsFromActorTransform()
        aff = affordance.createBoxAffordance(self.params)
        affordance.publishAffordance(aff)

        if hasattr(self, 'publishCallback'):
            self.publishCallback()

    def updateICPTransform(self, transform):
        delta = computeAToB(self.icpTransformInitial, transform)
        print 'initial:', self.icpTransformInitial.GetPosition(), self.icpTransformInitial.GetOrientation()
        print 'latest:', transform.GetPosition(), transform.GetOrientation()
        print 'delta:', delta.GetPosition(), delta.GetOrientation()
        newUserTransform = vtk.vtkTransform()
        newUserTransform.PostMultiply()
        newUserTransform.Identity()
        newUserTransform.Concatenate(self.baseTransform)
        newUserTransform.Concatenate(delta.GetLinearInverse())

        self.actor.SetUserTransform(newUserTransform)
        self._renderAllViews()


class FrameAffordanceItem(AffordanceItem):

    def setAffordanceParams(self, params):
        self.params = params
        assert 'otdf_type' in params

    def updateParamsFromActorTransform(self):

        t = self.actor.GetUserTransform()

        xaxis = np.array(t.TransformVector([1,0,0]))
        yaxis = np.array(t.TransformVector([0,1,0]))
        zaxis = np.array(t.TransformVector([0,0,1]))
        self.params['xaxis'] = xaxis
        self.params['yaxis'] = yaxis
        self.params['zaxis'] = zaxis
        self.params['origin'] = t.GetPosition()


    def publish(self):
        self.updateParamsFromActorTransform()
        aff = affordance.createFrameAffordance(self.params)
        affordance.publishAffordance(aff)


class CylinderAffordanceItem(AffordanceItem):

    def setAffordanceParams(self, params):
        self.params = params

    def updateParamsFromActorTransform(self):

        t = self.actor.GetUserTransform()

        xaxis = np.array(t.TransformVector([1,0,0]))
        yaxis = np.array(t.TransformVector([0,1,0]))
        zaxis = np.array(t.TransformVector([0,0,1]))
        self.params['axis'] = zaxis
        self.params['origin'] = t.GetPosition()


    def publish(self):
        self.updateParamsFromActorTransform()
        aff = affordance.createCylinderAffordance(self.params)
        affordance.publishAffordance(aff)


class FrameItem(om.PolyDataItem):

    def __init__(self, name, transform, view):

        scale = 1.0
        self.transform = transform
        polyData = self._createAxes(scale)

        om.PolyDataItem.__init__(self, name, polyData, view)

        self.colorBy('Axes')
        lut = self.mapper.GetLookupTable()
        lut.SetHueRange(0, 0.667)
        lut.Build()

        self.actor.SetUserTransform(transform)

        self.widget = vtk.vtkFrameWidget()
        self.widget.CreateDefaultRepresentation()
        self.widget.EnabledOff()
        self.rep = self.widget.GetRepresentation()
        self.rep.SetWorldSize(scale)
        self.rep.SetTransform(transform)

        self.addProperty('Scale', scale)
        self.addProperty('Edit', False)


    def _createAxes(self, scale):
        axes = vtk.vtkAxes()
        axes.SetComputeNormals(0)
        axes.SetScaleFactor(scale)
        axes.Update()

        #t = vtk.vtkTransformPolyDataFilter()
        #t.SetTransform(transform)
        #t.AddInputConnection(self.axes.GetOutputPort())
        #t.Update()

        return shallowCopy(axes.GetOutput())

    def addToView(self, view):
        om.PolyDataItem.addToView(self, view)

    def setFrame(self, transform):
        self.transform = transform
        polyData = self._createAxes(self.getProperty('Scale'))
        self.setPolyData(polyData)

    def _onPropertyChanged(self, propertyName):

        if propertyName == 'Scale':
            scale = self.getProperty(propertyName)
            self.rep.SetWorldSize(scale)
            self.setPolyData(self._createAxes(scale))
        elif propertyName == 'Edit':
            view = app.getCurrentRenderView()
            if view not in self.views:
                view = self.views[0]
            self.widget.SetInteractor(view.renderWindow().GetInteractor())
            self.widget.SetEnabled(self.getProperty(propertyName))

        om.PolyDataItem._onPropertyChanged(self, propertyName)

    def getPropertyAttributes(self, propertyName):

        if propertyName == 'Scale':
            return om.PropertyAttributes(decimals=2, minimum=0.01, maximum=100, singleStep=0.1, hidden=False)
        else:
            return om.PolyDataItem.getPropertyAttributes(self, propertyName)

    def onRemoveFromObjectModel(self):
        om.PolyDataItem.onRemoveFromObjectModel(self)

        self.widget.SetInteractor(None)
        self.widget.EnabledOff()
        for view in self.views:
            view.renderer().RemoveActor(self.actor)
            view.render()


def updatePolyData(polyData, name, **kwargs):

    obj = om.findObjectByName(name)
    obj = obj or showPolyData(polyData, name, **kwargs)
    obj.setPolyData(polyData)
    return obj


def updateFrame(frame, name, **kwargs):

    obj = om.findObjectByName(name)
    obj = obj or showFrame(frame, name, **kwargs)
    obj.setFrame(frame)
    return obj


def showFrame(frame, name, parent='segmentation', scale=0.35, visible=True):
    if isinstance(parent, str):
        parentObj = om.getOrCreateContainer(parent)
    else:
        parentObj = parent

    item = FrameItem(name, frame, app.getCurrentRenderView())
    om.addToObjectModel(item, parentObj)
    item.setProperty('Visible', visible)
    item.setProperty('Scale', scale)
    return item


def showPolyData(polyData, name, color=None, colorByName=None, colorByRange=None, alpha=1.0, visible=True, view=None, parent='segmentation', cls=None):

    view = view or app.getCurrentRenderView()
    if view is None:
        return

    cls = cls or om.PolyDataItem
    item = cls(name, polyData, view)

    if isinstance(parent, str):
        parentObj = om.getOrCreateContainer(parent)
    else:
        parentObj = parent

    om.addToObjectModel(item, parentObj)
    item.setProperty('Visible', visible)
    item.setProperty('Alpha', alpha)
    if color:
        color = [component * 255 for component in color]
        item.setProperty('Color', QtGui.QColor(*color))
        item.colorBy(None)
    else:
        item.colorBy(colorByName, colorByRange)
    return item


def pickPoint(displayPoint, obj=None, view=None, pickType='points', tolerance=0.01):

    view = view or app.getCurrentRenderView()
    assert view

    picker = vtk.vtkPointPicker() if pickType == 'points' else vtk.vtkCellPicker()

    if isinstance(obj, str):
        obj = om.findObjectByName(objName)
        assert obj

    if obj:
        picker.AddPickList(obj.actor)
        picker.PickFromListOn()

    picker.SetTolerance(tolerance)
    picker.Pick(displayPoint[0], displayPoint[1], 0, view.renderer())
    pickPoints = picker.GetPickedPositions()
    pickedPoint = np.array(pickPoints.GetPoint(0)) if pickPoints.GetNumberOfPoints() else None
    pickedDataset = picker.GetDataSet()

    if obj:
        return pickedPoint
    else:
        return pickedDataset, pickedPoint


def mapMousePosition(widget, mouseEvent):
    mousePosition = mouseEvent.pos()
    return mousePosition.x(), widget.height - mousePosition.y()


def getObjectByDataSet(polyData):
    for item, obj in om.objects.iteritems():
        if isinstance(obj, om.PolyDataItem) and obj.polyData == polyData:
            return obj


def findPickedObject(displayPoint, view=None):

    view = view or app.getCurrentRenderView()
    assert view

    polyData, pickedPoint = pickPoint(displayPoint, view=view, pickType='cells')
    return getObjectByDataSet(polyData), pickedPoint
