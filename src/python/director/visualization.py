import ddapp.objectmodel as om
import ddapp.applogic as app
from shallowCopy import shallowCopy
import ddapp.vtkAll as vtk
from ddapp.timercallback import TimerCallback
from ddapp import transformUtils
from ddapp import callbacks
from ddapp import frameupdater
import numpy as np
from PythonQt import QtCore, QtGui



import os
import weakref
import itertools

def computeAToB(a,b):

    t = vtk.vtkTransform()
    t.PostMultiply()
    t.Concatenate(b)
    t.Concatenate(a.GetLinearInverse())
    tt = vtk.vtkTransform()
    tt.SetMatrix(t.GetMatrix())
    return tt



class PolyDataItem(om.ObjectModelItem):

    def __init__(self, name, polyData, view):

        om.ObjectModelItem.__init__(self, name, om.Icons.Robot)

        self.views = []
        self.polyData = polyData
        self.mapper = vtk.vtkPolyDataMapper()
        self.mapper.SetInput(self.polyData)
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(self.mapper)
        self.shadowActor = None
        self.scalarBarWidget = None
        self.extraViewRenderers = {}

        self.rangeMap = {
            'intensity' : (400, 4000),
            #'z' : (0.0, 2.0),
            #'distance' : (0.5, 4.0),
            'spindle_angle' : (0, 360),
            'azimuth' : (-2.5, 2.5),
            'scan_delta' : (0.0, 0.3)
            }

        self.addProperty('Color By', 0, attributes=om.PropertyAttributes(enumNames=['Solid Color']))
        self.addProperty('Visible', True)
        self.addProperty('Alpha', 1.0,
                         attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=1.0, singleStep=0.1, hidden=False))
        self.addProperty('Point Size', self.actor.GetProperty().GetPointSize(),
                         attributes=om.PropertyAttributes(decimals=0, minimum=1, maximum=20, singleStep=1, hidden=False))

        self.addProperty('Surface Mode', 0,
                         attributes=om.PropertyAttributes(enumNames=['Surface', 'Wireframe', 'Surface with edges', 'Points'], hidden=True))

        self.addProperty('Color', [1.0, 1.0, 1.0])
        self.addProperty('Show Scalar Bar', False)

        self._updateSurfaceProperty()
        self._updateColorByProperty()

        if view is not None:
            self.addToView(view)

    def _renderAllViews(self):
        for view in self.views:
            view.render()

    def hasDataSet(self, dataSet):
        return dataSet == self.polyData

    def setPolyData(self, polyData):

        self.polyData = polyData
        self.mapper.SetInput(polyData)

        self._updateSurfaceProperty()
        self._updateColorByProperty()
        self._updateColorBy(retainColorMap=True)

        if self.getProperty('Visible'):
            self._renderAllViews()


    def getArrayNames(self):
        pointData = self.polyData.GetPointData()
        return [pointData.GetArrayName(i) for i in xrange(pointData.GetNumberOfArrays())]

    def setSolidColor(self, color):
        self.setProperty('Color', [float(c) for c in color])
        self.colorBy(None)


    def _isPointCloud(self):
        return self.polyData.GetNumberOfPoints() and (self.polyData.GetNumberOfCells() == self.polyData.GetNumberOfVerts())

    def colorBy(self, arrayName, scalarRange=None, lut=None):

        if not arrayName:
            self.mapper.ScalarVisibilityOff()
            self.polyData.GetPointData().SetActiveScalars(None)
            return

        array = self.polyData.GetPointData().GetArray(arrayName)
        if not array:
            print 'colorBy(%s): array not found' % arrayName
            self.mapper.ScalarVisibilityOff()
            self.polyData.GetPointData().SetActiveScalars(None)
            return

        self.polyData.GetPointData().SetActiveScalars(arrayName)

        if not lut:
            lut = self._getDefaultColorMap(array, scalarRange)

        #self.mapper.SetColorModeToMapScalars()
        self.mapper.ScalarVisibilityOn()
        self.mapper.SetUseLookupTableScalarRange(True)
        self.mapper.SetLookupTable(lut)
        self.mapper.SetInterpolateScalarsBeforeMapping(not self._isPointCloud())

        if self.getProperty('Visible'):
            self._renderAllViews()

    def getChildFrame(self):
        frameName = self.getProperty('Name') + ' frame'
        return self.findChild(frameName)

    def addToView(self, view):
        if view in self.views:
            return

        self.views.append(view)
        view.renderer().AddActor(self.actor)
        if self.shadowActor:
            view.renderer().AddActor(self.shadowActor)
        view.render()

    def _onPropertyChanged(self, propertySet, propertyName):
        om.ObjectModelItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Point Size':
            self.actor.GetProperty().SetPointSize(self.getProperty(propertyName))
        elif propertyName == 'Alpha':
            self.actor.GetProperty().SetOpacity(self.getProperty(propertyName))
            if self.shadowActor:
                self.shadowActor.GetProperty().SetOpacity(self.getProperty(propertyName))
        elif propertyName == 'Visible':
            self.actor.SetVisibility(self.getProperty(propertyName))
            if self.shadowActor:
                self.shadowActor.SetVisibility(self.getProperty(propertyName))

        elif propertyName == 'Surface Mode':
            mode = self.properties.getPropertyEnumValue(propertyName)
            prop =  self.actor.GetProperty()
            if mode == 'Surface':
                prop.SetRepresentationToSurface()
                prop.EdgeVisibilityOff()
            if mode == 'Wireframe':
                prop.SetRepresentationToWireframe()
            elif mode == 'Surface with edges':
                prop.SetRepresentationToSurface()
                prop.EdgeVisibilityOn()
            elif mode == 'Points':
                prop.SetRepresentationToPoints()

        elif propertyName == 'Color':
            color = self.getProperty(propertyName)
            self.actor.GetProperty().SetColor(color)

        elif propertyName == 'Color By':
            self._updateColorBy()

        elif propertyName == 'Show Scalar Bar':
            self._updateScalarBar()

        self._renderAllViews()

    def setScalarRange(self, rangeMin, rangeMax):
        arrayName = self.getPropertyEnumValue('Color By')
        if arrayName != 'Solid Color':
            lut = self.mapper.GetLookupTable()
            self.colorBy(arrayName, scalarRange=(rangeMin, rangeMax))

    def _updateSurfaceProperty(self):
        enableSurfaceMode = self.polyData.GetNumberOfPolys() or self.polyData.GetNumberOfStrips()
        self.properties.setPropertyAttribute('Surface Mode', 'hidden', not enableSurfaceMode)

    def _updateColorBy(self, retainColorMap=False):

        arrayName = self.getPropertyEnumValue('Color By')
        if arrayName == 'Solid Color':
            self.colorBy(None)
        else:
            lut = self.mapper.GetLookupTable() if retainColorMap else None
            self.colorBy(arrayName, lut=lut)

        self._updateScalarBar()

    def _updateColorByProperty(self):
        enumNames = ['Solid Color'] + self.getArrayNames()
        currentValue = self.properties.getProperty('Color By')
        if currentValue >= len(enumNames):
            self.setProperty('Color By', 0)
        self.properties.setPropertyAttribute('Color By', 'enumNames', enumNames)

    def _updateScalarBar(self):
        barEnabled = self.getProperty('Show Scalar Bar')
        colorBy = self.getProperty('Color By')
        if barEnabled and colorBy != 0:
            self._showScalarBar()
        else:
            self._hideScalarBar()

    def _hideScalarBar(self):
        if self.scalarBarWidget:
            self.scalarBarWidget.Off()
            self.scalarBarWidget.SetInteractor(None)
            self.scalarBarWidget = None
            self._renderAllViews()

    def _showScalarBar(self):
        title = self.properties.getPropertyEnumValue('Color By')
        view = self.views[0]
        lut = self.mapper.GetLookupTable()
        self.scalarBarWidget = createScalarBarWidget(view, lut, title)
        self._renderAllViews()

    def getCoolToWarmColorMap(self, scalarRange):

        f = vtk.vtkDiscretizableColorTransferFunction()
        f.DiscretizeOn()
        f.SetColorSpaceToDiverging()
        f.SetNumberOfValues(256)
        f.AddRGBPoint(scalarRange[0],  0.23, 0.299, 0.754)
        f.AddRGBPoint(scalarRange[1], 0.706, 0.016, 0.15)
        f.Build()
        return f

    def _getDefaultColorMap(self, array, scalarRange=None, hueRange=None):

        name = array.GetName()

        blueToRed = (0.667, 0)
        redtoBlue = (0, 0.667)

        hueMap = {
            'Axes' : redtoBlue
        }

        scalarRange = scalarRange or self.rangeMap.get(name, array.GetRange())
        hueRange = hueRange or hueMap.get(name, blueToRed)

        lut = vtk.vtkLookupTable()
        lut.SetNumberOfColors(256)
        lut.SetHueRange(hueRange)
        lut.SetRange(scalarRange)
        lut.Build()

        return lut
        #return self.getCoolToWarmColorMap(scalarRange)

    def shadowOn(self):

        if self.shadowActor:
            return

        mat =  [[1, 0, -1,  0],
                [0, 1, -1,  0],
                [0, 0,  0,  0],
                [0, 0,  0,  1]]

        shadowT = transformUtils.getTransformFromNumpy(mat)

        baseTransform = self.actor.GetUserTransform()
        if baseTransform:
            shadowT.PreMultiply()
            shadowT.Concatenate(baseTransform)

        self.shadowActor = vtk.vtkActor()
        self.shadowActor.SetMapper(self.mapper)
        self.shadowActor.SetUserTransform(shadowT)
        self.shadowActor.GetProperty().LightingOff()
        self.shadowActor.GetProperty().SetColor(0, 0, 0)

        for view in self.views:
            view.renderer().AddActor(self.shadowActor)

    def shadowOff(self):
        for view in self.views:
            view.renderer().RemoveActor(self.shadowActor)
        self.shadowActor = None

    def onRemoveFromObjectModel(self):
        om.ObjectModelItem.onRemoveFromObjectModel(self)
        self.removeFromAllViews()

    def removeFromAllViews(self):
        for view in list(self.views):
            self.removeFromView(view)
        assert len(self.views) == 0
        self._hideScalarBar()

    def removeFromView(self, view):
        assert view in self.views
        self.views.remove(view)
        view.renderer().RemoveActor(self.actor)
        if self.shadowActor:
            view.renderer().RemoveActor(self.shadowActor)
        for renderer in self.extraViewRenderers.get(view, []):
            renderer.RemoveActor(self.actor)
        view.render()



class TextItem(om.ObjectModelItem):

    def __init__(self, name, text='', view=None):

        om.ObjectModelItem.__init__(self, name)

        self.views = []
        self.actor = vtk.vtkTextActor()
        prop = self.actor.GetTextProperty()
        prop.SetFontSize(18)
        self.actor.SetPosition(10,10)
        self.actor.SetInput(text)

        self.addProperty('Visible', True)
        self.addProperty('Text', text)
        self.addProperty('Position', [10, 10], attributes=om.PropertyAttributes(minimum=0, maximum=3000, singleStep=1))
        self.addProperty('Font Size', 18, attributes=om.PropertyAttributes(minimum=6, maximum=128, singleStep=1))
        self.addProperty('Bold', False)
        self.addProperty('Italic', False)

        if view:
            self.addToView(view)


    def addToView(self, view):
        if view in self.views:
            return

        self.views.append(view)
        view.renderer().AddActor(self.actor)
        view.render()

    def _renderAllViews(self):
        for view in self.views:
            view.render()

    def onRemoveFromObjectModel(self):
        om.ObjectModelItem.onRemoveFromObjectModel(self)
        self.removeFromAllViews()

    def removeFromAllViews(self):
        for view in list(self.views):
            self.removeFromView(view)

    def removeFromView(self, view):
        assert view in self.views
        self.views.remove(view)
        view.renderer().RemoveActor(self.actor)
        view.render()

    def _onPropertyChanged(self, propertySet, propertyName):

        om.ObjectModelItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Visible':
            self.actor.SetVisibility(self.getProperty(propertyName))
            self._renderAllViews()
        elif propertyName == 'Text':
            view = app.getCurrentRenderView()
            self.actor.SetInput(self.getProperty(propertyName))
        elif propertyName == 'Position':
            pos = self.getProperty(propertyName)
            self.actor.SetPosition(pos[0], pos[1])
        elif propertyName == 'Font Size':
            self.actor.GetTextProperty().SetFontSize(self.getProperty(propertyName))
        elif propertyName == 'Bold Size':
            self.actor.GetTextProperty().SetBold(self.getProperty(propertyName))
        elif propertyName == 'Italic':
            self.actor.GetTextProperty().SetItalic(self.getProperty(propertyName))


        if self.getProperty('Visible'):
            self._renderAllViews()


def updateText(text, name, **kwargs):

    obj = om.findObjectByName(name)
    obj = obj or showText(text, name, **kwargs)
    obj.setProperty('Text', text)
    return obj


def showText(text, name, fontSize=18, position=(10, 10), parent=None, view=None):

    view = view or app.getCurrentRenderView()
    assert view

    item = TextItem(name, text, view=view)
    item.setProperty('Font Size', fontSize)
    item.setProperty('Position', list(position))

    if isinstance(parent, str):
        parentObj = om.getOrCreateContainer(parent)
    else:
        parentObj = parent

    om.addToObjectModel(item, parentObj)
    return item


class FrameItem(PolyDataItem):

    def __init__(self, name, transform, view):

        PolyDataItem.__init__(self, name, vtk.vtkPolyData(), view)

        self.transform = transform
        self._blockSignals = False

        self.actor.SetUserTransform(transform)

        self.widget = vtk.vtkFrameWidget()
        self.widget.CreateDefaultRepresentation()
        self.widget.EnabledOff()
        self.rep = self.widget.GetRepresentation()
        self.rep.SetTransform(transform)
        self.traceData = None
        self._frameSync = None

        self.addProperty('Scale', 1.0, attributes=om.PropertyAttributes(decimals=2, minimum=0.01, maximum=100, singleStep=0.1, hidden=False))
        self.addProperty('Edit', False)
        self.addProperty('Trace', False)
        self.addProperty('Tube', False)

        self.properties.setPropertyIndex('Edit', 0)
        self.properties.setPropertyIndex('Trace', 1)
        self.properties.setPropertyIndex('Tube', 2)

        self.callbacks.addSignal('FrameModified')
        self.onTransformModifiedCallback = None
        self.observerTag = self.transform.AddObserver('ModifiedEvent', self.onTransformModified)

        self._updateAxesGeometry()
        self.setProperty('Color By', 'Axes')
        self.setProperty('Icon', om.Icons.Axes)


    def connectFrameModified(self, func):
        return self.callbacks.connect('FrameModified', func)

    def disconnectFrameModified(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def onTransformModified(self, transform, event):
        if not self._blockSignals:
            if self.onTransformModifiedCallback:
                self.onTransformModifiedCallback(self)
            self.callbacks.process('FrameModified', self)

    def _createAxes(self, scale, useTube):
        axes = vtk.vtkAxes()
        axes.SetComputeNormals(0)
        axes.SetScaleFactor(scale)
        axes.Update()

        if useTube:
            tube = vtk.vtkTubeFilter()
            tube.SetInput(axes.GetOutput())
            tube.SetRadius(0.002)
            tube.SetNumberOfSides(12)
            tube.Update()
            axes = tube

        return shallowCopy(axes.GetOutput())

    def addToView(self, view):
        PolyDataItem.addToView(self, view)

    def copyFrame(self, transform):
        self._blockSignals = True
        self.transform.SetMatrix(transform.GetMatrix())
        self._blockSignals = False
        self.transform.Modified()
        parent = self.parent()
        if (parent and parent.getProperty('Visible')) or self.getProperty('Visible'):
            self._renderAllViews()

    def getFrameSync(self):
        if self._frameSync is None:
            self._frameSync = FrameSync()
            self._frameSync.addFrame(self)
        return self._frameSync

    def _updateAxesGeometry(self):
        scale = self.getProperty('Scale')
        self.rep.SetWorldSize(scale)
        self.setPolyData(self._createAxes(scale, self.getProperty('Tube')))

    def _onPropertyChanged(self, propertySet, propertyName):
        PolyDataItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Scale':
            scale = self.getProperty(propertyName)
            self.rep.SetWorldSize(scale)
            self._updateAxesGeometry()
        elif propertyName == 'Edit':
            view = app.getCurrentRenderView()
            if view not in self.views:
                view = self.views[0]
            self.widget.SetInteractor(view.renderWindow().GetInteractor())
            
            self.widget.SetEnabled(self.getProperty(propertyName))
            isEditing = self.getProperty(propertyName)
            if isEditing:
                frameupdater.registerFrame(self)
        elif propertyName == 'Trace':
            trace = self.getProperty(propertyName)
            if trace and not self.traceData:
                self.traceData = FrameTraceVisualizer(self)
            elif not trace and self.traceData:
                om.removeFromObjectModel(self.traceData.getTraceData())
                self.traceData = None
        elif propertyName == 'Tube':
            self._updateAxesGeometry()

    def onRemoveFromObjectModel(self):
        PolyDataItem.onRemoveFromObjectModel(self)

        self.transform.RemoveObserver(self.observerTag)

        self.widget.SetInteractor(None)
        self.widget.EnabledOff()
        for view in self.views:
            view.renderer().RemoveActor(self.actor)
            view.render()


class FrameTraceVisualizer(object):

    def __init__(self, frame):
        self.frame = frame
        self.traceName = '%s trace' % frame.getProperty('Name')
        self.lastPosition = np.array(frame.transform.GetPosition())
        self.lineCell = vtk.vtkLine()
        frame.connectFrameModified(self.onFrameModified)

    def getTraceData(self):
        t = self.frame.findChild(self.traceName)
        if not t:
            pts = vtk.vtkPoints()
            pts.SetDataTypeToDouble()
            pts.InsertNextPoint(self.frame.transform.GetPosition())
            pd = vtk.vtkPolyData()
            pd.SetPoints(pts)
            pd.SetLines(vtk.vtkCellArray())
            t = showPolyData(pd, self.traceName, parent=self.frame)
        return t

    def addPoint(self, point):
        traceData = self.getTraceData()
        pd = traceData.polyData

        pd.GetPoints().InsertNextPoint(point)
        numberOfPoints = pd.GetNumberOfPoints()
        line = self.lineCell
        ids = line.GetPointIds()
        ids.SetId(0, numberOfPoints-2)
        ids.SetId(1, numberOfPoints-1)
        pd.GetLines().InsertNextCell(line.GetPointIds())

        pd.Modified()
        traceData._renderAllViews()

    def onFrameModified(self, frame):
        position = np.array(frame.transform.GetPosition())
        if not np.allclose(position, self.lastPosition):
            self.addPoint(position)


class FrameSync(object):

    class FrameData(object):
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    def __init__(self):
        self.frames = {}
        self._blockCallbacks = False
        self._ids = itertools.count()

    def addFrame(self, frame, ignoreIncoming=False):

        if frame is None:
            return

        if self._findFrameId(frame) is not None:
            return

        frameId = self._ids.next()
        callbackId = frame.connectFrameModified(self._onFrameModified)

        self.frames[frameId] = FrameSync.FrameData(
            ref=weakref.ref(frame),
            baseTransform=self._computeBaseTransform(frame),
            callbackId=callbackId,
            ignoreIncoming=ignoreIncoming)

    def removeFrame(self, frame):

        frameId = self._findFrameId(frame)
        if frameId is None:
            raise KeyError(frame)

        frame.disconnectFrameModified(self.frames[frameId].callbackId)
        self._removeFrameId(frameId)

    def _computeBaseTransform(self, frame):

        currentDelta = None
        for frameId, frameData in self.frames.items():

            if frameData.ref() is None:
                self._removeFrameId(frameId)
            elif frameData.ref() is frame:
                continue
            else:
                currentDelta = transformUtils.copyFrame(frameData.baseTransform.GetLinearInverse())
                currentDelta.Concatenate(transformUtils.copyFrame(frameData.ref().transform))
                break

        t = transformUtils.copyFrame(frame.transform)
        t.PostMultiply()
        if currentDelta:
            t.Concatenate(currentDelta.GetLinearInverse())

        return t

    def _removeFrameId(self, frameId):
        del self.frames[frameId]

    def _findFrameId(self, frame):

        for frameId, frameData in self.frames.items():

            if frameData.ref() is None:
                self._removeFrameId(frameId)
            elif frameData.ref() is frame:
                return frameId

    def _moveFrame(self, frameId, modifiedFrameId):

        frameData = self.frames[frameId]
        modifiedFrameData = self.frames[modifiedFrameId]

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(frameData.baseTransform)
        t.Concatenate(modifiedFrameData.baseTransform.GetLinearInverse())
        t.Concatenate(modifiedFrameData.ref().transform)
        frameData.ref().copyFrame(t)

    def _onFrameModified(self, frame):

        if self._blockCallbacks:
            return

        modifiedFrameId = self._findFrameId(frame)
        assert modifiedFrameId is not None

        #print self, 'onFrameModified:', self.frames[modifiedFrameId].ref().getProperty('Name')

        if self.frames[modifiedFrameId].ignoreIncoming:
            self.frames[modifiedFrameId].baseTransform = self._computeBaseTransform(frame)
            return

        self._blockCallbacks = True

        for frameId, frameData in self.frames.items():
            if frameData.ref() is None:
                self._removeFrameId(frameId)
            elif frameId != modifiedFrameId:

                #print '  ', self, 'moving:', self.frames[frameId].ref().getProperty('Name')
                self._moveFrame(frameId, modifiedFrameId)

        self._blockCallbacks = False


class ViewOptionsItem(om.ObjectModelItem):

    def __init__(self, view):
        om.ObjectModelItem.__init__(self, 'view options')

        self.view = view
        self.addProperty('Camera projection', 0, attributes=om.PropertyAttributes(enumNames=['Perspective', 'Parallel']))
        self.addProperty('View angle', view.camera().GetViewAngle(), attributes=om.PropertyAttributes(minimum=2, maximum=180))
        self.addProperty('Key light intensity', view.lightKit().GetKeyLightIntensity(), attributes=om.PropertyAttributes(minimum=0, maximum=5, singleStep=0.1, decimals=2))
        self.addProperty('Light kit', True)
        self.addProperty('Eye dome lighting', False)
        self.addProperty('Orientation widget', True)
        self.addProperty('Interactive render', True)
        self.addProperty('Gradient background', True)
        self.addProperty('Background color', view.backgroundRenderer().GetBackground())
        self.addProperty('Background color 2', view.backgroundRenderer().GetBackground2())

    def _onPropertyChanged(self, propertySet, propertyName):

        om.ObjectModelItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName in ('Gradient background', 'Background color', 'Background color 2'):
            colors = [self.getProperty('Background color'), self.getProperty('Background color 2')]

            if not self.getProperty('Gradient background'):
                colors[1] = colors[0]

            self.view.renderer().SetBackground(colors[0])
            self.view.renderer().SetBackground2(colors[1])

        elif propertyName == 'Camera projection':

            if self.getPropertyEnumValue(propertyName) == 'Perspective':
                self.view.camera().ParallelProjectionOff()
            else:
                self.view.camera().ParallelProjectionOn()

        elif propertyName == 'Orientation widget':

            if self.getProperty(propertyName):
                self.view.orientationMarkerWidget().On()
            else:
                self.view.orientationMarkerWidget().Off()

        elif propertyName == 'View angle':

            angle = self.getProperty(propertyName)
            self.view.camera().SetViewAngle(angle)

        elif propertyName == 'Key light intensity':

            intensity = self.getProperty(propertyName)
            self.view.lightKit().SetKeyLightIntensity(intensity)

        elif propertyName == 'Light kit':

            self.view.setLightKitEnabled(self.getProperty(propertyName))

        elif propertyName == 'Eye dome lighting':

            if self.getProperty(propertyName):
                enableEyeDomeLighting(self.view)
            else:
                disableEyeDomeLighting(self.view)

        elif propertyName == 'Interactive render':
            if self.getProperty(propertyName):
                self.view.renderWindow().GetInteractor().EnableRenderOn()
            else:
                self.view.renderWindow().GetInteractor().EnableRenderOff()

        self.view.render()


def showGrid(view, cellSize=0.5, numberOfCells=25, name='grid', parent='sensors', color=[1,1,1], alpha=0.05, gridTransform=None):

    grid = vtk.vtkGridSource()
    grid.SetScale(cellSize)
    grid.SetGridSize(numberOfCells)
    grid.SetSurfaceEnabled(True)
    grid.Update()

    gridObj = showPolyData(grid.GetOutput(), 'grid', view=view, alpha=alpha, color=color, visible=True, parent=parent)
    gridObj.gridSource = grid
    gridObj.actor.GetProperty().LightingOff()
    gridObj.actor.SetPickable(False)

    gridTransform = gridTransform or vtk.vtkTransform()
    gridObj.actor.SetUserTransform(gridTransform)
    showFrame(gridTransform, 'grid frame', scale=0.2, visible=False, parent=gridObj, view=view)

    gridObj.setProperty('Surface Mode', 'Wireframe')

    def computeViewBoundsNoGrid():
        if not gridObj.getProperty('Visible'):
            return

        gridObj.actor.SetUseBounds(False)
        bounds = view.renderer().ComputeVisiblePropBounds()
        gridObj.actor.SetUseBounds(True)
        if vtk.vtkMath.AreBoundsInitialized(bounds):
            view.addCustomBounds(bounds)
        else:
            view.addCustomBounds([-1, 1, -1, 1, -1, 1])

    view.connect('computeBoundsRequest(ddQVTKWidgetView*)', computeViewBoundsNoGrid)
    return gridObj


def createScalarBarWidget(view, lookupTable, title):

    w = vtk.vtkScalarBarWidget()
    bar = w.GetScalarBarActor()
    bar.SetTitle(title)
    bar.SetLookupTable(lookupTable)
    w.SetRepositionable(True)
    w.SetInteractor(view.renderWindow().GetInteractor())
    w.On()

    rep = w.GetRepresentation()
    rep.SetOrientation(0)
    rep.SetPosition(0.77, 0.92)
    rep.SetPosition2(0.20, 0.07)

    return w


def updatePolyData(polyData, name, **kwargs):

    obj = om.findObjectByName(name)
    obj = obj or showPolyData(polyData, name, **kwargs)
    obj.setPolyData(polyData)
    return obj


def updateFrame(frame, name, **kwargs):

    obj = om.findObjectByName(name)
    obj = obj or showFrame(frame, name, **kwargs)
    obj.copyFrame(frame)
    return obj


def showFrame(frame, name, view=None, parent='segmentation', scale=0.35, visible=True):

    view = view or app.getCurrentRenderView()
    assert view

    if isinstance(parent, str):
        parentObj = om.getOrCreateContainer(parent)
    else:
        parentObj = parent

    item = FrameItem(name, frame, view)
    om.addToObjectModel(item, parentObj)
    item.setProperty('Visible', visible)
    item.setProperty('Scale', scale)
    return item


def showPolyData(polyData, name, color=None, colorByName=None, colorByRange=None, alpha=1.0, visible=True, view=None, parent='segmentation', cls=None):

    view = view or app.getCurrentRenderView()
    assert view

    cls = cls or PolyDataItem
    item = cls(name, polyData, view)

    if isinstance(parent, str):
        parentObj = om.getOrCreateContainer(parent)
    else:
        parentObj = parent

    om.addToObjectModel(item, parentObj)
    item.setProperty('Visible', visible)
    item.setProperty('Alpha', alpha)

    if colorByName and colorByName not in item.getArrayNames():
        print 'showPolyData(colorByName=%s): array not found' % colorByName
        colorByName = None

    if colorByName:
        item.setProperty('Color By', colorByName)
        item.colorBy(colorByName, colorByRange)

    else:
        color = [1.0, 1.0, 1.0] if color is None else color
        item.setProperty('Color', [float(c) for c in color])
        item.colorBy(None)

    return item


def addChildFrame(obj, initialTransform=None):
    '''
    Adds a child frame to the given PolyDataItem.  If initialTransform is given,
    the object's polydata is transformed using the inverse of initialTransform
    and then a child frame is assigned to the object to maintain its original
    position.
    '''
    pd = obj.polyData
    t = initialTransform

    if t is None:
        t = vtk.vtkTransform()
        t.PostMultiply()
    else:
        pd = transformPolyData(pd, t.GetLinearInverse())
        obj.setPolyData(pd)

    frame = obj.getChildFrame()
    if frame:
        frame.copyFrame(t)
    else:
        frame = showFrame(t, obj.getProperty('Name') + ' frame', parent=obj, scale=0.2, visible=False)
        obj.actor.SetUserTransform(t)

    return frame


def showHandCloud(hand='left', view=None):

    view = view or app.getCurrentRenderView()
    if view is None:
        return

    assert hand in ('left', 'right')

    maps = om.findObjectByName('Map Server')
    assert maps is not None

    viewId = 52 if hand == 'left' else 53
    reader = maps.source.reader

    def getCurrentViewId():
        return reader.GetCurrentMapId(viewId)

    p = vtk.vtkPolyData()
    obj = showPolyData(p, '%s hand cloud' % hand, view=view, parent='sensors')
    obj.currentViewId = -1

    def updateCloud():
        currentViewId = getCurrentViewId()
        #print 'updateCloud: current view id:', currentViewId
        if currentViewId != obj.currentViewId:
            reader.GetDataForMapId(viewId, currentViewId, p)
            #print 'updated poly data.  %d points.' % p.GetNumberOfPoints()
            obj._renderAllViews()

    t = TimerCallback()
    t.targetFps = 1
    t.callback = updateCloud
    t.start()
    obj.updater = t
    return obj


def showClusterObjects(clusters, parent):

    colors =  [ QtCore.Qt.red,
                QtCore.Qt.blue,
                QtCore.Qt.yellow,
                QtCore.Qt.green,
                QtCore.Qt.magenta,
                QtCore.Qt.cyan,
                QtCore.Qt.darkCyan,
                QtCore.Qt.darkGreen,
                QtCore.Qt.darkMagenta ]

    colors = [QtGui.QColor(c) for c in colors]
    colors = [(c.red()/255.0, c.green()/255.0, c.blue()/255.0) for c in colors]

    objects = []

    for i, cluster in enumerate(clusters):
        name = 'object %d' % i
        color = colors[i % len(colors)]
        clusterObj = showPolyData(cluster.mesh, name, color=color, parent=parent, alpha=1.0)
        clusterFrame = showFrame(cluster.frame, name + ' frame', scale=0.2, visible=False, parent=clusterObj)
        clusterBox = showPolyData(cluster.box, name + ' box', color=color, parent=clusterObj, alpha=0.6, visible=False)
        clusterPoints = showPolyData(cluster.points, name + ' points', color=color, parent=clusterObj, visible=False, alpha=1.0)
        clusterPoints.setProperty('Point Size', 7)
        clusterPoints.colorBy(None)
        objects.append(clusterObj)

        for obj in [clusterObj, clusterBox, clusterPoints]:
            obj.actor.SetUserTransform(cluster.frame)

    return objects


captionWidget = None

def hideCaptionWidget():
    global captionWidget
    if captionWidget is not None:
        captionWidget.Off()
        captionWidget.Render()


def showCaptionWidget(position, text, view=None):

    view = view or app.getCurrentRenderView()
    assert view

    global captionWidget

    if not captionWidget:
        rep = vtk.vtkCaptionRepresentation()
        rep.SetPosition(0.2, 0.8)
        w = vtk.vtkCaptionWidget()
        w.SetInteractor(view.renderWindow().GetInteractor())
        w.SetRepresentation(rep)
        w.On()
        captionWidget = w

    rep = captionWidget.GetRepresentation()
    rep.SetAnchorPosition(position)
    rep.GetCaptionActor2D().SetCaption(text)

    a = rep.GetCaptionActor2D()

    pr = a.GetTextActor().GetTextProperty()
    pr.SetJustificationToCentered()
    pr.SetVerticalJustificationToCentered()
    pr.SetItalic(0)
    pr.SetBold(0)
    pr.SetShadow(0)
    pr.SetFontFamilyToArial()

    c2 = rep.GetPosition2Coordinate()
    c2.SetCoordinateSystemToDisplay()
    c2.SetValue(12*len(text),30)

    # disable border
    #rep.SetShowBorder(0)

    a.SetThreeDimensionalLeader(0)
    a.SetLeaderGlyphSize(0.005)

    captionWidget.On()
    captionWidget.Render()


def getRayFromDisplayPoint(view, displayPoint):
    '''
    Given a view and an XY display point, returns two XYZ world points which
    are the display point at the near/far clipping planes of the view.
    '''
    worldPt1 = [0,0,0,0]
    worldPt2 = [0,0,0,0]
    renderer = view.renderer()

    vtk.vtkInteractorObserver.ComputeDisplayToWorld(renderer, displayPoint[0], displayPoint[1], 0, worldPt1)
    vtk.vtkInteractorObserver.ComputeDisplayToWorld(renderer, displayPoint[0], displayPoint[1], 1, worldPt2)

    worldPt1 = np.array(worldPt1[:3])
    worldPt2 = np.array(worldPt2[:3])
    return worldPt1, worldPt2


def pickImage(displayPoint, view, obj=None):

    picker = vtk.vtkCellPicker()

    if isinstance(obj, str):
        obj = om.findObjectByName(obj)
        assert obj

    if obj:
        picker.AddPickList(obj.actor)
        picker.PickFromListOn()

    picker.Pick(displayPoint[0], displayPoint[1], 0, view.renderer())
    pickedDataset = picker.GetDataSet()

    if obj:
        return picker.GetPointIJK()
    else:
        return pickedDataset, picker.GetPointIJK()


def pickProp(displayPoint, view):

    for tolerance in (0.0, 0.005, 0.01):
        pickType = 'render' if tolerance == 0.0 else 'cells'
        pickedPoint, pickedProp, pickedDataset = pickPoint(displayPoint, view, pickType=pickType, tolerance=tolerance)
        if pickedProp is not None:
            return pickedPoint, pickedProp, pickedDataset

    return None, None, None


def pickPoint(displayPoint, view, obj=None, pickType='points', tolerance=0.01, returnNormal=False):

    assert pickType in ('points', 'cells', 'render')

    view = view or app.getCurrentRenderView()
    assert view

    if isinstance(obj, str):
        obj = om.findObjectByName(obj)
        assert obj


    if pickType == 'render':
        picker = vtk.vtkPropPicker()
    else:
        picker = vtk.vtkPointPicker() if pickType == 'points' else vtk.vtkCellPicker()
        picker.SetTolerance(tolerance)


    if obj:
        if isinstance(obj, list):
            for o in obj:
                picker.AddPickList(o.actor)
            obj = None
        else:
            picker.AddPickList(obj.actor)
        picker.PickFromListOn()

    picker.Pick(displayPoint[0], displayPoint[1], 0, view.renderer())
    pickedProp = picker.GetViewProp()
    pickedPoint = np.array(picker.GetPickPosition())
    pickedDataset = pickedProp.GetMapper().GetInput() if isinstance(pickedProp, vtk.vtkActor) else None

    pickedNormal = np.zeros(3)

    if returnNormal:
        if pickType == 'cells':
          pickedNormal = np.array(picker.GetPickNormal())
        elif pickType == 'points' and pickedDataset:
          pointId = picker.GetPointId()
          normals = pickedDataset.GetPointData().GetNormals()
          if normals:
              pickedNormal = np.array(normals.GetTuple3(pointId))


    if obj:
        if returnNormal:
            return (pickedPoint, pickedNormal) if pickedProp else (None, None)
        else:
            return pickedPoint if pickedProp else None
    else:
        return (pickedPoint, pickedProp, pickedDataset, pickedNormal) if returnNormal else (pickedPoint, pickedProp, pickedDataset)


def mapMousePosition(widget, mouseEvent):
    mousePosition = mouseEvent.pos()
    return mousePosition.x(), widget.height - mousePosition.y()


def getObjectByDataSet(polyData):
    for obj in om.getObjects():
        if obj.hasDataSet(polyData):
            return obj

def getObjectByProp(prop):
    if not prop:
        return None
    for obj in om.getObjects():
        if isinstance(obj, FrameItem) and obj.widget.GetRepresentation() == prop:
            return obj
    if isinstance(prop, vtk.vtkActor):
        return getObjectByDataSet(prop.GetMapper().GetInput())


def findPickedObject(displayPoint, view):

    pickedPoint, pickedProp, pickedDataset = pickProp(displayPoint, view)
    obj = getObjectByProp(pickedProp)
    return obj, pickedPoint


def enableEyeDomeLighting(view):

    seq = vtk.vtkSequencePass()
    opaque = vtk.vtkOpaquePass()

    peeling = vtk.vtkDepthPeelingPass()
    peeling.SetMaximumNumberOfPeels(200)
    peeling.SetOcclusionRatio(0.1)

    translucent = vtk.vtkTranslucentPass()
    peeling.SetTranslucentPass(translucent)

    volume = vtk.vtkVolumetricPass()
    overlay = vtk.vtkOverlayPass()
    lights = vtk.vtkLightsPass()

    passes=vtk.vtkRenderPassCollection()
    passes.AddItem(lights)
    passes.AddItem(opaque)
    #passes.AddItem(peeling)
    passes.AddItem(translucent)
    #passes.AddItem(volume)
    #passes.AddItem(overlay)
    seq.SetPasses(passes)

    edlPass = vtk.vtkEDLShading()
    cameraPass = vtk.vtkCameraPass()

    edlPass.SetDelegatePass(cameraPass)
    cameraPass.SetDelegatePass(seq)
    view.renderer().SetPass(edlPass)


def disableEyeDomeLighting(view):
    view.renderer().SetPass(None)


def showImage(filename):
    '''
    Returns a QLabel displaying the image contents of given filename.
    Make sure to assign the label, it will destruct when it goes out
    of scope.
    '''
    image = QtGui.QImage(filename)
    assert not image.isNull()
    imageLabel = QtGui.QLabel()
    imageLabel.setPixmap(QtGui.QPixmap.fromImage(image))
    imageLabel.setScaledContents(True)
    imageLabel.resize(imageLabel.pixmap.size())
    imageLabel.setWindowTitle(os.path.basename(filename))
    imageLabel.show()
