import os
import re
import PythonQt
from PythonQt import QtCore, QtGui
from collections import namedtuple
from collections import OrderedDict

from ddapp import callbacks

from ddapp.fieldcontainer import FieldContainer
import vtk

class PropertyAttributes(FieldContainer):

    def __init__(self, **kwargs):

        self._add_fields(
          decimals = 5,
          minimum = -1e4,
          maximum = 1e4,
          singleStep = 1,
          hidden = False,
          enumNames = None,
          readOnly = False,
          )

        self._set_fields(**kwargs)


class Icons(object):

  Directory = QtGui.QApplication.style().standardIcon(QtGui.QStyle.SP_DirIcon)
  Eye = QtGui.QIcon(':/images/eye_icon.png')
  EyeOff = QtGui.QIcon(':/images/eye_icon_gray.png')
  Matlab = QtGui.QIcon(':/images/matlab_logo.png')
  Robot = QtGui.QIcon(':/images/robot_icon.png')
  Laser = QtGui.QIcon(':/images/laser_icon.jpg')
  Feet = QtGui.QIcon(':/images/feet.png')
  Hand = QtGui.QIcon(':/images/claw.png')


def cleanPropertyName(s):
    """
    Generate a valid python property name by replacing all non-alphanumeric characters with underscores and adding an initial underscore if the first character is a digit
    """
    return re.sub(r'\W|^(?=\d)','_',s).lower()  # \W matches non-alphanumeric, ^(?=\d) matches the first position if followed by a digit


class PropertyPanelHelper(object):

    @staticmethod
    def addPropertiesToPanel(properties, panel):

        for propertyName in properties.propertyNames():
            value = properties.getProperty(propertyName)
            attributes = properties._attributes[propertyName]
            if value is not None and not attributes.hidden:
                PropertyPanelHelper._addProperty(panel, propertyName, attributes, value)

    @staticmethod
    def onPropertyValueChanged(panel, properties, propertyName):
        prop = panel.findProperty(propertyName)
        if prop is not None:
            prop.setValue(properties.getProperty(propertyName))

    @staticmethod
    def _setPropertyAttributes(prop, attributes):

        prop.setAttribute('decimals', attributes.decimals)
        prop.setAttribute('minimum', attributes.minimum)
        prop.setAttribute('maximum', attributes.maximum)
        prop.setAttribute('singleStep', attributes.singleStep)
        if attributes.enumNames:
            prop.setAttribute('enumNames', attributes.enumNames)

    @staticmethod
    def _addProperty(panel, name, attributes, value):

        if isinstance(value, list) and not isinstance(value[0], str):
            groupName = '%s [%s]' % (name, ', '.join([str(v) for v in value]))
            groupProp = panel.addGroup(groupName)
            for v in value:
                p = panel.addSubProperty(name, v, groupProp)
                PropertyPanelHelper._setPropertyAttributes(p, attributes)
            return groupProp
        elif attributes.enumNames:
            p = panel.addEnumProperty(name, value)
            PropertyPanelHelper._setPropertyAttributes(p, attributes)
            return p
        else:
            p = panel.addProperty(name, value)
            PropertyPanelHelper._setPropertyAttributes(p, attributes)
            return p



class PropertySet(object):

    PROPERTY_CHANGED_SIGNAL = 'PROPERTY_CHANGED_SIGNAL'
    PROPERTY_ADDED_SIGNAL = 'PROPERTY_ADDED_SIGNAL'
    PROPERTY_ATTRIBUTE_CHANGED_SIGNAL = 'PROPERTY_ATTRIBUTE_CHANGED_SIGNAL'

    def __init__(self):
        self.callbacks = callbacks.CallbackRegistry([self.PROPERTY_CHANGED_SIGNAL,
                                                     self.PROPERTY_ADDED_SIGNAL,
                                                     self.PROPERTY_ATTRIBUTE_CHANGED_SIGNAL])

        self._properties = OrderedDict()
        self._attributes = {}
        self._alternateNames = {}

    def propertyNames(self):
        return self._properties.keys()

    def hasProperty(self, propertyName):
        return propertyName in self._properties

    def connectPropertyChanged(self, func):
        return self.callbacks.connect(self.PROPERTY_CHANGED_SIGNAL, func)

    def disconnectPropertyChanged(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def connectPropertyAdded(self, func):
        return self.callbacks.connect(self.PROPERTY_ADDED_SIGNAL, func)

    def disconnectPropertyAdded(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def connectPropertyAttributeChanged(self, func):
        return self.callbacks.connect(self.PROPERTY_ATTRIBUTE_CHANGED_SIGNAL, func)

    def disconnectPropertyAttributeChanged(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def getProperty(self, propertyName):
        assert self.hasProperty(propertyName)
        return self._properties[propertyName]

    def addProperty(self, propertyName, propertyValue, attributes=None):
        alternateName = cleanPropertyName(propertyName)
        if propertyName not in self._properties and alternateName in self._alternateNames:
            raise ValueError('Adding this property would conflict with a different existing property with alternate name {:s}'.format(alternateName))
        self._alternateNames[alternateName] = propertyName
        self._properties[propertyName] = propertyValue
        self._attributes[propertyName] = attributes or PropertyAttributes()

        self.callbacks.process(self.PROPERTY_ADDED_SIGNAL, self, propertyName)

    def setProperty(self, propertyName, propertyValue):
        assert self.hasProperty(propertyName)

        names = self.getPropertyAttribute(propertyName, 'enumNames')
        if names and type(propertyValue) != int:
            propertyValue = names.index(propertyValue)

        self.oldPropertyValue = (propertyName, self.getProperty(propertyName))
        self._properties[propertyName] = propertyValue
        self.oldPropertyValue = None
        self.callbacks.process(self.PROPERTY_CHANGED_SIGNAL, self, propertyName)

    def getPropertyAttribute(self, propertyName, propertyAttribute):
        assert self.hasProperty(propertyName)
        return getattr(self._attributes[propertyName], propertyAttribute)

    def setPropertyAttribute(self, propertyName, propertyAttribute, value):
        assert self.hasProperty(propertyName)
        attributes = self._attributes[propertyName]
        assert hasattr(attributes, propertyAttribute)
        setattr(attributes, propertyAttribute, value)
        self.callbacks.process(self.PROPERTY_ATTRIBUTE_CHANGED_SIGNAL, self, propertyName. propertyAttribute)

    def __getattribute__(self, name):
        try:
            alternateNames = object.__getattribute__(self, '_alternateNames')
            if name in alternateNames:
                return object.__getattribute__(self, 'getProperty')(alternateNames[name])
            else:
                raise AttributeError()
        except AttributeError:
            return object.__getattribute__(self, name)


class ObjectModelItem(object):

    def __init__(self, name, icon=Icons.Robot, tree=None, properties=None):

        self.properties = properties or PropertySet()
        self.properties.connectPropertyChanged(self._onPropertyChanged)
        self.properties.connectPropertyAdded(self._onPropertyAdded)
        self.properties.connectPropertyAttributeChanged(self._onPropertyAttributeChanged)

        self.icon = icon
        self._tree = tree
        self.addProperty('Name', name, attributes=PropertyAttributes(hidden=True))

    def setIcon(self, icon):
        self.icon = icon
        if self._tree:
            self._tree.updateObjectIcon(self)

    def propertyNames(self):
        return self.properties.propertyNames()

    def hasProperty(self, propertyName):
        return self.properties.hasProperty(propertyName)

    def getProperty(self, propertyName):
        return self.properties.getProperty(propertyName)

    def addProperty(self, propertyName, propertyValue, attributes=None):
        self.properties.addProperty(propertyName, propertyValue, attributes)

    def setProperty(self, propertyName, propertyValue):
        self.properties.setProperty(propertyName, propertyValue)

    def getPropertyAttribute(self, propertyName, propertyAttribute):
        self.properties.getPropertyAttribute(propertyname, propertyAttribute)

    def setPropertyAttribute(self, propertyName, propertyAttribute, value):
        self.properties.setPropertyAttribute(propertyName, propertyAttribute, value)

    def _onPropertyChanged(self, propertySet, propertyName):
        if self._tree is not None:
            self._tree._onPropertyValueChanged(self, propertyName)

    def _onPropertyAdded(self, propertySet, propertyName):
        pass

    def _onPropertyAttributeChanged(self, propertySet, propertyName, propertyAttribute):
        pass

    def hasDataSet(self, dataSet):
        return False

    def getActionNames(self):
        return []

    def onAction(self, action):
        pass

    def getObjectTree(self):
        return self._tree

    def onRemoveFromObjectModel(self):
        pass

    def parent(self):
        if self._tree is not None:
            return self._tree.getObjectParent(self)

    def children(self):
        if self._tree is not None:
            return self._tree.getObjectChildren(self)
        else:
            return []

    def findChild(self, name):
        if self._tree is not None:
            return self._tree.findChildByName(self, name)


class ContainerItem(ObjectModelItem):

    def __init__(self, name):
        ObjectModelItem.__init__(self, name, Icons.Directory)


class RobotModelItem(ObjectModelItem):

    def __init__(self, model):

        modelName = os.path.basename(model.filename())
        ObjectModelItem.__init__(self, modelName, Icons.Robot)

        self.model = model
        model.connect('modelChanged()', self.onModelChanged)
        self.modelChangedCallback = None

        self.addProperty('Filename', model.filename())
        self.addProperty('Visible', model.visible())
        self.addProperty('Alpha', model.alpha(),
                         attributes=PropertyAttributes(decimals=2, minimum=0, maximum=1.0, singleStep=0.1, hidden=False))
        self.addProperty('Color', model.color())
        self.views = []

    def _onPropertyChanged(self, propertySet, propertyName):
        ObjectModelItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Alpha':
            self.model.setAlpha(self.getProperty(propertyName))
        elif propertyName == 'Visible':
            self.model.setVisible(self.getProperty(propertyName))
        elif propertyName == 'Color':
            self.model.setColor(self.getProperty(propertyName))

        self._renderAllViews()

    def hasDataSet(self, dataSet):
        return len(self.model.getLinkNameForMesh(dataSet)) != 0

    def onModelChanged(self):
        if self.modelChangedCallback:
            self.modelChangedCallback(self)

        if self.getProperty('Visible'):
            self._renderAllViews()


    def _renderAllViews(self):
        for view in self.views:
            view.render()

    def getLinkFrame(self, linkName):
        t = vtk.vtkTransform()
        t.PostMultiply()
        if self.model.getLinkToWorld(linkName, t):
            return t
        else:
            return None

    def setModel(self, model):
        assert model is not None
        if model == self.model:
            return

        views = list(self.views)
        self.removeFromAllViews()
        self.model = model
        self.model.setAlpha(self.getProperty('Alpha'))
        self.model.setVisible(self.getProperty('Visible'))
        self.model.setColor(self.getProperty('Color'))
        self.setProperty('Filename', model.filename())
        model.connect('modelChanged()', self.onModelChanged)

        for view in views:
            self.addToView(view)
        self.onModelChanged()

    def addToView(self, view):
        if view in self.views:
            return
        self.views.append(view)
        self.model.addToRenderer(view.renderer())
        view.render()

    def onRemoveFromObjectModel(self):
        self.removeFromAllViews()

    def removeFromAllViews(self):
        for view in list(self.views):
            self.removeFromView(view)
        assert len(self.views) == 0

    def removeFromView(self, view):
        assert view in self.views
        self.views.remove(view)
        self.model.removeFromRenderer(view.renderer())
        view.render()

class PolyDataItem(ObjectModelItem):

    def __init__(self, name, polyData, view):

        ObjectModelItem.__init__(self, name, Icons.Robot)

        self.views = []
        self.polyData = polyData
        self.mapper = vtk.vtkPolyDataMapper()
        self.mapper.SetInput(self.polyData)
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(self.mapper)

        self.addProperty('Visible', True)
        self.addProperty('Point Size', self.actor.GetProperty().GetPointSize(),
                         attributes=PropertyAttributes(decimals=0, minimum=1, maximum=20, singleStep=1, hidden=False))
        self.addProperty('Alpha', 1.0,
                         attributes=PropertyAttributes(decimals=2, minimum=0, maximum=1.0, singleStep=0.1, hidden=False))
        self.addProperty('Color', QtGui.QColor(255,255,255))

        if view is not None:
            self.addToView(view)

    def _renderAllViews(self):
        for view in self.views:
            view.render()

    def hasDataSet(self, dataSet):
        return dataSet == self.polyData

    def setPolyData(self, polyData):

        arrayName = self.getColorByArrayName()

        self.polyData = polyData
        self.mapper.SetInput(polyData)
        self.colorBy(arrayName, lut=self.mapper.GetLookupTable())

        if self.getProperty('Visible'):
            self._renderAllViews()

    def getColorByArrayName(self):
        if self.polyData:
            scalars = self.polyData.GetPointData().GetScalars()
            if scalars:
                return scalars.GetName()

    def getArrayNames(self):
        pointData = self.polyData.GetPointData()
        return [pointData.GetArrayName(i) for i in xrange(pointData.GetNumberOfArrays())]

    def setSolidColor(self, color):

        color = [component * 255 for component in color]
        self.setProperty('Color', QtGui.QColor(*color))
        self.colorBy(None)

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
            if scalarRange is None:
                scalarRange = array.GetRange()

            lut = vtk.vtkLookupTable()
            lut.SetNumberOfColors(256)
            lut.SetHueRange(0.667, 0)
            lut.SetRange(scalarRange)
            lut.Build()


        #self.mapper.SetColorModeToMapScalars()
        self.mapper.ScalarVisibilityOn()
        self.mapper.SetUseLookupTableScalarRange(True)
        self.mapper.SetLookupTable(lut)
        self.mapper.InterpolateScalarsBeforeMappingOff()

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
        view.render()

    def _onPropertyChanged(self, propertySet, propertyName):
        ObjectModelItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Point Size':
            self.actor.GetProperty().SetPointSize(self.getProperty(propertyName))
        elif propertyName == 'Alpha':
            self.actor.GetProperty().SetOpacity(self.getProperty(propertyName))
        elif propertyName == 'Visible':
            self.actor.SetVisibility(self.getProperty(propertyName))
        elif propertyName == 'Color':
            color = self.getProperty(propertyName)
            color = [color.red()/255.0, color.green()/255.0, color.blue()/255.0]
            self.actor.GetProperty().SetColor(color)

        self._renderAllViews()

    def onRemoveFromObjectModel(self):
        self.removeFromAllViews()

    def removeFromAllViews(self):
        for view in list(self.views):
            self.removeFromView(view)
        assert len(self.views) == 0

    def removeFromView(self, view):
        assert view in self.views
        self.views.remove(view)
        view.renderer().RemoveActor(self.actor)
        view.render()


class ObjectModelTree(object):

    def __init__(self):
        self._treeWidget = None
        self._propertiesPanel = None
        self._objects = {}
        self._blockSignals = False

    def getTreeWidget(self):
        return self._treeWidget

    def getPropertiesPanel(self):
        return self._propertiesPanel

    def getActiveItem(self):
        items = self.getTreeWidget().selectedItems()
        return items[0] if len(items) == 1 else None

    def getObjectParent(self, obj):
        item = self._getItemForObject(obj)
        if item.parent():
            return self._getObjectForItem(item.parent())

    def getObjectChildren(self, obj):
        item = self._getItemForObject(obj)
        return [self._getObjectForItem(item.child(i)) for i in xrange(item.childCount())]

    def getActiveObject(self):
        item = self.getActiveItem()
        return self._objects[item] if item is not None else None

    def setActiveObject(self, obj):
        item = self._getItemForObject(obj)
        if item:
            tree = self.getTreeWidget()
            tree.setCurrentItem(item)
            tree.scrollToItem(item)

    def getObjects(self):
        return self._objects.values()

    def _getItemForObject(self, obj):
        for item, itemObj in self._objects.iteritems():
            if itemObj == obj:
                return item

    def _getObjectForItem(self, item):
        return self._objects[item]

    def findObjectByName(self, name, parent=None):
        if parent:
            return self.findChildByName(parent, name)
        for obj in self._objects.values():
            if obj.getProperty('Name') == name:
                return obj

    def findChildByName(self, parent, name):
        for child in self.getObjectChildren(parent):
            if child.getProperty('Name') == name:
                return child

    def onPropertyChanged(self, prop):

        if self._blockSignals:
            return

        if prop.isSubProperty():
            return

        obj = self.getActiveObject()
        obj.setProperty(prop.propertyName(), prop.value())

    def _onTreeSelectionChanged(self):

        panel = self.getPropertiesPanel()
        self._blockSignals = True
        panel.clear()
        self._blockSignals = False

        obj = self.getActiveObject()
        if not obj:
            return

        self._blockSignals = True
        PropertyPanelHelper.addPropertiesToPanel(obj.properties, panel)
        self._blockSignals = False

    def updateVisIcon(self, obj):

        if not obj.hasProperty('Visible'):
            return

        isVisible = obj.getProperty('Visible')
        icon = Icons.Eye if isVisible else Icons.EyeOff
        item = self._getItemForObject(obj)
        item.setIcon(1, icon)

    def updateObjectIcon(self, obj):
        item = self._getItemForObject(obj)
        item.setIcon(0, obj.icon)

    def updateObjectName(self, obj):
        item = self._getItemForObject(obj)
        item.setText(0, obj.getProperty('Name'))

    def _onPropertyValueChanged(self, obj, propertyName):

        if propertyName == 'Visible':
            self.updateVisIcon(obj)
        elif propertyName == 'Name':
            self.updateObjectName(obj)

        if obj == self.getActiveObject():
            self._blockSignals = True
            PropertyPanelHelper.onPropertyValueChanged(self.getPropertiesPanel(), obj.properties, propertyName)
            self._blockSignals = False

    def _onItemClicked(self, item, column):

        obj = self._objects[item]

        if column == 1 and obj.hasProperty('Visible'):
            obj.setProperty('Visible', not obj.getProperty('Visible'))
            self.updateVisIcon(obj)


    def _removeItemFromObjectModel(self, item):
        while item.childCount():
            self._removeItemFromObjectModel(item.child(0))

        try:
            obj = self._getObjectForItem(item)
        except KeyError:
            return

        obj.onRemoveFromObjectModel()
        obj._tree = None

        if item.parent():
            item.parent().removeChild(item)
        else:
            tree = self.getTreeWidget()
            tree.takeTopLevelItem(tree.indexOfTopLevelItem(item))

        del self._objects[item]


    def removeFromObjectModel(self, obj):
        if obj is None:
            return

        item = self._getItemForObject(obj)
        if item:
            self._removeItemFromObjectModel(item)


    def addToObjectModel(self, obj, parentObj=None):
        assert obj._tree is None

        parentItem = self._getItemForObject(parentObj)
        objName = obj.getProperty('Name')

        item = QtGui.QTreeWidgetItem(parentItem, [objName])
        item.setIcon(0, obj.icon)

        obj._tree = self

        self._objects[item] = obj
        self.updateVisIcon(obj)

        if parentItem is None:
            tree = self.getTreeWidget()
            tree.addTopLevelItem(item)
            tree.expandItem(item)


    def collapse(self, obj):
        item = self._getItemForObject(obj)
        if item:
            self.getTreeWidget().collapseItem(item)


    def expand(self, obj):
        item = self._getItemForObject(obj)
        if item:
            self.getTreeWidget().expandItem(item)


    def addContainer(self, name, parentObj=None):
        obj = ContainerItem(name)
        self.addToObjectModel(obj, parentObj)
        return obj


    def getOrCreateContainer(self, name, parentObj=None):
        containerObj = self.findObjectByName(name)
        if not containerObj:
            containerObj = self.addContainer(name, parentObj)
        return containerObj


    def _onShowContextMenu(self, clickPosition):

        obj = self.getActiveObject()
        if not obj:
            return

        globalPos = self.getTreeWidget().viewport().mapToGlobal(clickPosition)

        menu = QtGui.QMenu()

        actions = obj.getActionNames()

        for actionName in obj.getActionNames():
            if not actionName:
                menu.addSeparator()
            else:
                menu.addAction(actionName)

        menu.addSeparator()
        menu.addAction("Remove")

        selectedAction = menu.exec_(globalPos)
        if selectedAction is None:
            return

        if selectedAction.text == "Remove":
            self.removeFromObjectModel(obj)
        else:
            obj.onAction(selectedAction.text)


    def removeSelectedItems(self):
        for item in self.getTreeWidget().selectedItems():
            self._removeItemFromObjectModel(item)


    def _filterEvent(self, obj, event):
        if event.type() == QtCore.QEvent.KeyPress:
            if event.key() == QtCore.Qt.Key_Delete:
                self._eventFilter.setEventHandlerResult(True)
                self.removeSelectedItems()


    def init(self, treeWidget, propertiesPanel):

        self._treeWidget = treeWidget
        self._propertiesPanel = propertiesPanel
        propertiesPanel.clear()
        propertiesPanel.setBrowserModeToWidget()
        propertiesPanel.connect('propertyValueChanged(QtVariantProperty*)', self.onPropertyChanged)

        treeWidget.setColumnCount(2)
        treeWidget.setHeaderLabels(['Name', ''])
        treeWidget.headerItem().setIcon(1, Icons.Eye)
        treeWidget.header().setVisible(True)
        treeWidget.header().setStretchLastSection(False)
        treeWidget.header().setResizeMode(0, QtGui.QHeaderView.Stretch)
        treeWidget.header().setResizeMode(1, QtGui.QHeaderView.Fixed)
        treeWidget.setColumnWidth(1, 24)
        treeWidget.connect('itemSelectionChanged()', self._onTreeSelectionChanged)
        treeWidget.connect('itemClicked(QTreeWidgetItem*, int)', self._onItemClicked)
        treeWidget.connect('customContextMenuRequested(const QPoint&)', self._onShowContextMenu)

        self._eventFilter = PythonQt.dd.ddPythonEventFilter()
        self._eventFilter.addFilteredEventType(QtCore.QEvent.KeyPress)
        self._eventFilter.connect('handleEvent(QObject*, QEvent*)', self._filterEvent)
        treeWidget.installEventFilter(self._eventFilter)


#######################


_t = ObjectModelTree()

def getDefaultObjectModel():
    return _t

def getActiveItem():
    return _t.getActiveItem()

def getActiveObject():
    return _t.getActiveObject()

def setActiveObject(obj):
    _t.setActiveObject(obj)

def getObjects():
    return _t.getObjects()

def findObjectByName(name, parent=None):
    return _t.findObjectByName(name, parent)

def removeFromObjectModel(obj):
    _t.removeFromObjectModel(obj)

def addToObjectModel(obj, parentObj=None):
    _t.addToObjectModel(obj, parentObj)

def collapse(obj):
    _t.collapse(obj)

def expand(obj):
    _t.expand(obj)

def addContainer(name, parentObj=None):
    return _t.addContainer(name, parentObj)

def getOrCreateContainer(name, parentObj=None):
    return _t.getOrCreateContainer(name, parentObj)

def init(objectTree, propertiesPanel):
    _t.init(objectTree, propertiesPanel)
