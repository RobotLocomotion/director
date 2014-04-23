import os
import re
import PythonQt
from PythonQt import QtCore, QtGui
from collections import namedtuple
from collections import OrderedDict

from ddapp.fieldcontainer import FieldContainer
import vtk

class PropertyAttributes(FieldContainer):

    def __init__(self, **kwargs):

        self._add_fields(
          decimals    = 5,
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


class ObjectModelTree(object):

    def getObjectTree():
        return self


def getActiveItem():
    items = getObjectTree().selectedItems()
    return items[0] if len(items) == 1 else None


class ObjectModelItem(object):

    def __init__(self, name, icon=Icons.Robot):
        self.properties = OrderedDict()
        self.propertyAttributes = {}
        self.icon = icon
        self.alternateNames = {}
        self.addProperty('Name', name)

    def setIcon(self, icon):
        self.icon = icon
        item = getItemForObject(self)
        if item:
            item.setIcon(0, icon)

    def propertyNames(self):
        return self.properties.keys()

    def hasProperty(self, propertyName):
        return propertyName in self.properties

    def getProperty(self, propertyName):
        assert self.hasProperty(propertyName)
        return self.properties[propertyName]

    def addProperty(self, propertyName, propertyValue, attributes=None):
        alternateName = cleanPropertyName(propertyName)
        if propertyName not in self.properties and alternateName in self.alternateNames:
            raise ValueError('Adding this property would conflict with a different existing property with alternate name {:s}'.format(alternateName))
        self.alternateNames[alternateName] = propertyName
        self.properties[propertyName] = propertyValue
        if attributes is not None:
            self.propertyAttributes[propertyName] = attributes
        self._onPropertyAdded(propertyName)

    def setProperty(self, propertyName, propertyValue):
        assert self.hasProperty(propertyName)

        attributes = self.getPropertyAttributes(propertyName)
        if attributes.enumNames and type(propertyValue) != int:
            propertyValue = attributes.enumNames.index(propertyValue)

        self.oldPropertyValue = (propertyName, self.getProperty(propertyName))
        self.properties[propertyName] = propertyValue
        self._onPropertyChanged(propertyName)
        self.oldPropertyValue = None

    def hasDataSet(self, dataSet):
        return False

    def getActionNames(self):
        return []

    def onAction(self, action):
        pass

    def getPropertyAttributes(self, propertyName):

        if propertyName == 'Alpha':
            return PropertyAttributes(decimals=2, minimum=0.0, maximum=1.0, singleStep=0.1, hidden=False)
        elif propertyName == 'Name':
            return PropertyAttributes(decimals=0, minimum=0, maximum=0, singleStep=0, hidden=True)
        else:
            attributes = PropertyAttributes(decimals=0, minimum=0, maximum=0, singleStep=0, hidden=False)
            return self.propertyAttributes.setdefault(propertyName, attributes)

    def _onPropertyChanged(self, propertyName):
        updatePropertyPanel(self, propertyName)
        if propertyName == 'Visible':
            updateVisIcon(self)
        if propertyName == 'Name':
            getItemForObject(self).setText(0, self.getProperty('Name'))

    def _onPropertyAdded(self, propertyName):
        pass

    def onRemoveFromObjectModel(self):
        pass

    def children(self):
        return getObjectChildren(self)

    def findChild(self, name):
        return findChildByName(self, name)

    def __getattribute__(self, name):
        try:
            alternateNames = object.__getattribute__(self, 'alternateNames')
            if name in alternateNames:
                return object.__getattribute__(self, 'getProperty')(self.alternateNames[name])
            else:
                raise AttributeError()
        except AttributeError:
            return object.__getattribute__(self, name)


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
        self.addProperty('Alpha', model.alpha())
        self.addProperty('Color', model.color())
        self.views = []

    def _onPropertyChanged(self, propertyName):
        ObjectModelItem._onPropertyChanged(self, propertyName)

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
        self.addProperty('Point Size', self.actor.GetProperty().GetPointSize())
        self.addProperty('Alpha', 1.0)
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

    def _onPropertyChanged(self, propertyName):
        ObjectModelItem._onPropertyChanged(self, propertyName)

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

    def getPropertyAttributes(self, propertyName):

        if propertyName == 'Point Size':
            return PropertyAttributes(decimals=0, minimum=1, maximum=20, singleStep=1, hidden=False)
        else:
            return ObjectModelItem.getPropertyAttributes(self, propertyName)

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
        self._objectTree = None
        self._propertiesPanel = None
        self.objects = {}
        self._blockSignals = False

    def getObjectTree(self):
        return self._objectTree

    def getPropertiesPanel(self):
        return self._propertiesPanel

    def getActiveItem(self):
        items = self.getObjectTree().selectedItems()
        return items[0] if len(items) == 1 else None

    def getParentObject(self, obj):
        item = self.getItemForObject(obj)
        if item and item.parent():
            return self.getObjectForItem(item.parent())

    def getObjectChildren(self, obj):
        item = self.getItemForObject(obj)
        if not item:
            return

        return [self.getObjectForItem(item.child(i)) for i in xrange(item.childCount())]

    def getActiveObject(self):
        item = self.getActiveItem()
        return self.objects[item] if item is not None else None

    def setActiveObject(self, obj):
        item = self.getItemForObject(obj)
        if item:
            tree = self.getObjectTree()
            #tree.clearSelection()
            #item.setSelected(True)
            tree.setCurrentItem(item)
            tree.scrollToItem(item)

    def getItemForObject(self, obj):
        for item, itemObj in self.objects.iteritems():
            if itemObj == obj:
                return item

    def getObjectForItem(self, item):
        return self.objects[item]

    def findObjectByName(self, name, parent=None):
        if parent:
            return self.findChildByName(parent, name)
        for obj in self.objects.values():
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

    def addPropertiesToPanel(self, obj, p):
        for propertyName in obj.propertyNames():
            value = obj.getProperty(propertyName)
            attributes = obj.getPropertyAttributes(propertyName)
            if value is not None and not attributes.hidden:
                self.addProperty(p, propertyName, attributes, value)

    def onTreeSelectionChanged(self):

        item = self.getActiveItem()
        if not item:
            return

        obj = self.getObjectForItem(item)

        self._blockSignals = True

        p = self.getPropertiesPanel()
        p.clear()

        self.addPropertiesToPanel(obj, p)

        self._blockSignals = False

    def updateVisIcon(self, obj):

        item = self.getItemForObject(obj)
        if not item or not obj.hasProperty('Visible'):
            return

        isVisible = obj.getProperty('Visible')
        icon = Icons.Eye if isVisible else Icons.EyeOff
        item.setIcon(1, icon)

    def updatePropertyPanel(self, obj, propertyName):

        if self.getActiveObject() != obj:
            return

        p = self.getPropertiesPanel()
        prop = p.findProperty(propertyName)
        if prop is None:
            return

        self._blockSignals = True
        prop.setValue(obj.getProperty(propertyName))
        self._blockSignals = False

    def onItemClicked(self, item, column):

        obj = self.objects[item]

        if column == 1 and obj.hasProperty('Visible'):
            obj.setProperty('Visible', not obj.getProperty('Visible'))
            self.updateVisIcon(obj)

    def setPropertyAttributes(self, p, attributes):

        p.setAttribute('decimals', attributes.decimals)
        p.setAttribute('minimum', attributes.minimum)
        p.setAttribute('maximum', attributes.maximum)
        p.setAttribute('singleStep', attributes.singleStep)
        if attributes.enumNames:
            p.setAttribute('enumNames', attributes.enumNames)


    def addProperty(self, panel, name, attributes, value):

        if isinstance(value, list) and not isinstance(value[0], str):
            groupName = '%s [%s]' % (name, ', '.join([str(v) for v in value]))
            groupProp = panel.addGroup(groupName)
            for v in value:
                p = panel.addSubProperty(name, v, groupProp)
                self.setPropertyAttributes(p, attributes)
            return groupProp
        elif attributes.enumNames:
            p = panel.addEnumProperty(name, value)
            self.setPropertyAttributes(p, attributes)
            return p
        else:
            p = panel.addProperty(name, value)
            self.setPropertyAttributes(p, attributes)
            return p


    def initProperties(self):
        p = self.getPropertiesPanel()
        p.clear()
        p.connect('propertyValueChanged(QtVariantProperty*)', self.onPropertyChanged)


    def _removeItemFromObjectModel(self, item):
        while item.childCount():
            self._removeItemFromObjectModel(item.child(0))

        try:
            obj = self.getObjectForItem(item)
        except KeyError:
            return

        obj.onRemoveFromObjectModel()
        if item.parent():
            item.parent().removeChild(item)
        else:
            tree = self.getObjectTree()
            tree.takeTopLevelItem(tree.indexOfTopLevelItem(item))

        del self.objects[item]


    def removeFromObjectModel(self, obj):
        if obj is None:
            return

        item = self.getItemForObject(obj)
        if item:
            self._removeItemFromObjectModel(item)


    def addToObjectModel(self, obj, parentObj=None):
        parentItem = self.getItemForObject(parentObj)
        objName = obj.getProperty('Name')

        item = QtGui.QTreeWidgetItem(parentItem, [objName])
        item.setIcon(0, obj.icon)
        self.objects[item] = obj
        self.updateVisIcon(obj)

        if parentItem is None:
            tree = self.getObjectTree()
            tree.addTopLevelItem(item)
            tree.expandItem(item)


    def collapse(self, obj):
        item = self.getItemForObject(obj)
        if item:
            self.getObjectTree().collapseItem(item)


    def expand(self, obj):
        item = self.getItemForObject(obj)
        if item:
            self.getObjectTree().expandItem(item)


    def addContainer(self, name, parentObj=None):
        obj = ContainerItem(name)
        self.addToObjectModel(obj, parentObj)
        return obj


    def addRobotModel(self, model, parentObj):
        obj = RobotModelItem(model)
        self.addToObjectModel(obj, parentObj)
        return obj


    def addPlaceholder(self, name, icon, parentObj):
        obj = ObjectModelItem(name, icon)
        self.addToObjectModel(obj, parentObj)
        return obj


    def getOrCreateContainer(self, name, parentObj=None):
        containerObj = self.findObjectByName(name)
        if not containerObj:
            containerObj = self.addContainer(name, parentObj)
        return containerObj


    def onShowContextMenu(self, clickPosition):

        obj = self.getActiveObject()
        if not obj:
            return

        globalPos = self.getObjectTree().viewport().mapToGlobal(clickPosition)

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


    def onKeyPress(self, keyEvent):

        if keyEvent.key() == QtCore.Qt.Key_Delete:

            keyEvent.setAccepted(True)
            tree = self.getObjectTree()
            items = tree.selectedItems()
            for item in items:
                self._removeItemFromObjectModel(item)


    def initObjectTree(self):

        tree = self.getObjectTree()
        tree.setColumnCount(2)
        tree.setHeaderLabels(['Name', ''])
        tree.headerItem().setIcon(1, Icons.Eye)
        tree.header().setVisible(True)
        tree.header().setStretchLastSection(False)
        tree.header().setResizeMode(0, QtGui.QHeaderView.Stretch)
        tree.header().setResizeMode(1, QtGui.QHeaderView.Fixed)
        tree.setColumnWidth(1, 24)
        tree.connect('itemSelectionChanged()', onTreeSelectionChanged)
        tree.connect('itemClicked(QTreeWidgetItem*, int)', onItemClicked)
        tree.connect('customContextMenuRequested(const QPoint&)', onShowContextMenu)
        tree.connect('keyPressSignal(QKeyEvent*)', onKeyPress)


    def init(self, objectTree, propertiesPanel):

        self._objectTree = objectTree
        self._propertiesPanel = propertiesPanel
        propertiesPanel.setBrowserModeToWidget()

        self.initProperties()
        self.initObjectTree()


#######################


_t = ObjectModelTree()

def getObjectTree(self):
    return _t.getObjectTree()

def getPropertiesPanel():
    return _t.getPropertiesPanel()

def getActiveItem():
    return _t.getActiveItem()

def getParentObject(obj):
    return _t.getParentObject(obj)

def getObjectChildren(obj):
    return _t.getObjectChildren(obj)

def getActiveObject():
    return _t.getActiveObject()

def setActiveObject(obj):
    _t.setActiveObject(obj)

def getItemForObject(obj):
    return _t.getItemForObject(obj)

def getObjectForItem(item):
    return _t.getObjectForItem(item)

def findObjectByName(name, parent=None):
    return _t.findObjectByName(name, parent)

def findChildByName(parent, name):
    return _t.findChildByName(parent, name)

def onPropertyChanged(prop):
    return _t.onPropertyChanged(prop)

def addPropertiesToPanel(obj, p):
    _t.addPropertiesToPanel(obj, p)

def onTreeSelectionChanged():
    _t.onTreeSelectionChanged()

def updateVisIcon(obj):
    _t.updateVisIcon(obj)

def updatePropertyPanel(obj, propertyName):
    _t.updatePropertyPanel(obj, propertyName)

def onItemClicked(item, column):
    _t.onItemClicked(item, column)

def setPropertyAttributes(p, attributes):
    _t.setPropertyAttributes(p, attributes)

def addProperty(panel, name, attributes, value):
    return _t.addProperty(panel, name, attributes, value)

def initProperties():
    _t.initProperties()

def _removeItemFromObjectModel(item):
    _t._removeItemFromObjectModel(item)

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

def addRobotModel(model, parentObj):
    return _t.addRobotModel(model, parentObj)

def addPlaceholder(name, icon, parentObj):
    return _t.addPlaceholder(name, icon, parentObj)

def getOrCreateContainer(name, parentObj=None):
    return _t.getOrCreateContainer(name, parentObj)

def onShowContextMenu(clickPosition):
    _t.onShowContextMenu(clickPosition)

def onKeyPress(keyEvent):
    _t.onKeyPress(keyEvent)

def initObjectTree():
    _t.initObjectTree()

def init(objectTree, propertiesPanel):
    _t.init(objectTree, propertiesPanel)

