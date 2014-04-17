import os
import re
import PythonQt
from PythonQt import QtCore, QtGui
from collections import namedtuple
from collections import OrderedDict

from ddapp.fieldcontainer import FieldContainer
import vtk

_objectTree = None
_propertiesPanel = None

objects = {}

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

def cleanPropertyName(s):
    """
    Generate a valid python property name by replacing all non-alphanumeric characters with underscores and adding an initial underscore if the first character is a digit
    """
    return re.sub(r'\W|^(?=\d)','_',s).lower()  # \W matches non-alphanumeric, ^(?=\d) matches the first position if followed by a digit


class ObjectModelItem(object):

    def __init__(self, name, icon=Icons.Robot):
        self.properties = OrderedDict()
        self.propertyAttributes = {}
        self.icon = icon
        self.alternateNames = {}
        self.addProperty('Name', name)

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


def getObjectTree():
    return _objectTree


def getPropertiesPanel():
    return _propertiesPanel


def getActiveItem():
    items = getObjectTree().selectedItems()
    return items[0] if len(items) == 1 else None


def getParentObject(obj):
    item = getItemForObject(obj)
    if item and item.parent():
        return getObjectForItem(item.parent())


def getObjectChildren(obj):

    item = getItemForObject(obj)
    if not item:
        return

    return [getObjectForItem(item.child(i)) for i in xrange(item.childCount())]


def getActiveObject():
    item = getActiveItem()
    return objects[item] if item is not None else None


def setActiveObject(obj):
    item = getItemForObject(obj)
    if item:
        tree = getObjectTree()
        #tree.clearSelection()
        #item.setSelected(True)
        tree.setCurrentItem(item)
        tree.scrollToItem(item)

def getItemForObject(obj):
    global objects
    for item, itemObj in objects.iteritems():
        if itemObj == obj:
            return item


def getObjectForItem(item):
    return objects[item]


def findObjectByName(name, parent=None):
    if parent:
        return findChildByName(parent, name)
    for obj in objects.values():
        if obj.getProperty('Name') == name:
            return obj


def findChildByName(parent, name):
    for child in getObjectChildren(parent):
        if child.getProperty('Name') == name:
            return child


_blockSignals = False
def onPropertyChanged(prop):

    if _blockSignals:
        return

    if prop.isSubProperty():
        return

    obj = getActiveObject()
    obj.setProperty(prop.propertyName(), prop.value())


def addPropertiesToPanel(obj, p):
    for propertyName in obj.propertyNames():
        value = obj.getProperty(propertyName)
        attributes = obj.getPropertyAttributes(propertyName)
        if value is not None and not attributes.hidden:
            addProperty(p, propertyName, attributes, value)


def onTreeSelectionChanged():

    item = getActiveItem()
    if not item:
        return

    obj = getObjectForItem(item)

    global _blockSignals
    _blockSignals = True

    p = getPropertiesPanel()
    p.clear()

    addPropertiesToPanel(obj, p)

    _blockSignals = False


def updateVisIcon(obj):
    item = getItemForObject(obj)
    if not item or not obj.hasProperty('Visible'):
        return

    isVisible = obj.getProperty('Visible')
    icon = Icons.Eye if isVisible else Icons.EyeOff
    item.setIcon(1, icon)


def updatePropertyPanel(obj, propertyName):
    if getActiveObject() != obj:
        return

    p = getPropertiesPanel()
    prop = p.findProperty(propertyName)
    if prop is None:
        return

    global _blockSignals
    _blockSignals = True
    prop.setValue(obj.getProperty(propertyName))
    _blockSignals = False


def onItemClicked(item, column):

    global objects
    obj = objects[item]

    if column == 1 and obj.hasProperty('Visible'):
        obj.setProperty('Visible', not obj.getProperty('Visible'))
        updateVisIcon(obj)


def setPropertyAttributes(p, attributes):
    p.setAttribute('decimals', attributes.decimals)
    p.setAttribute('minimum', attributes.minimum)
    p.setAttribute('maximum', attributes.maximum)
    p.setAttribute('singleStep', attributes.singleStep)
    if attributes.enumNames:
        p.setAttribute('enumNames', attributes.enumNames)


def addProperty(panel, name, attributes, value):

    if isinstance(value, list) and not isinstance(value[0], str):
        groupName = '%s [%s]' % (name, ', '.join([str(v) for v in value]))
        groupProp = panel.addGroup(groupName)
        for v in value:
            p = panel.addSubProperty(name, v, groupProp)
            setPropertyAttributes(p, attributes)
        return groupProp
    elif attributes.enumNames:
        p = panel.addEnumProperty(name, value)
        setPropertyAttributes(p, attributes)
        return p
    else:
        p = panel.addProperty(name, value)
        setPropertyAttributes(p, attributes)
        return p


def initProperties():
    p = getPropertiesPanel()
    p.clear()
    p.connect('propertyValueChanged(QtVariantProperty*)', onPropertyChanged)


def _removeItemFromObjectModel(item):

    while item.childCount():
        _removeItemFromObjectModel(item.child(0))

    try:
        obj = getObjectForItem(item)
    except KeyError:
        return

    obj.onRemoveFromObjectModel()
    if item.parent():
        item.parent().removeChild(item)
    else:
        tree = getObjectTree()
        tree.takeTopLevelItem(tree.indexOfTopLevelItem(item))

    del objects[item]


def removeFromObjectModel(obj):

    if obj is None:
        return

    item = getItemForObject(obj)
    if item:
        _removeItemFromObjectModel(item)


def addToObjectModel(obj, parentObj=None):

    parentItem = getItemForObject(parentObj)
    objName = obj.getProperty('Name')

    item = QtGui.QTreeWidgetItem(parentItem, [objName])
    item.setIcon(0, obj.icon)
    objects[item] = obj
    updateVisIcon(obj)

    if parentItem is None:
        tree = getObjectTree()
        tree.addTopLevelItem(item)
        tree.expandItem(item)

def collapse(obj):
    item = getItemForObject(obj)
    if item:
        getObjectTree().collapseItem(item)


def expand(obj):
    item = getItemForObject(obj)
    if item:
        getObjectTree().expandItem(item)


def addContainer(name, parentObj=None):
    obj = ContainerItem(name)
    addToObjectModel(obj, parentObj)
    return obj

def addRobotModel(model, parentObj):
    obj = RobotModelItem(model)
    addToObjectModel(obj, parentObj)
    return obj

def addPlaceholder(name, icon, parentObj):
    obj = ObjectModelItem(name, icon)
    addToObjectModel(obj, parentObj)
    return obj

def getOrCreateContainer(name, parentObj=None):

    containerObj = findObjectByName(name)
    if not containerObj:
        containerObj = addContainer(name, parentObj)
    return containerObj

def onShowContextMenu(clickPosition):

    obj = getActiveObject()
    if not obj:
        return

    globalPos = getObjectTree().viewport().mapToGlobal(clickPosition)

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
        removeFromObjectModel(obj)
    else:
        obj.onAction(selectedAction.text)


def onKeyPress(keyEvent):


    if keyEvent.key() == QtCore.Qt.Key_Delete:

        keyEvent.setAccepted(True)
        tree = getObjectTree()
        items = tree.selectedItems()
        for item in items:
            _removeItemFromObjectModel(item)


def initObjectTree():

    tree = getObjectTree()
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



def init(objectTree, propertiesPanel):

    global _objectTree
    global _propertiesPanel
    _objectTree = objectTree
    _propertiesPanel = propertiesPanel
    propertiesPanel.setBrowserModeToWidget()

    initProperties()
    initObjectTree()
