import os
import PythonQt
from PythonQt import QtCore, QtGui
from collections import namedtuple
from collections import OrderedDict

import vtk

_objectTree = None
_propertiesPanel = None

objects = {}
PropertyAttributes = namedtuple('PropertyAttributes', ['decimals', 'minimum', 'maximum', 'singleStep', 'hidden'])


class Icons(object):

  Directory = QtGui.QApplication.style().standardIcon(QtGui.QStyle.SP_DirIcon)
  Eye = QtGui.QIcon(':/images/eye_icon.png')
  EyeOff = QtGui.QIcon(':/images/eye_icon_gray.png')
  Matlab = QtGui.QIcon(':/images/matlab_logo.png')
  Robot = QtGui.QIcon(':/images/robot_icon.png')
  Laser = QtGui.QIcon(':/images/laser_icon.jpg')



class ObjectModelItem(object):

    def __init__(self, name, icon=Icons.Robot):
        self.properties = OrderedDict()
        self.icon = icon
        self.addProperty('Name', name)

    def propertyNames(self):
        return self.properties.keys()

    def hasProperty(self, propertyName):
        return propertyName in self.properties

    def getProperty(self, propertyName):
        assert self.hasProperty(propertyName)
        return self.properties[propertyName]

    def addProperty(self, propertyName, propertyValue):
        self.properties[propertyName] = propertyValue
        self._onPropertyAdded(propertyName)

    def setProperty(self, propertyName, propertyValue):
        assert self.hasProperty(propertyName)
        self.oldPropertyValue = (propertyName, self.getProperty(propertyName))
        self.properties[propertyName] = propertyValue
        self._onPropertyChanged(propertyName)
        self.oldPropertyValue = None

    def getPropertyAttributes(self, propertyName):

        if propertyName == 'Alpha':
            return PropertyAttributes(decimals=2, minimum=0.0, maximum=1.0, singleStep=0.1, hidden=False)
        elif propertyName == 'Name':
            return PropertyAttributes(decimals=0, minimum=0, maximum=0, singleStep=0, hidden=True)
        else:
            return PropertyAttributes(decimals=0, minimum=0, maximum=0, singleStep=0, hidden=False)

    def _onPropertyChanged(self, propertyName):
        updatePropertyPanel(self, propertyName)
        if propertyName == 'Visible':
            updateVisIcon(self)

    def _onPropertyAdded(self, propertyName):
        pass


class ContainerItem(ObjectModelItem):

    def __init__(self, name):
        ObjectModelItem.__init__(self, name, Icons.Directory)


class RobotModelItem(ObjectModelItem):

    def __init__(self, model):

        modelName = os.path.basename(model.filename())
        ObjectModelItem.__init__(self, modelName, Icons.Robot)

        self.model = model
        self.addProperty('Filename', model.filename())
        self.addProperty('Visible', model.visible())
        self.addProperty('Alpha', model.alpha())
        self.addProperty('Color', QtGui.QColor(255,0,0))

    def _onPropertyChanged(self, propertyName):
        ObjectModelItem._onPropertyChanged(self, propertyName)

        if propertyName == 'Alpha':
            self.model.setAlpha(self.getProperty(propertyName))
        elif propertyName == 'Visible':
            self.model.setVisible(self.getProperty(propertyName))


class PolyDataItem(ObjectModelItem):

    def __init__(self, name, polyData, view):

        ObjectModelItem.__init__(self, name, Icons.Robot)

        self.polyData = polyData
        self.view = view
        self.mapper = vtk.vtkPolyDataMapper()
        self.mapper.SetInputData(self.polyData)
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(self.mapper)
        self.view.renderer().AddActor(self.actor)
        self.addProperty('Visible', True)
        self.addProperty('Point Size', self.actor.GetProperty().GetPointSize())
        self.addProperty('Alpha', 1.0)
        self.addProperty('Color', QtGui.QColor(255,255,255))


    def setPolyData(self, polyData):
        self.polyData = polyData
        self.mapper.SetInputData(polyData)
        self.view.render()

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

        self.view.render()

    def getPropertyAttributes(self, propertyName):

        if propertyName == 'Point Size':
            return PropertyAttributes(decimals=0, minimum=1, maximum=20, singleStep=1, hidden=False)
        else:
            return ObjectModelItem.getPropertyAttributes(self, propertyName)


def getObjectTree():
    return _objectTree


def getPropertiesPanel():
    return _propertiesPanel


def getActiveItem():
    items = getObjectTree().selectedItems()
    return items[0] if len(items) == 1 else None


def getActiveObject():
    item = getActiveItem()
    global objects
    return objects[item] if item is not None else None


def getItemForObject(obj):
    global objects
    for item, itemObj in objects.iteritems():
        if itemObj == obj:
            return item


def getObjectForItem(item):
    return objects[item]


def findObjectByName(name):
    for obj in objects.values():
        if obj.getProperty('Name') == name:
            return obj


_blockSignals = False
def onPropertyChanged(prop):

    if _blockSignals:
        return

    if prop.isSubProperty():
        return

    item, obj = getActiveItem(), getActiveObject()
    obj.setProperty(prop.propertyName(), prop.value())


def onTreeSelectionChanged():

    item, obj = getActiveItem(), getActiveObject()

    global _blockSignals
    _blockSignals = True

    p = getPropertiesPanel()
    p.clear()

    for propertyName in obj.propertyNames():

        value = obj.getProperty(propertyName)
        attributes = obj.getPropertyAttributes(propertyName)
        if value is not None and not attributes.hidden:
            addProperty(p, propertyName, attributes, value)

    _blockSignals = False


def updateVisIcon(obj):
    item = getItemForObject(obj)
    if not obj.hasProperty('Visible'):
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


def addProperty(panel, name, attributes, value):

    if isinstance(value, list):
        groupProp = panel.addGroup(name)
        for v in value:
            p = panel.addSubProperty(name, v, groupProp)
            setPropertyAttributes(p, attributes)
        return groupProp
    else:
        p = panel.addProperty(name, value)
        setPropertyAttributes(p, attributes)
        return p


def initProperties():
    p = getPropertiesPanel()
    p.clear()
    p.connect('propertyValueChanged(QtVariantProperty*)', onPropertyChanged)


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


def addContainer(name):
    obj = ContainerItem(name)
    addToObjectModel(obj)
    return obj

def addRobotModel(model, parentObj):
    obj = RobotModelItem(model)
    addToObjectModel(obj, parentObj)
    return obj

def addPlaceholder(name, icon, parentObj):
    obj = ObjectModelItem(name, icon)
    addToObjectModel(obj, parentObj)
    return obj


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


'''
_eventFilter = None
def _onEvent(obj, event):


    if event.type() == QtCore.QEvent.MouseButtonDblClick:
        eventFilter.setEventHandlerResult(True)
        switchToSegmentationView()
    else:
        eventFilter.setEventHandlerResult(False)


def installEventFilter():

    global _eventFilter
    _eventFilter = PythonQt.dd.ddPythonEventFilter()

    qvtkwidget = view.vtkWidget()
    qvtkwidget.installEventFilter(eventFilter)
    eventFilters[qvtkwidget] = eventFilter

    eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonDblClick)
    eventFilter.connect('handleEvent(QObject*, QEvent*)', func)
'''

def init(objectTree, propertiesPanel):

    global _objectTree
    global _propertiesPanel
    _objectTree = objectTree
    _propertiesPanel = propertiesPanel

    initProperties()
    initObjectTree()
