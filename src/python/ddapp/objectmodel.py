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
            return PropertyAttributes(decimals=0, minimum=0, maximum=0, singleStep=0, hidden=False)

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


class ContainerItem(ObjectModelItem):

    def __init__(self, name):
        ObjectModelItem.__init__(self, name, Icons.Directory)


class RobotModelItem(ObjectModelItem):

    def __init__(self, model):

        modelName = os.path.basename(model.filename())
        ObjectModelItem.__init__(self, modelName, Icons.Robot)

        self.model = model
        model.connect('modelChanged()', self.onModelChanged)

        self.addProperty('Filename', model.filename())
        self.addProperty('Visible', model.visible())
        self.addProperty('Alpha', model.alpha())
        self.addProperty('Color', QtGui.QColor(255,0,0))
        self.views = []

    def _onPropertyChanged(self, propertyName):
        ObjectModelItem._onPropertyChanged(self, propertyName)

        if propertyName == 'Alpha':
            self.model.setAlpha(self.getProperty(propertyName))
        elif propertyName == 'Visible':
            self.model.setVisible(self.getProperty(propertyName))
        elif propertyName == 'Color':
            self.model.setColor(self.getProperty(propertyName))

    def onModelChanged(self):
        for view in self.views:
            view.render()

    def getLinkFrame(self, linkName):
        t = vtk.vtkTransform()
        if self.model.getLinkToWorld(linkName, t):
            return t
        else:
            return None

    def addToView(self, view):
        if view in self.views:
            return
        self.views.append(view)
        self.model.addToRenderer(view.renderer())
        view.render()

    def onRemoveFromObjectModel(self):
        for view in self.views:
            self.model.removeFromRenderer(view.renderer())
            view.render()


class PolyDataItem(ObjectModelItem):

    def __init__(self, name, polyData, view):

        ObjectModelItem.__init__(self, name, Icons.Robot)

        self.polyData = polyData
        self.view = view
        self.views = [view]
        self.mapper = vtk.vtkPolyDataMapper()
        self.mapper.SetInput(self.polyData)
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(self.mapper)
        self.view.renderer().AddActor(self.actor)

        self.addProperty('Visible', True)
        self.addProperty('Point Size', self.actor.GetProperty().GetPointSize())
        self.addProperty('Alpha', 1.0)
        self.addProperty('Color', QtGui.QColor(255,255,255))

        self._renderAllViews()


    def _renderAllViews(self):
        for view in self.views:
            view.render()

    def setPolyData(self, polyData):

        arrayName = self.getColorByArrayName()

        self.polyData = polyData
        self.mapper.SetInput(polyData)
        self.colorBy(arrayName, lut=self.mapper.GetLookupTable())
        self._renderAllViews()

    def getColorByArrayName(self):
        if self.polyData:
            scalars = self.polyData.GetPointData().GetScalars()
            if scalars:
                return scalars.GetName()

    def colorBy(self, arrayName, scalarRange=None, lut=None):

        if not arrayName:
            self.mapper.ScalarVisibilityOff()
            return

        array = self.polyData.GetPointData().GetArray(arrayName)
        if not array:
            print 'colorBy(%s): array not found' % arrayName
            self.mapper.ScalarVisibilityOff()
            return

        self.polyData.GetPointData().SetScalars(array)

        if scalarRange is None:
            scalarRange = array.GetRange()

        if not lut:
            lut = vtk.vtkLookupTable()
            lut.SetNumberOfColors(256)
            lut.SetHueRange(0.667, 0)
            lut.Build()

        self.mapper.SetLookupTable(lut)
        self.mapper.ScalarVisibilityOn()
        self.mapper.SetScalarRange(scalarRange)
        self.mapper.InterpolateScalarsBeforeMappingOff()
        #self.mapper.SetInputArrayToProcess(0,0,0, vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS, arrayName)


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
        for view in self.views:
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

    obj = getActiveObject()
    obj.setProperty(prop.propertyName(), prop.value())


def onTreeSelectionChanged():

    item = getActiveItem()
    if not item:
        return

    obj = getObjectForItem(item)

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

    selectedAction = menu.exec_(globalPos);
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

    initProperties()
    initObjectTree()
