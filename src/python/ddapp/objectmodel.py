import os
import PythonQt
from PythonQt import QtCore, QtGui
from ddapp.propertyset import PropertySet, PropertyAttributes, PropertyPanelHelper

class Icons(object):

  Directory = QtGui.QApplication.style().standardIcon(QtGui.QStyle.SP_DirIcon)
  Eye = QtGui.QIcon(':/images/eye_icon.png')
  EyeOff = QtGui.QIcon(':/images/eye_icon_gray.png')
  Matlab = QtGui.QIcon(':/images/matlab_logo.png')
  Robot = QtGui.QIcon(':/images/robot_icon.png')
  Laser = QtGui.QIcon(':/images/laser_icon.jpg')
  Feet = QtGui.QIcon(':/images/feet.png')
  Hand = QtGui.QIcon(':/images/claw.png')


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
        return self.properties.getPropertyAttribute(propertyName, propertyAttribute)

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

        propertiesPanel = self.getPropertiesPanel()
        propertySet = self.getActiveObject().properties

        PropertyPanelHelper.setPropertyFromPanel(prop, propertiesPanel, propertySet)


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
