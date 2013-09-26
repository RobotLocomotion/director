import os
from PythonQt import QtCore, QtGui
from collections import namedtuple

_objectTree = None
_propertiesPanel = None

objects = {}
ItemProperty = namedtuple('ItemProperty', ['name', 'decimals', 'minimum', 'maximum', 'singleStep'])


class Icons(object):

  Directory = QtGui.QApplication.style().standardIcon(QtGui.QStyle.SP_DirIcon)
  Eye = QtGui.QIcon(':/images/eye_icon.png')
  EyeOff = QtGui.QIcon(':/images/eye_icon_gray.png')
  Matlab = QtGui.QIcon(':/images/matlab_logo.png')
  Robot = QtGui.QIcon(':/images/robot_icon.png')


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


def onPropertyChanged(prop):

    item, obj = getActiveItem(), getActiveObject()

    if prop.propertyName() == 'Alpha':
        obj['data'].setAlpha(prop.value())


def onTreeSelectionChanged():

    p = getPropertiesPanel()
    p.clear()

    item, obj = getActiveItem(), getActiveObject()

    if 'properties' not in obj:
        return

    for prop in obj['properties']:

        value = None
        if prop.name == 'Filename':
            value = obj['data'].filename()

        if prop.name == 'Alpha':
            value = obj['data'].alpha()

        if value is not None:
            addProperty(p, prop, value)



def updateVisItem(item):
    obj = objects[item]
    isVisible = obj['visible']
    icon = Icons.Eye if isVisible else Icons.EyeOff
    item.setIcon(1, icon)


def onItemClicked(item, column):

  global objects
  obj = objects[item]

  if column == 1 and 'visible' in obj:
      obj['visible'] = not obj['visible']
      obj['data'].setVisible(obj['visible'])
      updateVisItem(item)


def setPropertyAttributes(p, attributes):
    p.setAttribute('decimals', attributes.decimals)
    p.setAttribute('minimum', attributes.minimum)
    p.setAttribute('maximum', attributes.maximum)
    p.setAttribute('singleStep', attributes.singleStep)


def addProperty(panel, prop, value):

    if isinstance(value, list):
        groupProp = panel.addGroup(prop.name)
        for v in value:
            p = panel.addSubProperty(prop.name, v, groupProp)
            setPropertyAttributes(p, prop)
        return groupProp
    else:
        p = panel.addProperty(prop.name, value)
        setPropertyAttributes(p, prop)
        return p


def initProperties():
    p = getPropertiesPanel()
    p.clear()
    p.connect('propertyValueChanged(QtVariantProperty*)', onPropertyChanged)


def newAlphaProperty():
    return ItemProperty(name='Alpha', decimals=2, minimum=0.0, maximum=1.0, singleStep=0.1)


def newFileNameProperty():
    return ItemProperty(name='Filename', decimals=0, minimum=0, maximum=0, singleStep=0)


def addModelToObjectTree(model, parentItem):


    modelName = os.path.basename(model.filename())
    item = QtGui.QTreeWidgetItem(parentItem, [modelName])
    item.setIcon(0, Icons.Robot)

    obj = dict(data=model, visible=model.visible(), properties=[newAlphaProperty(), newFileNameProperty()])
    objects[item] = obj

    updateVisIcon(item)
    return item


def addModelsToObjectTree(models, parentItem):
    return [addModelToObjectTree(model, parentItem) for model in models]


def addBasicItemToObjectTree(name, icon, parentItem):
    item = QtGui.QTreeWidgetItem(parentItem, [name])
    item.setIcon(0, icon)
    objects[item] = dict(data=None)
    return item


def findItemByName(name):
    for item in objects.keys():
        if item.text(0) == name:
            return item


def addContainerToObjectTree(name):

    item = QtGui.QTreeWidgetItem([name])
    item.setIcon(0, Icons.Directory)
    objects[item] = dict(data=None)

    tree = getObjectTree()
    tree.addTopLevelItem(item)
    tree.expandItem(item)

    return item


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


def init(objectTree, propertiesPanel):

    global _objectTree
    global _propertiesPanel
    _objectTree = objectTree
    _propertiesPanel = propertiesPanel

    initProperties()
    initObjectTree()
