# This script is executed in the main console namespace so
# that all the variables defined here become console variables.

import os
import sys
import vtk
from PythonQt import QtCore, QtGui
import ddapp.applogic as app
from ddapp import matlab
from ddapp import jointcontrol
from ddapp import cameracontrol
from ddapp import ik
import numpy as np

from collections import namedtuple


quit = app.quit
exit = quit

app.startup(globals())


models = []
for name in ['model_minimal_contact_fixedjoint_hands.urdf', 'model.urdf', 'model_minimal_contact.urdf', 'model_minimal_contact_point_hands.urdf']:
    models.append(app.loadModelByName(name))
    if name != 'model_minimal_contact_fixedjoint_hands.urdf':
        models[-1].setVisible(False)


jc = jointcontrol.JointController(models[0])
jc.models = models
jc.addNominalPoseFromFile(app.getNominalPoseMatFile())
jc.setNominalPose()

spreadsheet = app.getSpreadsheetView()
view = app.getDRCView()
camera = view.camera()

camera.SetFocalPoint([0,0,0])
camera.SetPosition([1, 0, 0])
camera.SetViewUp([0,0,1])
view.resetCamera()

orbit = cameracontrol.OrbitController(view)



s = ik.AsyncIKCommunicator(jc)
s.outputConsole = app.getOutputConsole()
s.start()
s.startServerAsync()

def _displaySnoptInfo(info):
    print 'info:', info
    _mainWindow.statusBar().showMessage('Info: %d' % info)
s.infoFunc = _displaySnoptInfo

r = jointcontrol.JointControlTestRamp(jc)


def initSpreadsheetView():

    sv = app.getSpreadsheetView()
    model = sv.model()

    rowCount = 50
    columnCount = 26
    for row in xrange(rowCount):
        sv.appendRow(['' for column in xrange(columnCount)])
        for column in xrange(columnCount):
            model.item(row, column).setEditable(True)


def getActiveItem():
    items = tree.selectedItems()
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

    p = app.getMainWindow().propertiesPanel()
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


objects = {}
ItemProperty = namedtuple('ItemProperty', ['name', 'decimals', 'minimum', 'maximum', 'singleStep'])


def onItemClicked(item, column):

  global objects
  obj = objects[item]

  eyeIcon = QtGui.QIcon(':/images/eye_icon.png')
  eyeIconOff = QtGui.QIcon(':/images/eye_icon_gray.png')

  if column == 1 and 'visible' in obj:
      obj['visible'] = not obj['visible']
      icon = eyeIcon if obj['visible'] else eyeIconOff
      item.setIcon(1, icon)
      obj['data'].setVisible(obj['visible'])


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
    p = app.getMainWindow().propertiesPanel()
    p.clear()
    p.connect('propertyValueChanged(QtVariantProperty*)', onPropertyChanged)


def newAlphaProperty():
    return ItemProperty(name='Alpha', decimals=2, minimum=0.0, maximum=1.0, singleStep=0.1)


def newFileNameProperty():
    return ItemProperty(name='Filename', decimals=0, minimum=0, maximum=0, singleStep=0)


def addModelToObjectTree(model, parentItem):

    robotIcon = QtGui.QIcon(':/images/robot_icon.png')
    eyeIcon = QtGui.QIcon(':/images/eye_icon.png')
    eyeIconOff = QtGui.QIcon(':/images/eye_icon_gray.png')

    modelName = os.path.basename(model.filename())
    item = QtGui.QTreeWidgetItem(parentItem, [modelName])
    item.setIcon(0, robotIcon)
    item.setIcon(1, eyeIcon if model.visible() else eyeIconOff)

    obj = dict(data=model, visible=model.visible(), properties=[newAlphaProperty(), newFileNameProperty()])
    objects[item] = obj
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

    style = QtGui.QApplication.style()
    dirIcon = style.standardIcon(QtGui.QStyle.SP_DirIcon)

    item = QtGui.QTreeWidgetItem([name])
    item.setIcon(0, dirIcon)

    tree = app.getMainWindow().objectTree()
    tree.addTopLevelItem(item)
    tree.expandItem(item)
    objects[item] = dict(data=None)

    return item



def initObjectTree():

    matlabIcon = QtGui.QIcon(':/images/matlab_logo.png')
    eyeIcon = QtGui.QIcon(':/images/eye_icon.png')

    tree = app.getMainWindow().objectTree()
    tree.setColumnCount(2)
    tree.setHeaderLabels(['Name', ''])
    tree.headerItem().setIcon(1, eyeIcon)
    tree.header().setVisible(True)
    tree.header().setStretchLastSection(False)
    tree.header().setResizeMode(0, QtGui.QHeaderView.Stretch)
    tree.header().setResizeMode(1, QtGui.QHeaderView.Fixed)
    tree.setColumnWidth(1, 24)
    tree.connect('itemSelectionChanged()', onTreeSelectionChanged)
    tree.connect('itemClicked(QTreeWidgetItem*, int)', onItemClicked)

    parentItem = addContainerToObjectTree('robots')
    addBasicItemToObjectTree('model.urdf (IK server)', matlabIcon, parentItem)
    addModelsToObjectTree(models, parentItem)




initSpreadsheetView()
initProperties()
initObjectTree()

tree = app.getMainWindow().objectTree()


parentItem = addContainerToObjectTree('affordances')
tableModel = view.loadURDFModel(os.path.join(app.getDRCBase(), 'software/drake/systems/plants/test/table.urdf'))
addModelToObjectTree(tableModel, parentItem)

