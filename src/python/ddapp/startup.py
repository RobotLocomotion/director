# This script is executed in the main console namespace so
# that all the variables defined here become console variables.


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

model = app.loadModelByName('model_minimal_contact.urdf')
modelConvexHull = app.loadModelByName('model.urdf')


jc = jointcontrol.JointController(model)
jc.addModel(modelConvexHull)
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
#s.start()
#s.startServerAsync()

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


def initObjectTree():

    style = QtGui.QApplication.style()

    dirIcon = style.standardIcon(QtGui.QStyle.SP_DirIcon)
    matlabIcon = QtGui.QIcon(':/images/matlab_logo.png')
    robotIcon = QtGui.QIcon(':/images/robot_icon.png')
    eyeIcon = QtGui.QIcon(':/images/eye_icon.png')
    eyeIconOff = QtGui.QIcon(':/images/eye_icon_gray.png')

    robotsItem = QtGui.QTreeWidgetItem(['robots'])
    robotsItem.setIcon(0, dirIcon)

    ikmodelItem = QtGui.QTreeWidgetItem(robotsItem, ['model.urdf (IK server)'])
    ikmodelItem.setIcon(0, matlabIcon)
    objects[ikmodelItem] = dict(data=s, properties=[])

    visualItem = QtGui.QTreeWidgetItem(robotsItem, ['model_minimal_contact.urdf (visual)'])
    visualItem.setIcon(0, robotIcon)
    visualItem.setIcon(1, eyeIcon)
    objects[visualItem] = dict(data=model, visible=model.visible(), properties=[newAlphaProperty(), newFileNameProperty()])

    visualItem2 = QtGui.QTreeWidgetItem(robotsItem, ['model.urdf (visual)'])
    visualItem2.setIcon(0, robotIcon)
    visualItem2.setIcon(1, eyeIcon)
    objects[visualItem2] = dict(data=modelConvexHull, visible=modelConvexHull.visible(), properties=[newAlphaProperty(), newFileNameProperty()])

    tree = app.getMainWindow().objectTree()
    tree.setColumnCount(2)
    tree.setHeaderLabels(['Name', ''])
    tree.headerItem().setIcon(1, eyeIcon)

    tree.header().setVisible(True)
    tree.header().setStretchLastSection(False)
    tree.header().setResizeMode(0, QtGui.QHeaderView.Stretch)
    tree.header().setResizeMode(1, QtGui.QHeaderView.Fixed)

    tree.addTopLevelItem(robotsItem)
    tree.expandItem(robotsItem)
    tree.setColumnWidth(1, 24)
    #tree.resizeColumnToContents(0)
    #tree.resizeColumnToContents(1)

    tree.connect('itemSelectionChanged()', onTreeSelectionChanged)
    tree.connect('itemClicked(QTreeWidgetItem*, int)', onItemClicked)

    items = [robotsItem, ikmodelItem, visualItem, visualItem2]
    return items


initSpreadsheetView()
initProperties()
items = initObjectTree()


tree = app.getMainWindow().objectTree()

onItemClicked(items[-1], 1)
