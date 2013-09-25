# This script is executed in the main console namespace so
# that all the variables defined here become console variables.

import os
import sys
import vtk
import PythonQt
from PythonQt import QtCore, QtGui
import ddapp.applogic as app
from ddapp import matlab
from ddapp import jointcontrol
from ddapp import cameracontrol
from ddapp import ik
from ddapp import ikeditor
import numpy as np

from ddapp.timercallback import TimerCallback

from collections import namedtuple


quit = app.quit
exit = quit

app.startup(globals())


models = []
for name in ['model_minimal_contact_fixedjoint_hands.urdf', 'model.urdf', 'model_minimal_contact.urdf', 'model_minimal_contact_point_hands.urdf']:
    models.append(app.loadModelByName(name))
    if name != 'model_minimal_contact_fixedjoint_hands.urdf':
        models[-1].setVisible(False)

poseCollection = PythonQt.dd.ddSignalMap()
costCollection = PythonQt.dd.ddSignalMap()


jc = jointcontrol.JointController(models, poseCollection)
jc.addNominalPoseFromFile(app.getNominalPoseMatFile())
jc.setNominalPose()
jc.addPose('q_end', jc.poses['q_nom'])
jc.addPose('q_start', jc.poses['q_nom'])


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
#s.start()
#s.startServerAsync()


def _displaySnoptInfo(info):
    _mainWindow.statusBar().showMessage('Info: %d' % info)

s.infoFunc = _displaySnoptInfo

r = jointcontrol.JointControlTestRamp(jc)


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



def setSpreadsheetColumnData(columnIndex, name, data):

    sv = app.getSpreadsheetView()
    model = sv.model()

    model.item(0, columnIndex).setText(name)
    for i, value in enumerate(data):
          model.item(i + 1, columnIndex).setText(value)

def updateSpreadsheetPoses():

    poseMap = poseCollection.map()
    for i, poseName in enumerate(sorted(poseMap.keys())):
        setSpreadsheetColumnData(i + 3, poseName, poseMap[poseName])

poseCollection.connect('itemChanged(const QString&)', updateSpreadsheetPoses)
poseCollection.connect('itemAdded(const QString&)', updateSpreadsheetPoses)
poseCollection.connect('itemRemoved(const QString&)', updateSpreadsheetPoses)

def updateSpreadsheet():

    jointNames = [
      'base_x',
      'base_y',
      'base_z',
      'base_roll',
      'base_pitch',
      'base_yaw',
      'back_bkz',
      'back_bky',
      'back_bkx',
      'l_arm_usy',
      'l_arm_shx',
      'l_arm_ely',
      'l_arm_elx',
      'l_arm_uwy',
      'l_leg_hpz',
      'l_leg_hpx',
      'l_leg_hpy',
      'l_leg_kny',
      'l_leg_aky',
      'l_leg_akx',
      'l_arm_mwx',
      'r_arm_usy',
      'r_arm_shx',
      'r_arm_ely',
      'r_arm_elx',
      'r_arm_uwy',
      'r_leg_hpz',
      'r_leg_hpx',
      'r_leg_hpy',
      'r_leg_kny',
      'r_leg_aky',
      'r_leg_akx',
      'r_arm_mwx',
      'neck_ay',
    ]

    costData = [
      0.0,
      0.0,
      100.0,
      100.0,
      0.0,
    ]

    costData += [100.0 for i in xrange(len(jointNames) - len(costData))]
    costCollection.setItem('default_costs', costData)

    setSpreadsheetColumnData(0, 'joint_names', jointNames)
    setSpreadsheetColumnData(1, 'default_costs', costData)


initProperties()
initObjectTree()
updateSpreadsheet()
updateSpreadsheetPoses()

tree = app.getMainWindow().objectTree()


parentItem = addContainerToObjectTree('affordances')
tableModel = view.loadURDFModel(os.path.join(app.getDRCBase(), 'software/drake/systems/plants/test/table.urdf'))
tableModel.setVisible(False)
addModelToObjectTree(tableModel, parentItem)


e = ikeditor.IKEditor(app.getMainWindow(), s, poseCollection, costCollection)
app.addWidgetToDock(e.widget)



def motionEvent(obj, eventId):
    e.handleTDxMotionEvent(view.lastTDxMotion())

tdxStyle = view.renderWindow().GetInteractor().GetInteractorStyle().GetTDxStyle()
if tdxStyle:
    tdxSettings = tdxStyle.GetSettings()

    translationSensitivity = 0.01
    angleSensitivity = 0.1
    tdxSettings.SetAngleSensitivity(angleSensitivity)
    tdxSettings.SetTranslationXSensitivity(translationSensitivity)
    tdxSettings.SetTranslationYSensitivity(translationSensitivity)
    tdxSettings.SetTranslationZSensitivity(translationSensitivity)

    tdxStyle.AddObserver('TDxMotionEvent', motionEvent)

view.resetCamera()



sys.path.append('/home/pat/source/paraview/PCLPlugin/build/lib')
import vtkDRCFiltersPython as drc



class MultiSenseSource(TimerCallback):

    def __init__(self):
        TimerCallback.__init__(self)
        self.reader = None
        self.lastScanLine = -1
        self.numberOfActors = 250
        self.nextActorId = 0
        self._createActors()

    def _createActors(self):

        self.polyData = []
        self.mappers = []
        self.actors = []

        for i in xrange(self.numberOfActors):
            polyData = vtk.vtkPolyData()
            mapper = vtk.vtkPolyDataMapper()
            actor = vtk.vtkActor()

            mapper.SetInputData(polyData)
            actor.SetMapper(mapper)
            view.renderer().AddActor(actor)

            self.polyData.append(polyData)
            self.mappers.append(mapper)
            self.actors.append(actor)



    def start(self):
        if self.reader is None:
            self.reader = drc.vtkMultisenseSource()
            self.reader.Start()

        TimerCallback.start(self)


    def tick(self):

        robotState = vtk.vtkDoubleArray()
        self.reader.GetCurrentRobotState(robotState)
        #print 'robot position:', robotState.GetValue(0), robotState.GetValue(1), robotState.GetValue(2)

        robotState = [robotState.GetValue(i) for i in xrange(robotState.GetNumberOfTuples())]
        if len(robotState) == 7:
            return

        models[0].setEstRobotState(robotState)

        currentScanLine = self.reader.GetCurrentScanLine()
        scanLinesToUpdate = currentScanLine - self.lastScanLine

        #print 'current scanline:', currentScanLine
        #print 'scan lines to update:', scanLinesToUpdate

        for i in xrange(scanLinesToUpdate):
            polyData = self.polyData[(self.nextActorId + i) % self.numberOfActors]
            self.reader.GetDataForScanLine(self.lastScanLine + i + 1, polyData)

        self.lastScanLine = currentScanLine
        self.nextActorId = (self.nextActorId + scanLinesToUpdate) % self.numberOfActors

        if scanLinesToUpdate:
            view.render()


m = MultiSenseSource()
m.start()
