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
from ddapp import objectmodel as om
from ddapp import spreadsheet
from ddapp import tdx
from ddapp.timercallback import TimerCallback

import numpy as np

app.startup(globals())
quit = app.quit
exit = quit
view = app.getDRCView()
camera = view.camera()
tree = app.getMainWindow().objectTree()
orbit = cameracontrol.OrbitController(view)


###############################################################################


useIk = False
useTable = False

#modelsToLoad = ['model_minimal_contact_fixedjoint_hands.urdf', 'model.urdf',
#                'model_minimal_contact.urdf', 'model_minimal_contact_point_hands.urdf']
modelsToLoad = ['model_minimal_contact_fixedjoint_hands.urdf']
models = []

for name in modelsToLoad:
    models.append(app.loadModelByName(name))
    if name != 'model_minimal_contact_fixedjoint_hands.urdf':
        models[-1].setVisible(False)

if useTable:
  tableModel = view.loadURDFModel(os.path.join(app.getDRCBase(), 'software/drake/systems/plants/test/table.urdf'))
  tableModel.setVisible(True)

app.resetCamera(viewDirection=[-1,0,0])
poseCollection = PythonQt.dd.ddSignalMap()
costCollection = PythonQt.dd.ddSignalMap()
spreadsheet.init(app.getSpreadsheetView(), poseCollection, costCollection)
tdx.init(view)


jc = jointcontrol.JointController(models, poseCollection)
jc.addNominalPoseFromFile(app.getNominalPoseMatFile())
jc.setNominalPose()
jc.addPose('q_end', jc.poses['q_nom'])
jc.addPose('q_start', jc.poses['q_nom'])


if useIk:
    s = ik.AsyncIKCommunicator(jc)
    s.outputConsole = app.getOutputConsole()
    s.infoFunc = app.displaySnoptInfo
    #s.start()
    #s.startServerAsync()

    e = ikeditor.IKEditor(app.getMainWindow(), s, poseCollection, costCollection)
    app.addWidgetToDock(e.widget)



om.init(app.getMainWindow().objectTree(), app.getMainWindow().propertiesPanel())
parentItem = om.addContainerToObjectTree('robots')
om.addBasicItemToObjectTree('model.urdf (IK server)', om.Icons.Matlab, parentItem)
om.addModelsToObjectTree(models, parentItem)

if useTable:
    parentItem = om.addContainerToObjectTree('affordances')
    om.addModelToObjectTree(tableModel, parentItem)



