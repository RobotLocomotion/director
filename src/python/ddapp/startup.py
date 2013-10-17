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
from ddapp import perception
from ddapp import segmentation
from ddapp.timercallback import TimerCallback

import numpy as np
from vtkPointCloudUtils import vtkNumpy
from vtkPointCloudUtils.debugVis import DebugData
from vtkPointCloudUtils import io


app.startup(globals())
om.init(app.getMainWindow().objectTree(), app.getMainWindow().propertiesPanel())

quit = app.quit
exit = quit
view = app.getDRCView()
camera = view.camera()
tree = app.getMainWindow().objectTree()
orbit = cameracontrol.OrbitController(view)
showPolyData = segmentation.showPolyData
updatePolyData = segmentation.updatePolyData


###############################################################################


useIk = True
usePerception = False
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


poseCollection = PythonQt.dd.ddSignalMap()
costCollection = PythonQt.dd.ddSignalMap()
spreadsheet.init(app.getSpreadsheetView(), poseCollection, costCollection)
tdx.init(view)


jc = jointcontrol.JointController(models, poseCollection)
jc.addNominalPoseFromFile(app.getNominalPoseMatFile())
jc.setNominalPose()
jc.addPose('q_end', jc.poses['q_nom'])
jc.addPose('q_start', jc.poses['q_nom'])
app.resetCamera(viewDirection=[-1,0,0])


if useIk:
    s = ik.AsyncIKCommunicator(jc)
    s.outputConsole = app.getOutputConsole()
    s.infoFunc = app.displaySnoptInfo
    s.start()
    s.startServerAsync()

    e = ikeditor.IKEditor(app.getMainWindow(), s, poseCollection, costCollection)
    e.makeFrameWidget(view)
    app.addWidgetToDock(e.widget)




robotsFolder = om.addContainer('robots')

om.addPlaceholder('model.urdf (IK server)', om.Icons.Matlab, robotsFolder)
for model in models:
    om.addRobotModel(model, robotsFolder)

if useTable:
    affordancesFolder = om.addContainerToObjectTree('affordances')
    om.addRobotModel(tableModel, affordancesFolder)

if usePerception:
    perception.init(view, models)
    segmentation.init()
