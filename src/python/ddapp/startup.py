# This script is executed in the main console namespace so
# that all the variables defined here become console variables.

import ddapp

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
from ddapp.debugVis import DebugData
from ddapp import ioUtils as io


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
usePerception = True
useSpreadsheet = False




poseCollection = PythonQt.dd.ddSignalMap()
costCollection = PythonQt.dd.ddSignalMap()


if useSpreadsheet:
    spreadsheet.init(poseCollection, costCollection)


if useIk:


    ikview = app.getViewManager().createView('IK View')

    app.getViewManager().switchToView('IK View')

    ikFolder = om.addContainer('Drake IK')
    om.addPlaceholder('matlab server', om.Icons.Matlab, ikFolder)

    #modelsToLoad = ['model_minimal_contact_fixedjoint_hands.urdf', 'model.urdf',
    #                'model_minimal_contact.urdf', 'model_minimal_contact_point_hands.urdf']
    modelsToLoad = ['model_minimal_contact_fixedjoint_hands.urdf']
    models = []

    for name in modelsToLoad:
        model = ikview.loadURDFModel(os.path.join(app.getURDFModelDir(), name))
        om.addRobotModel(model, ikFolder)
        models.append(model)
        if name != 'model_minimal_contact_fixedjoint_hands.urdf':
            model.setVisible(False)


    useTable = False
    if useTable:
        tableModel = ikview.loadURDFModel(os.path.join(app.getDRCBase(), 'software/drake/systems/plants/test/table.urdf'))
        tableModel.setVisible(True)
        affordancesFolder = om.getOrCreateContainer('affordances')
        om.addRobotModel(tableModel, affordancesFolder)


    jc = jointcontrol.JointController(models, poseCollection)
    jc.addNominalPoseFromFile(app.getNominalPoseMatFile())
    jc.setNominalPose()
    jc.addPose('q_end', jc.poses['q_nom'])
    jc.addPose('q_start', jc.poses['q_nom'])

    s = ik.AsyncIKCommunicator(jc)
    s.outputConsole = app.getOutputConsole()
    s.infoFunc = app.displaySnoptInfo
    s.start()
    s.startServerAsync()

    e = ikeditor.IKEditor(app.getMainWindow(), s, poseCollection, costCollection)
    e.makeFrameWidget(ikview)
    app.addWidgetToDock(e.widget)
    tdx.init(ikview, e)

    app.resetCamera(viewDirection=[-1,0,0], view=ikview)


if usePerception:

    urdfFile = os.path.join(app.getURDFModelDir(), 'model_minimal_contact_fixedjoint_hands.urdf')
    robotStateModel = view.loadURDFModel(urdfFile)

    robotStateJointController = jointcontrol.JointController([robotStateModel])
    robotStateJointController.setZeroPose()

    perception.init(view, robotStateJointController)
    segmentation.init()

    sensorsFolder = om.getOrCreateContainer('sensors')
    obj = om.addRobotModel(robotStateModel, sensorsFolder)
    obj.setProperty('Name', 'ESTIMATED_ROBOT_STATE')

    def grabRobotState():
        poseName = 'ESTIMATED_ROBOT_STATE'
        robotStatePose = robotStateJointController.poses[poseName]
        s.sendPoseToServer(robotStatePose, poseName)
        s.forcePose(poseName)


app.resetCamera(viewDirection=[-1,0,0], view=view)
