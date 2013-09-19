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



quit = app.quit
exit = quit

app.startup(globals())

model = app.loadModelByName('model_minimal_contact.urdf')
modelConvexHull = app.loadModelByName('model.urdf')
modelConvexHull.setAlpha(0.0)


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



#s = ik.AsyncIKCommunicator(jc)
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


def onPropertyChanged(prop):
    if prop.propertyName() == 'alpha':
        modelConvexHull.setAlpha(prop.value())
        view.render()


def initProperties():

    p = app.getMainWindow().propertiesPanel()
    p.clear()

    alpha = p.addProperty('alpha', 0.0)
    alpha.setAttribute('decimals', 2)
    alpha.setAttribute('minimum', 0.0)
    alpha.setAttribute('maximum', 1.0)
    alpha.setAttribute('singleStep', 0.1)

    prop = p.addGroup("pose")
    s1 = p.addSubProperty("pose", 12.34, prop)
    s2 = p.addSubProperty("pose", 1.0, prop)
    s3 = p.addSubProperty("pose", 1.3, prop)

    s1.setAttribute('decimals', 4)
    s1.setAttribute('minimum', 0.0)
    s1.setAttribute('maximum', 50.0)


    p.connect('propertyValueChanged(QtVariantProperty*)', onPropertyChanged)


initSpreadsheetView()
initProperties()
