# This script is executed in the main console namespace so
# that all the variables defined here become console variables.


import vtk
from PythonQt import QtCore, QtGui
import ddapp.applogic as app
from ddapp import matlab
from ddapp import jointcontrol
from ddapp import cameracontrol

import numpy as np



quit = app.quit
exit = quit

app.startup(globals())
model = app.loadTestModel()
jc = jointcontrol.JointController(model)
jc.addNominalPoseFromFile(app.getNominalPoseMatFile())
jc.setNominalPose()

view = app.getDRCView()
camera = view.camera()

camera.SetFocalPoint([0,0,0])
camera.SetPosition([1, 0, 0])
camera.SetViewUp([0,0,1])
view.resetCamera()

orbit = cameracontrol.OrbitController(view)



def matlabTest():

    p = matlab.startIKServer()
    q_start = matlab.getFloatArray(p, 'q_start')
    jc.addPose('qstart', q_start)
    jc.setPose('qstart')
    return p


def footTest(p):

    for dist in np.linspace(0, 0.5, 50):

        print 'solve', dist

        #matlab.send(p, 'l_foot_target = vertcat(l_foot_target_start(1:2,:), l_foot_target_start(3,:)+%s);' % height)
        matlab.send(p, 'l_foot_target = vertcat(l_foot_target_start(1,:), l_foot_target_start(2,:)+%s, l_foot_target_start(3,:));' % dist)
        matlab.readForPrompt(p)

        matlab.send(p, 'kc2l = WorldPositionConstraint(r, l_foot, l_foot_pts, l_foot_target, l_foot_target, tspan);')
        matlab.readForPrompt(p)

        matlab.send(p, '[q_start, info] = inverseKin(r, q_seed, q_nom, qsc, kc2l, kc2r, kc3, kc4, s.ikoptions)')
        matlab.readForPrompt(p)

        q_start = matlab.getFloatArray(p, 'q_start')

        jc.addPose('q_start', q_start)
        jc.setPose('q_start')

        QtGui.QApplication.instance().processEvents()


p = matlabTest()


