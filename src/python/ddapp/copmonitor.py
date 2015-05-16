import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
import ddapp.objectmodel as om
from ddapp import lcmUtils
from ddapp import applogic as app
from ddapp.utime import getUtime
from ddapp.timercallback import TimerCallback
from ddapp import visualization as vis
from ddapp.debugVis import DebugData
from scipy.spatial import Delaunay
import drc as lcmdrc
import numpy as np
import math
from time import time
from time import sleep

class COPMonitor(object):
    LONG_FOOT_CONTACT_POINTS = [[-0.13, 0.0562, 0.0],
                                [-0.13, -0.0562, 0.0], 
                                [0.17, 0.0562, 0.0],
                                [0.17, -0.0562, 0.0]]
    SHRINK_FACTORS = [0.4, 0.7, 0.9] # should be ascending order from most strict to least strict
    COLORS = [[0.0, 1.0, 0.0], [1.0, 1.0, 0.0], [1.0, 0.6, 0.0]] # defaults to red if not in any of these shrinks
    COLORS_BUTTON = ['white', 'yellow', 'orange']

    def __init__(self, robotSystem, view):

        self.robotStateModel = robotSystem.robotStateModel
        self.robotStateJointController = robotSystem.robotStateJointController
        self.robotSystem = robotSystem
        self.lFootFtFrameId = self.robotStateModel.model.findLinkID('l_foot')
        self.rFootFtFrameId = self.robotStateModel.model.findLinkID('r_foot')
        self.robotStateModel.connectModelChanged(self.update)
        self.leftInContact = 0
        self.rightInContact = 0
        self.draw = False
        self.view = view

        self.warningButton = QtGui.QPushButton('COP Warning')
        self.warningButton.setStyleSheet("background-color:white")
        self.warningButton.connect('clicked()', self.toggleDrawing)

        app.getMainWindow().statusBar().insertPermanentWidget(0, self.warningButton)
        footContactSub = lcmUtils.addSubscriber('FOOT_CONTACT_ESTIMATE', lcmdrc.foot_contact_estimate_t, self.onFootContact)
        footContactSub.setSpeedLimit(5)

    def onFootContact(self, msg):
        self.leftInContact = msg.left_contact > 0.0
        self.rightInContact = msg.right_contact > 0.0

    def update(self, newRobotState):
        if (hasattr(self.robotStateJointController, 'lastRobotStateMessage') and 
            self.robotStateJointController.lastRobotStateMessage):

            if (self.rightInContact or self.leftInContact):

                lFootFt =  [self.robotStateJointController.lastRobotStateMessage.force_torque.l_foot_torque_x, 
                             self.robotStateJointController.lastRobotStateMessage.force_torque.l_foot_torque_y, 
                             0.0,
                             0.0, 
                             0.0, 
                             self.robotStateJointController.lastRobotStateMessage.force_torque.l_foot_force_z]
                rFootFt = [self.robotStateJointController.lastRobotStateMessage.force_torque.r_foot_torque_x, 
                             self.robotStateJointController.lastRobotStateMessage.force_torque.r_foot_torque_y, 
                             0.0,
                             0.0, 
                             0.0, 
                             self.robotStateJointController.lastRobotStateMessage.force_torque.r_foot_force_z]
                rFootTransform = self.robotStateModel.getLinkFrame('r_foot')
                lFootTransform = self.robotStateModel.getLinkFrame('l_foot')

                rFootOrigin = np.array(rFootTransform.TransformPoint([0, 0, -0.07645]))
                lFootOrigin = np.array(lFootTransform.TransformPoint([0, 0, -0.07645])) # down to sole

                measured_cop = self.robotStateModel.model.resolveCenterOfPressure([self.lFootFtFrameId, self.rFootFtFrameId], 
                                lFootFt + rFootFt, [0., 0., 1.], (self.rightInContact*rFootOrigin+self.leftInContact*lFootOrigin)/(self.leftInContact + self.rightInContact))

                inSafeSupportPolygon = False
                for shrink_factor_i in range(len(self.SHRINK_FACTORS)):
                    # alternatively: can we just do distance to hull?
                    shrink_factor = self.SHRINK_FACTORS[shrink_factor_i]
                    allFootContacts = np.empty([0, 2])
                    if self.rightInContact:
                        rFootContacts = np.array([rFootTransform.TransformPoint([i*shrink_factor for i in contact_point]) for contact_point in self.LONG_FOOT_CONTACT_POINTS])
                        allFootContacts = np.concatenate((allFootContacts, rFootContacts[:, 0:2]))
                    if self.leftInContact:
                        lFootContacts = np.array([lFootTransform.TransformPoint([i*shrink_factor for i in contact_point]) for contact_point in self.LONG_FOOT_CONTACT_POINTS])
                        allFootContacts = np.concatenate((allFootContacts, lFootContacts[:, 0:2]))
                    inSafeSupportPolygon = Delaunay(allFootContacts).find_simplex(measured_cop[0:2])>=0

                    if (inSafeSupportPolygon):
                        colorStatus = self.COLORS[shrink_factor_i]
                        colorStatusString = self.COLORS_BUTTON[shrink_factor_i]
                        break
                    # otherwise check next biggest shrink region

                if (not inSafeSupportPolygon):
                    colorStatus = [1.0, 0.0, 0.0]
                    colorStatusString = 'red'

                self.warningButton.setStyleSheet("background-color:"+colorStatusString)

                if (self.draw):
                    d = DebugData()
                    d.addSphere(measured_cop[0:3], radius=0.02)
                    vis.updatePolyData(d.getPolyData(), 'measured cop', view=self.view, parent='COP Monitor')
                    om.findObjectByName('measured cop').setProperty('Color', colorStatus)
                
    def toggleDrawing(self):
        self.draw = not self.draw
        if (not self.draw):
            om.removeFromObjectModel(om.findObjectByName('COP Monitor'))