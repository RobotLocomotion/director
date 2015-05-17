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

    DESIRED_INTERIOR_DISTANCE = 0.08

    UPDATE_RATE = 5
    def __init__(self, robotSystem, view):

        self.robotStateModel = robotSystem.robotStateModel
        self.robotStateJointController = robotSystem.robotStateJointController
        self.robotSystem = robotSystem
        self.lFootFtFrameId = self.robotStateModel.model.findLinkID('l_foot')
        self.rFootFtFrameId = self.robotStateModel.model.findLinkID('r_foot')
        self.updateTimer = TimerCallback(self.UPDATE_RATE)
        self.updateTimer.callback = self.update
        self.leftInContact = 0
        self.rightInContact = 0
        self.draw = False
        self.view = view
        self.ddDrakeWrapper = PythonQt.dd.ddDrakeWrapper()

        self.warningButton = QtGui.QPushButton('COP Warning')
        self.warningButton.setStyleSheet("background-color:white")
        self.warningButton.connect('clicked()', self.toggleDrawing)

        app.getMainWindow().statusBar().insertPermanentWidget(0, self.warningButton)
        footContactSub = lcmUtils.addSubscriber('FOOT_CONTACT_ESTIMATE', lcmdrc.foot_contact_estimate_t, self.onFootContact)
        footContactSub.setSpeedLimit(self.UPDATE_RATE)

        self.updateTimer.start()

    def onFootContact(self, msg):
        self.leftInContact = msg.left_contact > 0.0
        self.rightInContact = msg.right_contact > 0.0

    def update(self):
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

                measured_cop = self.ddDrakeWrapper.resolveCenterOfPressure(self.robotStateModel.model, [self.lFootFtFrameId, self.rFootFtFrameId], 
                                lFootFt + rFootFt, [0., 0., 1.], (self.rightInContact*rFootOrigin+self.leftInContact*lFootOrigin)/(self.leftInContact + self.rightInContact))

                allFootContacts = np.empty([0, 2])
                if self.rightInContact:
                    rFootContacts = np.array([rFootTransform.TransformPoint(contact_point) for contact_point in self.LONG_FOOT_CONTACT_POINTS])
                    allFootContacts = np.concatenate((allFootContacts, rFootContacts[:, 0:2]))
                if self.leftInContact:
                    lFootContacts = np.array([lFootTransform.TransformPoint(contact_point) for contact_point in self.LONG_FOOT_CONTACT_POINTS])
                    allFootContacts = np.concatenate((allFootContacts, lFootContacts[:, 0:2]))

                num_pts = allFootContacts.shape[0]
                dist = self.ddDrakeWrapper.drakeSignedDistanceInsideConvexHull(num_pts, allFootContacts.reshape(num_pts*2, 1), measured_cop[0:2])

                inSafeSupportPolygon = dist >= 0
                print dist
                # map dist to color -- green if inside threshold, red if not
                dist = min(max(0, dist), self.DESIRED_INTERIOR_DISTANCE)
                r = int(255. - 255. * dist / self.DESIRED_INTERIOR_DISTANCE )
                g = int(255. * dist / self.DESIRED_INTERIOR_DISTANCE )
                b = 0
                colorStatus = [r/255., g/255., b/255.]
                colorStatusString = 'rgb(%d, %d, %d)' % (r, g, b)
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