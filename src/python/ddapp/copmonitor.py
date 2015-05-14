import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
import ddapp.objectmodel as om
from ddapp import lcmUtils
from ddapp import applogic as app
from ddapp.utime import getUtime
from ddapp.timercallback import TimerCallback
from ddapp import visualization as vis
from ddapp.debugVis import DebugData


import numpy as np
import math
from time import time
from time import sleep

class COPMonitor(object):

    def __init__(self, robotStateModel, robotStateJointController, view):

        self.robotStateModel = robotStateModel
        self.robotStateJointController = robotStateJointController
        self.l_foot_ft_frame_id = robotStateModel.model.findLinkID('l_foot')
        self.r_foot_ft_frame_id = robotStateModel.model.findLinkID('r_foot')
        self.robotStateModel.connectModelChanged(self.update)
        self.view = view

    def update(self, newRobotState):
        if (hasattr(self.robotStateJointController, 'lastRobotStateMessage') and 
            self.robotStateJointController.lastRobotStateMessage):
            lfoot_ft =  [self.robotStateJointController.lastRobotStateMessage.force_torque.l_foot_torque_x, 
                         self.robotStateJointController.lastRobotStateMessage.force_torque.l_foot_torque_y, 
                         0.0,
                         0.0, 
                         0.0, 
                         self.robotStateJointController.lastRobotStateMessage.force_torque.l_foot_force_z]
            rfoot_ft = [self.robotStateJointController.lastRobotStateMessage.force_torque.r_foot_torque_x, 
                         self.robotStateJointController.lastRobotStateMessage.force_torque.r_foot_torque_y, 
                         0.0,
                         0.0, 
                         0.0, 
                         self.robotStateJointController.lastRobotStateMessage.force_torque.r_foot_force_z]
            measured_cop = self.robotStateModel.model.resolveCenterOfPressure([self.l_foot_ft_frame_id, self.r_foot_ft_frame_id], 
                lfoot_ft + rfoot_ft, [0., 0., 1.], [0., 0., 0.])
            
            d = DebugData()
            d.addSphere(measured_cop[0:3], radius=0.05, color=[1, 0.6, 0])
            vis.updatePolyData(d.getPolyData(), 'measured cop', view=self.view, parent='copmonitor')
            