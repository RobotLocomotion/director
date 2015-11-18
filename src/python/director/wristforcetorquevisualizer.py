import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
import director.objectmodel as om
from director import lcmUtils
from director import applogic as app
from director.utime import getUtime
from director.timercallback import TimerCallback
from director import visualization as vis
from director.debugVis import DebugData


import numpy as np
import math
from time import time
from time import sleep

class WristForceTorqueVisualizer(object):

    def __init__(self, robotStateModel, robotStateJointController, view):

        self.robotStateModel = robotStateModel
        self.robotStateJointController = robotStateJointController
    
        # Store calibration for wrist f/t sensors
        self.ft_right_bias = np.array([0.]*6)
        self.ft_left_bias = np.array([0.]*6)
        self.ft_left_calib_pts = []
        self.ft_left_calib_des = []
        self.ft_right_calib_pts = []
        self.ft_right_calib_des = []
        self.view = view

        # Draw on robot state update
        self.draw_left = False
        self.draw_right = False
        self.draw_torque = False
        self.robotStateModel.connectModelChanged(self.doFTDraw)

    def recomputeBiases(self):
        ft_right_bias_new = np.array([0.0]*6)
        for i in range(0, len(self.ft_right_calib_pts)):
            ft = self.ft_right_calib_pts[i]
            des = self.ft_right_calib_des[i]
            ft_right_bias_new[0:3] += (ft[0:3] - des)
            ft_right_bias_new[4:6] += ft[4:6]
        if len(self.ft_right_calib_pts) > 0:
            ft_right_bias_new /= float(len(self.ft_right_calib_pts))
        self.ft_right_bias = ft_right_bias_new

        ft_left_bias_new = np.array([0.0]*6)
        for i in range(0, len(self.ft_left_calib_pts)):
            ft = self.ft_left_calib_pts[i]
            des = self.ft_left_calib_des[i]
            ft_left_bias_new[0:3] += (ft[0:3] - des)
            ft_left_bias_new[4:6] += ft[4:6]
        if len(self.ft_left_calib_pts) > 0:
            ft_left_bias_new /= float(len(self.ft_left_calib_pts))
        self.ft_left_bias = ft_left_bias_new

    def calibRightFT(self):
        if (hasattr(self.robotStateJointController, 'lastRobotStateMessage') and 
            self.robotStateJointController.lastRobotStateMessage):
            ft = np.array(self.robotStateJointController.lastRobotStateMessage.force_torque.r_hand_force +
                  self.robotStateJointController.lastRobotStateMessage.force_torque.r_hand_torque)
            ft[0] = ft[0]*-1.
            ft[1] = ft[1]*-1.
            ft[3] = ft[3]*-1.
            ft[4] = ft[4]*-1.
            handMass = 1.0
            handCOM = [0., 0.1, 0.]
            # force is just down
            rHandFrame = self.robotStateModel.getLinkFrame('r_hand_force_torque')
            rHandForce = rHandFrame.GetInverse().TransformPoint([0,0,-9.8*handMass])
            # currently calibrating just force :P
            self.ft_right_calib_pts.append(ft)
            self.ft_right_calib_des.append(rHandForce)
        self.recomputeBiases()
            
    def calibLeftFT(self):
        if (hasattr(self.robotStateJointController, 'lastRobotStateMessage') and 
            self.robotStateJointController.lastRobotStateMessage):
            ft = np.array(self.robotStateJointController.lastRobotStateMessage.force_torque.l_hand_force +
                  self.robotStateJointController.lastRobotStateMessage.force_torque.l_hand_torque)
            handMass = 1.0
            handCOM = [0., 0.1, 0.]
            # force is just down
            lHandFrame = self.robotStateModel.getLinkFrame('l_hand_force_torque')
            lHandForce = lHandFrame.GetInverse().TransformPoint([0,0,-9.8*handMass])
            # currently calibrating just force :P
            self.ft_left_calib_pts.append(ft)
            self.ft_left_calib_des.append(lHandForce)
        self.recomputeBiases()

    def calibRightClearFT(self):
        self.ft_right_calib_pts = []
        self.ft_right_calib_des = []
        self.ft_right_bias = np.array([0.0]*6)
            
    def calibLeftClearFT(self):
        self.ft_left_calib_pts = []
        self.ft_left_calib_des = []
        self.ft_left_bias = np.array([0.0]*6)


    def tareRightFT(self):
        if (hasattr(self.robotStateJointController, 'lastRobotStateMessage') and 
            self.robotStateJointController.lastRobotStateMessage):
            self.ft_right_bias = np.array(self.robotStateJointController.lastRobotStateMessage.force_torque.r_hand_force +
                  self.robotStateJointController.lastRobotStateMessage.force_torque.r_hand_torque)
            self.ft_right_bias[0] = self.ft_right_bias[0]*-1.
            self.ft_right_bias[1] = self.ft_right_bias[1]*-1.
            self.ft_right_bias[3] = self.ft_right_bias[3]*-1.
            self.ft_right_bias[4] = self.ft_right_bias[4]*-1.
            
    def tareLeftFT(self):
        if (hasattr(self.robotStateJointController, 'lastRobotStateMessage') and 
            self.robotStateJointController.lastRobotStateMessage):
            self.ft_left_bias = np.array(self.robotStateJointController.lastRobotStateMessage.force_torque.l_hand_force +
                  self.robotStateJointController.lastRobotStateMessage.force_torque.l_hand_torque)
            
    def updateDrawSettings(self, draw_left, draw_right, draw_torque):
        self.draw_torque = draw_torque
        self.draw_right = draw_right
        self.draw_left = draw_left
        om.removeFromObjectModel(om.findObjectByName("wristft"))

    def doFTDraw(self, unusedrobotstate):
        frames = []
        fts = []
        vis_names = []
        if (hasattr(self.robotStateJointController, 'lastRobotStateMessage') and 
            self.robotStateJointController.lastRobotStateMessage):
            if self.draw_right:
                rft = np.array(self.robotStateJointController.lastRobotStateMessage.force_torque.r_hand_force +
                  self.robotStateJointController.lastRobotStateMessage.force_torque.r_hand_torque)
                rft[0] = -1.*rft[0] # right FT looks to be rotated 180deg around the z axis
                rft[1] = -1.*rft[1]
                rft[3] = -1.*rft[3]
                rft[4] = -1.*rft[4]
                rft -= self.ft_right_bias
                fts.append(rft)
                frames.append(self.robotStateModel.getLinkFrame('r_hand_force_torque'))
                vis_names.append('ft_right')
            if self.draw_left:
                lft = np.array(self.robotStateJointController.lastRobotStateMessage.force_torque.l_hand_force +
                  self.robotStateJointController.lastRobotStateMessage.force_torque.l_hand_torque)
                lft -= self.ft_left_bias
                fts.append(lft)
                frames.append(self.robotStateModel.getLinkFrame('l_hand_force_torque'))
                vis_names.append('ft_left')
            
        for i in range(0, len(frames)):
            frame = frames[i]
            ft = fts[i]
            offset = [0., 0., 0.]
            p1 = frame.TransformPoint(offset)
            scale = 1.0/25.0 # todo add slider for this?
            scalet = 1.0 / 2.5 
            p2f = frame.TransformPoint(offset + ft[0:3]*scale)
            p2t = frame.TransformPoint(offset + ft[3:6])
            normalt = (np.array(p2t) - np.array(p1))
            normalt = normalt /  np.linalg.norm(normalt)
            d = DebugData()
            # force
            if np.linalg.norm(np.array(p2f) - np.array(p1)) < 0.1:
                d.addLine(p1, p2f, color=[1.0, 0.0, 0.0])
            else:
                d.addArrow(p1, p2f, color=[1.,0.,0.])
            # torque
            if self.draw_torque:
                d.addCircle(p1, normalt, scalet*np.linalg.norm(ft[3:6]))
            # frame (largely for debug)
            vis.updateFrame(frame, vis_names[i]+'frame', view=self.view, parent='wristft', visible=False, scale=0.2)
            vis.updatePolyData(d.getPolyData(), vis_names[i], view=self.view, parent='wristft')