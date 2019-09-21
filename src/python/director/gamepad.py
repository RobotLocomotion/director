import os
import sys
import math
import time
import types
import numpy as np
from . import vtkAll as vtk
from director.timercallback import TimerCallback

from director import transformUtils
from director import lcmUtils
from director import objectmodel as om
from director import visualization as vis

import drc as lcmdrc


class Gamepad(object):
    def __init__(self, teleopPanel, teleopJointController, ikPlanner, view):
        lcmUtils.addSubscriber('GAMEPAD_CHANNEL', lcmdrc.gamepad_cmd_t, self.onGamepadCommand)
        self.speedMultiplier = 1.0
        self.maxSpeedMultiplier = 3.0
        self.minSpeedMultiplier = 0.2
        self.travel = 1
        self.angularTravel = 180
        self.baseFrame = None
        self.resetDeltas()
        self.timer = TimerCallback()
        self.timer.callback = self.tick
        self.teleopPanel = teleopPanel
        self.teleopJointController = teleopJointController
        self.ikPlanner = ikPlanner
        self.view = view
        self.camera = view.camera()
        self.cameraTarget = np.asarray(self.camera.GetFocalPoint())
        self.endEffectorFrames = []
        self.endEffectorIndex = 0
        self.teleopOn = False
        self.keyFramePoses = list()
        self.cameraMode = False
        self.timerStarted = False

    def findEndEffectors(self):
        planning = om.getOrCreateContainer('planning')
        if planning is not None:
            teleopFolder = planning.findChild('teleop plan')
            if teleopFolder is not None:
                framesFolder = teleopFolder.findChild('constraint frames')
                if framesFolder is not None:
                    self.endEffectorFrames = framesFolder.children()

    def cycleEndEffector(self):
        if len(self.endEffectorFrames) is not 0:
            self.clearKeyframePoses()
            self.endEffectorFrames[self.endEffectorIndex].setProperty('Edit', False)
            self.endEffectorIndex = (self.endEffectorIndex + 1) % len(self.endEffectorFrames)
            self.endEffectorFrames[self.endEffectorIndex].setProperty('Edit', True)
            om.setActiveObject(self.endEffectorFrames[self.endEffectorIndex])
            self.baseFrame = self.endEffectorFrames[self.endEffectorIndex]
            self.baseTransform = transformUtils.copyFrame(self.baseFrame.transform)
            self.resetDeltas()
            self.addKeyframePose()

    def clearKeyframePoses(self):
        self.keyFramePoses = list()

    def addKeyframePose(self):
        self.keyFramePoses.append(self.teleopJointController.q.copy())
    
    def finalizeTrajectory(self):
        self.ikPlanner.computeMultiPostureGoal(self.keyFramePoses)

    def tick(self):
        self.updateFrame()

    def resetDeltas(self):
        self.theta = 0.0
        self.phi = 0.0
        self.yaw = 0.0
        self.X = 0.0
        self.Y = 0.0
        self.Z = 0.0
        
        self.thetadot = 0.0
        self.phidot = 0.0
        self.yawdot = 0.0
        self.Xdot = 0.0
        self.Ydot = 0.0
        self.Zdot = 0.0

    def toggleTeleop(self):
        self.teleopOn = not self.teleopOn

        if self.teleopOn:
            self.teleopPanel.endEffectorTeleop.activate()
            self.teleopPanel.endEffectorTeleop.setLHandConstraint('ee fixed')
            self.teleopPanel.endEffectorTeleop.setRHandConstraint('ee fixed')
        else:
            self.teleopPanel.endEffectorTeleop.deactivate()


    def increaseTeleopSpeed(self):
        self.speedMultiplier = self.clampTeleopSpeed(self.speedMultiplier + 0.2)
        print('teleop speed: ' + str(self.speedMultiplier))
    
    def decreaseTeleopSpeed(self):
        self.speedMultiplier = self.clampTeleopSpeed(self.speedMultiplier - 0.2)
        print('teleop speed: ' + str(self.speedMultiplier))

    def clampTeleopSpeed(self, speed):
        return np.clip(speed, self.minSpeedMultiplier, self.maxSpeedMultiplier)
    
    def enableCameraMode(self):
        print('Gamepad camera mode on')
        self.cameraMode = True
    
    def disableCameraMode(self):
        print('Gamepad camera mode off')
        self.cameraMode = False
    
    def setCameraTarget(self):
        if self.baseFrame is not None:
            self.cameraTarget = np.asarray(self.baseFrame.transform.GetPosition())

    def onGamepadCommand(self, msg):
        if self.timerStarted is not True:
            self.timer.start()
            self.timerStarted = True

        if msg.button == msg.GAMEPAD_BUTTON_A and msg.button_a == 1:
            self.finalizeTrajectory()

        if msg.button == msg.GAMEPAD_DPAD_X and msg.button_dpad_x < 0:
            self.enableCameraMode()
            self.setCameraTarget()

        if msg.button == msg.GAMEPAD_DPAD_X and msg.button_dpad_x > 0:
            self.disableCameraMode()

        if msg.button == msg.GAMEPAD_BUTTON_Y and msg.button_y == 1:
            self.findEndEffectors()
            self.cycleEndEffector()
            self.setCameraTarget()

        if msg.button == msg.GAMEPAD_DPAD_Y and msg.button_dpad_y > 0:
            self.increaseTeleopSpeed()

        if msg.button == msg.GAMEPAD_DPAD_Y and msg.button_dpad_y < 0:
            self.decreaseTeleopSpeed()

        if msg.button == msg.GAMEPAD_BUTTON_START and msg.button_start == 1:
            self.toggleTeleop()

        if msg.button == msg.GAMEPAD_BUTTON_X and msg.button_x == 1:
            self.addKeyframePose()

        if msg.button == msg.GAMEPAD_BUTTON_LB:
            self.yawdot = -self.angularTravel * msg.button_lb

        if msg.button == msg.GAMEPAD_BUTTON_RB:
            self.yawdot = self.angularTravel * msg.button_rb

        if msg.button == msg.GAMEPAD_LEFT_X or msg.button == msg.GAMEPAD_LEFT_Y:
            self.thetadot = msg.thumbpad_left_x * self.angularTravel
            self.phidot = msg.thumbpad_left_y * self.angularTravel

        if msg.button == msg.GAMEPAD_RIGHT_X or msg.button == msg.GAMEPAD_RIGHT_Y:
            self.Xdot = self.travel * msg.thumbpad_right_x
            self.Zdot = self.travel * msg.thumbpad_right_y
        
        if msg.button == msg.GAMEPAD_TRIGGER_L:
            self.Ydot = -self.travel * msg.trigger_left
        
        if msg.button == msg.GAMEPAD_TRIGGER_R:
            self.Ydot = self.travel * msg.trigger_right

    def updateFrame(self):
        norm = np.linalg.norm(np.array([self.thetadot, self.phidot, self.yawdot, self.Xdot, self.Ydot, self.Zdot]))
        dt = 0.01 #self.timer.elapsed

        if self.baseFrame is not None:
            cameraFocus = np.asarray(self.camera.GetFocalPoint())
            cameraInterp = cameraFocus + (self.cameraTarget - cameraFocus)*0.02
            self.camera.SetFocalPoint(cameraInterp)
        
        if self.cameraMode is not True:
            if self.baseFrame is not None and norm > 0.1:
                t = vtk.vtkTransform()
                t.Concatenate(self.baseFrame.transform)
                t.RotateZ(-self.thetadot * dt * self.speedMultiplier)
                t.RotateX(self.phidot * dt * self.speedMultiplier)
                t.RotateY(self.yawdot * dt * self.speedMultiplier)
                t.Translate(self.Xdot * dt * self.speedMultiplier, self.Ydot * dt * self.speedMultiplier, self.Zdot * dt * self.speedMultiplier)
                self.baseFrame.copyFrame(t)
        else:
            
            self.camera.Elevation(self.Zdot * self.speedMultiplier)
            self.camera.Azimuth(self.Xdot * self.speedMultiplier)
            self.camera.Zoom(1.0 + self.phidot/1000.0)

        self.view.render()

