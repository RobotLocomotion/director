import os
import sys
import math
import time
import types
import numpy as np
import vtkAll as vtk
#from threading import Timer
from ddapp.timercallback import TimerCallback

from ddapp import transformUtils
from ddapp import lcmUtils
from ddapp import objectmodel as om
from ddapp import visualization as vis

import drc as lcmdrc


class Gamepad(object):
	def __init__(self, teleopPanel):
		print 'gamepad init'
		lcmUtils.addSubscriber('GAMEPAD_CHANNEL', lcmdrc.gamepad_cmd_t, self.onGamepadCommand)
		self.travel = 1
		self.angularTravel = 180
		self.baseFrame = None
		self.resetDeltas()
		self.timer = TimerCallback()
		self.timer.callback = self.tick
		self.timer.start()
		self.teleopPanel = teleopPanel

		self.endEffectorFrames = []
		self.endEffectorIndex = 0

	#todo: add keypoints for a multiPostureGoal using:
	#qi = teleopJointController.q.copy()
	#computeMultiPostureGoal([q1,...,qn])
	def findEndEffectors(self):
		planning = om.getOrCreateContainer('planning')
		if planning is not None:
			print 'found planning'
			teleopFolder = planning.findChild('teleop plan')
			if teleopFolder is not None:
				print 'found teleop'
				framesFolder = teleopFolder.findChild('constraint frames')
				if framesFolder is not None:
					print 'found frames'
					self.endEffectorFrames = framesFolder.children()

	def cycleEndEffector(self):
		if len(self.endEffectorFrames) is not 0:
			self.endEffectorFrames[self.endEffectorIndex].setProperty('Edit', False)
			self.endEffectorIndex = (self.endEffectorIndex + 1) % len(self.endEffectorFrames)
			self.endEffectorFrames[self.endEffectorIndex].setProperty('Edit', True)
			om.setActiveObject(self.endEffectorFrames[self.endEffectorIndex])
			self.baseFrame = self.endEffectorFrames[self.endEffectorIndex]
			self.baseTransform = transformUtils.copyFrame(self.baseFrame.transform)
			self.resetDeltas()

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


	def onGamepadCommand(self, msg):

		if msg.button == msg.GAMEPAD_BUTTON_Y and msg.button_y == 1:
			self.findEndEffectors()
			self.cycleEndEffector()

		if msg.button == msg.GAMEPAD_BUTTON_A and msg.button_a == 1:
			print 'activating end effector teleop'
			self.teleopPanel.endEffectorTeleop.activate()
			self.teleopPanel.endEffectorTeleop.setLHandConstraint('ee fixed')
			self.teleopPanel.endEffectorTeleop.setRHandConstraint('ee fixed')

		if msg.button == msg.GAMEPAD_BUTTON_B and msg.button_b == 1:
			self.teleopPanel.endEffectorTeleop.deactivate()

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
	
	def integrateVelocities(self):
		#todo: do this in numpy
		self.theta += self.thetadot * self.dt
		self.phi += self.phidot * self.dt
		self.X += self.Xdot * self.dt
		self.Y += self.Ydot * self.dt
		self.Z += self.Zdot * self.dt

	def updateFrame(self):
		norm = np.linalg.norm(np.array([self.thetadot, self.phidot, self.yawdot, self.Xdot, self.Ydot, self.Zdot]))
		if self.baseFrame is not None and norm > 0.1:
			dt = 0.01
			t = vtk.vtkTransform()
			t.Concatenate(self.baseFrame.transform)
			t.RotateZ(-self.thetadot * dt)
			t.RotateX(self.phidot * dt)
			t.RotateY(self.yawdot * dt)
			t.Translate(self.Xdot * dt, self.Ydot * dt, self.Zdot * dt)
			self.baseFrame.copyFrame(t)
