import os
import sys
import math
import time
import types
import numpy as np
import vtkAll as vtk
from threading import Timer

from ddapp import transformUtils
from ddapp import lcmUtils
from ddapp import objectmodel as om
from ddapp import visualization as vis

import drc as lcmdrc


class Gamepad(object):
	def __init__(self):
		lcmUtils.addSubscriber('GAMEPAD_CHANNEL', lcmdrc.gamepad_cmd_t, self.onGamepadCommand)
		self.travel = 0.2
		self.baseFrame = None
		self.resetDeltas()

	def resetDeltas(self):
		self.theta = 0.0
		self.phi = 0.0
		self.deltaX = 0.0
		self.deltaY = 0.0
		self.deltaZ = 0.0

	def onGamepadCommand(self, msg):
		frame = om.getActiveObject()
		if isinstance(frame, vis.FrameItem):
			if msg.button == msg.GAMEPAD_BUTTON_X and msg.button_x == 1:
				self.baseFrame = frame
				self.baseTransform = transformUtils.copyFrame(frame.transform)
				self.resetDeltas()

			if msg.button == msg.GAMEPAD_LEFT_X or msg.button == msg.GAMEPAD_LEFT_Y:
				self.theta = msg.thumbpad_left_x*90
				self.phi = msg.thumbpad_left_y*90

			if msg.button == msg.GAMEPAD_RIGHT_X or msg.button == msg.GAMEPAD_RIGHT_Y:
				self.deltaX = self.travel*msg.thumbpad_right_x
				self.deltaZ = self.travel*msg.thumbpad_right_y
			
			if msg.button == msg.GAMEPAD_TRIGGER_L:
				self.deltaY = -self.travel*msg.trigger_left
			
			if msg.button == msg.GAMEPAD_TRIGGER_R:
				self.deltaY = self.travel*msg.trigger_right

			self.updateFrame(frame)

	def updateFrame(self, frame):
		if self.baseFrame is not None:
			t = vtk.vtkTransform()
			t.Concatenate(self.baseTransform)		
			t.RotateZ(-self.theta)
			t.RotateX(self.phi)
			t.Translate(self.deltaX, self.deltaY, self.deltaZ)
			frame.copyFrame(t)
