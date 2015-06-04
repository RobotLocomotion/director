import PythonQt
from PythonQt import QtCore, QtGui
import numpy as np
import vtkAll as vtk

lastEditedFrame = None

def registerFrame(frame):
	global lastEditedFrame 
	lastEditedFrame = frame

def applyFrameTransform(x, y, z, yaw):
	if lastEditedFrame is not None and lastEditedFrame.getProperty('Edit'):
		t = vtk.vtkTransform()
		t.Concatenate(lastEditedFrame.transform)
		t.RotateZ(yaw)
		t.Translate(x, y, z)
		lastEditedFrame.copyFrame(t)	

def shiftFrameX(amount):
	applyFrameTransform(amount, 0, 0, 0)

def shiftFrameY(amount):
	applyFrameTransform(0, amount, 0, 0)

def shiftFrameYaw(amount):
	applyFrameTransform(0, 0, 0, amount)

def shiftFrameZ(amount):
	applyFrameTransform(0, 0, amount, 0)

def handleKey(event):
	linearDisplacement = 0.01
	angularDisplacement = 1
	multiplier = 5

	if event.modifiers() & QtCore.Qt.ControlModifier:
		linearDisplacement *= multiplier
		angularDisplacement *= multiplier

	if event.key() == QtCore.Qt.Key_Left:
		if event.modifiers() & QtCore.Qt.ShiftModifier:
			shiftFrameYaw(angularDisplacement)
		else:
			shiftFrameX(-linearDisplacement)
	elif event.key() == QtCore.Qt.Key_Right:
		if event.modifiers() & QtCore.Qt.ShiftModifier:
			shiftFrameYaw(-angularDisplacement)
		else:
			shiftFrameX(linearDisplacement)
	elif event.key() == QtCore.Qt.Key_Up:
		if event.modifiers() & QtCore.Qt.ShiftModifier:
			shiftFrameZ(linearDisplacement)
		else:
			shiftFrameY(linearDisplacement)
	elif event.key() == QtCore.Qt.Key_Down:
		if event.modifiers() & QtCore.Qt.ShiftModifier:
			shiftFrameZ(-linearDisplacement)
		else:
			shiftFrameY(-linearDisplacement)
	return False