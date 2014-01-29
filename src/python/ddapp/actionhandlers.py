import os

from PythonQt import QtCore, QtGui

import ddapp.applogic as app
import ddapp.objectmodel as om
import ddapp.ioUtils as io
import ddapp.visualization as vis

_lastDir = None

def getDefaultDirectory():
    return _lastDir or os.getcwd()


def storeDefaultDirectory(filename):

    global _lastDir
    if os.path.isfile(filename):
        filename = os.path.dirname(filename)
    if os.path.isdir(filename):
        _lastDir = filename

def onFileOpen():

    mainWindow = app.getMainWindow()

    fileFilters = "Geometry Files (*.obj *.pcd *.ply *.stl *.vtk *.vtp)";
    filename = QtGui.QFileDialog.getOpenFileName(mainWindow, "Open...", getDefaultDirectory(), fileFilters)
    if not filename:
        return

    polyData = io.readPolyData(filename)

    if not polyData or not polyData.GetNumberOfPoints():
        app.showErrorMessage('Failed to read any data from file: %s' % filename, title='Reader error')
        return

    vis.showPolyData(polyData, os.path.basename(filename), parent='files')
    storeDefaultDirectory(filename)


def onFileSaveData():

    obj = om.getActiveObject()
    if not obj or not hasattr(obj, 'polyData'):
        app.showErrorMessage('Please select an object that contains geometry data', title='Geometry object not selected')
        return

    mainWindow = app.getMainWindow()
    fileFilters = "PLY (*.ply);;STL (*.stl);;VTP (*.vtp)";
    filename = QtGui.QFileDialog.getSaveFileName(mainWindow, "Save Data...", getDefaultDirectory(), fileFilters, 'VTP (*.vtp)')

    if not filename:
        return

    if not os.path.splitext(filename)[1]:
        filename += '.vtp'

    polyData = io.writePolyData(obj.polyData, filename)
    storeDefaultDirectory(filename)


def init():
    mainWindow = app.getMainWindow()

    mainWindow.connect('fileOpen()', onFileOpen)
    mainWindow.connect('fileSaveData()', onFileSaveData)
