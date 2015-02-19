import os

from PythonQt import QtCore, QtGui

import ddapp.applogic as app
import ddapp.objectmodel as om
import ddapp.ioUtils as io
import ddapp.visualization as vis
from ddapp import roboturdf
from ddapp import otdfmodel

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

    fileFilters = "Data Files (*.obj *.pcd *.ply *.stl *.vtk *.vtp *.urdf *.otdf)";
    filename = QtGui.QFileDialog.getOpenFileName(mainWindow, "Open...", getDefaultDirectory(), fileFilters)
    if not filename:
        return

    storeDefaultDirectory(filename)

    if filename.lower().endswith('urdf'):
        onOpenUrdf(filename)
    elif filename.lower().endswith('otdf'):
        onOpenOtdf(filename)
    else:
        onOpenGeometry(filename)


def onOpenGeometry(filename):

    polyData = io.readPolyData(filename)

    if not polyData or not polyData.GetNumberOfPoints():
        app.showErrorMessage('Failed to read any data from file: %s' % filename, title='Reader error')
        return

    vis.showPolyData(polyData, os.path.basename(filename), parent='files')


def onOpenUrdf(filename):

    model = roboturdf.openUrdf(filename, app.getCurrentRenderView())
    if not model:
        app.showErrorMessage('Failed to read urdf file: %s' % filename, title='Read urdf error')


def onOpenOtdf(filename):
    model = otdfmodel.openOtdf(filename, app.getCurrentRenderView())


def onFileExportUrdf():
    obj = om.getActiveObject()
    if not obj or not isinstance(obj, otdfmodel.OtdfModelItem):
        app.showErrorMessage('Please select an OTDF object', title='OTDF object not selected')
        return

    mainWindow = app.getMainWindow()
    filename = QtGui.QFileDialog.getSaveFileName(mainWindow, "Save Data...", getDefaultDirectory(), 'URDF (*.urdf)', 'URDF (*.urdf)')

    if not os.path.splitext(filename)[1]:
        filename += '.urdf'

    storeDefaultDirectory(filename)
    urdfString = obj.parser.getUrdfFromOtdf()
    urdfFile = open(filename, 'w')
    urdfFile.write(urdfString)
    urdfFile.close()

def onFileSaveData():

    obj = om.getActiveObject()
    if not obj:
        app.showErrorMessage('Please select an object', title='No object selected')
        return
    if isinstance(obj, otdfmodel.OtdfModelItem):
        mainWindow = app.getMainWindow()
        filename = QtGui.QFileDialog.getSaveFileName(mainWindow, "Save Data...", getDefaultDirectory(), 'OTDF (*.otdf)', 'OTDF (*.otdf)')

        if not os.path.splitext(filename)[1]:
            filename += '.otdf'

        storeDefaultDirectory(filename)
        otdfString = obj.parser.getUpdatedOtdf()
        otdfFile = open(filename, 'w')
        otdfFile.write(otdfString)
        otdfFile.close()
    elif hasattr(obj, 'polyData'):
        mainWindow = app.getMainWindow()
        fileFilters = "PLY (*.ply);;STL (*.stl);;VTP (*.vtp)";
        filename = QtGui.QFileDialog.getSaveFileName(mainWindow, "Save Data...", getDefaultDirectory(), fileFilters, 'VTP (*.vtp)')

        if not filename:
            return

        if not os.path.splitext(filename)[1]:
            filename += '.vtp'

        polyData = io.writePolyData(obj.polyData, filename)
    else:
        app.showErrorMessage('Please select an object that contains geometry data or an OTDF object', title='Invalid object selected')
        return

    storeDefaultDirectory(filename)


def init():
    mainWindow = app.getMainWindow()

    mainWindow.connect('fileOpen()', onFileOpen)
    mainWindow.connect('fileSaveData()', onFileSaveData)
    mainWindow.connect('fileExportUrdf()', onFileExportUrdf)
