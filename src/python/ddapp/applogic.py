
import os
import time
import math
import PythonQt
from PythonQt import QtCore
from PythonQt import QtGui

from ddapp.timercallback import TimerCallback
from ddapp import midi

def getMainWindow():
    return _mainWindow

def quit():
    QtGui.QApplication.instance().quit()


def getDRCBase():
    return os.environ['DRC_BASE']


def getDRCView():
    return getMainWindow().viewManager().findView('DRC View')


def getSpreadsheetView():
    return getMainWindow().viewManager().findView('Spreadsheet View')


def getURDFModelDir():
    return os.path.join(getDRCBase(), 'software/models/mit_gazebo_models/mit_robot_drake')


def getNominalPoseMatFile():
    return os.path.join(getDRCBase(), 'software/drake/examples/Atlas/data/atlas_fp.mat')


def loadModelByName(name):
    filename = os.path.join(getURDFModelDir(), name)
    return getDRCView().loadURDFModel(filename)


def getDefaultDrakeModel():
    return getDRCView().models()[0]


def setupToolBar():

    def onComboChanged(text):
        loadModelByName(text)


    combo = QtGui.QComboBox()
    combo.addItem('Load URDF...')
    combo.addItem('model.urdf')
    combo.addItem('model_minimal_contact.urdf')
    combo.connect('currentIndexChanged(const QString&)', onComboChanged)
    toolbar = getMainWindow().toolBar()
    toolbar.addWidget(combo)


def showErrorMessage(message):
    QtGui.QMessageBox.warning(getMainWindow(), 'Error', message);


def startup(globals):

    global _mainWindow
    _mainWindow = globals['_mainWindow']

    if 'DRC_BASE' not in os.environ:
        showErrorMessage('DRC_BASE environment variable is not set')
        return

    if not os.path.isdir(getDRCBase()):
        showErrorMessage('DRC_BASE directory does not exist: ' + getDRCBase())
        return

    setupToolBar()

