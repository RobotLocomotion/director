
import os
import vtk
import time
import math
import PythonQt
from PythonQt import QtCore
from PythonQt import QtGui

def getMainWindow():
    return _mainWindow

def quit():
    QtGui.QApplication.instance().quit()


def getDRCBase():
    return os.environ['DRC_BASE']

def getDRCView():
    return getMainWindow().viewManager().findView('DRC View')


def getURDFModelDir():
    return os.path.join(getDRCBase(), 'software/models/mit_gazebo_models/mit_robot_drake')



def loadTestModel():
    filename = 'model_minimal_contact.urdf'
    filename = os.path.join(getURDFModelDir(), filename)
    getDRCView().loadURDFModel(filename)


def getDrakeModel():
    return getDRCView().models()[0]



def testJoint(jointId):

    runtime = 2.0
    fps = 30
    sleepDelta = 1.0 / (runtime * fps)
    model = getDrakeModel()

    global t
    t = 0.0

    def iterate():

        global t
        global renderTimer

        q = [0.0 for i in xrange(model.numberOfJoints())]

        if t < runtime:

            t += sleepDelta

            q[jointId] = math.sin( (t / runtime) * math.pi) * math.pi
            #print t/runtime, ':  ', math.degrees(q[jointId])

            model.setJointPositions(q)
            renderTimer.start(sleepDelta * 1000.0)
        else:

            model.setJointPositions(q)
            del renderTimer


    global renderTimer
    renderTimer = QtCore.QTimer()
    renderTimer.setSingleShot(True)
    renderTimer.connect('timeout()', iterate)

    iterate()


def setupToolBar():

    def onComboChanged(text):
        filename = os.path.join(getURDFModelDir(), text)
        print 'loading', filename
        getDRCView().loadURDFModel(filename)


    combo = QtGui.QComboBox()
    combo.addItem('Load URDF...')
    combo.addItem('model.urdf')
    combo.addItem('model_minimal_contact.urdf')
    combo.connect('currentIndexChanged(const QString&)', onComboChanged)
    toolbar = getMainWindow().toolBar()
    toolbar.addWidget(combo)


def setupConsoleGlobals(globals):
    '''Add some variables to be predefined in the console.'''
    globals['quit'] = quit
    globals['exit'] = quit
    globals['vtk'] = vtk
    globals['QtCore'] = QtCore
    globals['QtGui'] = QtGui


def startup(globals):

    if 'DRC_BASE' not in os.environ:
        showErrorMessage('DRC_BASE environment variable is not set')
        return

    if not os.path.isdir(getDRCBase()):
        showErrorMessage('DRC_BASE directory does not exist: ' + getDRCBase())
        return


    global _mainWindow
    _mainWindow = globals['_mainWindow']

    setupToolBar()
    setupConsoleGlobals(globals)

    loadTestModel()

