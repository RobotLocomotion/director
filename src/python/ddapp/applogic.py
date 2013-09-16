
import os
import vtk
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


def getURDFModelDir():
    return os.path.join(getDRCBase(), 'software/models/mit_gazebo_models/mit_robot_drake')



def loadTestModel():
    filename = 'model_minimal_contact.urdf'
    filename = os.path.join(getURDFModelDir(), filename)
    getDRCView().loadURDFModel(filename)


def getDrakeModel():
    return getDRCView().models()[0]


class MidiJointControl(TimerCallback):

    def __init__(self):
        TimerCallback.__init__(self)
        self.reader = midi.MidiReader()
        self.controller = JointController()

        self.channelToJoint = { 21: 13 }


    def _scaleMidiValue(self, midiValue):
        degrees = midiValue * 180.0/127.0
        return degrees


    def tick(self):
        messages = self.reader.getMessages()
        if not messages:
            return

        targets = {}
        for message in messages:
            channel = message[2]
            value = message[3]
            targets[channel] = value

        for channel, value in targets.iteritems():
            jointId = self.channelToJoint.get(channel)
            position = self._scaleMidiValue(value)

            if jointId is not None:
                self.controller.setJointPosition(jointId, position)


class JointController(object):

    def __init__(self):
        self.model = getDrakeModel()
        self.reset()

    def setJointPosition(self, jointId, position):

        assert jointId >= 0 and jointId < len(self.q)
        self.q[jointId] = math.radians(position % 360.0)
        self.push()

    def push(self):
        self.model.setJointPositions(self.q)

    def reset(self):
        self.q = [0.0 for i in xrange(self.model.numberOfJoints())]


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

