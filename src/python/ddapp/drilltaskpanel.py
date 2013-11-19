import PythonQt
from PythonQt import QtCore, QtGui

from ddapp import lcmUtils
from ddapp.utime import getUtime
from drc import drill_control_t

import ddapp.applogic as app

def _makeButton(text, func):

    b = QtGui.QPushButton(text)
    b.connect('clicked()', func)
    return b



class KeyboardNavigation(object):

    def __init__(self):
        self.eventFilter = None
        self.widget = QtGui.QWidget()
        l = QtGui.QVBoxLayout(self.widget)
        self.button = _makeButton('enable keyboard navigation', self.clicked)
        self.button.setCheckable(True)
        l.addWidget(self.button)
        self.callbacks = []

    def clicked(self):

        if self.eventFilter:
            self.widget.removeEventFilter(self.eventFilter)
            self.eventFilter = None
        else:
            self.eventFilter = PythonQt.dd.ddPythonEventFilter()
            self.eventFilter.addFilteredEventType(QtCore.QEvent.KeyPress)
            self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.keyPressEvent)
            self.widget.installEventFilter(self.eventFilter)
            self.widget.setFocus(True)

    def keyPressEvent(self, obj, event):
        self.eventFilter.setEventHandlerResult(True)
        for func in self.callbacks:
            func(event.key())


class DrillTaskPanel(object):

    def __init__(self):
        self.widget = QtGui.QGroupBox('Drill Planner')

        self.deltaSpinBoxes = [QtGui.QSpinBox() for i in xrange(3)]
        for spin in self.deltaSpinBoxes:
            spin.setMinimum(-100)
            spin.setMaximum(100)
            spin.setSingleStep(1)

        l = QtGui.QVBoxLayout(self.widget)
        l.addWidget(_makeButton('refit drill', self.refitDrill))
        l.addWidget(_makeButton('request nominal plan', self.nominalPlan))
        l.addWidget(_makeButton('request arm prepose plan', self.armPreposePlan))
        l.addWidget(_makeButton('request walking goal', self.walkingGoal))
        l.addWidget(_makeButton('request nominal fixed plan', self.nominalFixedPlan))
        l.addWidget(_makeButton('request pre-drill plan', self.preDrillPlan))
        l.addWidget(_makeButton('request drill-in plan', self.drillInPlan))
        l.addWidget(QtGui.QLabel(''))
        l.addWidget(_makeButton('next drill plan', self.nextDrillPlan))
        l.addWidget(QtGui.QLabel(''))

        hw = QtGui.QWidget()
        hl = QtGui.QHBoxLayout(hw)
        self.drillDeltaButton = _makeButton('drill delta', self.drillDelta)
        hl.addWidget(self.drillDeltaButton)
        for spin in self.deltaSpinBoxes:
            hl.addWidget(spin)
        hl.addWidget(QtGui.QLabel('cm'))
        l.addWidget(hw)

        self.keyPressNav = KeyboardNavigation()
        self.keyPressNav.callbacks.append(self.onKeyPress)
        l.addWidget(self.keyPressNav.widget)


    def sendControlMessage(self, command, data=None):
        m = drill_control_t()
        m.utime = getUtime()
        m.control_type = command
        m.data = data if data is not None else []
        m.data_length = len(m.data)
        lcmUtils.publish('DRILL_CONTROL', m)


    def onKeyPress(self, key):
        dist = 0.01
        if key == QtCore.Qt.Key_Left:
            delta = [-dist, 0.0, 0.0]
        elif key == QtCore.Qt.Key_Right:
            delta = [dist, 0.0, 0.0]
        elif key == QtCore.Qt.Key_Up:
            delta = [0.0, dist, 0.0]
        elif key == QtCore.Qt.Key_Down:
            delta = [0.0, -dist, 0.0]
        elif key == QtCore.Qt.Key_PageUp:
            delta = [0.0, 0.0, dist]
        elif key == QtCore.Qt.Key_PageDown:
            delta = [0.0, 0.0, -dist]
        else:
            print 'unknown key:', key
            return

        for spin, value in zip(self.deltaSpinBoxes, delta):
            spin.value = round(value * 100)

        self.drillDeltaButton.animateClick()
        self.sendControlMessage(drill_control_t.RQ_DRILL_DELTA_PLAN, delta)


    def refitDrill(self):
        self.sendControlMessage(drill_control_t.REFIT_DRILL)

    def nominalPlan(self):
        self.sendControlMessage(drill_control_t.RQ_NOMINAL_PLAN)

    def armPreposePlan(self):
        self.sendControlMessage(drill_control_t.RQ_ARM_PREPOSE_PLAN)

    def walkingGoal(self):
        self.sendControlMessage(drill_control_t.RQ_WALKING_GOAL)

    def nominalFixedPlan(self):
        self.sendControlMessage(drill_control_t.RQ_NOMINAL_FIXED_PLAN)

    def preDrillPlan(self):
        self.sendControlMessage(drill_control_t.RQ_PREDRILL_PLAN)

    def drillInPlan(self):
        self.sendControlMessage(drill_control_t.RQ_DRILL_IN_PLAN)

    def nextDrillPlan(self):
        self.sendControlMessage(drill_control_t.RQ_NEXT_DRILL_PLAN)

    def drillDelta(self):
        data = [float(spin.value)/100.0 for spin in self.deltaSpinBoxes]
        self.sendControlMessage(drill_control_t.RQ_DRILL_DELTA_PLAN, data)


