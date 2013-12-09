import PythonQt
from PythonQt import QtCore, QtGui

from ddapp import lcmUtils
from ddapp.utime import getUtime
from drc import drill_control_t

import numpy as np
import math

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


def sendControlMessage(command, data=None, channel='DRILL_CONTROL'):
    m = drill_control_t()
    m.utime = getUtime()
    m.control_type = command
    m.data = data if data is not None else []
    m.data_length = len(m.data)
    lcmUtils.publish(channel, m)


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
        l.addWidget(_makeButton('button preset posture', self.gotoButtonPreset))
        l.addWidget(_makeButton('button pre-pose plan', self.buttonPrePosePlan))
        l.addWidget(_makeButton('request nominal plan', self.nominalPlan))
        l.addWidget(_makeButton('request arm prepose plan', self.armPreposePlan))
        l.addWidget(_makeButton('request walking goal', self.walkingGoal))
        l.addWidget(_makeButton('request nominal fixed plan', self.nominalFixedPlan))
        l.addWidget(_makeButton('request pre-drill plan', self.preDrillPlan))
        l.addWidget(_makeButton('request drill-in plan', self.drillInPlan))
        l.addWidget(QtGui.QLabel(''))
        l.addWidget(_makeButton('next drill plan', self.nextDrillPlan))

        hw = QtGui.QWidget()
        hl = QtGui.QHBoxLayout(hw)
        hl.addWidget(_makeButton('set drill depth', self.setDrillDepth))
        self.drillDepthSpin = QtGui.QSpinBox()
        self.drillDepthSpin.setMinimum(-100)
        self.drillDepthSpin.setMaximum(100)
        self.drillDepthSpin.setSingleStep(1)
        hl.addWidget(self.drillDepthSpin)
        hl.addWidget(QtGui.QLabel('cm'))
        l.addWidget(hw)



        hw = QtGui.QWidget()
        hl = QtGui.QHBoxLayout(hw)
        self.drillDeltaCombo = QtGui.QComboBox()
        self.drillDeltaCombo.addItem('button')
        self.drillDeltaCombo.addItem('wall')
        self.drillDeltaButton = _makeButton('drill delta', self.drillDelta)
        hl.addWidget(self.drillDeltaButton)
        hl.addWidget(self.drillDeltaCombo)
        for spin in self.deltaSpinBoxes:
            hl.addWidget(spin)
        hl.addWidget(QtGui.QLabel('cm'))
        hl.addWidget(_makeButton('clear', self.clearDrillDelta))
        l.addWidget(QtGui.QLabel(''))
        l.addWidget(QtGui.QLabel(''))
        l.addWidget(hw)
        self.keyPressNav = KeyboardNavigation()
        self.keyPressNav.callbacks.append(self.onKeyPress)
        l.addWidget(self.keyPressNav.widget)


    def onKeyPress(self, key):

        dist = 0.01

        keyDeltas = {
                      QtCore.Qt.Key_Left   : np.array([0.0, dist, 0.0]),
                      QtCore.Qt.Key_Right : np.array([0.0, -dist, 0.0]),

                      QtCore.Qt.Key_Up       : np.array([0.0, 0.0, dist]),
                      QtCore.Qt.Key_Down     : np.array([0.0, 0.0, -dist]),

                      QtCore.Qt.Key_PageUp     : np.array([dist, 0.0, 0.0]),
                      QtCore.Qt.Key_PageDown    : np.array([-dist, 0.0, 0.0]),
                    }

        if key not in keyDeltas:
            print 'unknown key:', key
            return

        delta = np.array([spin.value / 100.0 for spin in self.deltaSpinBoxes])
        delta += keyDeltas[key]

        for spin, value in zip(self.deltaSpinBoxes, delta):
            spin.value = round(value * 100)

        self.drillDeltaButton.animateClick()
        self.sendDeltaMessage(delta)


    def sendDeltaMessage(self, data):
        deltaTypes = { 'button' : drill_control_t.RQ_BUTTON_DELTA_PLAN,
                       'wall' : drill_control_t.RQ_DRILL_DELTA_PLAN
                    }

        deltaType = deltaTypes[self.drillDeltaCombo.currentText]
        sendControlMessage(deltaType, data)

    def clearDrillDelta(self):
        for spin in self.deltaSpinBoxes:
            spin.value = 0

    def buttonPrePosePlan(self):
        sendControlMessage(drill_control_t.RQ_BUTTON_PREPOSE_PLAN)

    def refitDrill(self):
        sendControlMessage(drill_control_t.REFIT_DRILL)

    def nominalPlan(self):
        sendControlMessage(drill_control_t.RQ_NOMINAL_PLAN)

    def gotoButtonPreset(self):
        sendControlMessage(drill_control_t.RQ_GOTO_BUTTON_PRESET)

    def armPreposePlan(self):
        sendControlMessage(drill_control_t.RQ_ARM_PREPOSE_PLAN)

    def walkingGoal(self):
        sendControlMessage(drill_control_t.RQ_WALKING_GOAL)

    def nominalFixedPlan(self):
        sendControlMessage(drill_control_t.RQ_NOMINAL_FIXED_PLAN)

    def preDrillPlan(self):
        sendControlMessage(drill_control_t.RQ_PREDRILL_PLAN)

    def drillInPlan(self):
        sendControlMessage(drill_control_t.RQ_DRILL_IN_PLAN)

    def nextDrillPlan(self):
        sendControlMessage(drill_control_t.RQ_NEXT_DRILL_PLAN)

    def drillDelta(self):
        data = [float(spin.value)/100.0 for spin in self.deltaSpinBoxes]
        self.sendDeltaMessage(data)

    def setDrillDepth(self):
        depth = self.drillDepthSpin.value/100.0
        sendControlMessage(drill_control_t.SET_DRILL_DEPTH, data=[depth])

    def valveCirclePlan(self, angleInDegrees):
        sendControlMessage(drill_control_t.RQ_VALVE_CIRCLE_PLAN, data=[math.radians(angleInDegrees)])

