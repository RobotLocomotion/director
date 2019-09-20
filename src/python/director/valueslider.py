from director import callbacks
from director.flags import Flags
from director.timercallback import TimerCallback
from director import qtutils
import PythonQt
from PythonQt import QtCore, QtGui

import time
import numpy as np


class ValueSlider(object):

    events = Flags('VALUE_CHANGED')

    def __init__(self, minValue=0.0, maxValue=1.0, resolution=1000):
        self._value = 0.0
        self.spinbox = QtGui.QDoubleSpinBox()
        self.spinbox.setSuffix('s')
        self.slider = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.playButton = QtGui.QPushButton('Play')
        self.setValueRange(minValue, maxValue)
        self.setResolution(resolution)
        self.slider.connect('valueChanged(int)', self._onSliderValueChanged)
        self.spinbox.connect('valueChanged(double)', self._onSpinboxValueChanged)
        self.playButton.connect('clicked()', self._onPlayClicked)
        self.widget = QtGui.QWidget()
        layout = QtGui.QHBoxLayout(self.widget)
        layout.addWidget(self.playButton)
        layout.addWidget(self.spinbox)
        layout.addWidget(self.slider)

        self.animationPrevTime = 0.0
        self.animationRate = 1.0
        self.animationRateTarget = 1.0
        self.animationRateAlpha = 1.0
        self.animationTimer = TimerCallback(callback=self._tick, targetFps=60)
        self.useRealTime = True

        self.callbacks = callbacks.CallbackRegistry(self.events._fields)

        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self._filterEvent)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonPress)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseMove)
        self.slider.installEventFilter(self.eventFilter)

    def _tick(self):
        if self.useRealTime:
            tnow = time.time()
            dt = tnow - self.animationPrevTime
            self.animationPrevTime = tnow
        else:
            dt = (1.0 / self.animationTimer.targetFps)

        self.animationRate = (1.0 - self.animationRateAlpha)*self.animationRate + self.animationRateAlpha*self.animationRateTarget

        valueChange = dt * self.animationRate
        value = self.getValue() + valueChange
        if value > self.maxValue:
            self.setValue(self.maxValue)
            self.playButton.setText('Play')
            return False
        self.setValue(value)

    def setAnimationRate(self, animationRate, rateAlpha=1.0):
        self.animationRateTarget = animationRate
        self.animationRateAlpha = rateAlpha

    def play(self):
        self.playButton.setText('Pause')
        self.animationPrevTime = time.time()
        self.animationTimer.start()

    def pause(self):
        self.playButton.setText('Play')
        self.animationTimer.stop()

    def _onPlayClicked(self):
        if self.animationTimer.isActive():
            self.pause()
        else:
            self.play()

    def _filterEvent(self, obj, event):
        if event.type() == QtCore.QEvent.MouseButtonPress and event.button() == QtCore.Qt.LeftButton:
            val = QtGui.QStyle.sliderValueFromPosition(obj.minimum, obj.maximum, event.x(), obj.width)
            self.eventFilter.setEventHandlerResult(True)
            obj.setValue(val)
        elif event.type() == QtCore.QEvent.MouseMove:
            val = QtGui.QStyle.sliderValueFromPosition(obj.minimum, obj.maximum, event.x(), obj.width)
            self.eventFilter.setEventHandlerResult(True)
            obj.setValue(val)

    def setResolution(self, resolution):
        with qtutils.BlockSignals(self.slider):
            self.slider.maximum = resolution
        self._syncSlider()

    def setValueRange(self, minValue, maxValue):
        newValue = np.clip(self._value, minValue, maxValue)
        changed = newValue != self._value
        self.minValue = minValue
        self.maxValue = maxValue
        self._value = newValue
        with qtutils.BlockSignals(self.spinbox):
            self.spinbox.minimum = minValue
            self.spinbox.maximum = maxValue
        self._syncSpinBox()
        self._syncSlider()
        if changed:
            self._notifyValueChanged()

    def getValue(self):
        return self._value

    def setValue(self, value):
        self._value = np.clip(value, self.minValue, self.maxValue)
        self._syncSlider()
        self._syncSpinBox()
        self._notifyValueChanged()

    def connectValueChanged(self, callback):
        return self.callbacks.connect(self.events.VALUE_CHANGED, callback)

    def _notifyValueChanged(self):
        self.callbacks.process(self.events.VALUE_CHANGED, self._value)

    def _syncSlider(self):
        with qtutils.BlockSignals(self.slider):
            self.slider.value = self.slider.maximum * (self._value - self.minValue) / float(self.maxValue - self.minValue)

    def _syncSpinBox(self):
        with qtutils.BlockSignals(self.spinbox):
            self.spinbox.value = self._value

    def _onSpinboxValueChanged(self, spinboxValue):
        self._value = spinboxValue
        self._syncSlider()
        self._notifyValueChanged()

    def _onSliderValueChanged(self, sliderValue):
        self._value = (self.minValue + (self.maxValue - self.minValue) * (sliderValue / float(self.slider.maximum)))
        self._syncSpinBox()
        self._notifyValueChanged()
