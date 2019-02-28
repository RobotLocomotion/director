from director import callbacks
from director.flags import Flags
from director.timercallback import TimerCallback
from director import qtutils
from PythonQt import QtCore, QtGui


class ValueSlider(object):

    events = Flags('VALUE_CHANGED')

    def __init__(self, minValue=0.0, maxValue=1.0, resolution=1000):
        self.spinbox = QtGui.QDoubleSpinBox()
        self.spinbox.setSuffix('s')
        self.slider = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.setResolution(resolution)
        self.setValueRange(minValue, maxValue)
        self.slider.connect('valueChanged(int)', self._onSliderValueChanged)
        self.spinbox.connect('valueChanged(double)', self._onSpinboxValueChanged)
        self.widget = QtGui.QWidget()
        layout = QtGui.QHBoxLayout(self.widget)
        layout.addWidget(self.spinbox)
        layout.addWidget(self.slider)
        self.animationTimer = None
        self.callbacks = callbacks.CallbackRegistry(self.events._fields)

    def setResolution(self, resolution):
        self.slider.maximum = resolution

    def setValueRange(self, minValue, maxValue):
        self.minValue = minValue
        self.maxValue = maxValue
        self.spinbox.minimum = minValue
        self.spinbox.maximum = maxValue
        self.spinbox.value = minValue
        self.slider.value = 0

    def getValue(self):
        return self.spinbox.value

    def setValue(self, value):
        self.spinbox.value = np.clip(value, self.minValue, self.maxValue)

    def connectValueChanged(self, callback):
        return self.callbacks.connect(self.events.VALUE_CHANGED, callback)

    def _onSpinboxValueChanged(self, spinboxValue):
        sliderValue = self.slider.maximum * (spinboxValue - self.minValue) / float(self.maxValue - self.minValue)
        with qtutils.BlockSignals(self.slider):
            self.slider.value = sliderValue
        self.callbacks.process(self.events.VALUE_CHANGED, spinboxValue)

    def _onSliderValueChanged(self, sliderValue):
        value = (self.minValue + (self.maxValue - self.minValue) * (sliderValue / float(self.slider.maximum)))
        with qtutils.BlockSignals(self.spinbox):
            self.spinbox.value = value
        self.callbacks.process(self.events.VALUE_CHANGED, value)

    def animate(self, rate=1.0):
        if self.animationTimer:
            self.animationTimer.stop()
        tstart = time.time()
        def _tick():
            elapsed = (time.time() - tstart) * rate
            if elapsed > slider.maxValue:
                slider.spinbox.value = slider.maxValue
                return False
            slider.spinbox.value = elapsed
        self.animationTimer = TimerCallback(callback=_tick, targetFps=60)
        self.animationTimer.start()
