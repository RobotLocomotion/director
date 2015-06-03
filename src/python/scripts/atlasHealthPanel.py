from ddapp import atlasdriver
from ddapp import consoleapp
from ddapp.timercallback import TimerCallback
from PythonQt import QtCore, QtGui
from collections import namedtuple

FPS = 4
LABEL_DEFAULT_STYLE_SHEET = "font: 36pt; border-style: outset; border-width: 2px; border-color: black;"
TITLE_DEFAULT_STYLE_SHEET = "font: 24pt"
atlasDriver = atlasdriver.init()

w = QtGui.QWidget()
l = QtGui.QHBoxLayout(w)

def spawnBasicLabel(name):
  ql = QtGui.QLabel(name)
  ql.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
  qs = QtGui.QLabel('<>')
  qs.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
  ql.setStyleSheet(TITLE_DEFAULT_STYLE_SHEET)
  qs.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET)
  qs.setAlignment(QtCore.Qt.AlignHCenter)

  box = QtGui.QGroupBox(name)
  box.setAlignment(QtCore.Qt.AlignCenter) 
  boxLayout = QtGui.QVBoxLayout(box)
  boxLayout.addWidget(qs)
  l.addWidget(box)
  return qs


statusLabel = spawnBasicLabel("Behavior")
def statusUpdate():
  behavior = atlasDriver.getCurrentBehaviorName()
  statusLabel.setText(behavior)
  if (behavior != "user"):
    statusLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:red; color:white")
  else:
    statusLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:white; color:black")
statusTimer = TimerCallback(targetFps=FPS)
statusTimer.callback =  statusUpdate
statusTimer.start()

controllerLabel = spawnBasicLabel("Controller")
def controllerUpdate():
  status = atlasDriver.getControllerStatus()
  if not status:
    status = "Unknown"
  if len(status) > 5:
    status = status[0:5]
  rate = atlasDriver.getControllerRate()
  controllerLabel.setText(status + "\n" + str(rate))
  if (rate < 500):
    controllerLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:red; color:white")
  else:
    controllerLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:white; color:black")
controllerTimer = TimerCallback(targetFps=FPS)
controllerTimer.callback = controllerUpdate
controllerTimer.start()

batteryLabel = spawnBasicLabel("Battery")
def batteryUpdate():
  status = atlasDriver.getBatteryChargeRemaining()
  if not status:
    status = -1
  batteryLabel.setText(str(status) + "%")
  if (status < 10):
    batteryLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:red; color:white")
  else:
    batteryLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:white; color:black")
batteryTimer = TimerCallback(targetFps=FPS)
batteryTimer.callback = batteryUpdate
batteryTimer.start()


w.setWindowTitle('Atlas Health Panel')
w.show()
w.resize(1000,100)

consoleapp.ConsoleApp.start()
