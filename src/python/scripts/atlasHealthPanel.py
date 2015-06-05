from ddapp.consoleapp import ConsoleApp
from ddapp import atlasdriver
from ddapp import consoleapp
from ddapp.timercallback import TimerCallback
from ddapp import robotsystem
from PythonQt import QtCore, QtGui
from collections import namedtuple
import time, math

FPS = 4
LABEL_DEFAULT_STYLE_SHEET = "font: 36pt; border-style: outset; border-width: 2px; border-color: black;"
TITLE_DEFAULT_STYLE_SHEET = "font: 24pt"
atlasDriver = atlasdriver.init()

app = ConsoleApp()
app.setupGlobals(globals())
view = app.createView()
robotsystem.create(view, globals())

w = QtGui.QWidget()
l = QtGui.QHBoxLayout(w)


def wrapInVTitledItem(name, items):
  box = QtGui.QGroupBox(name)
  box.setAlignment(QtCore.Qt.AlignCenter) 
  boxLayout = QtGui.QVBoxLayout(box)
  for item in items:
    boxLayout.addWidget(item)
  return box
def wrapInHTitledItem(name, items):
  box = QtGui.QGroupBox(name)
  box.setAlignment(QtCore.Qt.AlignCenter) 
  boxLayout = QtGui.QHBoxLayout(box)
  for item in items:
    boxLayout.addWidget(item)
  return box
def spawnBasicLabel(txt = "<>"):
  qs = QtGui.QLabel(txt)
  qs.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
  qs.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET)
  qs.setAlignment(QtCore.Qt.AlignHCenter)
  return qs
def spawnBasicButton(txt = "<>"):
  qs = QtGui.QPushButton(txt)
  qs.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
  qs.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET)
  return qs
def spawnBasicTextEntry():
  qs = QtGui.QLineEdit('')
  qs.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
  qs.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET)
  return qs
def spawnProgressBar(max, min):
  ql = QtGui.QProgressBar()
  ql.setMaximum(max)
  ql.setMinimum(min)
  ql.setOrientation(QtCore.Qt.Vertical)
  return ql

statusLabel = spawnBasicLabel()
l.addWidget(wrapInVTitledItem("Behavior", [statusLabel]))

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

controllerLabel = spawnBasicLabel()
l.addWidget(wrapInVTitledItem("Controller", [controllerLabel]))
def controllerUpdate():
  status = atlasDriver.getControllerStatus()
  if not status:
    status = "Unknown"
  if len(status) > 6:
    status = status[0:6]
  rate = atlasDriver.getControllerRate()
  if not rate:
    rate = 0.0
  controllerLabel.setText("%s\n%06.1fhz" % (status, rate))
  if (rate < 500):
    controllerLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:red; color:white")
  else:
    controllerLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:white; color:black")
controllerTimer = TimerCallback(targetFps=FPS)
controllerTimer.callback = controllerUpdate
controllerTimer.start()
 
recoveryLabel = spawnBasicLabel()
bracingLabel = spawnBasicLabel()
l.addWidget(wrapInVTitledItem("Unstoppable", [recoveryLabel, bracingLabel]))
def unstoppableUpdate():
  recovery = atlasDriver.getRecoveryEnabledStatus()
  bracing = atlasDriver.getBracingEnabledStatus()
  if not recovery:
    recovery = "unknown"
  if not bracing:
    bracing = "unknown"
  
  if recovery == "enabled":
    recoveryLabel.setText("Rec On")
    recoveryLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:white; color:black")
  else:
    recoveryLabel.setText("Rec Off")
    recoveryLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:orange; color:white")

  if bracing == "enabled":
    bracingLabel.setText("Brc On")
    bracingLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:white; color:black")
  else:
    bracingLabel.setText("Brc Off")
    bracingLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:orange; color:white")

unstoppableTimer = TimerCallback(targetFps=FPS)
unstoppableTimer.callback = unstoppableUpdate
unstoppableTimer.start()


batteryLabel = spawnProgressBar(100, 0)
batteryVoltageLabel = spawnBasicLabel()
batteryTempLabel = spawnBasicLabel()
batteryTVBox = QtGui.QWidget()
batteryTVBoxLayout = QtGui.QVBoxLayout(batteryTVBox)
batteryTVBoxLayout.addWidget(batteryVoltageLabel)
batteryTVBoxLayout.addWidget(batteryTempLabel)
l.addWidget(wrapInHTitledItem("Battery", [batteryLabel, batteryTVBox]))
def batteryUpdate():
  percent = atlasDriver.getBatteryChargeRemaining()
  voltage = atlasDriver.getBatteryVoltage()
  temp = atlasDriver.getBatteryTemperature()
  if not percent:
    percent = 0
  if not voltage:
    voltage = 0
  if not temp:
    temp = 100
  batteryLabel.setValue(percent)
  batteryVoltageLabel.setText("%3.1fV" % voltage)
  if (voltage < 160):
    batteryVoltageLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:orange; color:white")
  elif (voltage < 150):
    batteryVoltageLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:red; color:white")
  else:
    batteryVoltageLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:white; color:black")
  batteryTempLabel.setText("%2.1f C" % temp)
  if (temp > 50):
    batteryTempLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:orange; color:white")
  elif (temp > 55):
    batteryTempLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:red; color:white")
  else:
    batteryTempLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:white; color:black")

batteryTimer = TimerCallback(targetFps=FPS)
batteryTimer.callback = batteryUpdate
batteryTimer.start()

spindleLabel = spawnBasicLabel()
l.addWidget(wrapInHTitledItem("Spindle", [spindleLabel]))
def spindleUpdate():
  rate = spindleMonitor.getAverageSpindleVelocity()
  if abs(rate) < 0.2:
    spindleLabel.setText('Stop')
    spindleLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:red; color:white")
  else:
    spindleLabel.setText('Spin')
    spindleLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:white; color:black")
spindleTimer = TimerCallback(targetFps=FPS)
spindleTimer.callback = spindleUpdate
spindleTimer.start()


pressureLabel = spawnBasicLabel()
l.addWidget(wrapInVTitledItem("Pressure", [pressureLabel]))
def pressureUpdate():
  supplyp = atlasDriver.getCurrentSupplyPressure()
  #returnp = atlasDriver.getCurrentReturnPressure()
  pressureLabel.setText("%06.1f\npsi" % supplyp) # + "\n(" + str(returnp) + ")")
  if (supplyp < 1450):
    pressureLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:red; color:white")
  elif (supplyp < 1950):
    pressureLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:orange; color:white")
  elif (supplyp > 2050):
    pressureLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:orange; color:white")
  else:
    pressureLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:white; color:black")
pressureTimer = TimerCallback(targetFps=FPS)
pressureTimer.callback = pressureUpdate
pressureTimer.start()

luwy = spawnProgressBar(100, 60)
lmwx = spawnProgressBar(100, 60)
llwy = spawnProgressBar(100, 60)
ruwy = spawnProgressBar(100, 60)
rmwx = spawnProgressBar(100, 60)
rlwy = spawnProgressBar(100, 60)
arms = [luwy, lmwx, llwy, ruwy, rmwx, rlwy]
l.addWidget(wrapInHTitledItem("Arm Temp", arms))
def armTempUpdate():
  for i in range(6):
    temp = atlasDriver.getElectricArmTemperature(i)
    arms[i].setValue(temp)

armTempTimer = TimerCallback(targetFps=FPS)
armTempTimer.callback = armTempUpdate
armTempTimer.start()

lHandLabel = spawnBasicLabel()
l.addWidget(wrapInHTitledItem("Left Hand", [lHandLabel]))
def lHandUpdate():
  state = robotStateJointController.lastRobotStateMessage
  if (state):
    status = atlasDriver.getControllerStatus()
    isSafeStatus = (status == "standing" or status == "manipulating")
    index = robotStateJointController.lastRobotStateMessage.joint_name.index('left_finger_1_joint_1')
    lHandOpen = state.joint_position[index]
    if (lHandOpen < 0.8 and isSafeStatus):
      lHandLabel.setText("Open\n(%02.1f)" % lHandOpen)
      lHandLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:orange; color:white")
    elif (lHandOpen < 0.8 and not isSafeStatus):
      lHandLabel.setText("Open\n(%02.1f)" % lHandOpen)
      lHandLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:red; color:white")
    else:
      lHandLabel.setText("Closed\n(%02.1f)" % lHandOpen)
      lHandLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:white; color:black")
lHandTimer = TimerCallback(targetFps=FPS)
lHandTimer.callback = lHandUpdate
lHandTimer.start()

rHandLabel = spawnBasicLabel()
l.addWidget(wrapInHTitledItem("Right Hand", [rHandLabel]))
def rHandUpdate():
  state = robotStateJointController.lastRobotStateMessage
  if (state):
    status = atlasDriver.getControllerStatus()
    isSafeStatus = (status == "standing" or status == "manipulating")
    index = robotStateJointController.lastRobotStateMessage.joint_name.index('right_finger_1_joint_1')
    rHandOpen = state.joint_position[index]
    if (rHandOpen < 0.8 and isSafeStatus):
      rHandLabel.setText("Open\n(%02.1f)" % rHandOpen)
      rHandLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:orange; color:white")
    elif (rHandOpen < 0.8 and not isSafeStatus):
      rHandLabel.setText("Open\n(%02.1f)" % rHandOpen)
      rHandLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:red; color:white")
    else:
      rHandLabel.setText("Closed\n(%02.1f)" % rHandOpen)
      rHandLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:white; color:black")
rHandTimer = TimerCallback(targetFps=FPS)
rHandTimer.callback = rHandUpdate
rHandTimer.start()

globalTimerLabel = spawnBasicLabel()
globalTimerSetBtn = spawnBasicButton("Set")
globalTimerSetTxt = spawnBasicTextEntry()
globalTimerAdjustBox = QtGui.QWidget()
globalTimerAdjustBoxLayout = QtGui.QHBoxLayout(globalTimerAdjustBox)
globalTimerAdjustBoxLayout.addWidget(globalTimerSetBtn)
globalTimerAdjustBoxLayout.addWidget(globalTimerSetTxt)
l.addWidget(wrapInVTitledItem("Run Timer", [globalTimerLabel, globalTimerAdjustBox]))
globalTimerStartTime = time.time()
globalTimerOffset = 0
def globalTimerUpdate():
  elapsed = time.time() - globalTimerStartTime + globalTimerOffset
  globalTimerLabel.setText("%02d:%02d" % (math.floor(elapsed/60), elapsed%60))
  if elapsed > (60*53):
      globalTimerLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:red; color:white")
  elif elapsed > (60*45):
      globalTimerLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:orange; color:white")
  else:
      globalTimerLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:white; color:black")

def globalTimerSet():
  global globalTimerOffset
  nums = [int(i) for i in str(globalTimerSetTxt.text).split(':')]
  if len(nums) == 1:
    # desired time = time.time() - globalTimerStartTime - offset
    # offset = des - (time.time - globaltimerstarttime)
    if nums[0] >= 0:
      globalTimerOffset = nums[0] - (time.time() - globalTimerStartTime)
      globalTimerSetTxt.setText('')
  elif len(nums) == 2:
    if nums[0] >= 0 and nums[1] >= 0 and nums[1] < 60:
      globalTimerOffset = (nums[0]*60 + nums[1]) - (time.time() - globalTimerStartTime)
      globalTimerSetTxt.setText('')
globalTimerTimer = TimerCallback(targetFps=FPS)
globalTimerTimer.callback = globalTimerUpdate
globalTimerTimer.start()
globalTimerSetBtn.connect('clicked()', globalTimerSet)


drillTimerLabel = spawnBasicLabel()
drillTimerShowBtn = spawnBasicButton("Start")
drillTimerHideBtn = spawnBasicButton("Done")
drillTimerLabel.hide()
drillTimerHideBtn.hide()
l.addWidget(wrapInVTitledItem("Drill Timer", [drillTimerShowBtn, drillTimerLabel, drillTimerHideBtn]))
drillTimerStartTime = time.time()
drillTimerOffset = 0
def drillTimerUpdate():
  elapsed = time.time() - drillTimerStartTime + drillTimerOffset
  drillTimerLabel.setText("%02d:%02d" % (math.floor(elapsed/60), elapsed%60))
  if elapsed > (60*5):
      drillTimerLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:red; color:white")
  elif elapsed > (60*4):
      drillTimerLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:orange; color:white")
  else:
      drillTimerLabel.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET + "background-color:white; color:black")


def drillTimerShow():
  global drillTimerOffset
  drillTimerLabel.show()
  drillTimerShowBtn.hide()
  drillTimerHideBtn.show()
  drillTimerOffset = - (time.time() - drillTimerStartTime)
def drillTimerHide():
  drillTimerLabel.hide()
  drillTimerShowBtn.show()
  drillTimerHideBtn.hide()

drillTimerTimer = TimerCallback(targetFps=FPS)
drillTimerTimer.callback = drillTimerUpdate
drillTimerTimer.start()
drillTimerShowBtn.connect('clicked()', drillTimerShow)
drillTimerHideBtn.connect('clicked()', drillTimerHide)

w.setWindowTitle('Atlas Health Panel')
w.show()
w.resize(1000,100)

app.start()
