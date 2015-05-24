import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from ddapp import applogic as app
from ddapp.timercallback import TimerCallback
from functools import partial


def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)



class AtlasDriverPanel(object):

    def __init__(self, driver):

        self.driver = driver

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddAtlasDriverPanel.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)
        self.widget.setWindowTitle('Atlas Driver Panel')
        self.ui = WidgetDict(self.widget.children())

        # Main Panel
        self.ui.calibrateEncodersButton.connect('clicked()', self.onCalibrateEncoders)
        self.ui.prepButton.connect('clicked()', self.onPrep)
        self.ui.combinedStandButton.connect('clicked()', self.onCombinedStand)
        self.ui.stopButton.connect('clicked()', self.onStop)
        self.ui.freezeButton.connect('clicked()', self.onFreeze)


        self.ui.calibrateNullBiasButton.connect('clicked()', self.onCalibrateNullBias)
        self.ui.calibrateElectricArmsButton.connect('clicked()', self.onCalibrateElectricArms)
        self.ui.initNavButton.connect('clicked()', self.onInitNav)
        self.ui.standButton.connect('clicked()', self.onStand)
        self.ui.mitStandButton.connect('clicked()', self.onMITStand)
        self.ui.userButton.connect('clicked()', self.onUser)
        self.ui.manipButton.connect('clicked()', self.onManip)
        self.ui.recoveryOnButton.connect('clicked()', self.driver.sendRecoveryEnable)
        self.ui.recoveryOffButton.connect('clicked()', self.driver.sendRecoveryDisable)
        self.ui.bracingOnButton.connect('clicked()', self.driver.sendBracingEnable)
        self.ui.bracingOffButton.connect('clicked()', self.driver.sendBracingDisable)
        for psi in [1000, 1500, 2000, 2400, 2650]:
            self.ui.__dict__['setPressure' + str(psi) + 'Button'].connect('clicked()', partial(self.driver.sendDesiredPumpPsi, psi))
        self.ui.sendCustomPressureButton.connect('clicked()', self.sendCustomPressure)
        self.setupElectricArmCheckBoxes()

        PythonQt.dd.ddGroupBoxHider(self.ui.calibrationGroupBox)
        PythonQt.dd.ddGroupBoxHider(self.ui.pumpStatusGroupBox)
        PythonQt.dd.ddGroupBoxHider(self.ui.electricArmStatusGroupBox)

        self.updateTimer = TimerCallback(targetFps=5)
        self.updateTimer.callback = self.updatePanel
        self.updateTimer.start()
        self.updatePanel()

    def updatePanel(self):
        self.updateBehaviorLabel()
        self.updateControllerStatusLabel()
        self.updateRecoveryEnabledLabel()
        self.updateBracingEnabledLabel()
        self.updateBatteryStatusLabel()
        self.updateStatus()
        self.updateButtons()
        self.updateElectricArmStatus()
        self.driver.updateCombinedStandLogic()

    def updateBehaviorLabel(self):
        self.ui.behaviorLabel.text = self.driver.getCurrentBehaviorName() or '<unknown>'

    def updateControllerStatusLabel(self):
        self.ui.controllerStatusLabel.text = self.driver.getControllerStatus() or '<unknown>'

    def updateBatteryStatusLabel(self):
        charge = self.driver.getBatteryChargeRemaining()
        self.ui.batteryStatusLabel.text = '<unknown>' if charge is None else '%d%%' % charge

    def updateRecoveryEnabledLabel(self):
        self.ui.recoveryEnabledLabel.text = self.driver.getRecoveryEnabledStatus() or '<unknown>'

    def updateBracingEnabledLabel(self):
        self.ui.bracingEnabledLabel.text = self.driver.getBracingEnabledStatus() or '<unknown>'

    def updateStatus(self):
        self.ui.inletPressure.value = self.driver.getCurrentInletPressure()
        self.ui.supplyPressure.value = self.driver.getCurrentSupplyPressure()
        self.ui.returnPressure.value = self.driver.getCurrentReturnPressure()
        self.ui.airSumpPressure.value = self.driver.getCurrentAirSumpPressure()
        self.ui.pumpRpm.value =  self.driver.getCurrentPumpRpm()
        self.ui.maxActuatorPressure.value = self.driver.getMaxActuatorPressure()

    def getElectricArmCheckBoxes(self):
        return [self.ui.armCheck1,
                  self.ui.armCheck2,
                  self.ui.armCheck3,
                  self.ui.armCheck4,
                  self.ui.armCheck5,
                  self.ui.armCheck6]

    def setupElectricArmCheckBoxes(self):
        for check in self.getElectricArmCheckBoxes():
            check.connect('clicked()', self.onEnableElectricArmChecked)

    def updateElectricArmStatus(self):

        temps = [self.ui.armTemp1,
                  self.ui.armTemp2,
                  self.ui.armTemp3,
                  self.ui.armTemp4,
                  self.ui.armTemp5,
                  self.ui.armTemp6]

        for i, check in enumerate(self.getElectricArmCheckBoxes()):
            enabled = self.driver.getElectricArmEnabledStatus(i)
            check.setText('yes' if enabled else 'no')

        for i, temp in enumerate(temps):
            temp.setValue(self.driver.getElectricArmTemperature(i))

    def updateButtons(self):

        behavior = self.driver.getCurrentBehaviorName()
        behaviorIsFreeze = behavior == 'freeze'

        self.ui.calibrateNullBiasButton.setEnabled(behaviorIsFreeze)
        self.ui.calibrateElectricArmsButton.setEnabled(behaviorIsFreeze)
        self.ui.calibrateEncodersButton.setEnabled(behaviorIsFreeze)
        self.ui.prepButton.setEnabled(behaviorIsFreeze)
        self.ui.standButton.setEnabled(behavior in ('prep', 'stand', 'user', 'manip', 'step', 'walk'))
        self.ui.mitStandButton.setEnabled(behavior=='user')
        self.ui.manipButton.setEnabled(behavior in ('stand', 'manip'))
        self.ui.userButton.setEnabled(behavior is not None)

    def onEnableElectricArmChecked(self):
        enabledState = [bool(check.checked) for check in self.getElectricArmCheckBoxes()]
        self.driver.sendElectricArmEnabledState(enabledState)

    def onFreeze(self):
        self.driver.sendFreezeCommand()

    def onStop(self):
        self.driver.sendStopCommand()

    def onCalibrateEncoders(self):
        self.driver.sendCalibrateEncodersCommand()

    def onCalibrateNullBias(self):
        self.driver.sendCalibrateNullBiasCommand()

    def onCalibrateElectricArms(self):
        self.driver.sendCalibrateElectricArmsCommand()

    def onInitNav(self):
        self.driver.sendInitAtZero()

    def onPrep(self):
        self.driver.sendPrepCommand()

    def onStand(self):
        self.driver.sendStandCommand()

    def onCombinedStand(self):
        self.driver.sendCombinedStandCommand()

    def onMITStand(self):
        self.driver.sendMITStandCommand()

    def onManip(self):
        self.driver.sendManipCommand()

    def onUser(self):
        self.driver.sendUserCommand()

    def sendCustomPressure(self):
        self.driver.sendDesiredPumpPsi(self.ui.customPumpPressure.value)



def _getAction():
    return app.getToolBarActions()['ActionAtlasDriverPanel']


def init(driver):

    global panel
    global dock

    panel = AtlasDriverPanel(driver)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()


    return panel
