import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from ddapp import applogic as app
from ddapp.timercallback import TimerCallback


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


        self.ui.calibrateBdiButton.connect('clicked()', self.onCalibrateBdi)
        self.ui.initNavButton.connect('clicked()', self.onInitNav)
        self.ui.standButton.connect('clicked()', self.onStand)
        self.ui.mitStandButton.connect('clicked()', self.onMITStand)
        self.ui.userButton.connect('clicked()', self.onUser)
        self.ui.manipButton.connect('clicked()', self.onManip)



        PythonQt.dd.ddGroupBoxHider(self.ui.calibrationGroupBox)
        PythonQt.dd.ddGroupBoxHider(self.ui.statusGroupBox)

        self.updateTimer = TimerCallback(targetFps=5)
        self.updateTimer.callback = self.updatePanel
        self.updateTimer.start()
        self.updatePanel()

    def updatePanel(self):
        self.updateBehaviorLabel()
        self.updateControllerStatusLabel()
        self.updateStatus()
        self.updateButtons()
        self.driver.updateCombinedStandLogic()

    def updateBehaviorLabel(self):
        self.ui.behaviorLabel.text = self.driver.getCurrentBehaviorName() or '<unknown>'

    def updateControllerStatusLabel(self):
        self.ui.controllerStatusLabel.text = self.driver.getControllerStatus() or '<unknown>'

    def updateStatus(self):
        self.ui.inletPressure.value = self.driver.getCurrentInletPressure()
        self.ui.supplyPressure.value = self.driver.getCurrentSupplyPressure()
        self.ui.returnPressure.value = self.driver.getCurrentReturnPressure()
        self.ui.airSumpPressure.value = self.driver.getCurrentAirSumpPressure()
        self.ui.pumpRpm.value =  self.driver.getCurrentPumpRpm()

    def updateButtons(self):

        behavior = self.driver.getCurrentBehaviorName()
        behaviorIsFreeze = behavior == 'freeze'

        self.ui.calibrateBdiButton.setEnabled(behaviorIsFreeze)
        self.ui.calibrateEncodersButton.setEnabled(behaviorIsFreeze)
        self.ui.prepButton.setEnabled(behaviorIsFreeze)
        self.ui.standButton.setEnabled(behavior in ('prep', 'stand', 'user', 'manip', 'step', 'walk'))
        self.ui.mitStandButton.setEnabled(behavior=='user')
        self.ui.manipButton.setEnabled(behavior in ('stand', 'manip'))
        self.ui.userButton.setEnabled(behavior is not None)

    def onFreeze(self):
        self.driver.sendFreezeCommand()

    def onStop(self):
        self.driver.sendStopCommand()

    def onCalibrateEncoders(self):
        self.driver.sendCalibrateEncodersCommand()

    def onCalibrateBdi(self):
        self.driver.sendCalibrateCommand()

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



def _getAction():
    return app.getToolBarActions()['ActionAtlasDriverPanel']


def init(driver):

    global panel
    global dock

    panel = AtlasDriverPanel(driver)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()


    return panel
