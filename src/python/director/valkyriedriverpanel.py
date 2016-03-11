import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from director import applogic as app
from director.timercallback import TimerCallback
from functools import partial


def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)



class ValkyrieDriverPanel(object):

    def __init__(self, driver):

        self.driver = driver

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddValkyrieDriverPanel.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)
        self.widget.setWindowTitle('Valkyrie Driver Panel')
        self.ui = WidgetDict(self.widget.children())

        # Main Panel
        self.ui.sendButton.connect('clicked()', self.onSendWholeBodyCommand)
        self.ui.modeBox.connect('currentIndexChanged(const QString&)', self.onModeBox)
        self.wholeBodyMode='Whole Body'

    def getModeInt(self, inputStr):
        if inputStr == 'Whole Body':
            return 0
        if inputStr == 'Left Arm':
            return 1
        if inputStr == 'Right Arm':
            return 2
        if inputStr == 'Both Arms':
            return 3
        return 0

    def onSendWholeBodyCommand(self):
        self.driver.sendWholeBodyCommand( self.getModeInt(self.wholeBodyMode) )

    def getComboText(self, combo):
        return str(combo.currentText)

    def onModeBox(self):
        self.wholeBodyMode = self.getComboText(self.ui.modeBox)

def _getAction():
    return app.getToolBarActions()['ActionValkyrieDriverPanel']

def init(driver):

    global panel
    global dock

    panel = ValkyrieDriverPanel(driver)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel
