from PythonQt import QtCore, QtGui
from director import applogic as app
import functools

class TaskLaunchPanel(object):

    def __init__(self, widgetMap):

        self.widget = QtGui.QTabWidget()
        self.widget.setWindowTitle('Task Panel')

        for name, widget in widgetMap.items():
            self.addTaskPanel(name, widget)

    def getTaskPanelNames(self):
        return [self.widget.tabText(i) for i in range(self.widget.count)]

    def removeTaskPanel(self, taskPanelName):
        names = self.getTaskPanelNames()
        assert taskPanelName in names
        self.widget.removeTab(names.index(taskPanelName))

    def clear(self):
        self.widget.clear()

    def addTaskPanel(self, taskPanelName, taskPanelWidget):
        self.widget.addTab(taskPanelWidget, taskPanelName)

    def showTaskLaunchPanel(self):

        widget = self.widget
        widget.show()
        widget.raise_()
        widget.activateWindow()


def _getAction():
    return app.getToolBarActions()['ActionTaskLauncher']


def init(widgetMap):

    global panel

    panel = TaskLaunchPanel(widgetMap)
    _getAction().connect('triggered()', panel.showTaskLaunchPanel)

    return panel
