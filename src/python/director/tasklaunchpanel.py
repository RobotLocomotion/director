from PythonQt import QtCore, QtGui
from ddapp import applogic as app
import functools

class TaskLaunchPanel(object):

    def __init__(self, widgetMap):

        self.widget = QtGui.QTabWidget()
        self.widget.setWindowTitle('Task Panel')

        for name, widget in widgetMap.iteritems():
            self.widget.addTab(widget, name)


    def showTaskPanel(self):

        widget = self.widget
        widget.show()
        widget.raise_()
        widget.activateWindow()


def _getAction():
    return app.getToolBarActions()['ActionTaskLauncher']


def init(widgetMap):

    global panel

    panel = TaskLaunchPanel(widgetMap)
    _getAction().connect('triggered()', panel.showTaskPanel)

    return panel
