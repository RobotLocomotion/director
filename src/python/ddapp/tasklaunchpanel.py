from PythonQt import QtCore, QtGui
from ddapp import applogic as app
import functools

class TaskLaunchPanel(object):

    def __init__(self, widgetMap):

        self.widget = QtGui.QWidget()
        self.widget.setWindowTitle('Task Launch Panel')
        l = QtGui.QVBoxLayout(self.widget)

        for name, widget in widgetMap.iteritems():
            b = QtGui.QPushButton(name)
            b.connect('clicked()', functools.partial(self.showWidget, widget))
            l.addWidget(b)

        l.addStretch()

    def showWidget(self, widget):

        widget.show()
        widget.raise_()
        widget.activateWindow()

        self.widget.parent().hide()


def _getAction():
    return app.getToolBarActions()['ActionTaskLauncher']


def init(widgetMap):

    global panel
    global dock

    panel = TaskLaunchPanel(widgetMap)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel
