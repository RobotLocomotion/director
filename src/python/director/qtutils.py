import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools


def addWidgetsToDict(widgets, d):
    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)

class WidgetDict(object):
    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)

def clearLayout(widget):
    children = widget.findChildren(QtGui.QWidget)
    for child in children:
        child.delete()

def loadUi(filename):

    loader = QtUiTools.QUiLoader()
    uifile = QtCore.QFile(filename)
    assert uifile.open(uifile.ReadOnly)

    widget = loader.load(uifile)
    ui = WidgetDict(widget.children())
    return widget, ui
