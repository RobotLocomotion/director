from PythonQt import QtCore, QtUiTools


def addWidgetsToDict(widgets, d):
    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):
    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


class UiPanel(object):

    def __init__(self, filename, windowTitle=None):

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/' + filename)
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)
        self.ui = WidgetDict(self.widget.children())
        if windowTitle is not None:
            self.widget.setWindowTitle(windowTitle)
