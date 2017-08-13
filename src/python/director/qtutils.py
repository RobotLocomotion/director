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

# @ref https://stackoverflow.com/a/35000974/7829525
class BlockSignals(object):
    """
    Block signals to a given set of objects using a `with` statement.

    Example:
    @code
        class MyWidget(object):
            def __init__(self):
                text = QtGui.QLineEdit()
                text.text = '0.0'
                text.connect('returnPressed()', self.onTextUpdate)

            def onTextUpdate(self, *args):
                doSomethingIntenseAndExpensive(self.text.text)
            def somethingElse(self):
                # In some code that you wish to update the text, without doing something
                # intense and expensive:
                with BlockSignals(self.text):
                    self.text.text = "Something Else"
    @endcode

    @see QObject.blockSignals(...)
    """
    def __init__(self, *args):
        self.objects = args
    def enable(self, value):
        """
        Block signals for all widgets.
        @param value True or False, whether or not to block the signals.
        """
        for obj in self.objects:
            obj.blockSignals(value)
    def __enter__(self, *args, **kwargs):
        self.enable(True)
    def __exit__(self, *args, **kwargs):
        self.enable(False)
