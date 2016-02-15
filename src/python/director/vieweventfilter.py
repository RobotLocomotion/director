import PythonQt
from PythonQt import QtCore, QtGui
import director.visualization as vis

class ViewEventFilter(object):

    def __init__(self, view):
        self.view = view
        self._leftMouseStart = None
        self._rightMouseStart = None
        self.installEventFilter()

    def installEventFilter(self):
        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.filterEvent)
        self.view.vtkWidget().installEventFilter(self.eventFilter)
        for eventType in self.getFilteredEvents():
            self.eventFilter.addFilteredEventType(eventType)

    def removeEventFilter(self):
        self.view.vtkWidget().removeEventFilter(self.eventFilter)
        self.eventFilter.disconnect('handleEvent(QObject*, QEvent*)', self.filterEvent)

    def getFilteredEvents(self):
        return [QtCore.QEvent.MouseButtonDblClick,
                QtCore.QEvent.MouseButtonPress,
                QtCore.QEvent.MouseButtonRelease,
                QtCore.QEvent.MouseMove,
                QtCore.QEvent.Wheel,
                QtCore.QEvent.KeyPress,
                QtCore.QEvent.KeyRelease]

    def getMousePositionInView(self, event):
        return vis.mapMousePosition(self.view, event)

    def getCursorDisplayPosition(self):
        cursorPos = self.view.mapFromGlobal(QtGui.QCursor.pos())
        return cursorPos.x(), self.view.height - cursorPos.y()

    def consumeEvent(self):
        self.eventFilter.setEventHandlerResult(True)

    def filterEvent(self, obj, event):

        if event.type() == QtCore.QEvent.MouseButtonDblClick and event.button() == QtCore.Qt.LeftButton:
            self.onLeftDoubleClick(event)

        elif event.type() == QtCore.QEvent.MouseButtonPress and event.button() == QtCore.Qt.LeftButton:
            self._leftMouseStart = QtCore.QPoint(event.pos())
            self.onLeftMousePress(event)

        elif event.type() == QtCore.QEvent.MouseButtonPress and event.button() == QtCore.Qt.RightButton:
            self._rightMouseStart = QtCore.QPoint(event.pos())

        elif event.type() == QtCore.QEvent.MouseMove:

            if self._rightMouseStart is None and self._leftMouseStart is None:
                self.onMouseMove(event)

            else:
                if self._rightMouseStart is not None:
                    delta = QtCore.QPoint(event.pos()) - self._rightMouseStart
                    if delta.manhattanLength() > 3:
                        self._rightMouseStart = None

                if self._leftMouseStart is not None:
                    delta = QtCore.QPoint(event.pos()) - self._leftMouseStart
                    if delta.manhattanLength() > 3:
                        self._leftMouseStart = None

        elif event.type() == QtCore.QEvent.MouseButtonRelease and event.button() == QtCore.Qt.LeftButton and self._leftMouseStart is not None:
            self._leftMouseStart = None
            self.onLeftClick(event)

        elif event.type() == QtCore.QEvent.MouseButtonRelease and event.button() == QtCore.Qt.RightButton and self._rightMouseStart is not None:
            self._rightMouseStart = None
            self.onRightClick(event)

        elif event.type() == QtCore.QEvent.Wheel:
            self.onMouseWheel(event)

        elif event.type() == QtCore.QEvent.KeyPress:
            if not event.isAutoRepeat():
                self.onKeyPress(event)
            self.onKeyPressRepeat(event)

        elif event.type() == QtCore.QEvent.KeyRelease and not event.isAutoRepeat():
            self.onKeyRelease(event)

    def onMouseWheel(self, event):
        pass

    def onMouseMove(self, event):
        pass

    def onLeftMousePress(self, event):
        pass

    def onLeftDoubleClick(self, event):
        pass

    def onLeftClick(self, event):
        pass

    def onRightClick(self, event):
        pass

    def onKeyPress(self, event):
        pass

    def onKeyPressRepeat(self, event):
        pass

    def onKeyRelease(self, event):
        pass
