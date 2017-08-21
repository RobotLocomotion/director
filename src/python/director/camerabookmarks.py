from director import applogic
from director import cameracontrol

from PythonQt import QtCore, QtGui


class CameraBookmarks(object):

    def __init__(self, view):
        self.bookmarks = {}
        self.view = view
        self.flyer = cameracontrol.Flyer(view)
        self.flyer.flyTime = 1.0

    def storeCameraBookmark(self, key):
        camera = self.view.camera()
        focal, position = camera.GetFocalPoint(), camera.GetPosition()
        self.bookmarks[key] = (focal, position)

    def clear(self):
        self.bookmarks = {}

    def getBookmark(self, key):
        return self.bookmarks.get(key)

    def flyToBookmark(self, key):
        focal, position = self.getBookmark(key)
        self.flyer.zoomTo(focal, position)


class CameraBookmarkWidget(object):

    def __init__(self, view):
        self.bookmarks = CameraBookmarks(view)
        self.widget = QtGui.QScrollArea()
        self.widget.setWindowTitle('Camera Bookmarks')
        self.storeMapper = QtCore.QSignalMapper()
        self.flyMapper = QtCore.QSignalMapper()
        self.storeMapper.connect('mapped(QObject*)', self.onStoreCamera)
        self.flyMapper.connect('mapped(QObject*)', self.onFlyToCamera)
        self.numberOfBookmarks = 8
        self.updateLayout()

    def updateLayout(self):
        self.storeButtons = []
        self.flyButtons = []

        w = QtGui.QWidget()
        l = QtGui.QGridLayout(w)

        for i in range(self.numberOfBookmarks):
            storeButton = QtGui.QPushButton('set')
            flyButton = QtGui.QPushButton('fly')
            textEdit = QtGui.QLineEdit('camera %d' % i)
            storeButton.connect('clicked()', self.storeMapper, 'map()')
            flyButton.connect('clicked()', self.flyMapper, 'map()')
            self.storeMapper.setMapping(storeButton, storeButton)
            self.flyMapper.setMapping(flyButton, flyButton)
            self.storeButtons.append(storeButton)
            self.flyButtons.append(flyButton)
            l.addWidget(storeButton, i, 0)
            l.addWidget(flyButton, i, 1)
            l.addWidget(textEdit, i, 2)
            flyButton.setEnabled(False)

        self.flySpeedSpinner = QtGui.QDoubleSpinBox()
        self.flySpeedSpinner.setMinimum(0)
        self.flySpeedSpinner.setMaximum(60)
        self.flySpeedSpinner.setDecimals(1)
        self.flySpeedSpinner.setSingleStep(0.5)
        self.flySpeedSpinner.setSuffix(' seconds')
        self.flySpeedSpinner.setValue(1.0)

        l.addWidget(QtGui.QLabel('Fly speed:'), i+1, 0, 2)
        l.addWidget(self.flySpeedSpinner, i+1, 2)

        self.widget.setWidget(w)

    def onStoreCamera(self, button):
        index = self.storeButtons.index(button)
        self.bookmarks.storeCameraBookmark(index)
        self.flyButtons[index].setEnabled(True)

    def onFlyToCamera(self, button):
        index = self.flyButtons.index(button)
        self.bookmarks.flyer.flyTime = self.flySpeedSpinner.value
        self.bookmarks.flyToBookmark(index)


def init(view):
    global widget, dock
    widget = CameraBookmarkWidget(view)
    dock = applogic.addWidgetToDock(widget.widget, action=None)
    dock.hide()
