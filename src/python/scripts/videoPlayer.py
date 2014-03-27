import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from ddapp.timercallback import TimerCallback
import ddapp.objectmodel as om
from ddapp import lcmUtils
from ddapp import cameraview
from ddapp import vtkAll as vtk
import json
import drc as lcmdrc


CAPTURE_CHANNEL = 'DECKLINK_VIDEO_CAPTURE'

def startApplication(enableQuitTimer=False):
    appInstance = QtGui.QApplication.instance()
    if enableQuitTimer:
        quitTimer = TimerCallback()
        quitTimer.callback = appInstance.quit
        quitTimer.singleShot(0.1)
    appInstance.exec_()


def addShortcut(keySequence, widget, slot):
    shortcut = QtGui.QShortcut(QtGui.QKeySequence(keySequence), widget)
    shortcut.connect('activated()', slot)


def addConsoleShortcut(widget):
    addShortcut('F8', widget, _console.show)


def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):
    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


def clearLayout(w):
    children = w.findChildren(QtGui.QWidget)
    for child in children:
        child.delete()


class FieldData(object):

    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

    def __repr__(self):
        keys = self.__dict__.keys()
        return 'FieldData(%s)' % ', '.join(['%s=%r' % (k,v) for k, v in self.__dict__.iteritems()])


class VideoPlayer(object):

    def __init__(self):

        self.loadUi()
        self.setupUi()
        self.setupImageStreams()

    def loadUi(self):

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddVideoPlayer.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)
        uifile.close()

        self.ui = WidgetDict(self.widget.children())

    def setupImageStreams(self):

        streams = [
            CAPTURE_CHANNEL,
            'VIDEO_PLAYBACK_IMAGE']

        self.imageManager = cameraview.CameraView()

        for channelName in streams:
            self.imageManager.queue.addCameraStream(channelName)
            self.imageManager.addImage(channelName)

        self.currentStream = streams[0]
        self.cameraView = cameraview.CameraImageView(self.imageManager, self.currentStream, view=self.view)
        self.cameraView.eventFilterEnabled = False
        self.view.renderWindow().GetInteractor().SetInteractorStyle(vtk.vtkInteractorStyleImage())

    def printImageDimensions(self):
        image = self.imageManager.images[self.currentStream]
        print image.GetDimensions()

    def setupUi(self):

        addConsoleShortcut(self.widget)
        _pythonManager.setupConsole(self.widget)
        addShortcut('Ctrl+Q', self.widget, self.widget.close)
        addShortcut('Ctrl+W', self.widget, self.widget.close)

        self.view = PythonQt.dd.ddQVTKWidgetView()
        self.view.orientationMarkerWidget().Off()
        self.view.renderer().SetBackground([0,0,0])
        self.view.renderer().SetBackground2([0,0,0])

        self.ui.slider.connect('valueChanged(int)', self.onSliderChanged)
        self.ui.resumeButton.connect('clicked()', self.onResumeClicked)

        self.ui.mainWidget.layout().addWidget(self.view)
        self.widget.resize(1280, 1024 + self.ui.controlsFrame.height)
        self.widget.show()
        self.ui.slider.clearFocus()

    def sendCommand(self, channel, **kwargs):
        msg = lcmdrc.atlas_behavior_command_t()
        msg.command = json.dumps(kwargs)
        lcmUtils.publish(channel, msg)

    def unwrapCommand(self, msg):
        argDict = json.loads(msg.command)
        return FieldData(**argDict)

    def onResumeClicked(self):
        self.ui.slider.setValue(self.ui.slider.maximum)
        self.cameraView.setImageName(CAPTURE_CHANNEL)
        self.sendCommand('VIDEO_PLAYBACK_CONTROL', command='resume')


    def onSliderChanged(self, sliderValue):
        self.sendCommand('VIDEO_PLAYBACK_CONTROL', command='request_frame', value=(sliderValue/float(self.ui.slider.maximum)))
        self.cameraView.setImageName('VIDEO_PLAYBACK_IMAGE')


def main():

    global videoPlayer
    videoPlayer = VideoPlayer()
    videoPlayer.widget.show()
    startApplication()


if __name__ == '__main__':
    main()
