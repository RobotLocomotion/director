import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from director import applogic as app
from director.timercallback import TimerCallback
from director.simpletimer import FPSCounter
from director import ioUtils as io
import director.vtkAll as vtk
import os
import glob
import time
import datetime
import itertools

def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)



class ScreenGrabberPanel(object):

    def __init__(self, view):

        self.view = view

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddScreenGrabber.ui')
        assert uifile.open(uifile.ReadOnly)

        self.frameCount = 0

        self.widget = loader.load(uifile)
        self.ui = WidgetDict(self.widget.children())

        self.ui.lockViewSizeCheck.connect('clicked()', self.onLockViewSize)
        self.ui.screenshotOutputBrowseButton.connect('clicked()', self.onChooseScreenshotOutputDir)
        self.ui.movieOutputBrowseButton.connect('clicked()', self.onChooseMovieOutputDir)

        self.ui.saveScreenshotButton.connect('clicked()', self.onSaveScreenshot)
        self.ui.recordMovieButton.connect('clicked()', self.onRecordMovie)

        self.ui.viewSizeCombo.connect('currentIndexChanged(const QString&)', self.updateViewSize)

        self.ui.viewHeightSpin.connect('valueChanged(int)', self.onViewSizeChanged)
        self.ui.viewWidthSpin.connect('valueChanged(int)', self.onViewSizeChanged)

        self.updateViewSize()
        self.onLockViewSize()

        self.recordTimer = QtCore.QTimer()
        self.recordTimer.connect('timeout()', self.onRecordTimer)
        self.fpsCounter = FPSCounter()

        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        self.ui.scrollArea.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.Resize)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.onEvent)


    def onEvent(self, obj, event):
        minSize = self.ui.scrollArea.widget().minimumSizeHint.width() + self.ui.scrollArea.verticalScrollBar().width
        self.ui.scrollArea.setMinimumWidth(minSize)


    def dateTimeString(self):
        return datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H:%M:%S')

    def movieOutputDirectory(self):
        return os.path.expanduser(self.ui.movieOutputDirectory.text)

    def screenshotOutputDirectory(self):
        return os.path.expanduser(self.ui.screenshotOutputDirectory.text)

    def captureRate(self):
        return self.ui.captureRateSpin.value

    def chooseDirectory(self):
        return QtGui.QFileDialog.getExistingDirectory(app.getMainWindow(), "Choose directory...", self.movieOutputDirectory())

    def ensureDirectoryIsWritable(self, dirname):

        if not os.path.isdir(dirname):

            try:
                os.makedirs(dirname)
            except OSError:
                app.showErrorMessage('Error creating directory: %s' % dirname)
                return False

        if not os.access(dirname, os.W_OK | os.X_OK):
                app.showErrorMessage('Directory is not writable: %s' % dirname)
                return False

        return True


    def onChooseScreenshotOutputDir(self):
        newDir = self.chooseDirectory()
        if newDir:
            self.ui.screenshotOutputDirectory.text = newDir

    def onChooseMovieOutputDir(self):
        newDir = self.chooseDirectory()
        if newDir:
            self.ui.movieOutputDirectory.text = newDir

    def onSaveScreenshot(self):

        outDir = self.screenshotOutputDirectory()
        if not self.ensureDirectoryIsWritable(outDir):
            return

        filename = os.path.join(outDir, 'Screenshot-' + self.dateTimeString() + '.png')
        saveScreenshot(self.view, filename)
        app.getMainWindow().statusBar().showMessage('Saved: ' + filename, 2000)


    def nextMovieFileName(self):
        filename = os.path.join(self.movieOutputDirectory(), 'frame_%07d.tiff' % self.frameCount)
        self.frameCount += 1
        return filename

    def updateRecordingStats(self):
        isRecordMode = self.ui.recordMovieButton.checked

        currentRate = 0.0
        writeQueue = 0.0

        self.ui.currentRateValueLabel.setText('%.1f' % currentRate)
        self.ui.writeQueueValueLabel.setText('%.1f' % currentRate)

    def isRecordMode(self):
        return self.ui.recordMovieButton.checked

    def updateRecordingButtons(self):

        isRecordMode = self.isRecordMode()

        self.ui.movieOutputDirectory.setEnabled(not isRecordMode)
        self.ui.movieOutputBrowseButton.setEnabled(not isRecordMode)
        self.ui.captureRateSpin.setEnabled(not isRecordMode)
        self.ui.captureRateLabel.setEnabled(not isRecordMode)
        self.ui.moveOutputDirectoryLabel.setEnabled(not isRecordMode)
        self.ui.currentRateLabel.setEnabled(isRecordMode)
        self.ui.currentRateValueLabel.setEnabled(isRecordMode)
        self.ui.writeQueueLabel.setEnabled(isRecordMode)
        self.ui.writeQueueValueLabel.setEnabled(isRecordMode)

        self.updateRecordingStats()

    def onRecordMovie(self):
        # Enforce even width number, otherwise avconv will fail
        _width = (self.view.width if self.view.width % 2 == 0 else self.view.width + 1)
        _height = (self.view.height if self.view.height % 2 == 0 else self.view.height + 1)
        self.view.setFixedSize(_width, _height)

        if self.isRecordMode():
            self.startRecording()
        else:
            self.stopRecording()

        self.updateRecordingButtons()

    def startRecording(self):

        self.frameCount = 0

        if not self.ensureDirectoryIsWritable(self.movieOutputDirectory()):
            self.ui.recordMovieButton.checked = False
            return

        existingFiles = glob.glob(os.path.join(self.movieOutputDirectory(), '*.tiff'))
        if len(existingFiles):

            choice = QtGui.QMessageBox.question(app.getMainWindow(), 'Continue?',
              'There are existing image files in the output directory.  They will be deleted prior to recording.  Continue?',
              QtGui.QMessageBox.Yes | QtGui.QMessageBox.No,
              QtGui.QMessageBox.No)

            if choice == QtGui.QMessageBox.No:
                self.ui.recordMovieButton.checked = False
                return

        for fileToRemove in existingFiles:
            os.remove(fileToRemove)

        self.fpsCounter.tick()
        self.startT = time.time()
        interval = int(round(1000.0 / self.captureRate()))

        self.recordTimer.setInterval(interval)
        self.recordTimer.start()

    def stopRecording(self):
        self.recordTimer.stop()
        if self.frameCount > 0:
            self.showEncodingDialog()

    def showEncodingDialog(self):

        msg = 'Recorded %d frames.  For encoding, use this command line:\n\n\n' % self.frameCount
        msg += '    cd "%s"\n\n' % self.movieOutputDirectory()
        msg += '    avconv -r %d -i frame_%%07d.tiff \\\n' % self.captureRate()
        msg += '           -vcodec libx264 \\\n'
        msg += '           -preset slow \\\n'
        msg += '           -crf 18 \\\n'
        msg += '           -pix_fmt yuv420p \\\n'
        msg += '           output.mp4\n\n\n'

        app.showInfoMessage(msg, title='Recording Stopped')


    def updateViewSize(self):

        current = str(self.ui.viewSizeCombo.currentText)
        useCustom = (current == 'Custom')

        self.ui.viewWidthSpin.setEnabled(useCustom)
        self.ui.viewHeightSpin.setEnabled(useCustom)

        if useCustom:
            return
        else:
            viewSize = [int(value) for value in current.split(' ')[0].split('x')]
            self.ui.viewWidthSpin.value = viewSize[0]
            self.ui.viewHeightSpin.value = viewSize[1]

    def onViewSizeChanged(self):
        self.onLockViewSize()

    def viewSize(self):
        return self.ui.viewWidthSpin.value, self.ui.viewHeightSpin.value

    def lockViewSize(self):
        self.view.setFixedSize(*self.viewSize())
        self.ui.viewSizeFrame.setEnabled(True)

    def unlockViewSize(self):
        self.ui.viewSizeFrame.setEnabled(False)
        qtwidgetMaxViewSize = 16777215
        self.view.setFixedSize(qtwidgetMaxViewSize, qtwidgetMaxViewSize)

    def onLockViewSize(self):
        if self.ui.lockViewSizeCheck.checked:
            self.lockViewSize()
        else:
            self.unlockViewSize()

    def onRecordTimer(self):

        saveScreenshot(self.view, self.nextMovieFileName(), shouldRender=False)

        self.fpsCounter.tick()
        tNow = time.time()
        if tNow - self.startT > 1.0:
            self.startT = tNow
            self.ui.currentRateValueLabel.text = '%.1f' % self.fpsCounter.getAverageFPS()


def saveScreenshot(view, filename, shouldRender=True, shouldWrite=True):

    if shouldRender:
        view.forceRender()

    grabber = vtk.vtkWindowToImageFilter()
    grabber.SetInput(view.renderWindow())
    grabber.SetInputBufferTypeToRGB()
    grabber.ReadFrontBufferOff()
    grabber.SetShouldRerender(False)
    grabber.Update()

    if shouldWrite:
        io.writeImage(grabber.GetOutput(), filename)
    return grabber.GetOutput()



def test(n=30, height=1080, aspect=16/9.0, ext='tiff', shouldRender=True, shouldWrite=True):

    view.resize(height*aspect, height)

    tStart = time.time()
    tPrev = tStart

    orbitTime = 5.0
    speed = 360.0 / orbitTime



    for i in range(n):

        tNow = time.time()
        elapsed = tNow - tPrev
        tPrev = tNow
        camera.Azimuth(elapsed * speed)

        saveScreenshot('out_%04d.%s' % (i,ext), shouldRender, shouldWrite )

    elapsed = time.time() - tStart
    print(n, 'frames')
    print('%.3f' % elapsed, 'seconds')
    print('%.2f' % (n/elapsed), 'fps')


def _getAction():
    return app.getToolBarActions()['ActionScreenGrabberPanel']


def init(view):

    global panel
    global dock

    panel = ScreenGrabberPanel(view)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel
