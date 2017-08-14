from director import lcmUtils
from director.timercallback import TimerCallback
from director.qtutils import BlockSignals

import lcm
import numpy as np
from PythonQt import QtCore, QtGui


class LcmLogPlayer(object):

    def __init__(self, lcmHandle=None):
        if not lcmHandle:
            lcmHandle = lcmUtils.getGlobalLCM()
        self.lcmHandle = lcmHandle
        self.log = None
        self.filePositions = []
        self.playbackFactor = 1.0
        self.timer = TimerCallback()
        self.timestamps = np.array([])
        self.timestampOffset = 0.0

    def findEventIndex(self, timestampRequest):
        requestIndex = self.timestamps.searchsorted(timestampRequest)
        if requestIndex >= len(self.timestamps):
            requestIndex = len(self.timestamps)-1
        return requestIndex

    def resetPlayPosition(self, playTime):
        self.nextEventIndex = self.findEventIndex(playTime*1e6)
        filepos = self.filePositions[self.nextEventIndex]
        self.log.seek(filepos)

    def advanceTime(self, playLength, onFrame=None, fixedRate=None):

        numEvents = len(self.timestamps)
        if self.nextEventIndex >= numEvents:
            return

        startTimestamp = self.timestamps[self.nextEventIndex]
        endTimestamp = startTimestamp + playLength*1e6
        nextHit = startTimestamp # startTimestamp

        good = True

        while good:

            event = self.log.read_next_event()
            self.nextEventIndex += 1

            self.lcmHandle.publish(event.channel, event.data)

            good = (self.nextEventIndex < numEvents
                    and self.timestamps[self.nextEventIndex] <= endTimestamp)
            if onFrame and good:
                doFrame = False
                currentTimestamp = self.timestamps[self.nextEventIndex]
                if fixedRate is None:
                    doFrame = True
                else:
                    if currentTimestamp >= nextHit:
                        doFrame = True
                        nextHit += fixedRate * 1e6
                if doFrame:
                    onFrame(currentTimestamp / 1.e6)

    def skipToTime(self, timeRequest, playLength=0.0):
        self.resetPlayPosition(timeRequest)
        self.advanceTime(playLength)

    def getEndTime(self):
        assert len(self.timestamps)
        return self.timestamps[-1]*1e-6

    def stop(self):
        self.timer.stop()

    def playback(self, startTime, playLength, onFrame=None, onStop=None, fixedRate=None):

        self.resetPlayPosition(startTime)

        startTimestamp = self.timestamps[self.nextEventIndex]
        endTimestamp = startTimestamp + playLength*1e6

        def onTick():
            elapsed = self.timer.elapsed * self.playbackFactor
            if fixedRate != None:
                dt = fixedRate * self.playbackFactor
            else:
                dt = elapsed
            self.advanceTime(dt, onFrame, fixedRate)

            good = (self.nextEventIndex < len(self.timestamps)
                    and self.timestamps[self.nextEventIndex] <= endTimestamp)

            if onFrame and good and fixedRate is not None:
                onFrame(self.timestamps[self.nextEventIndex] / 1.e6)
            if onStop and not good:
                onStop()

            return bool(good) #convert numpy.bool to bool

        self.timer.callback = onTick
        self.timer.start()

    def readLog(self, filename, eventTimeFunction=None, progressFunction=None):
        log = lcm.EventLog(filename, 'r')
        self.log = log

        timestamps = []
        filePositions = []
        offsetIsDefined = False
        timestampOffset = 0
        lastEventTimestamp = 0
        nextProgressTime = 0.0

        while True:

            filepos = log.tell()

            event = log.read_next_event()
            if not event:
                break

            if eventTimeFunction:
                eventTimestamp = eventTimeFunction(event)
            else:
                eventTimestamp = event.timestamp

            if eventTimestamp is None:
                eventTimestamp = lastEventTimestamp
            elif not offsetIsDefined:
                timestampOffset = eventTimestamp
                offsetIsDefined = True

            lastEventTimestamp = eventTimestamp
            timestamp = eventTimestamp - timestampOffset

            if progressFunction:
                progressTime = timestamp*1e-6
                if progressTime >= nextProgressTime:
                    nextProgressTime += 1.0
                    if not progressFunction(timestamp*1e-6):
                        break

            filePositions.append(filepos)
            timestamps.append(timestamp)

        self.filePositions = filePositions
        self.timestamps = np.array(timestamps)
        self.timestampOffset = timestampOffset


class LcmLogPlayerGui(object):

    def __init__(self, logPlayer):

        self.logPlayer = logPlayer

        w = QtGui.QWidget()
        w.windowTitle = 'LCM Log Playback'

        playButton = QtGui.QPushButton('Play')
        playAndRecordButton = QtGui.QPushButton('Play + Record')
        stopButton = QtGui.QPushButton('Stop')
        slider = QtGui.QSlider(QtCore.Qt.Horizontal)
        slider.maximum = int(logPlayer.getEndTime()*100)
        text = QtGui.QLineEdit()
        text.text = '0.0'
        playButton.connect('clicked()', self.onPlay)
        playAndRecordButton.connect('clicked()', self.onPlayAndRecord)
        stopButton.connect('clicked()', self.onStop)
        slider.connect('valueChanged(int)', self.onSlider)
        text.connect('returnPressed()', self.onText)

        l = QtGui.QHBoxLayout(w)
        l.addWidget(slider)
        l.addWidget(text)
        l.addWidget(playButton)
        l.addWidget(playAndRecordButton)
        l.addWidget(stopButton)

        self.slider = slider
        self.text = text
        self.widget = w
        self.widget.show()

    def _getSliderTime(self, value=None):
        if value is None:
            value = self.slider.value
        t = self.logPlayer.getEndTime()*value/self.slider.maximum
        return t

    def _getSliderValue(self, t):
        value = t / self.logPlayer.getEndTime() * self.slider.maximum
        return int(round(value))

    def _updateTime(self, t):
        with BlockSignals(self.slider, self.text):
            self.slider.value = self._getSliderValue(t)
            self.text.text = str(t)

    def onPlay(self, onStop=None, onFrame=None, fixedRate=None):
        def onFrameExt(t):
            self._updateTime(t)
            if onFrame:
                onFrame(t)
        self.logPlayer.playback(self._getSliderTime(),
            self.logPlayer.getEndTime(), onFrame=onFrameExt, onStop=onStop, fixedRate=fixedRate)

    def onPlayAndRecord(self):
        from director.screengrabberpanel import ScreenGrabberPanel
        recorder = getattr(ScreenGrabberPanel, 'instance', None)
        if recorder is not None:
            fixedRate = 2 * 1. / recorder.captureRate()
            # HACK(eric.cousineau): Tunnel directly through.
            with BlockSignals(recorder.ui.recordMovieButton):
                recorder.ui.recordMovieButton.click()
            # This should block the thread if needed.
            recorder.startRecording(useTimer=False)
            def onFrame(t):
                recorder.onRecordTimer()
            def onStop():
                recorder.stopRecording(useTimer=False)
            self.onPlay(onFrame = onFrame, onStop = onStop, fixedRate = fixedRate)
        else:
            print "No screen grabber found"

    def onStop(self):
        self.logPlayer.timer.stop()

    def skipTo(self, t):
        self._updateTime(t)
        self.logPlayer.skipToTime(t, playLength=0.0)

    def onSlider(self, value):
        t = self._getSliderTime(value)
        self.skipTo(t)

    def onText(self, *args):
        try:
            t = float(self.text.text)
            self.skipTo(t)
        except ValueError:
            pass

if __name__ == '__main__':

    import sys
    from director import consoleapp

    if len(sys.argv) < 2:
        print 'usage: %s <lcm log file>' % sys.argv[0]
        sys.exit(1)

    filename = sys.argv[1]
    print 'reading', filename

    logPlayer = LcmLogPlayer()
    logPlayer.readLog(filename)

    gui = LcmLogPlayerGui(logPlayer)
    consoleapp.ConsoleApp.showPythonConsole()
    consoleapp.ConsoleApp.start()
