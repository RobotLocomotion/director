from director import lcmUtils
from director.timercallback import TimerCallback

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

    def advanceTime(self, playLength, onFrame=None):

        numEvents = len(self.timestamps)
        if self.nextEventIndex >= numEvents:
            return

        startTimestamp = self.timestamps[self.nextEventIndex]
        endTimestamp = startTimestamp + playLength*1e6

        good = True

        while good:

            event = self.log.read_next_event()
            self.nextEventIndex += 1

            self.lcmHandle.publish(event.channel, event.data)

            good = (self.nextEventIndex < numEvents
                    and self.timestamps[self.nextEventIndex] <= endTimestamp)
            if onFrame and good:
                onFrame(self.timestamps[self.nextEventIndex] / 1.e6)

    def skipToTime(self, timeRequest, playLength=0.0):
        self.resetPlayPosition(timeRequest)
        self.advanceTime(playLength)

    def getEndTime(self):
        assert len(self.timestamps)
        return self.timestamps[-1]*1e-6

    def stop(self):
        self.timer.stop()

    def playback(self, startTime, playLength, onFrame=None, onStop=None):

        self.resetPlayPosition(startTime)

        startTimestamp = self.timestamps[self.nextEventIndex]
        endTimestamp = startTimestamp + playLength*1e6

        def onTick():
            elapsed = self.timer.elapsed * self.playbackFactor
            self.advanceTime(elapsed, onFrame)

            good = (self.nextEventIndex < len(self.timestamps)
                    and self.timestamps[self.nextEventIndex] <= endTimestamp)

            if onFrame and good:
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
        stopButton = QtGui.QPushButton('Stop')
        slider = QtGui.QSlider(QtCore.Qt.Horizontal)
        slider.maximum = int(logPlayer.getEndTime()*100)
        playButton.connect('clicked()', self.onPlay)
        stopButton.connect('clicked()', self.onStop)
        slider.connect('valueChanged(int)', self.onSlider)

        l = QtGui.QHBoxLayout(w)
        l.addWidget(slider)
        l.addWidget(playButton)
        l.addWidget(stopButton)

        self.slider = slider
        self.widget = w
        self.widget.show()

    def _getTime(self, value=None):
        if value is None:
            value = self.slider.value
        t = self.logPlayer.getEndTime()*value/self.slider.maximum
        return t

    def _getValue(self, t):
        value = t / self.logPlayer.getEndTime() * self.slider.maximum
        return int(round(value))

    def onPlay(self):
        def onFrame(t):
            with BlockSignals(self.slider):
                self.slider.value = self._getValue(t)
        self.logPlayer.playback(self._getTime(), self.logPlayer.getEndTime(), onFrame)

    def onStop(self):
        self.logPlayer.timer.stop()

    def onSlider(self, value):
        t = self._getTime(value)
        self.logPlayer.skipToTime(t, playLength=0.0)

# @ref https://stackoverflow.com/a/35000974/7829525
class BlockSignals(object):
    def __init__(self, *args):
        self.widgets = args
    def blockSignals(self, value):
        for widget in self.widgets:
            widget.blockSignals(value)
    def __enter__(self, *args, **kwargs):
        self.blockSignals(True)
    def __exit__(self, *args, **kwargs):
        self.blockSignals(False)

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
