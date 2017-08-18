import time
from PythonQt import QtCore
import traceback

class TimerCallback(object):

    def __init__(self, targetFps=30, callback=None):
        '''
        Construct TimerCallback.  The targetFps defines frames per second, the
        frequency for the ticks() callback method.
        '''
        self.targetFps = targetFps
        self.timer = QtCore.QTimer()
        self.enableScheduledTimer()

        self.singleShotTimer = QtCore.QTimer()
        self.singleShotTimer.setSingleShot(True)
        self.callback = callback

    def start(self):
        '''
        Start the timer.
        '''
        if not self.timer.isActive():
            self.timer.connect('timeout()', self._timerEvent)

        self.startTime = time.time()
        self.lastTickTime = self.startTime

        if self.useScheduledTimer:
            self.timer.start(0)
        else:
            self.timer.start(int(1000.0 / self.targetFps))

    def stop(self):
        '''
        Stop the timer.
        '''
        self.timer.stop()
        self.timer.disconnect('timeout()', self._timerEvent)
        self.singleShotTimer.stop()
        self.singleShotTimer.disconnect('timeout()', self._singleShotTimerEvent)

    def tick(self):
        '''
        Timer event callback method.  Subclasses can override this method.
        '''
        if self.callback:
            return self.callback()

    def isActive(self):
        '''
        Return whether or not the timer is active.
        '''
        return self.timer.isActive()

    def enableScheduledTimer(self):
        self.useScheduledTimer = True
        self.timer.setSingleShot(True)

    def disableScheduledTimer(self):
        self.useScheduledTimer = False
        self.timer.setSingleShot(False)

    def singleShot(self, timeoutInSeconds):
        if not self.singleShotTimer.isActive():
            self.singleShotTimer.connect('timeout()', self._singleShotTimerEvent)
        self.singleShotTimer.start(int(timeoutInSeconds * 1000))

    def _singleShotTimerEvent(self):
        self.tick()
        self.singleShotTimer.disconnect('timeout()', self._singleShotTimerEvent)

    def _schedule(self, elapsedTimeInSeconds):
        '''
        This method is given an elapsed time since the start of the last
        call to ticks().  It schedules a timer event to achieve the targetFps.
        '''
        fpsDelayMilliseconds = int(1000.0 / self.targetFps)
        elapsedMilliseconds = int(elapsedTimeInSeconds*1000.0)
        waitMilliseconds = fpsDelayMilliseconds - elapsedMilliseconds
        self.timer.start(waitMilliseconds if waitMilliseconds > 0 else 1)

    def _timerEvent(self):
        '''
        Internal timer callback method.  Calls tick() and measures elapsed time.
        '''
        startTime = time.time()
        self.elapsed = startTime - self.lastTickTime

        try:
            result = self.tick()
        except:
            self.stop()
            raise

        if result is not False:
            self.lastTickTime = startTime
            if self.useScheduledTimer:
                self._schedule(time.time() - startTime)
        else:
            self.stop()

