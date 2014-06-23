import time
from PythonQt import QtCore
import traceback

class TimerCallback(object):

    def __init__(self, targetFps=30):
        '''
        Construct TimerCallback.  The targetFps defines frames per second, the
        frequency for the ticks() callback method.
        '''
        self.targetFps = targetFps
        self.timer = QtCore.QTimer()
        self.timer.setSingleShot(True)

        self.singleShotTimer = QtCore.QTimer()
        self.singleShotTimer.setSingleShot(True)
        self.callback = None

    def start(self):
        '''
        Start the timer.
        '''
        self.startTime = time.time()
        self.lastTickTime = self.startTime
        self.timer.connect('timeout()', self._timerEvent)
        self.timer.start(0)

    def stop(self):
        '''
        Stop the timer.
        '''
        self.timer.stop()
        self.timer.disconnect('timeout()', self._timerEvent)

    def tick(self):
        '''
        Timer event callback method.  Subclasses can override this method.
        '''
        if self.callback:
            try:
                return self.callback()
            except:
                print traceback.format_exc()
                return False

    def singleShot(self, timeoutInSeconds):
        self.singleShotTimer.connect('timeout()', self._singleShotTimerEvent)
        self.singleShotTimer.start(int(timeoutInSeconds * 1000))

    def _singleShotTimerEvent(self):
        self.tick()
        self.singleShotTimer.disconnect('timeout()', self._singleShotTimerEvent)

    def _schedule(self, elapsedTimeInSeconds):
        '''
        This method is given an elapsed time since the start of the last
        call to ticks().  It schedules a timer event to acheive the targetFps.
        '''
        fpsDelayMilliseconds = int(1000.0 / self.targetFps)
        elapsedMilliseconds = int(elapsedTimeInSeconds*1000.0)
        waitMilliseconds = fpsDelayMilliseconds - elapsedMilliseconds
        self.timer.start(waitMilliseconds if waitMilliseconds > 0 else 1)

    def _timerEvent(self):
        '''
        Internal timer callback method.  Calls ticks() and measures elapsed time.
        '''
        startTime = time.time()
        self.elapsed = startTime - self.lastTickTime
        if self.tick() is not False:
            self.lastTickTime = startTime
            self._schedule(time.time() - startTime)
        else:
            self.stop()


