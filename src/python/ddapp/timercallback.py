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
        self.timer.connect('timeout()', self._timerEvent)
        self.callback = None

    def start(self):
        '''
        Start the timer.
        '''
        self.lastTickTime = time.time()
        self.timer.start(0)

    def stop(self):
        '''
        Stop the timer.
        '''
        self.timer.stop()

    def tick(self):
        '''
        Timer event callback method.  Subclasses can override this method.
        '''
        if self.callback:
            try:
                self.callback()
            except:
                print traceback.format_exc()

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
        self.tick()
        self.lastTickTime = startTime
        self._schedule(time.time() - startTime)


