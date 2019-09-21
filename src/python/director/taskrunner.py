import sys
import time
import sys
from threading import Thread

from director import asynctaskqueue
from director.timercallback import TimerCallback


class TaskRunner(object):

    def __init__(self):
        self.interval = 1/60.0
        sys.setcheckinterval(1000)
        try:
            sys.setswitchinterval(self.interval)
        except AttributeError:
            # sys.setswitchinterval is only python3
            pass

        self.taskQueue = asynctaskqueue.AsyncTaskQueue()
        self.pendingTasks = []
        self.threads = []
        self.timer = TimerCallback(callback=self._onTimer, targetFps=1/self.interval)

        # call timer.start here to initialize the QTimer now on the main thread
        self.timer.start()

    def _onTimer(self):

        # add all tasks in self.pendingTasks to the AsyncTaskQueue
        if self.pendingTasks:
            while True:
                try:
                    self.taskQueue.addTask(self.pendingTasks.pop(0))
                except IndexError:
                    break

            # start the AsyncTaskQueue if it's not already running
            if self.taskQueue.tasks and not self.taskQueue.isRunning:
                self.taskQueue.start()

        # only retain the live threads
        self.threads = [t for t in self.threads if t.is_alive()]

        if self.threads:
            # Give up control to other python threads that are running
            time.sleep(self.interval)

    def callOnMain(self, func, *args, **kwargs):
        self.pendingTasks.append(lambda: func(*args, **kwargs))

    def callOnThread(self, func, *args, **kwargs):
        t = Thread(target=lambda: func(*args, **kwargs))
        self.threads.append(t)
        t.start()
        return t
