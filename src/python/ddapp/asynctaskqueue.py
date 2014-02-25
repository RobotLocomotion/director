import types
from ddapp.timercallback import TimerCallback



class AsyncTaskQueue(object):

    class PauseException(Exception):
        pass

    def __init__(self):
        self.tasks = []
        self.timer = TimerCallback()
        self.timer.callback = self.callbackLoop

    def start(self):
        self.timer.start()

    def stop(self):
        self.timer.stop()

    def addTask(self, task):
        self.tasks.append(task)

    def callbackLoop(self):

        for i in xrange(10):
            if not self.tasks:
                break
            task = self.tasks[0]

            try:
                self.handleAsyncTask(task)
            except AsyncTaskQueue.PauseException:
                print 'pausing task queue'
                return False

        return len(self.tasks)

    def handleAsyncTask(self, task):

        if hasattr(task, '__call__'):
            self.tasks.remove(task)
            result = task()
            if isinstance(result, types.GeneratorType):
                self.tasks.insert(0, result)
        elif isinstance(task, types.GeneratorType):
            try:
                task.next()
            except StopIteration:
                self.tasks.remove(task)

        return len(self.tasks)
