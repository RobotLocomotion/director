import types
from PythonQt import QtCore, QtGui
from ddapp.timercallback import TimerCallback
from ddapp.simpletimer import SimpleTimer


class AsyncTaskQueue(object):

    class PauseException(Exception):
        pass

    def __init__(self):
        self.tasks = []
        self.timer = TimerCallback(targetFps=10)
        self.timer.callback = self.callbackLoop
        self.currentTask = None

    def start(self):
        self.timer.start()

    def stop(self):
        self.timer.stop()

    def addTask(self, task):
        self.tasks.append(task)

    def callbackLoop(self):

        #if self.currentTask:
        #    print self.currentTask

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
            self.currentTask = task
            if isinstance(result, types.GeneratorType):
                self.tasks.insert(0, result)
            else:
                self.currentTask = None

        elif isinstance(task, types.GeneratorType):
            try:
                task.next()
            except StopIteration:
                self.tasks.remove(task)
                self.currentTask = None

        return len(self.tasks)



class AsyncTask(object):

    def __init__(self):
        pass

    def getStatus(self):
        return 'none'

    def __call__(self):
        pass


class PrintTask(AsyncTask):

    def __init__(self, message):
        self.message = message

    def __call__(self):
        print self.message


class UserPromptTask(AsyncTask):

    promptsEnabled = True

    def __init__(self, message, force=False, testingValue=None):
        self.message = message
        self.force = force
        self.testingValue = testingValue

    def showDialog(self):

        self.d = QtGui.QDialog()

        buttons = QtGui.QDialogButtonBox()
        buttons.addButton('Yes', QtGui.QDialogButtonBox.AcceptRole)
        buttons.addButton('No', QtGui.QDialogButtonBox.RejectRole)
        buttons.connect('accepted()', self.d.accept)
        buttons.connect('rejected()', self.d.reject)

        l = QtGui.QVBoxLayout(self.d)
        l.addWidget(QtGui.QLabel(self.message))
        l.addWidget(buttons)

        self.d.setAttribute(QtCore.Qt.WA_QuitOnClose, False)
        self.d.show()
        self.d.raise_()
        self.d.connect('accepted()', self.onYes)
        self.d.connect('rejected()', self.onNo)

    def onYes(self):
        self.result = True

    def onNo(self):
        self.result = False

    def __call__(self):

        if not self.promptsEnabled and not self.force:
            return

        self.showDialog()

        self.result = None

        if self.testingValue is not None:
            self.d.close()
            self.result = self.testingValue

        while self.result is None:
            yield

        if not self.result:
            raise AsyncTaskQueue.PauseException()


class DelayTask(AsyncTask):

    def __init__(self, delayTimeInSeconds):
        self.delayTimeInSeconds = delayTimeInSeconds

    def __call__(self):

        t = SimpleTimer()
        while t.elapsed() < self.delayTimeInSeconds:
            yield


class PauseTask(AsyncTask):

    def __init__(self):
        pass

    def __call__(self):
        raise AsyncTaskQueue.PauseException()


class QuitTask(AsyncTask):

    def __init__(self):
        pass

    def __call__(self):
        QtCore.QCoreApplication.instance().quit()
