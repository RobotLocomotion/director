import types
import traceback
from PythonQt import QtCore, QtGui
from ddapp.timercallback import TimerCallback
from ddapp.simpletimer import SimpleTimer
from ddapp import callbacks


class AsyncTaskQueue(object):

    QUEUE_STARTED_SIGNAL = 'QUEUE_STARTED_SIGNAL'
    QUEUE_STOPPED_SIGNAL = 'QUEUE_STOPPED_SIGNAL'
    TASK_STARTED_SIGNAL = 'TASK_STARTED_SIGNAL'
    TASK_ENDED_SIGNAL = 'TASK_ENDED_SIGNAL'
    TASK_PAUSED_SIGNAL = 'TASK_PAUSED_SIGNAL'
    TASK_FAILED_SIGNAL = 'TASK_FAILED_SIGNAL'
    TASK_EXCEPTION_SIGNAL = 'TASK_EXCEPTION_SIGNAL'

    class PauseException(Exception):
        pass

    class FailException(Exception):
        pass

    def __init__(self):
        self.tasks = []
        self.generators = []
        self.timer = TimerCallback(targetFps=10)
        self.timer.callback = self.callbackLoop
        self.callbacks = callbacks.CallbackRegistry([self.QUEUE_STARTED_SIGNAL,
                                                     self.QUEUE_STOPPED_SIGNAL,
                                                     self.TASK_STARTED_SIGNAL,
                                                     self.TASK_ENDED_SIGNAL,
                                                     self.TASK_PAUSED_SIGNAL,
                                                     self.TASK_FAILED_SIGNAL,
                                                     self.TASK_EXCEPTION_SIGNAL])
        self.currentTask = None
        self.isRunning = False

    def reset(self):
        assert not self.isRunning
        assert not self.generators
        self.tasks = []

    def start(self):
        self.isRunning = True
        self.callbacks.process(self.QUEUE_STARTED_SIGNAL, self)
        self.timer.start()

    def stop(self):
        self.isRunning = False
        self.currentTask = None
        self.generators = []
        self.timer.stop()
        self.callbacks.process(self.QUEUE_STOPPED_SIGNAL, self)

    def wrapGenerator(self, generator):
        def generatorWrapper():
            return generator
        return generatorWrapper

    def addTask(self, task):

        if isinstance(task, types.GeneratorType):
            task = self.wrapGenerator(task)

        assert hasattr(task, '__call__')
        self.tasks.append(task)

    def callbackLoop(self):

        try:
            for i in xrange(10):
                self.doWork()
            if not self.tasks:
                self.stop()

        except AsyncTaskQueue.PauseException:
            assert self.currentTask
            self.callbacks.process(self.TASK_PAUSED_SIGNAL, self, self.currentTask)
            self.stop()
        except AsyncTaskQueue.FailException:
            assert self.currentTask
            self.callbacks.process(self.TASK_FAILED_SIGNAL, self, self.currentTask)
            self.stop()
        except:
            assert self.currentTask
            self.callbacks.process(self.TASK_EXCEPTION_SIGNAL, self, self.currentTask)
            self.stop()
            raise

        return self.isRunning

    def popTask(self):
        assert not self.isRunning
        assert not self.currentTask
        if self.tasks:
            self.tasks.pop(0)

    def completePreviousTask(self):
        assert self.currentTask
        self.tasks.remove(self.currentTask)
        self.callbacks.process(self.TASK_ENDED_SIGNAL, self, self.currentTask)
        self.currentTask = None

    def startNextTask(self):
        self.currentTask = self.tasks[0]
        self.callbacks.process(self.TASK_STARTED_SIGNAL, self, self.currentTask)
        result = self.currentTask()
        if isinstance(result, types.GeneratorType):
            self.generators.insert(0, result)

    def doWork(self):
        if self.generators:
            self.handleGenerator(self.generators[0])
        else:
            if self.currentTask:
                self.completePreviousTask()
            if self.tasks:
                self.startNextTask()

    def handleGenerator(self, generator):
        try:
            result = generator.next()
        except StopIteration:
            self.generators.remove(generator)
        else:
            if isinstance(result, types.GeneratorType):
                self.generators.insert(0, result)

    def connectQueueStarted(self, func):
        return self.callbacks.connect(self.QUEUE_STARTED_SIGNAL, func)

    def disconnectQueueStarted(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def connectQueueStopped(self, func):
        return self.callbacks.connect(self.QUEUE_STOPPED_SIGNAL, func)

    def disconnectQueueStopped(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def connectTaskStarted(self, func):
        return self.callbacks.connect(self.TASK_STARTED_SIGNAL, func)

    def disconnectTaskStarted(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def connectTaskEnded(self, func):
        return self.callbacks.connect(self.TASK_ENDED_SIGNAL, func)

    def disconnectTaskEnded(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def connectTaskPaused(self, func):
        return self.callbacks.connect(self.TASK_PAUSED_SIGNAL, func)

    def disconnectTaskPaused(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def connectTaskFailed(self, func):
        return self.callbacks.connect(self.TASK_FAILED_SIGNAL, func)

    def disconnectTaskFailed(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def connectTaskException(self, func):
        return self.callbacks.connect(self.TASK_EXCEPTION_SIGNAL, func)

    def disconnectTaskException(self, callbackId):
        self.callbacks.disconnect(callbackId)


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
