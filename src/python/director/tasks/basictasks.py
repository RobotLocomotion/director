import copy
import inspect
import re

from director import asynctaskqueue as atq
from director import propertyset
from director.simpletimer import SimpleTimer

from PythonQt import QtCore
from PythonQt import QtGui

def _splitCamelCase(name):
    name = re.sub('(.)([A-Z][a-z]+)', r'\1 \2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1 \2', name)


class AsyncTask(object):
    '''
    AsyncTask documentation.
    '''

    def __init__(self, **kwargs):
        self.statusMessage = ''
        self.failReason = ''
        self.properties = propertyset.PropertySet()
        self.properties.addProperty('Name', _splitCamelCase(self.__class__.__name__).lower())

        for cls in reversed(inspect.getmro(self.__class__)):
            if hasattr(cls, 'getDefaultProperties'):
                cls.getDefaultProperties(self.properties)

        for name, value in kwargs.iteritems():
            self.properties.setProperty(_splitCamelCase(name).capitalize(), value)


    def __call__(self):
        return self.run()

    def stop(self):
        pass

    def run(self):
        pass

    def fail(self, reason):
        self.failReason = reason
        raise atq.AsyncTaskQueue.FailException(reason)

    def copy(self):
        return copy.deepcopy(self)


class PrintTask(AsyncTask):
    '''
    Name: Print Task
    Short Description: prints a string
    Description:

    This task prints a message string.
    '''

    printFunction = None

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Message', '<empty message>')

    def run(self):
        if self.printFunction:
            self.printFunction(self.properties.message)
        else:
            print self.properties.message


class CallbackTask(AsyncTask):

    def __init__(self, callback=None, **kwargs):
        AsyncTask.__init__(self, **kwargs)
        self.callback = callback

    def run(self):
        if self.callback:
            yield self.callback()


class ExceptionTask(AsyncTask):

    def run(self):
        raise Exception('Task exception')


class UserPromptTask(AsyncTask):

    promptsEnabled = True
    promptFunction = None


    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Message', 'continue?')
        properties.addProperty('Always', False)

    def showUserPrompt(self):
        if self.promptFunction:
            self.promptFunction(self, self.properties.message)
        else:
            self.showDialog()

    def showDialog(self):

        self.d = QtGui.QDialog()

        buttons = QtGui.QDialogButtonBox()
        buttons.addButton('Yes', QtGui.QDialogButtonBox.AcceptRole)
        buttons.addButton('No', QtGui.QDialogButtonBox.RejectRole)
        buttons.connect('accepted()', self.d.accept)
        buttons.connect('rejected()', self.d.reject)

        l = QtGui.QVBoxLayout(self.d)
        l.addWidget(QtGui.QLabel(self.properties.message))
        l.addWidget(buttons)

        self.d.setAttribute(QtCore.Qt.WA_QuitOnClose, False)
        self.d.show()
        self.d.raise_()
        self.d.connect('accepted()', self.accept)
        self.d.connect('rejected()', self.reject)

    def accept(self):
        self.result = True

    def reject(self):
        self.result = False

    def run(self):

        if not self.promptsEnabled and not self.properties.getProperty('Always'):
            return

        self.result = None

        self.showUserPrompt()

        while self.result is None:
            yield

        if not self.result:
            raise atq.AsyncTaskQueue.PauseException()


class DelayTask(AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Delay time', 1.0, attributes=propertyset.PropertyAttributes(minimum=0.0, maximum=1e4, singleStep=0.1))

    def run(self):

        delayTime = self.properties.getProperty('Delay time')
        t = SimpleTimer()

        while True:

            elapsed = t.elapsed()
            if elapsed >= delayTime:
                break

            self.statusMessage = 'Waiting %.1f seconds' % (delayTime - elapsed)
            yield


class PauseTask(AsyncTask):

    def run(self):
        raise atq.AsyncTaskQueue.PauseException()


class QuitTask(AsyncTask):

    def run(self):
        QtCore.QCoreApplication.instance().quit()
