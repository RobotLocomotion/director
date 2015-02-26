from PythonQt import QtCore, QtGui
from ddapp import lcmUtils
from ddapp.simpletimer import SimpleTimer
from ddapp.timercallback import TimerCallback
import subprocess
import os

class LCMLoggerWidget(object):

    def __init__(self, statusBar=None):
        self.manager = lcmUtils.LCMLoggerManager()
        self.statusBar = statusBar

        self.lastActiveLogFile = None
        self.numProcesses = 0
        self.numLogFiles = 0
        self.userTag = ''

        self.button = QtGui.QPushButton('')
        self.button.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.button.connect('customContextMenuRequested(const QPoint&)', self.showContextMenu)
        self.button.connect('clicked()', self.onClick)

        self.timer = TimerCallback(targetFps=0.25)
        self.timer.callback = self.updateState
        self.timer.start()


    def updateState(self):

        t = SimpleTimer()
        self.manager.updateExistingLoggerProcesses()

        activeLogFiles = self.manager.getActiveLogFilenames()
        self.numProcesses = len(self.manager.getActiveLoggerPids())
        self.numLogFiles = len(activeLogFiles)

        if self.numLogFiles == 1:
            self.lastActiveLogFile = activeLogFiles[0]

        if self.numProcesses == 0:
            self.button.text = 'start logger'
        elif self.numProcesses == 1:
            self.button.text = 'stop logger'
        elif self.numProcesses > 1:
            self.button.text = 'stop all loggers'

        statusDescription = 'active' if self.numProcesses else 'last'
        logFileDescription = self.lastActiveLogFile or '<unknown>'
        self.button.setToolTip('%s log file: %s' % (statusDescription, logFileDescription))


    def onClick(self):
        if self.numProcesses == 0:
            self.manager.startNewLogger(tag=self.userTag)
            self.updateState()
            self.showStatusMessage('start logging: ' + self.lastActiveLogFile)
        else:
            self.manager.killAllLoggingProcesses()
            self.showStatusMessage('stopped logging')
            self.updateState()

    def showStatusMessage(self, msg, timeout=2000):
        if self.statusBar:
            self.statusBar.showMessage(msg, timeout)

    def showContextMenu(self, clickPosition):

        globalPos = self.button.mapToGlobal(clickPosition)

        menu = QtGui.QMenu()

        action = menu.addAction('Stop logger')
        action.enabled = (self.numProcesses > 0)

        action = menu.addAction('Stop and delete log file')
        action.enabled = (self.numProcesses > 0 and self.lastActiveLogFile)

        action = menu.addAction('Set logger tag')
        action.enabled = (self.numProcesses == 0)

        action = menu.addAction('Copy log filename')
        action.enabled = (self.lastActiveLogFile is not None)

        action = menu.addAction('Review log')
        action.enabled = (self.lastActiveLogFile is not None)


        selectedAction = menu.exec_(globalPos)
        if selectedAction is None:
            return

        if selectedAction.text == 'Copy log filename':
            clipboard = QtGui.QApplication.instance().clipboard()
            clipboard.setText(self.lastActiveLogFile)
            self.showStatusMessage('copy to clipboard: ' + self.lastActiveLogFile)

        elif selectedAction.text == 'Stop logger':
            self.manager.killAllLoggingProcesses()
            self.showStatusMessage('stopped logger')
            self.updateState()

        elif selectedAction.text == 'Stop and delete log file':
            logFileToRemove = self.lastActiveLogFile
            self.manager.killAllLoggingProcesses()
            self.updateState()
            os.remove(logFileToRemove)
            self.showStatusMessage('deleted: ' + logFileToRemove)

        elif selectedAction.text == 'Set logger tag':
            inputDialog = QtGui.QInputDialog()
            inputDialog.setInputMode(inputDialog.TextInput)
            inputDialog.setLabelText('Log file tag:')
            inputDialog.setWindowTitle('Enter tag')
            inputDialog.setTextValue(self.userTag)
            result = inputDialog.exec_()

            if result:
                tag = inputDialog.textValue()
                self.userTag = tag
                self.showStatusMessage('Set lcm logger tag: ' + self.userTag)

        elif selectedAction.text == 'Review log':
            newEnv = dict(os.environ)
            newEnv['LCM_DEFAULT_URL'] = newEnv['LCM_REVIEW_DEFAULT_URL']
            devnull = open(os.devnull, 'w')
            subprocess.Popen('drake-designer', stdout=devnull, stderr=devnull, env=newEnv)
            subprocess.Popen(['lcm-logplayer-gui', self.lastActiveLogFile], stdout=devnull, stderr=devnull, env=newEnv)
            subprocess.Popen(['bot-procman-sheriff', '-o'], stdout=devnull, stderr=devnull, env=newEnv)
