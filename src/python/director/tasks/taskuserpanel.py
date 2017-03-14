from director.timercallback import TimerCallback
from director import objectmodel as om
from director import propertyset
from director import asynctaskqueue as atq
import director.tasks.basictasks as basictasks
import director.tasks.taskmanagerwidget as tmw

import numpy as np
import traceback
import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools


def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


class TaskUserPanel(object):

    def __init__(self, windowTitle='Task Panel'):


        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddTaskUserPanel.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)
        self.ui = WidgetDict(self.widget.children())
        self.widget.setWindowTitle(windowTitle)

        self.manualButtons = {}
        self.imageViewLayout = QtGui.QHBoxLayout(self.ui.imageFrame)

        self._setupParams()
        self._setupPropertiesPanel()
        self._initTaskPanel()


    def addManualButton(self, text, callback):
        b = QtGui.QPushButton(text)
        b.connect('clicked()', callback)
        self.manualButtons[text] = b
        self.addManualWidget(b)
        return b

    def addManualSpacer(self):

        line = QtGui.QFrame()
        line.setFrameShape(QtGui.QFrame.HLine)
        line.setFrameShadow(QtGui.QFrame.Sunken)
        self.addManualWidget(line)

    def addManualWidget(self, widget):
        self.ui.manualButtonsLayout.insertWidget(self.ui.manualButtonsLayout.count()-1, widget)

    def initImageView(self, imageView, activateAffordanceUpdater=True):
        if activateAffordanceUpdater:
            from director import affordanceupdater
            self.affordanceUpdater = affordanceupdater.AffordanceInCameraUpdater(segmentation.affordanceManager, imageView)
            self.affordanceUpdater.timer.start()
        self.imageViewLayout.addWidget(imageView.view)


    def _setupParams(self):
        self.params = om.ObjectModelItem('Task Params')
        self.params.properties.connectPropertyChanged(self.onPropertyChanged)


    def _setupPropertiesPanel(self):
        l = QtGui.QVBoxLayout(self.ui.propertyFrame)
        l.setMargin(0)
        self.propertiesPanel = PythonQt.dd.ddPropertiesPanel()
        self.propertiesPanel.setBrowserModeToWidget()
        l.addWidget(self.propertiesPanel)
        self.panelConnector = propertyset.PropertyPanelConnector(self.params.properties, self.propertiesPanel)


    def onPropertyChanged(self, propertySet, propertyName):
        pass

    def getNextTasks(self):
        return self.taskTree.getTasks(fromSelected=True)

    def onContinue(self):

        self._activatePrompts()
        self.completedTasks = []
        self.taskQueue.reset()
        for obj in self.getNextTasks():
            self.taskQueue.addTask(obj.task)

        self.taskQueue.start()


    def _activatePrompts(self):
        basictasks.UserPromptTask.promptFunction = self.onTaskPrompt
        basictasks.PrintTask.printFunction = self.appendMessage

    def onStep(self):

        assert not self.taskQueue.isRunning
        self._activatePrompts()

        tasks = self.getNextTasks()
        if not tasks:
            return

        task = tasks[0].task
        self.nextStepTask = tasks[1].task if len(tasks) > 1 else None

        self.completedTasks = []
        self.taskQueue.reset()
        self.taskQueue.addTask(task)
        self.taskQueue.start()

    def updateTaskButtons(self):
        self.ui.taskStepButton.setEnabled(not self.taskQueue.isRunning)
        self.ui.taskContinueButton.setEnabled(not self.taskQueue.isRunning)
        self.ui.taskPauseButton.setEnabled(self.taskQueue.isRunning)

    def onPause(self):

        if not self.taskQueue.isRunning:
            return

        self.nextStepTask = None
        currentTask = self.taskQueue.currentTask
        self.taskQueue.stop()
        if currentTask:
            currentTask.stop()

        self.appendMessage('<font color="red">paused</font>')

    def onQueueStarted(self, taskQueue):
        self.updateTaskButtons()

    def onQueueStopped(self, taskQueue):
        self.clearPrompt()
        self.updateTaskButtons()

    def onTaskStarted(self, taskQueue, task):
        msg = task.properties.getProperty('Name')  + ' ... <font color="green">start</font>'
        self.appendMessage(msg)

        self.taskTree.selectTask(task)
        item = self.taskTree.findTaskItem(task)
        if len(self.completedTasks) and item.getProperty('Visible'):
            self.appendMessage('<font color="red">paused</font>')
            raise atq.AsyncTaskQueue.PauseException()

    def onTaskEnded(self, taskQueue, task):
        msg = task.properties.getProperty('Name') + ' ... <font color="green">end</font>'
        self.appendMessage(msg)

        self.completedTasks.append(task)

        if self.taskQueue.tasks:
            self.taskTree.selectTask(self.taskQueue.tasks[0])
        elif self.nextStepTask:
            self.taskTree.selectTask(self.nextStepTask)
        #else:
        #    self.taskTree.selectTask(self.completedTasks[0])

    def onTaskFailed(self, taskQueue, task):
        msg = task.properties.getProperty('Name')  + ' ... <font color="red">failed: %s</font>' % task.failReason
        self.appendMessage(msg)

    def onTaskPaused(self, taskQueue, task):
        msg = task.properties.getProperty('Name')  + ' ... <font color="red">paused</font>'
        self.appendMessage(msg)

    def onTaskException(self, taskQueue, task):
        msg = task.properties.getProperty('Name')  + ' ... <font color="red">exception:\n\n%s</font>' % traceback.format_exc()
        self.appendMessage(msg)


    def appendMessage(self, msg):
        if msg == self.lastStatusMessage:
            return

        self.lastStatusMessage = msg
        self.ui.outputConsole.append(msg.replace('\n', '<br/>'))

    def updateTaskStatus(self):

        currentTask = self.taskQueue.currentTask
        if not currentTask or not currentTask.statusMessage:
            return

        name = currentTask.properties.getProperty('Name')
        status = currentTask.statusMessage
        msg = name + ': ' + status
        self.appendMessage(msg)

    def clearPrompt(self):
        self.promptTask = None
        self.ui.promptLabel.text = ''
        self.ui.promptAcceptButton.enabled = False
        self.ui.promptRejectButton.enabled = False

    def onAcceptPrompt(self):
        self.promptTask.accept()
        self.clearPrompt()

    def onRejectPrompt(self):
        self.promptTask.reject()
        self.clearPrompt()

    def onTaskPrompt(self, task, message):
        self.promptTask = task
        self.ui.promptLabel.text = message
        self.ui.promptAcceptButton.enabled = True
        self.ui.promptRejectButton.enabled = True

    def _initTaskPanel(self):

        self.lastStatusMessage = ''
        self.nextStepTask = None
        self.completedTasks = []
        self.taskQueue = atq.AsyncTaskQueue()
        self.taskQueue.connectQueueStarted(self.onQueueStarted)
        self.taskQueue.connectQueueStopped(self.onQueueStopped)
        self.taskQueue.connectTaskStarted(self.onTaskStarted)
        self.taskQueue.connectTaskEnded(self.onTaskEnded)
        self.taskQueue.connectTaskPaused(self.onTaskPaused)
        self.taskQueue.connectTaskFailed(self.onTaskFailed)
        self.taskQueue.connectTaskException(self.onTaskException)

        self.timer = TimerCallback(targetFps=2)
        self.timer.callback = self.updateTaskStatus
        self.timer.start()

        self.taskTree = tmw.TaskTree()
        self.ui.taskFrame.layout().insertWidget(0, self.taskTree.treeWidget)

        l = QtGui.QVBoxLayout(self.ui.taskPropertiesGroupBox)
        l.addWidget(self.taskTree.propertiesPanel)
        PythonQt.dd.ddGroupBoxHider(self.ui.taskPropertiesGroupBox)


        self.ui.taskStepButton.connect('clicked()', self.onStep)
        self.ui.taskContinueButton.connect('clicked()', self.onContinue)
        self.ui.taskPauseButton.connect('clicked()', self.onPause)

        self.ui.promptAcceptButton.connect('clicked()', self.onAcceptPrompt)
        self.ui.promptRejectButton.connect('clicked()', self.onRejectPrompt)
        self.clearPrompt()
        self.updateTaskButtons()
