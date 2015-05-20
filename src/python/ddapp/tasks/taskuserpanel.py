import os
import sys
import vtkAll as vtk
from ddapp import botpy
import math
import time
import types
import functools
import numpy as np

from ddapp import transformUtils
from ddapp import lcmUtils
from ddapp.timercallback import TimerCallback
from ddapp.asynctaskqueue import AsyncTaskQueue
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp import applogic as app
from ddapp.debugVis import DebugData
from ddapp import ik
from ddapp import ikplanner
from ddapp import ioUtils
from ddapp.simpletimer import SimpleTimer
from ddapp.utime import getUtime
from ddapp.pointpicker import ImagePointPicker
from ddapp import affordanceitems
from ddapp import affordanceupdater
from ddapp import robotstate
from ddapp import robotplanlistener
from ddapp import cameraview
from ddapp import segmentation
from ddapp import planplayback
from ddapp import propertyset
from ddapp import asynctaskqueue as atq

import ddapp.tasks.robottasks as rt
import ddapp.tasks.taskmanagerwidget as tmw

import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools


import drc as lcmdrc
import traceback
from PythonQt import QtCore, QtGui




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

    def initImageView(self, imageView):
        self.affordanceUpdater = affordanceupdater.AffordanceInCameraUpdater(segmentation.affordanceManager, imageView)
        self.affordanceUpdater.timer.start()
        self.imageViewLayout.addWidget(self.fitter.imageView.view)


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
        rt.UserPromptTask.promptFunction = self.onTaskPrompt
        rt.PrintTask.printFunction = self.appendMessage

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


class ImageBasedAffordanceFit(object):

    def __init__(self, imageView=None, numberOfPoints=1):

        self.imageView = imageView or cameraview.CameraImageView(cameraview.imageManager, 'CAMERA_LEFT', 'image view')
        self.imagePicker = ImagePointPicker(self.imageView, numberOfPoints=numberOfPoints)
        self.imagePicker.connectDoubleClickEvent(self.onImageViewDoubleClick)
        self.imagePicker.annotationFunc = self.onImageAnnotation
        self.imagePicker.start()

        self.pointCloudSource = 'lidar'
        self.pickLineRadius = 0.05
        self.pickNearestToCamera = True

    def getPointCloud(self):
        assert self.pointCloudSource in ('lidar', 'stereo')
        if self.pointCloudSource == 'stereo':
            return segmentation.getDisparityPointCloud(decimation=1, removeOutliers=False)
        else:
            return segmentation.getCurrentRevolutionData()

    def onImageAnnotation(self, *points):
        polyData = self.getPointCloud()
        points = [self.getPointCloudLocationFromImage(p, self.imageView, polyData) for p in points]
        self.fit(polyData, points)

    def getPointCloudLocationFromImage(self, imagePixel, imageView, polyData):
        cameraPos, ray = imageView.getWorldPositionAndRay(imagePixel)
        return segmentation.extractPointsAlongClickRay(cameraPos, ray, polyData, distanceToLineThreshold=self.pickLineRadius, nearestToCamera=self.pickNearestToCamera)

    def onImageViewDoubleClick(self, displayPoint, modifiers, imageView):
        pass

    def fit(self, pointData, points):
        pass
