from ddapp.tasks.robottasks import *
from ddapp.tasks.descriptions import loadTaskDescription
import ddapp.applogic as app


class TaskItem(om.ObjectModelItem):

    def __init__(self, task):
        om.ObjectModelItem.__init__(self, task.properties.getProperty('Name'), properties=task.properties)
        #self.addProperty('Visible', False)
        self.task = task


class TaskLibraryWidget(object):

    def __init__(self):

        self.taskTree = TaskTree()

        self.widget = QtGui.QWidget()
        self.addButton = QtGui.QPushButton('add task')

        l = QtGui.QGridLayout(self.widget)
        l.addWidget(self.taskTree.treeWidget, 0, 0)
        l.addWidget(self.taskTree.propertiesPanel, 1, 0)
        l.addWidget(self.addButton, 2, 0)


        self.widget.setWindowTitle('Task Library')
        self.addButton.connect('clicked()', self.onAddTaskClicked)
        self.callbacks = callbacks.CallbackRegistry(['OnAddTask'])


        #p = self.treeWidget.palette
        #p.setColor(QtGui.QPalette.Highlight, QtCore.Qt.green)
        #p.setColor(QtGui.QPalette.Normal, QtGui.QPalette.Highlight, QtCore.Qt.red)
        #self.treeWidget.setPalette(p)

        #item.setBackground(0, QtGui.QBrush(QtCore.Qt.green))

    def onAddTaskClicked(self):
        task = self.taskTree.getSelectedTask()
        if task:
            self.callbacks.process('OnAddTask', task.task)

    def addTask(self, task, parent=None):
        self.taskTree.onAddTask(task, parent)


class TaskTree(object):


    def __init__(self):
        self.treeWidget = QtGui.QTreeWidget()
        self.propertiesPanel = PythonQt.dd.ddPropertiesPanel()
        self.objectModel = om.ObjectModelTree()
        self.objectModel.init(self.treeWidget, self.propertiesPanel)
        self.treeWidget.setColumnCount(1)

    def onSave(self):

        def helper(obj, children):
            if isinstance(obj, TaskItem):
                obj = obj.task
            elif obj:
                obj = obj.getProperty('Name')

            result = [obj, []]
            for child in children:
                result[1].append(helper(child, child.children()))
            return result

        state = helper(None, self.objectModel.getTopLevelObjects())
        return pickle.dumps(state)

    def onLoad(self, serializedData):

        state = pickle.loads(serializedData)
        assert len(state) == 2
        assert state[0] is None

        def helper(parent, children):

            for child in children:
                child, grandChildren = child

                if isinstance(child, str):
                    child = self.addGroup(child, parent)
                else:
                    child = self.onAddTask(child, parent)

                helper(child, grandChildren)

        helper(state[0], state[1])

    def loadTaskDescription(self, description):

        taskQueue = self

        def helper(obj, children):
            for child in children:
                assert isinstance(child, list)
                assert len(child) == 2
                if isinstance(child[0], str):
                    child, grandChildren = child
                    group = taskQueue.addGroup(child, obj)
                    helper(group, grandChildren)
                else:
                    taskClass, args = child
                    assert isinstance(args, dict)
                    task = taskClass()
                    for propertyName, propertyValue in args.iteritems():
                        assert isinstance(propertyName, str)
                        task.properties.setProperty(propertyName, propertyValue)
                    taskQueue.onAddTask(task, obj)

        helper(None, description)

    def assureSelection(self):
      objs = self.objectModel.getTopLevelObjects()
      if len(objs) == 1:
          self.objectModel.setActiveObject(objs[0])

    def onAddTask(self, task, parent=None):
        obj = TaskItem(task.copy())
        self.objectModel.addToObjectModel(obj, parentObj=parent)
        self.assureSelection()
        return obj

    def addGroup(self, groupName, parent=None):
        obj = self.objectModel.addContainer(groupName, parent)
        obj.removeProperty('Visible')
        self.assureSelection()
        return obj

    def removeAllTasks(self):
        for obj in self.objectModel.getObjects():
            self.objectModel.removeFromObjectModel(obj)

    def findTaskItem(self, task):
        for obj in self.objectModel.getObjects():
            if isinstance(obj, TaskItem) and obj.task == task:
                return obj

    def selectTask(self, task):
        obj = self.findTaskItem(task)
        if obj:
            self.objectModel.setActiveObject(obj)

    def getSelectedTask():
        if not isinstance(self.objectModel.getActiveObject(), TaskItem):
            return None
        return self.objectModel.getActiveObject().task

    def getSelectedTasks(self):
        obj = self.objectModel.getActiveObject()
        assert obj

        container = obj if isinstance(obj, om.ContainerItem) else obj.parent()
        tasks = self.getTasks(container)
        if obj != container:
            tasks = tasks[tasks.index(obj):]

        return tasks

    def getTasks(self, parent=None):

        def helper(objs, tasks):
            for obj in objs:
                if isinstance(obj, om.ContainerItem):
                    helper(obj.children(), tasks)
                    continue
                tasks.append(obj)
            return tasks

        return helper(self.objectModel.getTopLevelObjects() if parent is None else parent.children(), [])


class TaskQueueWidget(object):

    def __del__(self):
      self.timer.stop()


    def __init__(self):

        self.taskTree = TaskTree()

        self.taskQueue = atq.AsyncTaskQueue()
        self.taskQueue.connectTaskStarted(self.onTaskStarted)
        self.taskQueue.connectTaskEnded(self.onTaskEnded)
        self.completedTasks = []

        self.timer = TimerCallback(targetFps=5)
        self.timer.callback = self.updateDisplay
        self.timer.start()

        self.widget = QtGui.QWidget()
        l = QtGui.QGridLayout(self.widget)

        self.queueCombo = QtGui.QComboBox()
        self.startButton = QtGui.QPushButton('start')
        self.stepButton = QtGui.QPushButton('step')
        self.clearButton = QtGui.QPushButton('clear')
        self.currentTaskLabel = QtGui.QLabel('')
        self.statusTaskLabel = QtGui.QLabel('')
        self.statusTaskLabel.visible = False

        l.addWidget(self.queueCombo, 0, 0)
        l.addWidget(self.taskTree.treeWidget, 1, 0)
        l.addWidget(self.taskTree.propertiesPanel, 2, 0)
        l.addWidget(self.startButton, 3, 0)
        l.addWidget(self.stepButton, 4, 0)
        l.addWidget(self.clearButton, 5, 0)
        l.addWidget(self.currentTaskLabel, 6, 0)
        l.addWidget(self.statusTaskLabel, 7, 0)

        self.widget.setWindowTitle('Task Queue')


        self.descriptions = {}
        self.queueCombo.insertSeparator(0)
        self.queueCombo.addItem('Create new...')

        self.queueCombo.connect('currentIndexChanged(const QString&)', self.onQueueComboChanged)
        self.startButton.connect('clicked()', self.onStart)
        self.stepButton.connect('clicked()', self.onStep)
        self.clearButton.connect('clicked()', self.onClear)



    def loadTaskDescription(self, name, description):

        self.descriptions[name] = description
        insertIndex = self.queueCombo.count - 2
        self.queueCombo.insertItem(insertIndex, name)
        #self.onQueueComboChanged(name)


    def onQueueComboChanged(self, name):

        self.taskTree.removeAllTasks()

        if name == 'Create new...':
            return
        elif not name:
            return
        else:
            description = self.descriptions[name]

        self.taskTree.loadTaskDescription(description)


    def onClear(self):

        assert not self.taskQueue.isRunning

        self.taskQueue.reset()

        self.taskTree.removeAllTasks()

        self.updateDisplay()


    def onTaskStarted(self, taskQueue, task):
        print 'starting task:', task.properties.getProperty('Name')
        self.taskTree.selectTask(task)
        item = self.taskTree.findTaskItem(task)
        #if item.getProperty('Visible'):
        #    raise atq.AsyncTaskQueue.PauseException()

    def onTaskEnded(self, taskQueue, task):
        self.completedTasks.append(task)
        self.taskTree.selectTask(self.completedTasks[0])

    def updateStatusMessage(self):

        currentTask = self.taskQueue.currentTask
        if currentTask and currentTask.statusMessage:
            text = vis.updateText(currentTask.statusMessage, 'task status message', parent='planning')
            text.setProperty('Visible', True)
        else:
            text = om.findObjectByName('task status message')
            if text:
                text.setProperty('Visible', False)


    def updateDisplay(self):

        isRunning = self.taskQueue.isRunning
        isEmpty = len(self.taskTree.objectModel._objects) == 0

        if isRunning:
            self.startButton.text = 'stop'
        else:
            self.startButton.text = 'start'

        self.startButton.setEnabled(not isEmpty)
        self.stepButton.setEnabled(not isRunning and not isEmpty)
        self.clearButton.setEnabled(not isRunning and not isEmpty)

        currentTask = self.taskQueue.currentTask

        if currentTask:
            self.currentTaskLabel.text = 'Task: <b>%s</b>' % currentTask.properties.getProperty('Name')
        else:
            self.currentTaskLabel.text = ''

        self.updateStatusMessage()


    def onStep(self):
        assert not self.taskQueue.isRunning

        task = taskTree.getSelectedTask()
        if not task:
            return

        self.completedTasks = []
        self.taskQueue.reset()
        self.taskQueue.addTask(task)
        self.taskQueue.start()
        self.updateDisplay()


    def onStart(self):

        if self.taskQueue.isRunning:
            currentTask = self.taskQueue.currentTask
            self.taskQueue.stop()
            if currentTask:
                currentTask.stop()

        else:

            self.completedTasks = []
            self.taskQueue.reset()
            for obj in self.taskTree.getSelectedTasks():
                #print 'adding task:', obj.task.properties.name
                self.taskQueue.addTask(obj.task)

            self.taskQueue.start()

        self.updateDisplay()


class TaskWidgetManager(object):

    def __init__(self):

        self.taskLibraryWidget = TaskLibraryWidget()
        self.taskQueueWidget = TaskQueueWidget()
        self.taskLibraryWidget.callbacks.connect('OnAddTask', self.taskQueueWidget.taskTree.onAddTask)
        self.addDefaultTasksToLibrary()


    def addDefaultTasksToLibrary(self):

        desc = loadTaskDescription('taskLibrary')
        self.taskLibraryWidget.taskTree.loadTaskDescription(desc)


def init():

    global panel
    global dock
    panel = TaskWidgetManager()
    dock = app.addWidgetToDock(panel.taskQueueWidget.widget, action=app.getToolBarActions()['ActionActionManagerPanel'])
    dock.hide()

    dock = app.addWidgetToDock(panel.taskLibraryWidget.widget)
    return panel

