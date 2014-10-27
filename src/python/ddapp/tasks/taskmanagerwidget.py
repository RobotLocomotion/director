from ddapp.tasks.robottasks import *
import ddapp.applogic as app


class TaskItem(om.ObjectModelItem):

    def __init__(self, task):
        om.ObjectModelItem.__init__(self, task.properties.getProperty('Name'), properties=task.properties)
        #self.addProperty('Visible', False)
        self.task = task


class TaskLibraryWidget(object):

    def __init__(self):
        self.widget = QtGui.QWidget()
        l = QtGui.QGridLayout(self.widget)

        self.treeWidget = QtGui.QTreeWidget()


        self.propertiesPanel = PythonQt.dd.ddPropertiesPanel()
        self.addButton = QtGui.QPushButton('add task')

        l.addWidget(self.treeWidget, 0, 0)
        l.addWidget(self.propertiesPanel, 1, 0)
        l.addWidget(self.addButton, 2, 0)


        self.widget.setWindowTitle('Task Library')

        self.objectModel = om.ObjectModelTree()
        self.objectModel.init(self.treeWidget, self.propertiesPanel)

        self.treeWidget.setColumnCount(1)
        self.treeWidget.setHeaderLabels(['Tasks'])

        self.addButton.connect('clicked()', self.onAddTaskClicked)

        self.callbacks = callbacks.CallbackRegistry(['OnAddTask'])


        #p = self.treeWidget.palette
        #p.setColor(QtGui.QPalette.Highlight, QtCore.Qt.green)
        #p.setColor(QtGui.QPalette.Normal, QtGui.QPalette.Highlight, QtCore.Qt.red)
        #self.treeWidget.setPalette(p)

        #item.setBackground(0, QtGui.QBrush(QtCore.Qt.green))


    def onAddTaskClicked(self):
        task = self.objectModel.getActiveObject().task
        self.callbacks.process('OnAddTask', task)

    def addTask(self, task, groupName=None):
        groupName = groupName or self.defaultTaskGroupName
        folder = self.objectModel.getOrCreateContainer(groupName)
        obj = TaskItem(task)
        self.objectModel.addToObjectModel(obj, parentObj=folder)
        if len(self.objectModel.getTopLevelObjects()) == 1:
            self.objectModel.setActiveObject(obj)


class TaskQueueWidget(object):

    def __del__(self):
      self.timer.stop()


    def __init__(self):

        self.taskQueue = atq.AsyncTaskQueue()
        self.taskQueue.connectTaskStarted(self.onTaskStarted)
        self.taskQueue.connectTaskEnded(self.onTaskEnded)
        self.completedTasks = []

        self.timer = TimerCallback(targetFps=5)
        self.timer.callback = self.updateDisplay
        self.timer.start()

        self.widget = QtGui.QWidget()
        l = QtGui.QGridLayout(self.widget)

        self.treeWidget = QtGui.QTreeWidget()
        self.propertiesPanel = PythonQt.dd.ddPropertiesPanel()
        self.startButton = QtGui.QPushButton('start')
        self.stepButton = QtGui.QPushButton('step')
        self.clearButton = QtGui.QPushButton('clear')
        self.currentTaskLabel = QtGui.QLabel('')
        self.statusTaskLabel = QtGui.QLabel('')
        self.statusTaskLabel.visible = False

        l.addWidget(self.treeWidget, 0, 0)
        l.addWidget(self.propertiesPanel, 1, 0)
        l.addWidget(self.startButton, 2, 0)
        l.addWidget(self.stepButton, 3, 0)
        l.addWidget(self.clearButton, 4, 0)
        l.addWidget(self.currentTaskLabel, 5, 0)
        l.addWidget(self.statusTaskLabel, 6, 0)

        self.widget.setWindowTitle('Task Queue')

        self.objectModel = om.ObjectModelTree()
        self.objectModel.init(self.treeWidget, self.propertiesPanel)

        self.treeWidget.setColumnCount(1)
        #self.treeWidget.setHeaderLabels(['Tasks', 'Break'])

        self.startButton.connect('clicked()', self.onStart)
        self.stepButton.connect('clicked()', self.onStep)
        self.clearButton.connect('clicked()', self.onClear)


    def getChildItems(self, item):
        return []

    def serializeObject(self, obj):
        state = []
        state.append(pickle.dumps(obj.task))

        for child in obj.children():
            state.append(self.serializeObject(child))

        return state

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


    def loadTaskDescription(self, name, description):

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

    def onClear(self):

        assert not self.taskQueue.isRunning

        self.taskQueue.reset()

        for obj in self.objectModel.getObjects():
            self.objectModel.removeFromObjectModel(obj)
        self.updateDisplay()


    def findTaskItem(self, task):
        for obj in self.objectModel.getObjects():
            if isinstance(obj, TaskItem) and obj.task == task:
                return obj

    def selectTask(self, task):
        obj = self.findTaskItem(task)
        if obj:
            self.objectModel.setActiveObject(obj)

    def getSelectedTasks(self):
        obj = self.objectModel.getActiveObject()
        assert obj

        container = obj if isinstance(obj, om.ContainerItem) else obj.parent()
        tasks = self.getTasks(container)
        if obj != container:
            tasks = tasks[tasks.index(obj):]

        return tasks

    def onTaskStarted(self, taskQueue, task):
        print 'starting task:', task.properties.getProperty('Name')
        self.selectTask(task)
        item = self.findTaskItem(task)
        #if item.getProperty('Visible'):
        #    raise atq.AsyncTaskQueue.PauseException()

    def onTaskEnded(self, taskQueue, task):
        self.completedTasks.append(task)
        self.selectTask(self.completedTasks[0])

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
        isEmpty = len(self.objectModel._objects) == 0

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


    def getTasks(self, parent=None):

        def helper(objs, tasks):
            for obj in objs:
                if isinstance(obj, om.ContainerItem):
                    helper(obj.children(), tasks)
                    continue
                tasks.append(obj)
            return tasks

        return helper(self.objectModel.getTopLevelObjects() if parent is None else parent.children(), [])

    def onStep(self):
        assert not self.taskQueue.isRunning

        if not isinstance(self.objectModel.getActiveObject(), TaskItem):
            return

        task = self.objectModel.getActiveObject().task
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
            for obj in self.getSelectedTasks():
                #print 'adding task:', obj.task.properties.name
                self.taskQueue.addTask(obj.task)

            self.taskQueue.start()

        self.updateDisplay()


class TaskWidgetManager(object):

    def __init__(self):

        self.taskLibraryWidget = TaskLibraryWidget()
        self.taskQueueWidget = TaskQueueWidget()
        self.taskLibraryWidget.callbacks.connect('OnAddTask', self.taskQueueWidget.onAddTask)
        self.addDefaultTasksToLibrary()


    def addDefaultTasksToLibrary(self):

        self.taskLibraryWidget.addTask(PrintTask(), 'utils')
        self.taskLibraryWidget.addTask(UserPromptTask(), 'utils')
        self.taskLibraryWidget.addTask(DelayTask(), 'utils')
        self.taskLibraryWidget.addTask(PauseTask(), 'utils')
        self.taskLibraryWidget.addTask(QuitTask(), 'utils')
        self.taskLibraryWidget.addTask(WaitForMultisenseLidar(), 'perception')
        self.taskLibraryWidget.addTask(SnapshotSelectedPointcloud(), 'perception')
        self.taskLibraryWidget.addTask(SnapshotMultisensePointcloud(), 'perception')
        self.taskLibraryWidget.addTask(SnapshotStereoPointcloud(), 'perception')
        self.taskLibraryWidget.addTask(FitDrill(), 'perception')
        self.taskLibraryWidget.addTask(FindHorizontalSurfaces(), 'perception')
        self.taskLibraryWidget.addTask(PublishAffordance(), 'affordances')



def init():

    global panel
    global dock
    panel = TaskWidgetManager()
    dock = app.addWidgetToDock(panel.taskQueueWidget.widget, action=app.getToolBarActions()['ActionActionManagerPanel'])
    dock.hide()
    return panel

