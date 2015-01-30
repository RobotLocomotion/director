
from ddapp.consoleapp import ConsoleApp

from ddapp import visualization as vis
import ddapp.objectmodel as om
import PythonQt
from PythonQt import QtCore, QtGui

import ddapp.tasks.robottasks as rt
from ddapp.tasks.taskmanagerwidget import TaskWidgetManager
from ddapp import robotsystem
from ddapp.fieldcontainer import FieldContainer


###############################
app = ConsoleApp()

app.setupGlobals(globals())
app.showPythonConsole()

view = app.createView()
view.show()

useRobotSystem = True
if useRobotSystem:
    robotSystem = robotsystem.create(view)
    rt.robotSystem = robotSystem




m = TaskWidgetManager()

actions = [


    ['ground pick demo', [

        ['fit object', [
          [rt.SnapshotSelectedPointcloud, {}],
          [rt.FindHorizontalSurfaces, { 'Normal estimation search radius' : 0.04}],
          [rt.PrintTask, {'Message' : 'all done', 'Name' : 'special print task'}],
         ]],


        ['walk to table', [
          [rt.PlanPostureGoal, {}]
        ]],

     ]],
]


m.taskQueueWidget.loadTaskDescription('ground pick', actions)
m.taskQueueWidget.loadTaskDescription('ground pick copy', actions)


m.taskQueueWidget.setCurrentQueue('Ground pick copy')
m.taskQueueWidget.setCurrentQueue('Ground pick')


m.taskQueueWidget.widget.show()
m.taskLibraryWidget.widget.show()



app.start()



