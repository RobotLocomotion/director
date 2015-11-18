
from director.consoleapp import ConsoleApp

from director import visualization as vis
import director.objectmodel as om
import PythonQt
from PythonQt import QtCore, QtGui

import director.tasks.robottasks as rt
from director.tasks.taskmanagerwidget import TaskWidgetManager
from director import robotsystem
from director.fieldcontainer import FieldContainer


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



