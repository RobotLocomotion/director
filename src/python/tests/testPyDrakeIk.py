# import pydrakeik first.  This is a workaround for the issue:
# https://github.com/RobotLocomotion/director/issues/467
from director import pydrakeik

from director import mainwindowapp
from director import robotsystem
from director import applogic
from PythonQt import QtCore

def makeRobotSystem(view):
    factory = robotsystem.ComponentFactory()
    factory.register(robotsystem.RobotSystemFactory)
    options = factory.getDisabledOptions()
    factory.setDependentOptions(options, usePlannerPublisher=True, useTeleop=True)
    return factory.construct(view=view, options=options)

app = mainwindowapp.construct()


robotSystem = makeRobotSystem(app.view)

app.app.addWidgetToDock(robotSystem.teleopPanel.widget, QtCore.Qt.RightDockWidgetArea)
app.app.addWidgetToDock(robotSystem.playbackPanel.widget, QtCore.Qt.BottomDockWidgetArea)

# use pydrake ik backend
ikPlanner = robotSystem.ikPlanner
ikPlanner.planningMode = 'pydrake'
ikPlanner.plannerPub._setupLocalServer()

applogic.resetCamera(viewDirection=[-1,0,0], view=app.view)
app.app.start()
