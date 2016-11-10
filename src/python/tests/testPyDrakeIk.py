from director import mainwindowapp
from director import robotsystem
from director import applogic
from PythonQt import QtCore

def makeRobotSystem(view):
    factory = robotsystem.RobotSystemFactory()
    options = factory.getDisabledOptions()
    factory.setDependentOptions(options, usePlannerPublisher=True, useTeleop=True)
    return factory.construct(view=view, options=options)


app = mainwindowapp.MainWindowAppFactory().construct()
robotSystem = makeRobotSystem(app.view)

app.app.addWidgetToDock(robotSystem.teleopPanel.widget, QtCore.Qt.RightDockWidgetArea)
app.app.addWidgetToDock(robotSystem.playbackPanel.widget, QtCore.Qt.BottomDockWidgetArea)

# use pydrake ik backend
ikPlanner = robotSystem.ikPlanner
ikPlanner.planningMode = 'pydrake'
ikPlanner.plannerPub._setupLocalServer()

applogic.resetCamera(viewDirection=[-1,0,0], view=app.view)
app.app.start()
