import ddapp
from ddapp.consoleapp import ConsoleApp
from ddapp import robotsystem
from ddapp import kinematicposeplanner as kpp


def onIkStartup(ikServer, startSuccess):
    kinematicPosePlanner.init()
    panel.testDemo()
    panel.ikServer.taskQueue.addTask(app.startTestingModeQuitTimer)


app = ConsoleApp()
app.setupGlobals(globals())

if app.getTestingInteractiveEnabled():
    app.showPythonConsole()

view = app.createView()
view.show()

robotsystem.create(view, globals())

ikServer.connectStartupCompleted(onIkStartup)
startIkServer()

kinematicPosePlanner = kpp.KinematicPosePlanner(ikServer)
panel = kpp.KinematicPosePlannerPanel(kinematicPosePlanner, ikServer, teleopJointController)

robotStateModel.setProperty('Visible', False)
teleopRobotModel.setProperty('Visible', True)

app.start(enableAutomaticQuit=False)
