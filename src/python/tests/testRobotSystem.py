
from director.consoleapp import ConsoleApp
from director import robotsystem

app = ConsoleApp()

app.setupGlobals(globals())

if app.getTestingInteractiveEnabled():
    app.showPythonConsole()

view = app.createView()
view.show()

testMinimalOptions = True

if testMinimalOptions:

    factory = robotsystem.RobotSystemFactory()

    options = factory.getDisabledOptions()
    options.useDirectorConfig = True
    options.useAffordances = True
    options.useRobotState = True
    options.usePlanning = True
    options.usePlayback = True
    options.useTeleop = True

    robotSystem = factory.construct(view=view, options=options)

else:

    robotSystem = robotsystem.create(view)


app.start()
