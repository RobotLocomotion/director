
from director.consoleapp import ConsoleApp
from director import robotsystem

app = ConsoleApp()

app.setupGlobals(globals())

if app.getTestingInteractiveEnabled():
    app.showPythonConsole()

view = app.createView()
view.show()

useMinimalOptions = False

if useMinimalOptions:

    factory = robotsystem.RobotSystemFactory()

    options = factory.getDisabledOptions()
    options.useDirectorConfig = True
    options.useRobotState = True

    robotSystem = factory.construct(view=view, options=options)

else:

    robotSystem = robotsystem.create(view)


app.start()
