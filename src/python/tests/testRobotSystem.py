
from director.consoleapp import ConsoleApp
from director import robotsystem

app = ConsoleApp()

app.setupGlobals(globals())

if app.getTestingInteractiveEnabled():
    app.showPythonConsole()

view = app.createView()
view.show()

robotsystem.create(view, globals())

app.start()
