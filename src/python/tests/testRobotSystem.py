
from ddapp.consoleapp import ConsoleApp
from ddapp import robotsystem

app = ConsoleApp()

app.setupGlobals(globals())

if app.getTestingInteractiveEnabled():
    app.showPythonConsole()

view = app.createView()
view.show()

robotsystem.create(view, globals())

app.start()
