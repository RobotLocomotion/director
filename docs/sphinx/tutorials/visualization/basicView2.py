from director.consoleapp import ConsoleApp

# initialize application components
app = ConsoleApp()
view = app.createView()
view.showMaximized()

# start the application
app.start()
