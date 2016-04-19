from director.consoleapp import ConsoleApp
from director import visualization as vis
from director import roboturdf

app = ConsoleApp()
view = app.createView()

robotStateModel, robotStateJointController = roboturdf.loadRobotModel('robot state model', view, parent='sensors', color=roboturdf.getRobotGrayColor(), visible=True)

robotStateModel.addToView(view)

if app.getTestingInteractiveEnabled():
    view.show()
    app.start()
