from ddapp.consoleapp import ConsoleApp
from ddapp import visualization as vis
from ddapp import roboturdf

app = ConsoleApp()
view = app.createView()

robotStateModel, robotStateJointController = roboturdf.loadRobotModel('robot state model', view, parent='sensors', color=roboturdf.getRobotGrayColor(), visible=True)

<<<<<<< HEAD
robotStateModel.addToView(view)

if app.getTestingInteractiveEnabled():

=======
if app.getTestingInteractiveEnabled():
>>>>>>> sandbox/unified-plans
    view.show()
    app.start()
