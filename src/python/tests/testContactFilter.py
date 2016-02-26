__author__ = 'manuelli'

from ddapp.consoleapp import ConsoleApp
from ddapp import visualization as vis
from ddapp import roboturdf
from ddapp import robotsystem
from ddapp import contactfilter

app = ConsoleApp()
view = app.createView()


# robotStateModel, robotStateJointController = roboturdf.loadRobotModel('robot state model', view, parent='sensors', color=roboturdf.getRobotGrayColor(), visible=True)
# robotStateJointController.addLCMUpdater('EST_ROBOT_STATE')
# robotStateModel.addToView(view)

robotSystem = robotsystem.create(view)
contactFilter = contactfilter.ContactFilter(robotSystem.robotStateModel, robotSystem.robotStateJointController)
contactFilter.start()

if app.getTestingInteractiveEnabled():
    view.show()
    app.start()