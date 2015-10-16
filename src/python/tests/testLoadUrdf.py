from ddapp.consoleapp import ConsoleApp
from ddapp import visualization as vis
from ddapp import roboturdf

app = ConsoleApp()
view = app.createView()

robotStateModel, robotStateJointController = roboturdf.loadRobotModel('robot state model', view, parent='sensors', color=roboturdf.getRobotGrayColor(), visible=True)

print 'urdf file:', robotStateModel.getProperty('Filename')

for joint in robotStateModel.model.getJointNames():
    print 'joint:', joint

for link in robotStateModel.model.getLinkNames():
    print 'link:', link
    robotStateModel.getLinkFrame(link)

robotStateModel.addToView(view)

if app.getTestingInteractiveEnabled():
    view.show()
    app.start()
