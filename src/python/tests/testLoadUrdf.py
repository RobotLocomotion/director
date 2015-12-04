from director.consoleapp import ConsoleApp
from director import visualization as vis
from director import roboturdf
from director import jointcontrol
import argparse


def getArgs():
    parser = argparse.ArgumentParser()
    parser.add_argument('--urdf', type=str, default=None, help='urdf filename to load')
    args, unknown = parser.parse_known_args()
    return args


app = ConsoleApp()
view = app.createView()

args = getArgs()
if args.urdf:

    robotModel = roboturdf.openUrdf(args.urdf, view)

    jointNames = robotModel.model.getJointNames()
    jointController = jointcontrol.JointController([robotModel], jointNames=jointNames)

else:
    robotModel, jointController = roboturdf.loadRobotModel('robot model', view)



print 'urdf file:', robotModel.getProperty('Filename')

for joint in robotModel.model.getJointNames():
    print 'joint:', joint

for link in robotModel.model.getLinkNames():
    print 'link:', link
    robotModel.getLinkFrame(link)


if app.getTestingInteractiveEnabled():
    view.show()
    app.start()
