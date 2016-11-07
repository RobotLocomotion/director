import os
import fnmatch
import argparse
import traceback

from director import mainwindowapp
from director import visualization as vis
from director import roboturdf
from director import jointcontrol


def getArgs():
    parser = argparse.ArgumentParser()
    parser.add_argument('--urdf', type=str, default=None, help='urdf filename to load')
    parser.add_argument('--glob-dir', type=str, default=None, help='recursively search this directory and load every urdf found')
    args, unknown = parser.parse_known_args()
    return args


app = mainwindowapp.MainWindowAppFactory().construct()
view = app.view

args = getArgs()

if args.urdf:

    robotModel, jointController = roboturdf.loadRobotModel(urdfFile=args.urdf, view=view, useConfigFile=False)

    jointNames = robotModel.model.getJointNames()
    jointController = jointcontrol.JointController([robotModel], jointNames=jointNames)

elif args.glob_dir:

    urdfFiles = []
    for root, dirnames, filenames in os.walk(args.glob_dir):
        for filename in fnmatch.filter(filenames, '*.urdf'):
            urdfFiles.append(os.path.join(root, filename))

    failedFiles = []
    for urdfFile in urdfFiles:
        print 'loading:', urdfFile
        try:
            robotModel, jointController = roboturdf.loadRobotModel(urdfFile=urdfFile, view=view, visible=False, useConfigFile=False)
        except:
            print traceback.format_exc()
            failedFiles.append(urdfFile)

    if failedFiles:
        print 'failed to load urdf files:'
        print '\n'.join(failedFiles)

else:

    robotModel, jointController = roboturdf.loadRobotModel('robot model', view)

    print 'urdf file:', robotModel.getProperty('Filename')

    for joint in robotModel.model.getJointNames():
        print 'joint:', joint

    for link in robotModel.model.getLinkNames():
        print 'link:', link
        robotModel.getLinkFrame(link)



app.app.start()
