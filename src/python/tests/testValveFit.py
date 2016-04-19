import os
import math
from director.consoleapp import ConsoleApp
from director import ioUtils
from director import segmentation
from director import segmentationroutines
from director import applogic
from director import visualization as vis

from director import roboturdf

app = ConsoleApp()

# create a view
view = app.createView()
segmentation._defaultSegmentationView = view
segmentation.initAffordanceManager(view)

robotStateModel, robotStateJointController = roboturdf.loadRobotModel('robot state model', view, parent='sensors', color=roboturdf.getRobotGrayColor(), visible=True)
segmentationroutines.SegmentationContext.initWithRobot(robotStateModel)

# Move robot to near to valve wall:
# 0degrees
#robotStateJointController.q[5] = math.radians(120)
#robotStateJointController.q[0] = 0
#robotStateJointController.q[1] = 0
# 30,60,90
robotStateJointController.q [5] = math.radians(-120)
robotStateJointController.q [0] = 1
robotStateJointController.q [1] = 1

robotStateJointController.q[2] = 0.85
robotStateJointController.push()

# load poly data
dataDir = app.getTestingDataDirectory()
#polyData = ioUtils.readPolyData(os.path.join(dataDir, 'valve/valve-lever-scene.vtp'))
#polyData = ioUtils.readPolyData(os.path.join(dataDir, 'valve/valve-lever-scene-30.vtp'))
#polyData = ioUtils.readPolyData(os.path.join(dataDir, 'valve/valve-lever-scene-60.vtp'))
polyData = ioUtils.readPolyData(os.path.join(dataDir, 'valve/valve-lever-scene-90.vtp'))
vis.showPolyData(polyData, 'pointcloud snapshot')

segmentation.segmentValveWallAuto(.2, mode='both', removeGroundMethod=segmentation.removeGround )

if app.getTestingInteractiveEnabled():
    view.show()
    app.showObjectModel()
    app.start()
