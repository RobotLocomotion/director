import os
import math
from ddapp.consoleapp import ConsoleApp
from ddapp import ioUtils
from ddapp import segmentation
from ddapp import segmentationroutines
from ddapp import applogic
from ddapp import visualization as vis

from ddapp import roboturdf

app = ConsoleApp()

# create a view
view = app.createView()
segmentation._defaultSegmentationView = view

robotStateModel, robotStateJointController = roboturdf.loadRobotModel('robot state model', view, parent='sensors', color=roboturdf.getRobotGrayColor(), visible=True)
segmentationroutines.SegmentationContext.initWithRobot(robotStateModel)

# Move robot to near to table:
robotStateJointController.q [5] = math.radians(120)
robotStateJointController.q[0] = -1.5
robotStateJointController.q[1] = 2
robotStateJointController.q[2] = 0.95
robotStateJointController.push()

# load poly data
dataDir = app.getTestingDataDirectory()
polyData = ioUtils.readPolyData(os.path.join(dataDir, 'tabletop/table-and-bin-scene.vtp'))
vis.showPolyData(polyData, 'pointcloud snapshot')

#segmentation.segmentTableScene(polyData, [-1.58661389,  2.91242337,  0.79958105] )
segmentation.segmentTableSceneClusters(polyData, [-1.58661389,  2.91242337,  0.79958105], clusterInXY=True )

if app.getTestingInteractiveEnabled():
    view.show()
    app.showObjectModel()
    app.start()
