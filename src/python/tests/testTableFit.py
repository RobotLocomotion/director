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

p1 = [-1.58661389,  2.91242337,  0.79958105]
data = segmentation.segmentTableScene(polyData, p1)
vis.showClusterObjects(data.clusters, parent='segmentation')
segmentation.showTable(data.table, parent='segmentation')

if app.getTestingInteractiveEnabled():
    view.show()
    app.showObjectModel()
    app.start()
