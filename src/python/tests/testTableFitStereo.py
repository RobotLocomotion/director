import os

from director.consoleapp import ConsoleApp
from director import ioUtils
from director import segmentation
from director import segmentationroutines
from director import applogic
from director import visualization as vis

#from director import roboturdf


app = ConsoleApp()

# create a view
view = app.createView()
segmentation._defaultSegmentationView = view


'''
robotStateModel, robotStateJointController = roboturdf.loadRobotModel('robot state model', view, parent='sensors', color=roboturdf.getRobotGrayColor(), visible=True)

segmentationroutines.SegmentationContext.initWithRobot(robotStateModel)

# Move Robot in front of the table:
robotStateJointController.q[5] = -0.63677# math.radians(120)
robotStateJointController.q[0] = 0.728
robotStateJointController.q[1] = -0.7596
robotStateJointController.q[2] = 0.79788
robotStateJointController.q[33] = 0.5 # head down
robotStateJointController.push()
'''

groundHeight = 0.0
viewFrame = segmentation.transformUtils.frameFromPositionAndRPY([1, -1, groundHeight + 1.5], [0, 0, -35])

segmentationroutines.SegmentationContext.initWithUser(groundHeight, viewFrame)

# load poly data
dataDir = app.getTestingDataDirectory()
polyData = ioUtils.readPolyData(os.path.join(dataDir, 'tabletop/table-sparse-stereo.vtp'))
vis.showPolyData(polyData, 'pointcloud snapshot original', colorByName='rgb_colors')
polyData = segmentationroutines.sparsifyStereoCloud( polyData )
vis.showPolyData(polyData, 'pointcloud snapshot')

# Use only returns near the robot:
polyData = segmentation.addCoordArraysToPolyData(polyData)
polyData = segmentation.thresholdPoints(polyData, 'distance_along_view_x', [0, 1.3])

# I removed segmentTableThenFindDrills() because it used a depreciated function: 
#segmentation.segmentTableThenFindDrills(polyData, [1.2864902,  -0.93351376,  1.10208917])
segmentation.segmentTableSceneClusters(polyData, [1.2864902,  -0.93351376,  1.10208917], clusterInXY=True )

if app.getTestingInteractiveEnabled():

    v = applogic.getCurrentView()
    v.camera().SetPosition([3,3,3])
    v.camera().SetFocalPoint([0,0,0])

    view.show()
    app.showObjectModel()
    app.start()
