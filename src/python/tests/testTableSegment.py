import os
import math
from director.consoleapp import ConsoleApp
from director import ioUtils
from director import segmentation
from director import segmentationroutines
from director import applogic
from director import visualization as vis

from director import roboturdf
from director import transformUtils

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



p1 = [-1.58661389,  2.91242337,  0.79958105]
# p2 to be depreciated
p2 = p1


mode = 0


if (mode == 0):
    # previous: Faulty frames
    data = segmentation.segmentTableScene(polyData, p1)
    vis.showClusterObjects(data.clusters + [data.table], parent='segmentation')
    print data

elif (mode == 1):
    # perferred table segmentation approach

    tableData = segmentation.segmentTableEdge(polyData, p1, p2)
    print tableData
    vis.showClusterObjects([tableData], parent='segmentation')


    pose = transformUtils.poseFromTransform(tableData.frame)
    desc = dict(classname='MeshAffordanceItem', Name='table', Color=[0,1,0], pose=pose)
    #aff = self.affordanceManager.newAffordanceFromDescription(desc)
    #aff.setPolyData(tableData.mesh)

    tableBox = vis.showPolyData(tableData.box, 'table box', parent='segmentation', color=[0,1,0], visible=False)
    tableBox.actor.SetUserTransform(tableData.frame)

elif (mode == 2):
	# code taken from tableboxdemo, to be removed
    tableData = segmentation.segmentTableEdge(self.getInputPointCloud(), p1, p2)

    pose = transformUtils.poseFromTransform(tableData.frame)
    desc = dict(classname='MeshAffordanceItem', Name='table', Color=[0,1,0], pose=pose)
    aff = self.affordanceManager.newAffordanceFromDescription(desc)
    aff.setPolyData(tableData.mesh)

    self.tableData = tableData

    tableBox = vis.showPolyData(tableData.box, 'table box', parent=aff, color=[0,1,0], visible=False)
    tableBox.actor.SetUserTransform(tableData.frame)

    # how far back to stand - from the middle of the table
    # -0.6 is too far. reduced 0.5 was too low. now trying -0.55
    relativeStance = transformUtils.frameFromPositionAndRPY([-0.55, 0, 0],[0,0,0])
    self.computeTableStanceFrame(relativeStance)

    # automatically add the box on the table (skip user segmentation)
    boxFrame = transformUtils.copyFrame(tableData.frame)
    boxFrame.PreMultiply()
    tableToBoxFrame = transformUtils.frameFromPositionAndRPY([-0.05, 0, 0.15], [0,0, 0])
    boxFrame.Concatenate(tableToBoxFrame)
    self.spawnBlockAffordanceAtFrame(boxFrame)

    safeStance = transformUtils.frameFromPositionAndRPY([-1, 0, 0],[0,0,0])
    self.computeTableStanceFrame(safeStance, 'safe stance frame')

    facingStance = transformUtils.frameFromPositionAndRPY([-1, 0, 0],[0,0,-60.0])
    self.computeTableStanceFrame(facingStance, 'facing stance frame')



if app.getTestingInteractiveEnabled():
    view.show()
    app.showObjectModel()
    app.start()
