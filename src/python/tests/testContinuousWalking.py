import os
import math
from ddapp.consoleapp import ConsoleApp
from ddapp import ioUtils
from ddapp import segmentation
from ddapp import segmentationroutines
from ddapp import footstepsdriver
from ddapp import footstepsdriverpanel
from ddapp import applogic
from ddapp import visualization as vis
from ddapp import continuouswalkingdemo
from ddapp import perception
import ddapp.objectmodel as om
from ddapp import roboturdf

app = ConsoleApp()
dataDir = app.getTestingDataDirectory()
# create a view
view = app.createView()
segmentation._defaultSegmentationView = view


#multisenseDriver, mapServerSource = perception.init(view)
robotStateModel, robotStateJointController = roboturdf.loadRobotModel('robot state model', view, parent='sensors', color=roboturdf.getRobotGrayColor(), visible=True)
footstepsDriver = footstepsdriver.FootstepsDriver(robotStateJointController)
#footstepsPanel = footstepsdriverpanel.init(footstepsDriver, robotStateModel, robotStateJointController, mapServerSource)
footstepsPanel = None
segmentationroutines.SegmentationContext.initWithRobot(robotStateModel)





def processSingleBlock(robotStateModel, whichFile=0):
    if (whichFile == 0):
        polyData = ioUtils.readPolyData(os.path.join(dataDir, 'tabletop/table_top_45.vtp'))
    else:
        polyData = ioUtils.readPolyData(os.path.join(dataDir, 'terrain/block_top.vtp'))

    standingFootName = 'l_foot'
    standingFootFrame = robotStateModel.getLinkFrame(standingFootName)
    cwdemo.findMinimumBoundingRectangle(polyData, standingFootFrame)


def processSnippet():

    obj = om.getOrCreateContainer('continuous')
    om.getOrCreateContainer('cont debug', obj)

    if (continuouswalkingDemo.processContinuousStereo):
        polyData = ioUtils.readPolyData(os.path.join(dataDir, 'terrain/block_snippet_stereo.vtp'))
        polyData = segmentation.applyVoxelGrid(polyData, leafSize=0.01)
    else:
        polyData = ioUtils.readPolyData(os.path.join(dataDir, 'terrain/block_snippet.vtp'))


    vis.updatePolyData( polyData, 'walking snapshot trimmed', parent='continuous')
    standingFootName = 'l_foot'

    standingFootFrame = robotStateModel.getLinkFrame(standingFootName)
    vis.updateFrame(standingFootFrame, standingFootName, parent='continuous', visible=False)

    # Step 2: find all the surfaces in front of the robot (about 0.75sec)
    clusters = segmentation.findHorizontalSurfaces(polyData)
    if (clusters is None):
        print "No cluster found, stop walking now!"
        return

    # Step 3: find the corners of the minimum bounding rectangles
    blocks,groundPlane = cwdemo.extractBlocksFromSurfaces(clusters, standingFootFrame)

    footsteps = cwdemo.placeStepsOnBlocks(blocks, groundPlane, standingFootName, standingFootFrame)

    cwdemo.drawFittedSteps(footsteps)
    # cwdemo.sendPlanningRequest(footsteps)


continuouswalkingDemo = continuouswalkingdemo.ContinousWalkingDemo(robotStateModel, footstepsPanel, robotStateJointController)
cwdemo = continuouswalkingDemo

# test 1
processSingleBlock(robotStateModel, 1)
# test 2 - Table:
processSingleBlock(robotStateModel, 0)

# test 3
processSnippet()

# test 4
continuouswalkingDemo.processContinuousStereo = True
processSnippet()

if app.getTestingInteractiveEnabled():
    view.show()
    app.showObjectModel()
    app.start()
