import os
import math
from ddapp.consoleapp import ConsoleApp
from ddapp import ioUtils
from ddapp import segmentation
from ddapp import applogic
from ddapp import visualization as vis
from ddapp import continuouswalkingdemo
from ddapp import robotsystem
from ddapp import objectmodel as om
from ddapp import ikplanner
from ddapp import navigationpanel
from ddapp import cameraview

app = ConsoleApp()
dataDir = app.getTestingDataDirectory()
# create a view
view = app.createView()
segmentation._defaultSegmentationView = view

#footstepsPanel = footstepsdriverpanel.init(footstepsDriver, robotStateModel, robotStateJointController, mapServerSource)
footstepsPanel = None
robotsystem.create(view, globals())



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


#navigationPanel = navigationpanel.init(robotStateJointController, footstepsDriver)
navigationPanel = None
continuouswalkingDemo = continuouswalkingdemo.ContinousWalkingDemo(robotStateModel, footstepsPanel, robotStateJointController, ikPlanner,
                                                                       teleopJointController, navigationPanel, cameraview)
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
