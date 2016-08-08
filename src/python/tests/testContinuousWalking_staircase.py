import os
import math
from director import robotsystem
from director.consoleapp import ConsoleApp
from director import ioUtils
from director import segmentation
from director import applogic
from director import visualization as vis
from director import continuouswalkingdemo
from director import objectmodel as om
from director import ikplanner
from director import navigationpanel
from director import cameraview
from director import playbackpanel
from director import drcargs

import drc as lcmdrc

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
        polyData = ioUtils.readPolyData(os.path.join(dataDir, 'terrain/terrain_simple_ihmc.vtp'))
        vis.updatePolyData( polyData, 'terrain_simple_ihmc.vtp', parent='continuous')
    else:
        polyData = ioUtils.readPolyData(os.path.join(dataDir, 'terrain/terrain_flagstones_ihmc.vtp'))
        cwdemo.chosenTerrain = 'stairs'
        cwdemo.supportContact = lcmdrc.footstep_params_t.SUPPORT_GROUPS_MIDFOOT_TOE
        vis.updatePolyData( polyData, 'terrain_stairs_ihmc.vtp', parent='continuous')
    
    if drcargs.args().directorConfigFile.find('atlas') != -1:
    	standingFootName = 'l_foot'
    else:
        standingFootName = 'leftFoot'

    standingFootFrame = robotStateModel.getLinkFrame(standingFootName)
    vis.updateFrame(standingFootFrame, standingFootName, parent='continuous', visible=False)
    
    # Step 1: filter the data down to a box in front of the robot:
    polyData = cwdemo.getRecedingTerrainRegion(polyData, footstepsDriver.getFeetMidPoint(robotStateModel))

    # Step 2: find all the surfaces in front of the robot (about 0.75sec)
    clusters = segmentation.findHorizontalSurfaces(polyData)
    if (clusters is None):
        print "No cluster found, stop walking now!"
        return
    
    # Step 3: find the corners of the minimum bounding rectangles
    blocks,match_idx,groundPlane = cwdemo.extractBlocksFromSurfaces(clusters, standingFootFrame)

    footsteps = cwdemo.placeStepsOnBlocks(blocks, groundPlane, standingFootName, standingFootFrame)

    cwdemo.drawFittedSteps(footsteps)
    

def processSnippet():

    obj = om.getOrCreateContainer('continuous')
    om.getOrCreateContainer('cont debug', obj)

    if (continuouswalkingDemo.processContinuousStereo):
        polyData = ioUtils.readPolyData(os.path.join(dataDir, 'terrain/block_snippet_stereo.vtp'))
        polyData = segmentation.applyVoxelGrid(polyData, leafSize=0.01)
    else:
        polyData = ioUtils.readPolyData(os.path.join(dataDir, 'terrain/block_snippet.vtp'))


    vis.updatePolyData( polyData, 'walking snapshot trimmed', parent='continuous')
    if drcargs.args().directorConfigFile.find('atlas') != -1:
    	standingFootName = 'l_foot'
    else:
        standingFootName = 'leftFoot'

    standingFootFrame = robotStateModel.getLinkFrame(standingFootName)
    vis.updateFrame(standingFootFrame, standingFootName, parent='continuous', visible=False)

    # Step 2: find all the surfaces in front of the robot (about 0.75sec)
    clusters = segmentation.findHorizontalSurfaces(polyData)
    if (clusters is None):
        print "No cluster found, stop walking now!"
        return

    # Step 3: find the corners of the minimum bounding rectangles
    blocks,match_idx,groundPlane = cwdemo.extractBlocksFromSurfaces(clusters, standingFootFrame)

    footsteps = cwdemo.placeStepsOnBlocks(blocks, groundPlane, standingFootName, standingFootFrame)

    cwdemo.drawFittedSteps(footsteps)
    # cwdemo.sendPlanningRequest(footsteps)


#navigationPanel = navigationpanel.init(robotStateJointController, footstepsDriver)
navigationPanel = None
continuouswalkingDemo = continuouswalkingdemo.ContinousWalkingDemo(robotStateModel, footstepsPanel, footstepsDriver, playbackpanel, robotStateJointController, ikPlanner,
                                                                       teleopJointController, navigationPanel, cameraview)

cwdemo = continuouswalkingDemo

# test 1 - Simple:
#processSingleBlock(robotStateModel, 0)
# test 2 - Staircase:
processSingleBlock(robotStateModel, 1)

'''
# test 3
processSnippet()

# test 4
continuouswalkingDemo.processContinuousStereo = True
processSnippet()
'''

if app.getTestingInteractiveEnabled():
    view.show()
    app.showObjectModel()
    app.start()

