import os

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

# load poly data
dataDir = app.getTestingDataDirectory()
polyData = ioUtils.readPolyData(os.path.join(dataDir, 'valve/valve-sparse-stereo.pcd'))
vis.showPolyData(polyData, 'pointcloud snapshot original', colorByName='rgb_colors')
polyData = segmentationroutines.sparsifyStereoCloud( polyData )
vis.showPolyData(polyData, 'pointcloud snapshot')

# fit valve and lever
segmentation.segmentValveWallAuto(.2, mode='valve', removeGroundMethod=segmentation.removeGroundSimple )

if app.getTestingInteractiveEnabled():

    v = applogic.getCurrentView()
    v.camera().SetPosition([3,3,3])
    v.camera().SetFocalPoint([0,0,0])

    view.show()
    app.showObjectModel()
    app.start()
