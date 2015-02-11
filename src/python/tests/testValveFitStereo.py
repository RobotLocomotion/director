import os

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
