import os

from ddapp.consoleapp import ConsoleApp
from ddapp import ioUtils
from ddapp import segmentation
from ddapp import applogic
from ddapp import visualization as vis


app = ConsoleApp()

# create a view
view = app.createView()
segmentation._defaultSegmentationView = view

# load poly data
dataDir = app.getTestingDataDirectory()
polyData = ioUtils.readPolyData(os.path.join(dataDir, 'valve-pod-wall.vtp'))
vis.showPolyData(polyData, 'pointcloud snapshot')


# fit valve
applogic.resetCamera(viewDirection=[-1,1,0], view=view)
segmentation.segmentValveWallAuto(.195)


if app.getTestingInteractiveEnabled():
    view.show()
    app.showObjectModel()
    app.start()
