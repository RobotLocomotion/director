import time
from ddapp import segmentationroutines
from ddapp import segmentation
from ddapp.timercallback import TimerCallback
from ddapp.visualization import *

class TrackDrillOnTable(object):

    def __init__(self):
        self.tableCentroid = None

    def updateFit(self):
        # get and display: .1sec
        polyData = segmentation.getDisparityPointCloud()
        if (polyData is None):
            return

        updatePolyData(polyData, 'pointcloud snapshot', colorByName='rgb_colors', visible=False)

        t0 = time.time()
        if (self.tableCentroid is None):
            # initial fit .75 sec
            print "Boot Strapping tracker"
            self.tableCentroid = segmentation.findAndFitDrillBarrel(polyData)
        else:
            # refit .07 sec
            #print "current centroid"
            #print self.tableCentroid

            viewFrame = segmentationroutines.SegmentationContext.getGlobalInstance().getViewFrame()
            forwardDirection = np.array([1.0, 0.0, 0.0])
            viewFrame.TransformVector(forwardDirection, forwardDirection)
            robotOrigin = viewFrame.GetPosition()
            robotForward =forwardDirection    

            fitResults = []
            drillFrame = segmentation.segmentDrillBarrelFrame(self.tableCentroid, polyData, robotForward)
            clusterObj = updatePolyData(polyData, 'surface cluster refit', color=[1,1,0], parent=segmentation.getDebugFolder(), visible=False)
            fitResults.append((clusterObj, drillFrame))

            segmentation.sortFittedDrills(fitResults, robotOrigin, robotForward)




class PointerTracker(object):
    '''
    See segmentation.estimatePointerTip() documentation.
    '''
    def __init__(self, robotModel, stereoPointCloudItem):
        self.robotModel = robotModel
        self.stereoPointCloudItem = stereoPointCloudItem
        self.timer = TimerCallback(targetFps=5)
        self.timer.callback = self.updateFit

    def start(self):
        self.timer.start()

    def stop(self):
        self.timer.stop()

    def cleanup(self):
        om.removeFromObjectModel(om.findObjectByName('segmentation'))

    def updateFit(self, polyData=None):

        #if not self.stereoPointCloudItem.getProperty('Visible'):
        #    return

        if not polyData:
            self.stereoPointCloudItem.update()
            polyData = self.stereoPointCloudItem.polyData

        if not polyData or not polyData.GetNumberOfPoints():
            self.cleanup()
            return

        self.tipPosition = segmentation.estimatePointerTip(self.robotModel, polyData)
        if self.tipPosition is None:
            self.cleanup()

    def getPointerTip(self):
        return self.tipPosition
