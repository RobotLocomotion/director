from director import lcmUtils
from director import objectmodel as om
from director import visualization as vis
from director import vtkAll as vtk
from director import transformUtils
from director.debugVis import DebugData
from director.shallowCopy import shallowCopy
import vicon as lcmvicon
import time
import numpy as np


class ViconVisualizer(object):
    '''
    Usage:

      viconVis = ViconVisualizer('VICON_CHANNEL_NAME')


      # You can enable visualization of edges between vicon markers,
      # but this visualization is slower.  To enable:

      viconVis.drawEdges = True

      # By default the lcm update rate is throttled to 10 hz.
      # To increase the update rate:

      viconVis.subscriber.setSpeedLimit(100)

      # Note, the constructor calls initSubscriber() automatically.
      # To remove the lcm subscriber:

      viconVis.removeSubscriber()

    '''

    def __init__(self, channel):
        self.channel = channel
        self.subscriber = None
        self.lastMessage = None
        self.unitConversion = 0.001
        self.models = {}
        self.markerGeometry = None
        self.drawEdges = False
        self.initSubscriber()

    def initSubscriber(self):
        self.subscriber = lcmUtils.addSubscriber(self.channel, lcmvicon.vicon_t, self.onMessage)
        self.subscriber.setSpeedLimit(10)

    def removeSubscriber(self):
        if not self.subscriber:
            return
        lcmUtils.removeSubscriber(self.subscriber)
        self.subscriber = None

    def getRootFolder(self):
        folder = om.getOrCreateContainer(self.channel)
        return folder

    def removeRootFolder(self):
        om.removeFromObjectModel(self.getRootFolder())

    def onMessage(self, msg):
        self.lastMessage = msg
        self.drawModels(msg)

    def getMarkerGeometry(self):

        if self.markerGeometry is None:
            d = DebugData()
            d.addSphere(np.zeros(3), radius=0.007, resolution=12)
            self.markerGeometry = shallowCopy(d.getPolyData())

        return self.markerGeometry

    def drawModels(self, msg):

        tNow = time.time()

        for model in msg.models:
            self.drawModel(model)

        elapsed = time.time() - tNow
        #print 'rate:', 1/elapsed

    def createMarkerObjects(self, numberOfMarkers, modelFolder, modelName, modelColor):

        geom = self.getMarkerGeometry()

        def makeMarker(i):
            obj = vis.showPolyData(shallowCopy(geom), modelName + ' marker %d' % i, color=modelColor, parent=modelFolder)
            vis.addChildFrame(obj)
            return obj

        return [makeMarker(i) for i in range(numberOfMarkers)]

    def drawModel(self, model):

        modelFolder = om.getOrCreateContainer(model.name, parentObj=self.getRootFolder())
        markerFolder = om.getOrCreateContainer('markers', parentObj=modelFolder)
        modelName = model.name

        markerObjects = markerFolder.children()

        if len(markerObjects) != model.nummarkers:
            for obj in markerObjects:
                om.removeFromObjectModel(obj)

            modelColor = vis.getRandomColor()
            markerObjects = self.createMarkerObjects(model.nummarkers, markerFolder, modelName, modelColor)
            self.models[modelName] = markerObjects

        if len(markerObjects):
            modelColor = markerObjects[0].getProperty('Color')

        for i, marker in enumerate(model.markers):
            xyz = np.array(marker.xyz)*self.unitConversion
            markerFrame = vtk.vtkTransform()
            markerFrame.Translate(xyz)
            markerObjects[i].getChildFrame().copyFrame(markerFrame)

        if self.drawEdges:
            d = DebugData()
            for m1 in model.markers:
                xyz = np.array(m1.xyz)*self.unitConversion
                for m2 in model.markers:
                    xyz2 = np.array(m2.xyz)*self.unitConversion
                    d.addLine(xyz, xyz2)
            edges = shallowCopy(d.getPolyData())
            vis.updatePolyData(edges, modelName + ' edges', color=modelColor, parent=modelFolder)
        else:
            edgesObj = om.findObjectByName(modelName + ' edges')
            om.removeFromObjectModel(edgesObj)

        computeModelFrame = True
        if computeModelFrame:
            pos = np.array(model.segments[0].T)*self.unitConversion
            rpy = np.array(model.segments[0].A)
            modelFrame = transformUtils.frameFromPositionAndRPY(pos, np.degrees(rpy))
            vis.updateFrame(modelFrame, modelName + ' frame', parent=modelFolder, scale=0.1)
