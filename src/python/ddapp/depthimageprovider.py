import ddapp.vtkAll as vtk
import ddapp.vtkNumpy as vnp
import drc as lcmdrc
import numpy as np
import time

class DepthImageProvider(object):

    def __init__(self):

        self.source = vtk.vtkMapServerSource()
        self.source.Start()

    def waitForSceneHeight(self):
        viewId = lcmdrc.data_request_t.HEIGHT_MAP_SCENE
        while self.source.GetCurrentMapId(viewId) < 0:
            time.sleep(0.1)

    def getSceneHeightData(self):
        return self.getDepthMapData(lcmdrc.data_request_t.HEIGHT_MAP_SCENE)

    def getDepthMapData(self, viewId):

        mapId = self.source.GetCurrentMapId(viewId)
        if mapId < 0:
            return None, None

        depthImage = vtk.vtkImageData()
        transform = vtk.vtkTransform()
        # print "getting depth image for viewId {:d} mapId {:d}".format(viewId, mapId)
        self.source.GetDataForMapId(viewId, mapId, depthImage, transform)

        dims = depthImage.GetDimensions()
        d = vnp.getNumpyFromVtk(depthImage, 'ImageScalars')
        d = d.reshape(dims[1], dims[0])
        t = np.array([[transform.GetMatrix().GetElement(r, c) for c in xrange(4)] for r in xrange(4)])

        return d, t
