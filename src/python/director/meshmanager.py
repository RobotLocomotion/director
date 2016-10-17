from director import lcmobjectcollection
from director import geometryencoder
from director import ioUtils
from director.uuidutil import newUUID
import os


class MeshManager(object):

    def __init__(self):
        self.meshes = {}
        self.cacheDirectory = '/tmp'
        self.cacheDataType = 'stl'
        self.collection = lcmobjectcollection.LCMObjectCollection(channel='MESH_COLLECTION_COMMAND')
        self.collection.connectDescriptionUpdated(self._onDescriptionUpdated)

    def add(self, polyData, publish=True):
        meshId = newUUID()
        self.meshes[meshId] = polyData
        if publish and self.collection:
            self._publishMesh(meshId)
        return meshId

    def get(self, meshId):
        return self.meshes.get(meshId)

    def getFilesystemFilename(self, meshId):
        if meshId in self.meshes:
            filename = os.path.join(self.cacheDirectory, '%s.%s' % (meshId, self.cacheDataType))
            if not os.path.isfile(filename):
                ioUtils.writePolyData(self.get(meshId), filename)
            return filename
        return None

    def _publishMesh(self, meshId):
        polyData = self.meshes[meshId]
        self.collection.updateDescription(dict(uuid=meshId, data=geometryencoder.encodePolyData(polyData)), notify=False)

    def _onDescriptionUpdated(self, collection, descriptionId):
        desc = collection.getDescription(descriptionId)
        meshId = desc['uuid']
        if meshId not in self.meshes:
            polyData = geometryencoder.decodePolyData(desc['data'])
            self.meshes[meshId] = polyData
            #print 'decoded polydata with %d points' % polyData.GetNumberOfPoints()
