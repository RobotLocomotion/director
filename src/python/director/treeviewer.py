import json
import math
import os
import re
import time
import warnings
import numpy as np
from collections import namedtuple

from director import objectmodel as om
from director import applogic as app
from director import lcmUtils
from director import transformUtils
from director.debugVis import DebugData
from director import ioUtils
from director import filterUtils
from director.shallowCopy import shallowCopy
from director import vtkAll as vtk
from director import vtkNumpy as vnp
from director import visualization as vis
from director import packagepath
from director.shallowCopy import shallowCopy

import robotlocomotion as lcmrl

from PythonQt import QtGui

USE_TEXTURE_MESHES = True
USE_SHADOWS = False
DEFAULT_COLOR = [1, 1, 1, 1]

class ViewerStatus:
    OK = 0
    MISSING_PATHS = 1
    ERROR_UNKNOWN_FORMAT = -1
    ERROR_UNKNOWN_FORMAT_VERSION = -2
    ERROR_HANDLING_REQUEST = -3
    ERROR_UKNOWN_REQUEST_TYPE = -4


class ViewerResponse(namedtuple("ViewerResponse", ["status", "data"])):
    def toJson(self):
        return dict(status=self.status, **self.data)


def transformFromDict(pose_data):
    return transformUtils.transformFromPose(
        pose_data.get("translation", [0, 0, 0]),
        pose_data.get("quaternion", [1, 0, 0, 0]))


class Geometry(object):
    TextureCache = {}
    PackageMap = None

    @staticmethod
    def createBox(params):
        d = DebugData()
        color = params.get("color", DEFAULT_COLOR)[:3]
        d.addCube(dimensions=params["lengths"], center=(0, 0, 0), color=color)
        return [d.getPolyData()]

    @staticmethod
    def createSphere(params):
        d = DebugData()
        color = params.get("color", DEFAULT_COLOR)[:3]
        d.addSphere(center=(0, 0, 0), radius=params["radius"], color=color)
        return [d.getPolyData()]

    @staticmethod
    def createCylinder(params):
        d = DebugData()
        color = params.get("color", DEFAULT_COLOR)[:3]
        d.addCylinder(center=(0, 0, 0),
                      axis=(0, 0, 1),
                      radius=params["radius"],
                      length=params["length"],
                      color=color)
        return [d.getPolyData()]

    @staticmethod
    def createCapsule(params):
        d = DebugData()
        radius = params["radius"]
        length = params["length"]
        color = params.get("color", DEFAULT_COLOR)[:3]
        d.addCylinder(center=(0, 0, 0),
                      axis=(0, 0, 1),
                      radius=radius,
                      length=length,
                      color=color)
        d.addSphere(center=(0, 0, length / 2.0), radius=radius, color=color)
        d.addSphere(center=(0, 0, -length / 2.0), radius=radius, color=color)
        return [d.getPolyData()]

    @staticmethod
    def createEllipsoid(params):
        d = DebugData()
        color = params.get("color", DEFAULT_COLOR)[:3]
        radii = params["radii"]
        d.addEllipsoid(center=(0, 0, 0), radii=radii, color=color)
        return [d.getPolyData()]

    @staticmethod
    def createMeshFromFile(params):
        polyDataList = Geometry.loadPolyDataMeshes(params["filename"])
        if "scale" in params:
            polyDataList = Geometry.scaleGeometry(polyDataList,
                                                  params["scale"])
        return polyDataList

    @staticmethod
    def createMeshFromData(params):
        verts = np.asarray(params["vertices"])
        faces = np.asarray(params["faces"])
        return [Geometry.createPolyDataFromMeshArrays(verts, faces)]

    @staticmethod
    def createPointcloud(params):
        polyData = vnp.numpyToPolyData(np.asarray(params["points"]),
                                       createVertexCells=True)
        if "channels" in params:
            Geometry.addColorChannels(polyData, params["channels"])
        return [polyData]

    @staticmethod
    def createPlanarLidar(params):
        ranges = np.asarray(params["ranges"])
        angle_stop = (params["angle_start"] +
                      len(ranges) * params["angle_step"])
        angles = np.arange(
            params["angle_start"],
            angle_stop,
            params["angle_step"])
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros(x.shape)
        points = np.vstack((x, y, z)).T
        polyData = vnp.numpyToPolyData(points, createVertexCells=True)
        if "channels" in params:
            Geometry.addColorChannels(polyData, params["channels"])
        return [polyData]

    @staticmethod
    def createTriad(params):
        d = DebugData()
        d.addFrame(vtk.vtkTransform(), scale=params.get("scale", 1.0),
                   tubeRadius=0.002 if params.get("tube", False) else 0.0)
        return [d.getPolyData()]

    @staticmethod
    def createPolyLine(params):
        d = DebugData()
        points = [np.asarray(p) for p in params["points"]]
        color = params.get("color", DEFAULT_COLOR)[:3]
        radius = params.get("radius", 0.01)
        startHead = params.get("start_head", False)
        endHead = params.get("end_head", False)
        headRadius = params.get("head_radius", 0.05)
        headLength = params.get("head_length", headRadius)
        isClosed = params.get("closed", False)
        if startHead:
            normal = points[0] - points[1]
            normal = normal / np.linalg.norm(normal)
            points[0] = points[0] - 0.5 * headLength * normal
            d.addCone(origin=points[0], normal=normal, radius=headRadius,
                      height=headLength, color=color, fill=True)
        if endHead:
            normal = points[-1] - points[-2]
            normal = normal / np.linalg.norm(normal)
            points[-1] = points[-1] - 0.5 * headLength * normal
            d.addCone(origin=points[-1], normal=normal, radius=headRadius,
                      height=headLength, color=color, fill=True)
        d.addPolyLine(points, isClosed, radius=radius, color=color)
        return [d.getPolyData()]

    @staticmethod
    def createPolyData(params):
        if params["type"] == "box":
            return Geometry.createBox(params)
        elif params["type"] == "sphere":
            return Geometry.createSphere(params)
        elif params["type"] == "cylinder":
            return Geometry.createCylinder(params)
        elif params["type"] == "capsule":
            return Geometry.createCapsule(params)
        elif params["type"] == "ellipsoid":
            return Geometry.createEllipsoid(params)
        elif params["type"] == "mesh_file":
            return Geometry.createMeshFromFile(params)
        elif params["type"] == "mesh_data":
            return Geometry.createMeshFromData(params)
        elif params["type"] == "pointcloud":
            return Geometry.createPointcloud(params)
        elif params["type"] == "planar_lidar":
            return Geometry.createPlanarLidar(params)
        elif params["type"] == "triad":
            return Geometry.createTriad(params)
        elif params["type"] == "line":
            return Geometry.createPolyLine(params)
        else:
            raise Exception(
                "Unsupported geometry type: {}".format(params["type"]))

    @staticmethod
    def createPolyDataFromMeshArrays(pts, faces):
        pd = vtk.vtkPolyData()
        pd.SetPoints(vtk.vtkPoints())

        if pts.size > 0:
            pd.GetPoints().SetData(vnp.getVtkFromNumpy(pts.copy()))

            cells = vtk.vtkCellArray()
            tri = vtk.vtkTriangle()
            setId = tri.GetPointIds().SetId  # bind the method for convenience
            for face in faces:
                assert len(face) == 3, "Non-triangular faces are not supported."
                setId(0, face[0])
                setId(1, face[1])
                setId(2, face[2])
                cells.InsertNextCell(tri)

            pd.SetPolys(cells)
        return pd

    @staticmethod
    def scaleGeometry(polyDataList, scale):
        if len(scale) == 1:
            scale_x = scale_y = scale_z = scale
        else:
            assert len(scale) == 3
            scale_x, scale_y, scale_z = scale

        if scale_x != 1.0 or scale_y != 1.0 or scale_z != 1.0:
            t = vtk.vtkTransform()
            t.Scale(scale_x, scale_y, scale_z)
            polyDataList = [filterUtils.transformPolyData(polyData, t) for polyData in polyDataList]

        return polyDataList

    @staticmethod
    def transformGeometry(polyDataList, geom):
        if "transform" in geom:
            t = transformFromDict(geom["transform"])
            polyDataList = [filterUtils.transformPolyData(polyData, t) for polyData in polyDataList]
        return polyDataList

    @staticmethod
    def computeNormals(polyDataList):

        def addNormals(polyData):
            hasNormals = polyData.GetPointData().GetNormals() is not None
            return polyData if hasNormals else filterUtils.computeNormals(polyData)

        return [addNormals(polyData) for polyData in polyDataList]


    @staticmethod
    def getTextureFileName(polyData):
        textureArray = vtk.vtkStringArray.SafeDownCast(polyData.GetFieldData().GetAbstractArray('texture_filename'))
        if not textureArray:
            return None
        return textureArray.GetValue(0)

    @staticmethod
    def loadTextureForMesh(polyData, meshFileName):

        textureFileName = Geometry.getTextureFileName(polyData)
        if textureFileName in Geometry.TextureCache or textureFileName is None:
            return

        if not os.path.isabs(textureFileName):
            baseDir = os.path.dirname(meshFileName)
            imageFile = os.path.join(baseDir, textureFileName)
        else:
            imageFile = textureFileName

        if not os.path.isfile(imageFile):
            print('cannot find texture file:', textureFileName)
            return

        image = ioUtils.readImage(imageFile)
        if not image:
            print('failed to load image file:', imageFile)
            return

        texture = vtk.vtkTexture()
        texture.SetInputData(image)
        texture.EdgeClampOn()
        texture.RepeatOn()

        Geometry.TextureCache[textureFileName] = texture

    @staticmethod
    def resolvePackageFilename(filename):

        if not packagepath.PackageMap.isPackageUrl(filename):
            return filename

        if Geometry.PackageMap is None:
            import pydrake
            m = packagepath.PackageMap()
            m.populateFromSearchPaths([pydrake.getDrakePath()])
            m.populateFromEnvironment(['DRAKE_PACKAGE_PATH', 'ROS_PACKAGE_PATH'])
            Geometry.PackageMap = m

        return Geometry.PackageMap.resolveFilename(filename) or filename

    @staticmethod
    def loadPolyDataMeshes(filename):

        filename = Geometry.resolvePackageFilename(filename)
        basename, ext = os.path.splitext(filename)

        preferredExtensions = ['.vtm', '.vtp', '.obj']

        for x in preferredExtensions:
            if os.path.isfile(basename + x):
                filename = basename + x
                break

        if not os.path.isfile(filename):
            print('warning, cannot find file:', filename)
            return []

        if filename.endswith('vtm'):
            polyDataList = ioUtils.readMultiBlock(filename)
        else:
            polyDataList = [ioUtils.readPolyData(filename)]

        if USE_TEXTURE_MESHES:
            for polyData in polyDataList:
                Geometry.loadTextureForMesh(polyData, filename)

        return polyDataList

    @staticmethod
    def createPolyDataForGeometry(geom):
        polyDataList = Geometry.createPolyData(geom)
        polyDataList = Geometry.transformGeometry(polyDataList, geom)
        polyDataList = Geometry.computeNormals(polyDataList)
        return polyDataList

    @staticmethod
    def addColorChannels(polyData, channels):
        if "intensity" in channels:
            colorBy = "intensity"
            intensity = np.asarray(channels["intensity"]) * 255
            vnp.addNumpyToVtk(polyData,
                              intensity.astype(np.uint8),
                              "intensity")
        if "rgb" in channels:
            colorBy = "rgb"  # default to rgb if provided
            colorArray = np.asarray(channels["rgb"]) * 255
            vnp.addNumpyToVtk(polyData, colorArray.astype(np.uint8), "rgb")

    def __init__(self, geomDatas):
        polyDatas = []
        for geomData in geomDatas:
            polyDatas.extend(Geometry.createPolyDataForGeometry(geomData))

        self.polyData = filterUtils.appendPolyData(polyDatas)
        if geomDatas:
            self.color = geomDatas[0].get("color", DEFAULT_COLOR)
        else:
            self.color = DEFAULT_COLOR

    def createPolyDataItem(self, name="geometry"):
        polyDataItem = vis.PolyDataItem(name, self.polyData, view=None)
        polyDataItem.setProperty("Point Size", 2)
        self.updatePolyDataItemProperties(polyDataItem)
        return polyDataItem

    def updatePolyDataItemProperties(self, polyDataItem):
        polyDataItem._updateColorByProperty()

        polyDataItem.setProperty('Alpha', self.color[3])
        polyDataItem.actor.SetTexture(
            Geometry.TextureCache.get(
                Geometry.getTextureFileName(self.polyData)))

        if polyDataItem.actor.GetTexture():
            polyDataItem.setProperty('Color',
                                     QtGui.QColor(255, 255, 255))
        else:
            polyDataItem.setProperty(
                'Color',
                QtGui.QColor(*(255 * np.asarray(self.color[:3]))))

        if USE_SHADOWS:
            polyDataItem.shadowOn()


def findPathToAncestor(fromItem, toItem):
    path = [fromItem]
    while fromItem is not toItem:
        parent = fromItem.parent()
        if parent is None:
            raise ValueError(
                "Cannot find a path from {} to {}".format(fromItem, toItem))
        path.append(parent)
        fromItem = parent
    return path


class TreeViewer(object):
    name = "Remote Tree Viewer"

    def __init__(self, view):

        self.subscriber = None
        self.view = view
        self.itemToPathCache = {}
        self.pathToItemCache = {}
        self.client_id_regex = re.compile(r'\<(.*)\>')
        self.enable()
        self.sendStatusMessage(
            0, ViewerResponse(ViewerStatus.OK, {"ready": True}))

    def _addSubscriber(self):
        self.subscriber = lcmUtils.addSubscriber(
            'DIRECTOR_TREE_VIEWER_REQUEST.*',
            lcmrl.viewer2_comms_t,
            self.onViewerRequest,
            callbackNeedsChannel=True)
        # Note: from discussion with @patmarion, there's a bug in the lcmUtils subscriber
        # when dealing with regex channels:
        #   > If you subscribe to MY_CHANNEL_*, and two messages arrive back to back
        #   > (MY_CHANNEL_FOO, foo_data) and (MY_CHANNEL_BAR, bar_data), then the default
        #   > behavior in the lcm subscriber is to only call your callback once (if notify
        #   > all messages = false), and it calls callback(MY_CHANNEL_FOO, bar_data)
        #
        # However, this can be avoided by notifying for *all* messages, which is what we
        # want anyway:
        self.subscriber.setNotifyAllMessagesEnabled(True)

    def _removeSubscriber(self):
        lcmUtils.removeSubscriber(self.subscriber)
        self.subscriber = None

    def isEnabled(self):
        return self.subscriber is not None

    def setEnabled(self, enabled):
        if enabled and not self.isEnabled():
            self._addSubscriber()
        elif not enabled and self.isEnabled():
            self._removeSubscriber()

    def enable(self):
        self.setEnabled(True)

    def disable(self):
        self.setEnabled(False)

    def sendStatusMessage(self, timestamp, response, client_id=""):
        msg = lcmrl.viewer2_comms_t()
        msg.format = "treeviewer_json"
        msg.format_version_major = 1
        msg.format_version_minor = 0
        data = dict(timestamp=timestamp, **response.toJson())
        msg.data = bytearray(json.dumps(data), encoding='utf-8')
        msg.num_bytes = len(msg.data)
        if client_id:
            channel = "DIRECTOR_TREE_VIEWER_RESPONSE_<{:s}>".format(client_id)
        else:
            channel = "DIRECTOR_TREE_VIEWER_RESPONSE"
        lcmUtils.publish(channel, msg)

    def decodeCommsMsg(self, msg):
        if msg.format == "treeviewer_json":
            if msg.format_version_major == 1 and msg.format_version_minor == 0:
                data = json.loads(msg.data.decode())
                return data, ViewerResponse(ViewerStatus.OK, {})
            else:
                return None, ViewerResponse(ViewerStatus.ERROR_UNKNOWN_FORMAT_VERSION,
                                            {"supported_formats": {
                                                 "treeviewer_json": ["1.0"]
                                            }})
        else:
            return None, ViewerResponse(ViewerStatus.ERROR_UNKNOWN_FORMAT,
                                        {"supported_formats": {
                                             "treeviewer_json": ["1.0"]
                                        }})

    def onViewerRequest(self, msg, channel="DIRECTOR_TREE_VIEWER_REQUEST"):
        match = self.client_id_regex.search(channel)
        if match:
            client_id = match.group(1)  # MatchObject.group is 1-indexed
        else:
            warnings.warn("To reduce cross-talk, clients should append a unique client ID inside <> characters to their DIRECTOR_TREE_VIEWER_REQUEST channel name. For example: DIRECTOR_TREE_VIEWER_REQUEST_<foo>. The client should also subscribe to the equivalent response channel: DIRECTOR_TREE_VIEWER_RESPONSE_<foo>")
            client_id = ""
        data, response = self.decodeCommsMsg(msg)
        if data is None:
            self.sendStatusMessage(msg.utime,
                                   response, client_id)
        else:
            response = self.handleViewerRequest(data)
            self.sendStatusMessage(msg.utime,
                                   response, client_id)

    def handleViewerRequest(self, data):
        deletedPaths = set()
        addedGeometries = set()
        setTransforms = set()
        missingPaths = set()
        for command in data["delete"]:
            deletedPaths.add(tuple(self.handleDeletePath(command)))
        if "load" in data:
            warnings.warn("The 'load' comand has been deprecated. Please use 'setgeometry' instead", DeprecationWarning)
            data["setgeometry"] = data["load"]
        if "draw" in data:
            warnings.warn("The 'draw' cmmand has been deprecated. Please use 'settransform' instead", DeprecationWarning)
            data["settransform"] = data["draw"]
        for command in data["setgeometry"]:
            addedGeometries.add(tuple(self.handleSetGeometry(command)))
        for command in data["settransform"]:
            path, missingGeometry = self.handleSetTransform(command)
            setTransforms.add(tuple(path))
            if missingGeometry:
                missingPaths.add(tuple(path))
        result = {
            "deleted_paths": list(deletedPaths),
            "added_geometries": list(addedGeometries),
            "set_transforms": list(setTransforms),
            "missing_paths": list(missingPaths)
        }
        self.view.render()
        # print "result:", result
        if not missingPaths:
            return ViewerResponse(ViewerStatus.OK, result)
        else:
            return ViewerResponse(ViewerStatus.MISSING_PATHS, result)

    def handleSetGeometry(self, command):
        path = command["path"]
        if "geometry" in command:
            geometry = Geometry([command["geometry"]])
        else:
            geometry = Geometry(command["geometries"])
        return self.setGeometry(path, geometry)

    @staticmethod
    def setDefaultColorBy(item):
        availableColorModes = set(
            item.getPropertyAttribute('Color By', 'enumNames'))
        for colorBy in ["rgb", "intensity", "RGB255"]:
            if colorBy in availableColorModes:
                item.setProperty("Color By", colorBy)
                break

    def setGeometry(self, path, geometry):
        folder = self.getPathFolder(path)
        ancestors = findPathToAncestor(folder, self.getRootFolder())
        geomTransform = vtk.vtkTransform()
        for item in reversed(ancestors):
            if not hasattr(item, "transform"):
                item.transform = vtk.vtkTransform()
                item.transform.PostMultiply()
            geomTransform.Concatenate(item.transform)

        geometryName = folder.getProperty("Name")
        item = folder.findChild(geometryName)
        if item is None:
            item = geometry.createPolyDataItem(name=geometryName)
            item.addToView(self.view)
            om.addToObjectModel(item, parentObj=folder)
        else:
            item.setPolyData(geometry.polyData)
            geometry.updatePolyDataItemProperties(item)

        self.setDefaultColorBy(item)
        item.actor.SetUserTransform(geomTransform)

        return path

    def getPathForItem(self, item):
        return [x.getProperty("Name") for x in reversed(findPathToAncestor(
            item, self.getRootFolder())[:-1])]

    def handleSetTransform(self, command):
        return self._setTransform(command["path"],
                                  transformFromDict(command["transform"]))

    def _setTransform(self, path, transform):
        folder = self.getPathFolder(path)
        if not hasattr(folder, "transform"):
            folder.transform = transform
        else:
            folder.transform.SetMatrix(transform.GetMatrix())
        return path, len(folder.children()) == 0

    def handleDeletePath(self, command):
        path = command["path"]
        item = self.getPathFolder(path)
        if item is not None:
            om.removeFromObjectModel(item)
        return path

    def getRootFolder(self):
        path = tuple()
        if path in self.pathToItemCache:
            return self.pathToItemCache[path]
        else:
            folder = om.getOrCreateContainer(
                self.name.lower(),
                parentObj=om.findObjectByName('scene'))
            self.pathToItemCache[path] = folder
            self.itemToPathCache[folder] = path
            folder.connectRemovedFromObjectModel(self.onItemRemoved)
            return folder

    def onItemRemoved(self, objModel, item):
        if item in self.itemToPathCache:
            path = self.itemToPathCache[item]
            del self.itemToPathCache[item]
            if path in self.pathToItemCache:
                del self.pathToItemCache[path]

    def getPathFolder(self, path):
        path = tuple(path)
        if path in self.pathToItemCache:
            # print "hit for path:", path
            return self.pathToItemCache[path]
        else:
            # print "miss for path:", path
            folder = self.getRootFolder()
            for element in path:
                folder = om.getOrCreateContainer(element, parentObj=folder)
                folder.connectRemovedFromObjectModel(self.onItemRemoved)
            self.pathToItemCache[path] = folder
            self.itemToPathCache[folder] = path
            return folder
