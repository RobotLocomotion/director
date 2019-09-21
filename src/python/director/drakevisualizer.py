import math, os
import numpy as np
from director import lcmUtils

import director.objectmodel as om
import director.applogic as app
from director import transformUtils
from director.debugVis import DebugData
from director import ioUtils
from director import filterUtils
from director.shallowCopy import shallowCopy
from director import vtkAll as vtk
from director import vtkNumpy as vnp
from director import visualization as vis
from director import packagepath
from director.fieldcontainer import FieldContainer

import bot_core as lcmbot

# Currently, viewer lcm message types are in bot_core_lcmtypes and
# robotlocomotion-lcmtypes, but drake only builds bot_core_lcmtypes.
# When drake starts using robotlocomotion-lcmtypes, then the following
# import can be used instead of getting viewer messages from bot_core_lcmtypes.
#import robotlocomotion as lcmrl
lcmrl = lcmbot

# pydrake is only used to provide an additional source of mesh package lookup
# paths, so failing to find it should not be fatal
try:
    import pydrake
    HAVE_PYDRAKE = True
except ImportError:
    HAVE_PYDRAKE = False

from PythonQt import QtGui

class Geometry(object):

    MeshCache = {}
    TextureCache = {}

    PackageMap = None

    @staticmethod
    def createPolyDataFromPrimitive(geom):

        if geom.type == lcmrl.viewer_geometry_data_t.BOX:
            d = DebugData()
            d.addCube(dimensions=geom.float_data[0:3], center=[0,0,0])
            return d.getPolyData()

        elif geom.type == lcmrl.viewer_geometry_data_t.SPHERE:
            d = DebugData()
            d.addSphere(center=(0,0,0), radius=geom.float_data[0])
            return d.getPolyData()

        elif geom.type == lcmrl.viewer_geometry_data_t.CYLINDER:
            d = DebugData()
            d.addCylinder(center=(0,0,0), axis=(0,0,1), radius=geom.float_data[0], length=geom.float_data[1])
            return d.getPolyData()

        elif geom.type == lcmrl.viewer_geometry_data_t.CAPSULE:
            d = DebugData()
            radius = geom.float_data[0]
            length = geom.float_data[1]
            d.addCylinder(center=(0,0,0), axis=(0,0,1), radius=radius, length=length)
            d.addSphere(center=(0,0,length/2.0), radius=radius)
            d.addSphere(center=(0,0,-length/2.0), radius=radius)
            return d.getPolyData()

        elif hasattr(lcmrl.viewer_geometry_data_t, "ELLIPSOID") and geom.type == lcmrl.viewer_geometry_data_t.ELLIPSOID:
            d = DebugData()
            radii = geom.float_data[0:3]
            d.addEllipsoid(center=(0,0,0), radii=radii)
            return d.getPolyData()

        raise Exception('Unsupported geometry type: %s' % geom.type)

    @staticmethod
    def createPolyDataFromMeshMessage(geom):

        assert len(geom.float_data) >= 2

        numPoints = int(geom.float_data[0])
        numTris = int(geom.float_data[1])

        headerOffset = 2
        ptsOffset = 3*numPoints
        trisOffset = 3*numTris

        assert len(geom.float_data) == headerOffset + ptsOffset + trisOffset

        pts = np.array(geom.float_data[headerOffset:headerOffset+ptsOffset])
        pts = pts.reshape((numPoints, 3))
        tris = np.array(geom.float_data[headerOffset+ptsOffset:], dtype=int)

        return Geometry.createPolyDataFromMeshArrays(pts, tris)

    @staticmethod
    def createPolyDataFromMeshArrays(pts, faces):
        pd = vtk.vtkPolyData()
        pd.SetPoints(vtk.vtkPoints())
        pd.GetPoints().SetData(vnp.getVtkFromNumpy(pts.copy()))

        assert len(faces) % 3 == 0
        cells = vtk.vtkCellArray()
        for i in range(len(faces)/3):
            tri = vtk.vtkTriangle()
            tri.GetPointIds().SetId(0, faces[i*3 + 0])
            tri.GetPointIds().SetId(1, faces[i*3 + 1])
            tri.GetPointIds().SetId(2, faces[i*3 + 2])
            cells.InsertNextCell(tri)

        pd.SetPolys(cells)
        return pd

    @staticmethod
    def scaleGeometry(polyDataList, geom):
        if len(geom.float_data) == 1:
            scale_x = scale_y = scale_z = geom.float_data[0]
        elif len(geom.float_data) == 3:
            scale_x = geom.float_data[0]
            scale_y = geom.float_data[1]
            scale_z = geom.float_data[2]
        else:
            scale_x = scale_y = scale_z = 1.0

        if scale_x != 1.0 or scale_y != 1.0 or scale_z != 1.0:
            t = vtk.vtkTransform()
            t.Scale(scale_x, scale_y, scale_z)
            polyDataList = [filterUtils.transformPolyData(polyData, t) for polyData in polyDataList]

        return polyDataList


    @staticmethod
    def transformGeometry(polyDataList, geom):
        t = transformUtils.transformFromPose(geom.position, geom.quaternion)
        return [filterUtils.transformPolyData(polyData, t) for polyData in polyDataList]

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
            print('failed to find image file:', imageFile)
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
            m = packagepath.PackageMap()
            if HAVE_PYDRAKE:
                m.populateFromSearchPaths([pydrake.getDrakePath()])
            m.populateFromEnvironment(['DRAKE_PACKAGE_PATH', 'ROS_PACKAGE_PATH'])
            Geometry.PackageMap = m

        return Geometry.PackageMap.resolveFilename(filename) or filename

    @staticmethod
    def createPolyDataFromFiles(geom):

        filename = Geometry.resolvePackageFilename(geom.string_data)
        basename, ext = os.path.splitext(filename)

        if filename in Geometry.MeshCache:
            return Geometry.MeshCache[filename]

        preferredExtensions = ['.vtm', '.vtp', '.obj']

        for x in preferredExtensions:
            if os.path.isfile(basename + x):
                filename = basename + x
                break

        if not os.path.isfile(filename):
            print('warning, cannot find file:', filename)
            return []

        visInfo = None

        if filename.endswith('vtm'):
            polyDataList = ioUtils.readMultiBlock(filename)
        else:
            if filename.endswith('obj'):
                polyDataList, actors = ioUtils.readObjMtl(filename)
                if actors:
                    visInfo = Geometry.makeVisInfoFromActors(actors)
            else:
                polyDataList = [ioUtils.readPolyData(filename)]

        for polyData in polyDataList:
            Geometry.loadTextureForMesh(polyData, filename)

        result = (polyDataList, visInfo)
        Geometry.MeshCache[filename] = result
        return result

    @staticmethod
    def makeVisInfoFromMessage(polyDataList, geom):

        def make(polyData):
            texture = Geometry.TextureCache.get(Geometry.getTextureFileName(polyData))
            color = [1.0, 1.0, 1.0] if texture else geom.color[:3]
            alpha = geom.color[3]
            return FieldContainer(color=color, alpha=alpha, texture=texture)

        return [make(polyData) for polyData in polyDataList]

    @staticmethod
    def makeVisInfoFromActors(actors):

        def make(actor):
            color = actor.GetProperty().GetColor()
            alpha = actor.GetProperty().GetOpacity()
            texture = actor.GetTexture()
            return FieldContainer(color=color, alpha=alpha, texture=texture)

        return [make(actor) for actor in actors]

    @staticmethod
    def createGeometry(name, geom):

        visInfo = None

        if geom.type != lcmrl.viewer_geometry_data_t.MESH:
            polyDataList = [Geometry.createPolyDataFromPrimitive(geom)]
        elif not geom.string_data:
            polyDataList = [Geometry.createPolyDataFromMeshMessage(geom)]
        else:
            polyDataList, visInfo = Geometry.createPolyDataFromFiles(geom)
            polyDataList = Geometry.scaleGeometry(polyDataList, geom)

        if not visInfo:
            visInfo = Geometry.makeVisInfoFromMessage(polyDataList, geom)

        assert len(polyDataList) == len(visInfo)

        polyDataList = Geometry.transformGeometry(polyDataList, geom)
        polyDataList = Geometry.computeNormals(polyDataList)

        return [Geometry(name, polyData, visInfo) for polyData, visInfo in zip(polyDataList, visInfo)]


    def __init__(self, name, polyData, visInfo):
        self.polyDataItem = vis.PolyDataItem(name, polyData, view=None)
        self.polyDataItem.setProperty('Color', visInfo.color)
        self.polyDataItem.setProperty('Alpha', visInfo.alpha)
        self.polyDataItem.actor.SetTexture(visInfo.texture)


class Link(object):

    def __init__(self, link):
        self.transform = vtk.vtkTransform()
        self.geometry = []
        for g in link.geom:
            self.geometry.extend(Geometry.createGeometry(link.name + ' geometry data', g))

    def setTransform(self, pos, quat):
        self.transform = transformUtils.transformFromPose(pos, quat)
        for g in self.geometry:
            childFrame = g.polyDataItem.getChildFrame()
            if childFrame:
                childFrame.copyFrame(self.transform)
            else:
                g.polyDataItem.actor.SetUserTransform(self.transform)


class DrakeVisualizer(object):
    name = 'Drake Visualizer'

    def __init__(self, view):

        self.subscribers = []
        self.view = view
        self.robots = {}
        self.linkWarnings = set()
        self.enable()
        self.sendStatusMessage('loaded')

    def _addSubscribers(self):
        self.subscribers.append(lcmUtils.addSubscriber('DRAKE_VIEWER_LOAD_ROBOT', lcmrl.viewer_load_robot_t, self.onViewerLoadRobot))
        self.subscribers.append(lcmUtils.addSubscriber('DRAKE_VIEWER_ADD_ROBOT', lcmrl.viewer_load_robot_t, self.onViewerAddRobot))
        self.subscribers.append(lcmUtils.addSubscriber('DRAKE_VIEWER_DRAW', lcmrl.viewer_draw_t, self.onViewerDraw))
        self.subscribers.append(lcmUtils.addSubscriber('DRAKE_PLANAR_LIDAR_.*', lcmbot.planar_lidar_t, self.onPlanarLidar, callbackNeedsChannel=True))
        self.subscribers.append(lcmUtils.addSubscriber('DRAKE_POINTCLOUD_.*', lcmbot.pointcloud_t, self.onPointCloud, callbackNeedsChannel=True))

    def _removeSubscribers(self):
        for sub in self.subscribers:
            lcmUtils.removeSubscriber(sub)
        self.subscribers = []

    def isEnabled(self):
        return bool(self.subscribers)

    def setEnabled(self, enabled):
        if enabled and not self.isEnabled():
            self._addSubscribers()
        elif not enabled and self.isEnabled():
            self._removeSubscribers()

    def enable(self):
        self.setEnabled(True)

    def disable(self):
        self.setEnabled(False)

    def onViewerLoadRobot(self, msg):
        self.removeAllRobots()
        self.addLinksFromLCM(msg)
        self.sendStatusMessage('successfully loaded robot')

    def onViewerAddRobot(self, msg):
        robotNumsToReplace = set(link.robot_num for link in msg.link)
        for robotNum in robotNumsToReplace:
            self.removeRobot(robotNum)
        self.addLinksFromLCM(msg)
        self.sendStatusMessage('successfully added robot')

    def getRootFolder(self):
        return om.getOrCreateContainer(self.name.lower(), parentObj=om.findObjectByName('scene'))

    def getRobotFolder(self, robotNum):
        return om.getOrCreateContainer('robot %d' % robotNum, parentObj=self.getRootFolder())

    def getLinkFolder(self, robotNum, linkName):
        return om.getOrCreateContainer(linkName, parentObj=self.getRobotFolder(robotNum))

    def getPointCloudFolder(self):
        return om.getOrCreateContainer('pointclouds', parentObj=self.getRootFolder())

    def addLinksFromLCM(self, load_msg):
        for link in load_msg.link:
            self.addLink(Link(link), link.robot_num, link.name)

    def addLink(self, link, robotNum, linkName):
        self.robots.setdefault(robotNum, {})[linkName] = link
        linkFolder = self.getLinkFolder(robotNum, linkName)
        for geom in link.geometry:
            self.addLinkGeometry(geom, linkName, linkFolder)

    def addLinkGeometry(self, geom, linkName, linkFolder):
        geom.polyDataItem.addToView(self.view)
        om.addToObjectModel(geom.polyDataItem, parentObj=linkFolder)

        if linkName == 'world':
            #geom.polyDataItem.actor.SetUseBounds(False)
            #geom.polyDataItem.actor.GetProperty().LightingOff()
            geom.polyDataItem.actor.GetProperty().SetSpecular(0.0)
        else:
            geom.polyDataItem.actor.GetProperty().SetSpecular(0.9)
            geom.polyDataItem.actor.GetProperty().SetSpecularPower(20)

    def getLink(self, robotNum, linkName):
        return self.robots[robotNum][linkName]

    def removeAllRobots(self):
        for child in self.getRootFolder().children():
            if child.getProperty('Name') != "pointclouds":
                om.removeFromObjectModel(child)
        self.robots = {}

    def removeRobot(self, robotNum):
        if robotNum in self.robots:
            om.removeFromObjectModel(self.getRobotFolder(robotNum))
            del self.robots[robotNum]

    def sendStatusMessage(self, message):
        msg = lcmrl.viewer_command_t()
        msg.command_type = lcmrl.viewer_command_t.STATUS
        msg.command_data = message
        lcmUtils.publish('DRAKE_VIEWER_STATUS', msg)

    def onViewerDraw(self, msg):

        for i in range(msg.num_links):

            pos = msg.position[i]
            quat = msg.quaternion[i]
            robotNum = msg.robot_num[i]
            linkName = msg.link_name[i]

            try:
                link = self.getLink(robotNum, linkName)
            except KeyError:
                if linkName not in self.linkWarnings:
                    print('Error locating link name:', linkName)
                    self.linkWarnings.add(linkName)
            else:
                link.setTransform(pos, quat)

        self.view.render()

    def onPlanarLidar(self, msg, channel):

        linkName = channel.replace('DRAKE_PLANAR_LIDAR_', '', 1)
        robotNum, linkName = linkName.split('_', 1)
        robotNum = int(robotNum)

        try:
            link = self.getLink(robotNum, linkName)
        except KeyError:
            if linkName not in self.linkWarnings:
                print('Error locating link name:', linkName)
                self.linkWarnings.add(linkName)
        else:
            if len(link.geometry):
                polyData = link.geometry[0].polyDataItem
            else:
                polyData = vtk.vtkPolyData()

                name = linkName + ' geometry data'
                visInfo = FieldContainer(color=[1.0,0.0,0.0], alpha=1.0, texture=None)
                g = Geometry(name, polyData, visInfo)
                link.geometry.append(g)

                linkFolder = self.getLinkFolder(robotNum, linkName)
                self.addLinkGeometry(g, linkName, linkFolder)
                g.polyDataItem.actor.SetUserTransform(link.transform)

            points = vtk.vtkPoints()
            verts = vtk.vtkCellArray()

            t = msg.rad0
            for r in msg.ranges:
                if r >= 0:
                    x = r * math.cos(t)
                    y = r * math.sin(t)

                    pointId = points.InsertNextPoint([x,y,0])
                    verts.InsertNextCell(1)
                    verts.InsertCellPoint(pointId)

                t += msg.radstep

            polyData.SetPoints(points)
            polyData.SetVerts(verts)


    def onPointCloud(self, msg, channel):
        pointcloudName = channel.replace('DRAKE_POINTCLOUD_', '', 1)

        polyData = vnp.numpyToPolyData(np.asarray(msg.points), createVertexCells=True)

        # If the user provided color channels, then use them to colorize
        # the pointcloud.
        channels = {msg.channel_names[i]: msg.channels[i] for i in range(msg.n_channels)}
        if "r" in channels and "g" in channels and "b" in channels:
            colorized = True
            colorArray = np.empty((msg.n_points, 3), dtype=np.uint8)
            for (colorIndex, color) in enumerate(["r", "g", "b"]):
                colorArray[:, colorIndex] = 255 * np.asarray(channels[color])
            vnp.addNumpyToVtk(polyData, colorArray, "rgb")
        else:
            colorized = False

        folder = self.getPointCloudFolder()

        # If there was an existing point cloud by this name, then just
        # set its polyData to the new point cloud.
        # This has the effect of preserving all the user-specified properties
        # like point size, coloration mode, alpha, etc.
        previousPointcloud = folder.findChild(pointcloudName)
        if previousPointcloud is not None:
            previousPointcloud.setPolyData(polyData)
            previousPointcloud._updateColorByProperty()
        else:
            item = vis.PolyDataItem(pointcloudName, polyData, view=None)
            item.addToView(self.view)
            if colorized:
                item._updateColorByProperty()
                item.setProperty("Color By", "rgb")
            om.addToObjectModel(item, parentObj=folder)
