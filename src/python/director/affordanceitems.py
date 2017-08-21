import director
import director.objectmodel as om
from director import visualization as vis
from director.visualization import PolyDataItem
from director import filterUtils
from director import ioUtils
from director import meshmanager
from director import transformUtils
from director.uuidutil import newUUID
from director.debugVis import DebugData
from director import vtkAll as vtk
import numpy as np

import os
import uuid
from collections import OrderedDict


class AffordanceItem(PolyDataItem):

    COPY_MODE_ALL = 0 # copies all properties from affordance descriptions
    COPY_MODE_SKIP_LOCAL = 1 # skips properties that should keep local values such as visibility
    LOCAL_PROPERTY_NAMES = ('Visible')

    def __init__(self, name, polyData, view):
        PolyDataItem.__init__(self, name, polyData, view)
        self.params = {}
        self.addProperty('uuid', newUUID(), attributes=om.PropertyAttributes(hidden=True))
        self.addProperty('Collision Enabled', True)
        self.addProperty('Origin', [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], attributes=om.PropertyAttributes(hidden=True))
        self.addProperty('Camera Texture Enabled', False)

        self.properties.setPropertyIndex('Collision Enabled', 0)
        self.setProperty('Icon', om.Icons.Hammer)

    def getPose(self):
        childFrame = self.getChildFrame()
        t = childFrame.transform if childFrame else vtk.vtkTransform()
        return transformUtils.poseFromTransform(t)

    def getDescription(self):
        d = OrderedDict()
        d['classname'] = type(self).__name__
        d.update(self.properties._properties)
        d['pose'] = self.getPose()
        return d

    def _onPropertyChanged(self, propertySet, propertyName):
        PolyDataItem._onPropertyChanged(self, propertySet, propertyName)
        if propertyName == 'Origin':
            self.updateGeometryFromProperties()

    def updateGeometryFromProperties():
        pass

    def setPolyData(self, polyData):

        if polyData.GetNumberOfPoints():
            originPose = self.getProperty('Origin')
            pos, quat = originPose[:3], originPose[3:]
            t = transformUtils.transformFromPose(pos, quat)
            polyData = filterUtils.transformPolyData(polyData, t.GetLinearInverse())
        PolyDataItem.setPolyData(self, polyData)

    def repositionFromDescription(self, desc):
        position, quat = desc['pose']
        t = transformUtils.transformFromPose(position, quat)
        self.getChildFrame().copyFrame(t)

    def loadDescription(self, desc, copyMode=COPY_MODE_ALL):
        self.syncProperties(desc, copyMode)
        self.repositionFromDescription(desc)
        self._renderAllViews()

    def syncProperties(self, desc, copyMode=COPY_MODE_ALL):
        for propertyName, propertyValue in desc.items():
            if copyMode == self.COPY_MODE_SKIP_LOCAL:
                if propertyName in self.LOCAL_PROPERTY_NAMES:
                    continue

            if self.hasProperty(propertyName) and (self.getProperty(propertyName) != propertyValue):
                #print 'syncing property %s: %r' % (propertyName, propertyValue)
                self.setProperty(propertyName, propertyValue)

    def onRemoveFromObjectModel(self):
        PolyDataItem.onRemoveFromObjectModel(self)


class BoxAffordanceItem(AffordanceItem):

    def __init__(self, name, view):
        AffordanceItem.__init__(self, name, vtk.vtkPolyData(), view)
        self.addProperty('Dimensions', [0.25, 0.25, 0.25], attributes=om.PropertyAttributes(decimals=3, singleStep=0.01, minimum=0.0, maximum=1e4))
        self.addProperty('Subdivisions', 0, attributes=om.PropertyAttributes(minimum=0, maximum=1000))
        self.properties.setPropertyIndex('Dimensions', 0)
        self.properties.setPropertyIndex('Subdivisions', 1)
        self.updateGeometryFromProperties()

    def updateGeometryFromProperties(self):
        d = DebugData()
        d.addCube(self.getProperty('Dimensions'), (0,0,0), subdivisions=self.getProperty('Subdivisions'))
        self.setPolyData(d.getPolyData())

    def _onPropertyChanged(self, propertySet, propertyName):
        AffordanceItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName in ('Dimensions', 'Subdivisions'):
            self.updateGeometryFromProperties()


class SphereAffordanceItem(AffordanceItem):

    def __init__(self, name, view):
        AffordanceItem.__init__(self, name, vtk.vtkPolyData(), view)
        self.addProperty('Radius', 0.15, attributes=om.PropertyAttributes(decimals=3, singleStep=0.01, minimum=0.0, maximum=1e4))
        self.properties.setPropertyIndex('Radius', 0)
        self.updateGeometryFromProperties()

    def updateGeometryFromProperties(self):
        d = DebugData()
        d.addSphere((0,0,0), self.getProperty('Radius'))
        self.setPolyData(d.getPolyData())

    def _onPropertyChanged(self, propertySet, propertyName):
        AffordanceItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Radius':
            self.updateGeometryFromProperties()


class CylinderAffordanceItem(AffordanceItem):

    def __init__(self, name, view):
        AffordanceItem.__init__(self, name, vtk.vtkPolyData(), view)
        self.addProperty('Radius', 0.03, attributes=om.PropertyAttributes(decimals=3, singleStep=0.01, minimum=0.0, maximum=1e4))
        self.addProperty('Length', 0.5, attributes=om.PropertyAttributes(decimals=3, singleStep=0.01, minimum=0.0, maximum=1e4))
        self.properties.setPropertyIndex('Radius', 0)
        self.properties.setPropertyIndex('Length', 1)
        self.updateGeometryFromProperties()

    def updateGeometryFromProperties(self):
        d = DebugData()
        length = self.getProperty('Length')
        d.addCylinder(center=(0,0,0), axis=(0,0,1), length=self.getProperty('Length'), radius=self.getProperty('Radius'))
        self.setPolyData(d.getPolyData())

    def _onPropertyChanged(self, propertySet, propertyName):
        AffordanceItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName in ('Length', 'Radius'):
            self.updateGeometryFromProperties()


class CapsuleAffordanceItem(AffordanceItem):

    def __init__(self, name, view):
        AffordanceItem.__init__(self, name, vtk.vtkPolyData(), view)
        self.addProperty('Radius', 0.03, attributes=om.PropertyAttributes(decimals=3, singleStep=0.01, minimum=0.0, maximum=1e4))
        self.addProperty('Length', 0.5, attributes=om.PropertyAttributes(decimals=3, singleStep=0.01, minimum=0.0, maximum=1e4))
        self.properties.setPropertyIndex('Radius', 0)
        self.properties.setPropertyIndex('Length', 1)
        self.updateGeometryFromProperties()

    def updateGeometryFromProperties(self):
        d = DebugData()
        length = self.getProperty('Length')
        d.addCapsule(center=(0,0,0), axis=(0,0,1), length=self.getProperty('Length'), radius=self.getProperty('Radius'))
        self.setPolyData(d.getPolyData())

    def _onPropertyChanged(self, propertySet, propertyName):
        AffordanceItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName in ('Length', 'Radius'):
            self.updateGeometryFromProperties()


class CapsuleRingAffordanceItem(AffordanceItem):

    def __init__(self, name, view):
        AffordanceItem.__init__(self, name, vtk.vtkPolyData(), view)
        self.setProperty('Collision Enabled', False)
        self.addProperty('Radius', 0.15, attributes=om.PropertyAttributes(decimals=3, singleStep=0.01, minimum=0.0, maximum=1e4))
        self.addProperty('Tube Radius', 0.02, attributes=om.PropertyAttributes(decimals=3, singleStep=0.01, minimum=0.0, maximum=1e4))
        self.addProperty('Segments', 8, attributes=om.PropertyAttributes(decimals=3, singleStep=1, minimum=3, maximum=100))

        self.properties.setPropertyIndex('Radius', 0)
        self.properties.setPropertyIndex('Tube Radius', 1)
        self.properties.setPropertyIndex('Segments', 2)

        self.updateGeometryFromProperties()

    def updateGeometryFromProperties(self):
        radius = self.getProperty('Radius')
        circlePoints = np.linspace(0, 2*np.pi, self.getProperty('Segments')+1)
        spokes = [(0.0, np.sin(x), np.cos(x)) for x in circlePoints]
        spokes = [radius*np.array(x)/np.linalg.norm(x) for x in spokes]
        d = DebugData()
        for a, b in zip(spokes, spokes[1:]):
            d.addCapsule(center=(a+b)/2.0, axis=(b-a), length=np.linalg.norm(b-a), radius=self.getProperty('Tube Radius'))
        self.setPolyData(d.getPolyData())

    def _onPropertyChanged(self, propertySet, propertyName):
        AffordanceItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName in ('Radius', 'Tube Radius', 'Segments'):
            self.updateGeometryFromProperties()


class MeshAffordanceItem(AffordanceItem):

    _meshManager = None

    def __init__(self, name, view):
        AffordanceItem.__init__(self, name, vtk.vtkPolyData(), view)
        self.setProperty('Collision Enabled', False)

        self.addProperty('Filename', '')
        self.addProperty('Scale', [1.0, 1.0, 1.0], attributes=om.PropertyAttributes(decimals=3, singleStep=0.01, minimum=0.0, maximum=1e4))

        self.properties.setPropertyIndex('Filename', 0)
        self.properties.setPropertyIndex('Scale', 1)

        # attempt to reload geometry if it is currently empty
        if self.getProperty('Filename') and not self.polyData.GetNumberOfPoints():
            self.updateGeometryFromProperties()


    def updateGeometryFromProperties(self):
        filename = self.getProperty('Filename')
        scale = self.getProperty('Scale')

        if not filename:
            polyData = vtk.vtkPolyData()
        else:
            polyData = self.getMeshManager().get(filename)

        if not polyData:
            if not os.path.isabs(filename):
                filename = os.path.join(director.getDRCBaseDir(), filename)

            if os.path.isfile(filename):
                polyData = ioUtils.readPolyData(filename)

                if not scale == [1, 1, 1]:
                    transform = vtk.vtkTransform()
                    transform.Scale(scale)
                    polyData = filterUtils.transformPolyData(polyData, transform)
            else:
                # use axes as a placeholder mesh
                d = DebugData()
                d.addFrame(vtk.vtkTransform(), scale=0.1, tubeRadius=0.005)
                polyData = d.getPolyData()

        self.setPolyData(polyData)

    @classmethod
    def getMeshManager(cls):
        if cls._meshManager is None:
            cls._meshManager = meshmanager.MeshManager()
        return cls._meshManager

    @classmethod
    def promotePolyDataItem(cls, obj):
        parent = obj.parent()
        view = obj.views[0]
        name = obj.getProperty('Name')
        polyData = obj.polyData
        props = obj.properties._properties
        childFrame = obj.getChildFrame()
        if childFrame:
            t = transformUtils.copyFrame(childFrame.transform)
        else:
            t = vtk.vtkTransform()
            t.PostMultiply()
            t.Translate(filterUtils.computeCentroid(polyData))
            polyData = filterUtils.transformPolyData(polyData, t.GetLinearInverse())

        children = [c for c in obj.children() if c is not childFrame]

        meshId = cls.getMeshManager().add(polyData)

        om.removeFromObjectModel(obj)
        obj = MeshAffordanceItem(name, view)
        obj.setProperty('Filename', meshId)
        om.addToObjectModel(obj, parentObj=parent)
        frame = vis.addChildFrame(obj)
        frame.copyFrame(t)

        for child in children:
            om.addToObjectModel(child, parentObj=obj)

        obj.syncProperties(props)
        return obj


    def _onPropertyChanged(self, propertySet, propertyName):
        AffordanceItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName in ('Filename', 'Scale'):
            self.updateGeometryFromProperties()


class FrameAffordanceItem(AffordanceItem):

    def setAffordanceParams(self, params):
        self.params = params

    def updateParamsFromActorTransform(self):

        t = self.actor.GetUserTransform()

        xaxis = np.array(t.TransformVector([1,0,0]))
        yaxis = np.array(t.TransformVector([0,1,0]))
        zaxis = np.array(t.TransformVector([0,0,1]))
        self.params['xaxis'] = xaxis
        self.params['yaxis'] = yaxis
        self.params['zaxis'] = zaxis
        self.params['origin'] = t.GetPosition()
