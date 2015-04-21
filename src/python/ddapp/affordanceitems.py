import ddapp
import ddapp.objectmodel as om
from ddapp import affordance
from ddapp import visualization as vis
from ddapp.visualization import PolyDataItem
from ddapp import filterUtils
from ddapp import ioUtils
from ddapp import meshmanager
from ddapp import transformUtils
from ddapp.uuidutil import newUUID
from ddapp.debugVis import DebugData
from ddapp import vtkAll as vtk
import numpy as np

import os
import uuid
from collections import OrderedDict


class AffordanceItem(PolyDataItem):

    def __init__(self, name, polyData, view):
        PolyDataItem.__init__(self, name, polyData, view)
        self.params = {}
        self.addProperty('uuid', newUUID(), attributes=om.PropertyAttributes(hidden=True))
        self.addProperty('Collision Enabled', True)
        self.properties.setPropertyIndex('Collision Enabled', 0)
        self.setProperty('Icon', om.Icons.Hammer)

    def publish(self):
        pass

    def getActionNames(self):
        actions = ['Publish affordance']
        return PolyDataItem.getActionNames(self) + actions

    def onAction(self, action):
        if action == 'Publish affordance':
            self.publish()
        else:
            PolyDataItem.onAction(self, action)

    def getDescription(self):
        d = OrderedDict()
        d['classname'] = type(self).__name__
        d.update(self.properties._properties)
        pose = transformUtils.poseFromTransform(self.getChildFrame().transform)
        d['pose'] = pose
        return d

    def repositionFromDescription(self, desc):
        position, quat = desc['pose']
        t = transformUtils.transformFromPose(position, quat)
        self.getChildFrame().copyFrame(t)

    def loadDescription(self, desc):
        self.syncProperties(desc)
        self.repositionFromDescription(desc)
        self._renderAllViews()

    def syncProperties(self, desc):
        for propertyName, propertyValue in desc.iteritems():
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

    def loadDescription(self, desc):
        AffordanceItem.loadDescription(self, desc)

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

    def loadDescription(self, desc):
        AffordanceItem.loadDescription(self, desc)

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

    def loadDescription(self, desc):
        AffordanceItem.loadDescription(self, desc)

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

    def loadDescription(self, desc):
        AffordanceItem.loadDescription(self, desc)

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

    def loadDescription(self, desc):
        AffordanceItem.loadDescription(self, desc)

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
        self.properties.setPropertyIndex('Filename', 0)

    def loadDescription(self, desc):
        AffordanceItem.loadDescription(self, desc)

        # attempt to reload geometry if it is currently empty
        if self.getProperty('Filename') and not self.polyData.GetNumberOfPoints():
            self.updateGeometryFromProperties()


    def updateGeometryFromProperties(self):
        filename = self.getProperty('Filename')

        if not filename:
            polyData = vtk.vtkPolyData()
        else:
            polyData = self.getMeshManager().get(filename)

        if not polyData:

            if not os.path.isabs(filename):
                filename = os.path.join(ddapp.getDRCBaseDir(), filename)

            if os.path.isfile(filename):
                polyData = ioUtils.readPolyData(filename)
            else:
                # todo: put placeholder geometry to indicate a missing mesh
                polyData = vtk.vtkPolyData()

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

        if propertyName == 'Filename':
            self.updateGeometryFromProperties()


class FrameAffordanceItem(AffordanceItem):

    def setAffordanceParams(self, params):
        self.params = params
        assert 'otdf_type' in params

    def updateParamsFromActorTransform(self):

        t = self.actor.GetUserTransform()

        xaxis = np.array(t.TransformVector([1,0,0]))
        yaxis = np.array(t.TransformVector([0,1,0]))
        zaxis = np.array(t.TransformVector([0,0,1]))
        self.params['xaxis'] = xaxis
        self.params['yaxis'] = yaxis
        self.params['zaxis'] = zaxis
        self.params['origin'] = t.GetPosition()


    def publish(self):
        self.updateParamsFromActorTransform()
        aff = affordance.createFrameAffordance(self.params)
        affordance.publishAffordance(aff)
