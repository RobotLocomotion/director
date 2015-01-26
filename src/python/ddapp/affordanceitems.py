import ddapp.objectmodel as om
from ddapp import affordance
from ddapp.visualization import PolyDataItem
from ddapp import vtkAll as vtk
from ddapp import transformUtils
from ddapp.debugVis import DebugData
import numpy as np

import uuid
from collections import OrderedDict


def newUUID():
    return str(uuid.uuid1())


class AffordanceItem(PolyDataItem):

    def __init__(self, name, polyData, view):
        PolyDataItem.__init__(self, name, polyData, view)
        self.params = {}
        self.addProperty('uuid', newUUID(), attributes=om.PropertyAttributes(hidden=True))
        self.addProperty('Server updates enabled', True, attributes=om.PropertyAttributes(hidden=True))

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
        del d['Color']
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

    def syncProperties(self, desc):
        for propertyName, propertyValue in desc.iteritems():
            if self.hasProperty(propertyName):
                self.setProperty(propertyName, propertyValue)

    def onRemoveFromObjectModel(self):
        PolyDataItem.onRemoveFromObjectModel(self)



class BoxAffordanceItem(AffordanceItem):

    def __init__(self, name, view):
        AffordanceItem.__init__(self, name, vtk.vtkPolyData(), view)
        self.addProperty('Dimensions', [0.5, 0.5, 0.5], attributes=om.PropertyAttributes(decimals=3, singleStep=0.01, minimum=0.0, maximum=1e4))
        self.updateGeometryFromProperties()

    def loadDescription(self, desc):
        AffordanceItem.loadDescription(self, desc)
        self.updateGeometryFromProperties()

    def updateGeometryFromProperties(self):
        d = DebugData()
        d.addCube(self.getProperty('Dimensions'), (0,0,0))
        self.setPolyData(d.getPolyData())

    def _onPropertyChanged(self, propertySet, propertyName):
        AffordanceItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Dimensions':
            self.updateGeometryFromProperties()


class BlockAffordanceItem(AffordanceItem):

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


    def publish(self):
        self.updateParamsFromActorTransform()
        aff = affordance.createBoxAffordance(self.params)
        affordance.publishAffordance(aff)

        if hasattr(self, 'publishCallback'):
            self.publishCallback()

    def updateICPTransform(self, transform):
        delta = computeAToB(self.icpTransformInitial, transform)
        print 'initial:', self.icpTransformInitial.GetPosition(), self.icpTransformInitial.GetOrientation()
        print 'latest:', transform.GetPosition(), transform.GetOrientation()
        print 'delta:', delta.GetPosition(), delta.GetOrientation()
        newUserTransform = vtk.vtkTransform()
        newUserTransform.PostMultiply()
        newUserTransform.Identity()
        newUserTransform.Concatenate(self.baseTransform)
        newUserTransform.Concatenate(delta.GetLinearInverse())

        self.actor.SetUserTransform(newUserTransform)
        self._renderAllViews()


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


class CylinderAffordanceItem(AffordanceItem):

    def setAffordanceParams(self, params):
        self.params = params

    def updateParamsFromActorTransform(self):

        t = self.actor.GetUserTransform()

        xaxis = np.array(t.TransformVector([1,0,0]))
        yaxis = np.array(t.TransformVector([0,1,0]))
        zaxis = np.array(t.TransformVector([0,0,1]))
        self.params['axis'] = zaxis
        self.params['origin'] = t.GetPosition()


    def publish(self):
        self.updateParamsFromActorTransform()
        aff = affordance.createCylinderAffordance(self.params)
        affordance.publishAffordance(aff)
