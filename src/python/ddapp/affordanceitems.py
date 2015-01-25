import ddapp.objectmodel as om
from ddapp import affordance
from ddapp.visualization import PolyDataItem
from ddapp import vtkAll as vtk
import numpy as np

class AffordanceItem(PolyDataItem):

    def __init__(self, name, polyData, view):
        PolyDataItem.__init__(self, name, polyData, view)
        self.params = {}

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


    def onRemoveFromObjectModel(self):
        PolyDataItem.onRemoveFromObjectModel(self)



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
