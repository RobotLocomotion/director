import ddapp.vtkAll as vtk
import ddapp.thirdparty.numpyjsoncoder as nje
from collections import OrderedDict
from ddapp import fieldcontainer
from ddapp import transformUtils
from ddapp import lcmUtils
from ddapp.utime import getUtime

import drc as lcmdrc

import pprint
import json

class ConstraintEncoder(nje.NumpyEncoder):
    def default(self, obj):
        if isinstance(obj, vtk.vtkTransform):
            pos, quat = transformUtils.poseFromTransform(obj)
            return OrderedDict(position=pos, quaternion=quat)
        elif isinstance(obj, fieldcontainer.FieldContainer):
            d = OrderedDict()
            d['class'] = type(obj).__name__
            for key in obj._fields:
                d[key] = getattr(obj, key)
            return d
        return nje.NumpyEncoder.default(self, obj)



def ConstraintDecoder(dct):
    return nje.NumpyDecoder(dct)

def encodeConstraints(dataObj, **kwargs):
    return json.dumps(dataObj, cls=ConstraintEncoder, **kwargs)

def decodeConstraints(dataStream):
    return json.loads(dataStream, object_hook=ConstraintDecoder)

def getPlanPoses(constraints, ikPlanner):
    '''
    Given a list of constraints, returns a dictionary of poses containing all
    the poses that are references by the constraints by name
    '''

    poses = sorted([c.postureName for c in constraints if hasattr(c, 'postureName')])
    poses = {poseName:list(ikPlanner.jointController.getPose(poseName)) for poseName in poses}
    return poses


class IKConstraintEncoder(object):
    def __init__(self,ikPlanner):
        self.ikPlanner = ikPlanner

    def publishConstraints(self,constraints,messageName='PLANNER_REQUEST'):
        poses = getPlanPoses(constraints, self.ikPlanner)

        #poseJsonStr = json.dumps(poses, indent=4)
        #constraintsJsonStr = encodeConstraints(constraints, indent=4)
        poseJsonStr = json.dumps(poses)
        constraintsJsonStr = encodeConstraints(constraints)

        msg = lcmdrc.planner_request_t()
        msg.utime = getUtime()
        msg.poses = poseJsonStr
        msg.constraints = constraintsJsonStr
        lcmUtils.publish(messageName, msg)

    def decodeConstraints(self,dataStream):
        return decodeConstraints(dataStream)
