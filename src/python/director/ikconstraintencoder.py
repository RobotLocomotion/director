import director.vtkAll as vtk
import director.thirdparty.numpyjsoncoder as nje
from collections import OrderedDict
from director import fieldcontainer
from director import transformUtils

import json

class ConstraintEncoder(nje.NumpyConvertEncoder):
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
        return nje.NumpyConvertEncoder.default(self, obj)


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
