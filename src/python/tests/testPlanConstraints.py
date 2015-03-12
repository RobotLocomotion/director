from ddapp.consoleapp import ConsoleApp
from ddapp import robotsystem
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp import objectmodel as om
from ddapp import ikplanner
from ddapp import fieldcontainer
from ddapp import transformUtils

import ddapp.vtkAll as vtk

from PythonQt import QtCore, QtGui
import numpy as np

import pprint
import json
import ddapp.thirdparty.numpyjsoncoder as nje
from collections import OrderedDict

######################

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
    return NumpyDecoder(dct)

def encode(dataObj, **kwargs):
    return json.dumps(dataObj, cls=ConstraintEncoder, **kwargs)

def decode(dataStream):
    return json.loads(dataStream, object_hook=ConstraintDecoder)




######################
def getRobotState():
    return robotStateJointController.q.copy()


def buildConstraints():
    '''
    For testing, build some constraints and return them in a list.
    '''

    startPose = getRobotState()

    startPoseName = 'plan_start'
    endPoseName = 'plan_end'

    ikPlanner.addPose(startPose, startPoseName)
    ikPlanner.addPose(startPose, endPoseName)

    constraints = []
    constraints.extend(ikPlanner.createFixedFootConstraints(startPoseName))
    constraints.append(ikPlanner.createMovingBaseSafeLimitsConstraint())
    constraints.append(ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
    constraints.append(ikPlanner.createLockedRightArmPostureConstraint(startPoseName))
    constraints.append(ikPlanner.createLockedRightArmPostureConstraint(startPoseName))

    targetFrame = ikPlanner.getLinkFrameAtPose(ikPlanner.getHandLink('left'), startPose)

    p, o = ikPlanner.createPositionOrientationGraspConstraints('left', targetFrame)
    p.tspan = [1.0, 1.0]
    o.tspan = [1.0, 1.0]
    constraints.extend([p, o])

    return constraints


def getPlanPoses(constraints):
    '''
    Given a list of constraints, returns a dictionary of poses containing all
    the poses that are references by the constraints by name
    '''

    poses = sorted([c.postureName for c in constraints if hasattr(c, 'postureName')])
    poses = {poseName:list(ikPlanner.jointController.getPose(poseName)) for poseName in poses}
    return poses


def testPlanConstraints():

    # this is required for now, makes it avoid communication with matlab
    # inside the call to ikPlanner.addPose
    ikPlanner.pushToMatlab = False



    constraints = buildConstraints()
    poses = getPlanPoses(constraints)

    poseJsonStr = json.dumps(poses, indent=4)
    constraintsJsonStr = encode(constraints, indent=4)

    print poseJsonStr
    print constraintsJsonStr

    #pprint.pprint(constraints)



app = ConsoleApp()
app.setupGlobals(globals())

view = app.createView()
robotsystem.create(view, globals())


testPlanConstraints()

#app.start()
