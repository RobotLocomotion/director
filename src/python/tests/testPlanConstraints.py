from director.consoleapp import ConsoleApp
from director import robotsystem
from director import visualization as vis
from director import objectmodel as om
from director import ikplanner
from director import ikconstraintencoder as ce
from director import ikconstraints
from director import transformUtils

import numpy as np
import pprint
import json


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


def reconstructConstraints(constraints):
    '''
    Convert dicts (decoded from json) back to the original
    constraint classes using the 'class' information in the dict
    '''
    objs = []

    for c in constraints:
        objClass = getattr(ikconstraints, c['class'])
        del c['class']
        obj = objClass()
        objs.append(obj)

        for attr, value in c.iteritems():
            if isinstance(value, dict) and 'position' in value and 'quaternion' in value:
                value = transformUtils.transformFromPose(value['position'], value['quaternion'])
            setattr(obj, attr, value)

    return objs


def testPlanConstraints():

    ikPlanner.planningMode = 'dummy'

    constraints = buildConstraints()
    poses = ce.getPlanPoses(constraints, ikPlanner)


    poseJsonStr = json.dumps(poses, indent=4)
    constraintsJsonStr = ce.encodeConstraints(constraints, indent=4)

    print poseJsonStr
    print constraintsJsonStr

    print '--------------decoding--------------------'
    constraints = ce.decodeConstraints(constraintsJsonStr)
    pprint.pprint(constraints)

    print '--------------reconstructing--------------'
    constraints = reconstructConstraints(constraints)

    print '--------------matlab commands---------------'
    for c in constraints:
        c.printCommands()



app = ConsoleApp()
view = app.createView()


factory = robotsystem.RobotSystemFactory()
options = factory.getDisabledOptions()
factory.setDependentOptions(options, usePlannerPublisher=True)
robotSystem = factory.construct(view=view, options=options)


app.setupGlobals(globals())
globals().update(dict(robotSystem))


testPlanConstraints()
