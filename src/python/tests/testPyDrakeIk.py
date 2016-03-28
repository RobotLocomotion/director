from director import robotsystem

import os
import numpy as np
import pydrake
from pydrake.solvers import ik
from director import drcargs


from director.consoleapp import ConsoleApp
from director import visualization as vis
from director import objectmodel as om
from director import ikplanner
from director import ikconstraintencoder as ce
from director import roboturdf
from director import transformUtils

import numpy as np
import pprint
import json


def getRobotState():
    return robotSystem.robotStateJointController.q.copy()

def getIkUrdfFilename():
    return robotSystem.ikPlanner.robotModel.getProperty('Filename')


def buildConstraints():
    '''
    For testing, build some constraints and return them in a list.
    '''

    ikPlanner = robotSystem.ikPlanner

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



def loadRigidBodyTree(urdfFile):

    assert os.path.isfile(urdfFile)

    packageMap = pydrake.rbtree.mapStringString()
    for path in roboturdf.getBuiltinPackagePaths():
        packageMap[os.path.basename(path)] = path

    urdfString = open(urdfFile, 'r').read()

    rigidBodyTree = pydrake.rbtree.RigidBodyTree()
    rigidBodyTree.addRobotFromURDFString(urdfString, packageMap)
    return  rigidBodyTree


def testIkPlan():

    ikPlanner = robotSystem.ikPlanner
    ikPlanner.pushToMatlab = False

    constraints = buildConstraints()
    poses = ce.getPlanPoses(constraints, ikPlanner)

    robot = loadRigidBodyTree(getIkUrdfFilename())

    #for i, body in enumerate(robot.bodies):
    #    print i, body.linkname


    tspan = np.array([1.0, 1.0])

    handLinkNames = [ikPlanner.getHandLink('left'), ikPlanner.getHandLink('right')]
    footLinkNames = [robotSystem.directorConfig['leftFootLink'], robotSystem.directorConfig['rightFootLink']]

    leftHandBodyId = robot.findLink(str(handLinkNames[0])).body_index
    rightHandBodyId = robot.findLink(str(handLinkNames[1])).body_index
    leftFootBodyId = robot.findLink(str(footLinkNames[0])).body_index
    rightFootBodyId = robot.findLink(str(footLinkNames[1])).body_index

    lfootContactPts, rfootContactPts = robotSystem.footstepsDriver.getContactPts()


    footConstraints = []

    for linkName in footLinkNames:
        t = robotSystem.robotStateModel.getLinkFrame(linkName)
        pos, quat = transformUtils.poseFromTransform(t)

        pc = ik.WorldPositionConstraint(robot, robot.findLink(str(linkName)).body_index, np.zeros((3,)), pos, pos, tspan)
        qc = ik.WorldQuatConstraint(robot, robot.findLink(str(linkName)).body_index, quat, 0.0, tspan)
        footConstraints.append(pc)
        footConstraints.append(qc)


    qsc = ik.QuasiStaticConstraint(robot, tspan)
    qsc.setActive(True)
    qsc.setShrinkFactor(0.5)
    qsc.addContact([leftFootBodyId], lfootContactPts.transpose())
    qsc.addContact([rightFootBodyId], rfootContactPts.transpose())


    #q_seed = robot.getZeroConfiguration()
    q_seed = robotSystem.robotStateJointController.getPose('q_nom').copy()

    # todo, joint costs, and other options
    options = ik.IKoptions(robot)

    def solveAndDraw(constraints):

        results = ik.inverseKinSimple(robot, q_seed, q_seed, constraints, options)
        pose = results.q_sol
        robotSystem.robotStateJointController.setPose('pose_end', pose)


    def onFrameModified(obj):

        pos, quat = transformUtils.poseFromTransform(obj.transform)

        pc = ik.WorldPositionConstraint(robot, leftHandBodyId,
                                              np.zeros((3,)),
                                              pos,
                                              pos, tspan)

        qc = ik.WorldQuatConstraint(robot, leftHandBodyId,
                          quat, 0.0, tspan)

        solveAndDraw([qsc, pc, qc] + footConstraints)


    frameObj = vis.updateFrame(robotSystem.robotStateModel.getLinkFrame(ikPlanner.getHandLink('left')), 'left hand frame')
    frameObj.setProperty('Edit', True)
    frameObj.setProperty('Scale', 0.2)
    frameObj.connectFrameModified(onFrameModified)


app = ConsoleApp()
app.setupGlobals(globals())

view = app.createView()
robotSystem = robotsystem.create(view)


testIkPlan()

view.show()
app.start()





'''
Matlab constraints example:

default_shrink_factor = 0.2;
qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(default_shrink_factor);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(links.l_foot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.r_foot, r_foot_pts);


posture_constraint_1 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.neck_ay];
joints_lower_limit = reach_start(joint_inds) + [0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0];
posture_constraint_1 = posture_constraint_1.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.l_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_2 = WorldPositionConstraint(r, links.l_foot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_2 = WorldQuatConstraint(r, links.l_foot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.r_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_3 = WorldPositionConstraint(r, links.r_foot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_3 = WorldQuatConstraint(r, links.r_foot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


posture_constraint_4 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.back_bkx; joints.back_bky; joints.back_bkz];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_4 = posture_constraint_4.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_6 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.l_leg_kny; joints.r_leg_kny];
joints_lower_limit = q_zero(joint_inds) + [0.59999999999999998; 0.59999999999999998];
joints_upper_limit = q_zero(joint_inds) + [2.5; 2.5];
posture_constraint_6 = posture_constraint_6.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [-7.8430774985704943e-08; -0.24449999999997435; -2.7755575615628914e-17];
ref_frame = [2.5835740031888761e-07, 0.99981862375284958, -0.01904519878844263, 0.27344406436719226; 8.9298994930582085e-07, -0.019045198788666409, -0.99981862375247987, 0.74112606401726799; -0.99999999999956801, 2.4130336935024374e-07, -8.9774845025705755e-07, 1.2501258752837239; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_7 = WorldPositionInFrameConstraint(r, links.l_hand, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


quat_constraint_8 = WorldQuatConstraint(r, links.l_hand, [0.50473839491665085; 0.49521580829234357; -0.50473929121040906; 0.49521581540045545], 0.0, [1.0, 1.0]);


posture_constraint_9 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.r_arm_shz; joints.r_arm_shx; joints.r_arm_ely; joints.r_arm_elx; joints.r_arm_uwy; joints.r_arm_mwx; joints.r_arm_lwy];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_9 = posture_constraint_9.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);

'''


