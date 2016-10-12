from director import robotsystem

import os
import numpy as np
import pydrake
import pydrake.solvers.ik as pydrakeik
from director import drcargs


from director.consoleapp import ConsoleApp
from director import visualization as vis
from director import objectmodel as om
from director import ikconstraintencoder as ce
from director import ikconstraints
from director import ikplanner
from director import roboturdf
from director import transformUtils
from director.fieldcontainer import FieldContainer
from director import vtkAll as vtk

from director.pydrakeik import PyDrakeIkServer

from PythonQt import QtCore, QtGui
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
    #constraints.extend(ikPlanner.createFixedFootConstraints(startPoseName))

    constraints.extend(ikPlanner.createSixDofLinkConstraints(startPoseName, ikPlanner.leftFootLink))
    constraints.extend(ikPlanner.createSixDofLinkConstraints(startPoseName, ikPlanner.rightFootLink))

    constraints.append(ikPlanner.createMovingBaseSafeLimitsConstraint())
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
    for path in roboturdf.getPackagePaths():
        packageMap[os.path.basename(path)] = path

    urdfString = open(urdfFile, 'r').read()

    rigidBodyTree = pydrake.rbtree.RigidBodyTree()
    rigidBodyTree.addRobotFromURDFString(urdfString, packageMap)
    return  rigidBodyTree


def getFootContactPoints():

    contact_pts_left = np.zeros((4,3))
    contact_pts_right = np.zeros((4,3))

    modelName = 'valkyrie'

    if modelName == 'atlas':

        contact_pts_left[0,:] = [-0.0876,  0.0626, -0.07645]
        contact_pts_left[1,:] = [-0.0876, -0.0626, -0.07645]
        contact_pts_left[2,:] = [0.1728,   0.0626, -0.07645]
        contact_pts_left[3,:] = [0.1728,  -0.0626, -0.07645]
        contact_pts_right = contact_pts_left.copy()

    elif modelName == 'valkyrie':

        contact_pts_left[0,:] = [-0.038,  0.055, -0.09]
        contact_pts_left[1,:] = [-0.038, -0.055, -0.09]
        contact_pts_left[2,:] = [0.172,   0.055, -0.09]
        contact_pts_left[3,:] = [0.172,  -0.055, -0.09]
        contact_pts_right = contact_pts_left.copy()

    else:
        raise Exception('unknown model name: ' + modelName)

    return contact_pts_left, contact_pts_right

def testIkPlan():

    ikPlanner = robotSystem.ikPlanner

    constraints = buildConstraints()
    poses = ce.getPlanPoses(constraints, ikPlanner)

    global robot

    robot = loadRigidBodyTree(getIkUrdfFilename())


    tspan = np.array([1.0, 1.0])

    handLinkNames = [ikPlanner.getHandLink('left'), ikPlanner.getHandLink('right')]
    footLinkNames = [robotSystem.directorConfig['leftFootLink'], robotSystem.directorConfig['rightFootLink']]

    leftHandBodyId = robot.FindBody(str(handLinkNames[0])).get_body_index()
    rightHandBodyId = robot.FindBody(str(handLinkNames[1])).get_body_index()
    leftFootBodyId = robot.FindBody(str(footLinkNames[0])).get_body_index()
    rightFootBodyId = robot.FindBody(str(footLinkNames[1])).get_body_index()

    lfootContactPts, rfootContactPts = getFootContactPoints()
    #lfootContactPts, rfootContactPts = = robotSystem.footstepsDriver.getContactPts()


    for i in xrange(robot.get_num_bodies()):
        body = robot.get_body(i)
        print i, body.get_name()
        assert robot.FindBodyIndex(body.get_name()) == i

    q = np.zeros(robot.get_num_positions())
    v = np.zeros(robot.get_num_velocities())
    kinsol = robot.doKinematics(q,v)

    t = robot.relativeTransform(kinsol, robot.FindBodyIndex('world'), robot.FindBodyIndex(str(footLinkNames[0])))
    tt = transformUtils.getTransformFromNumpy(t)

    pos = transformUtils.transformations.translation_from_matrix(t)
    quat = transformUtils.transformations.quaternion_from_matrix(t)



    footConstraints = []

    for linkName in footLinkNames:
        t = robotSystem.robotStateModel.getLinkFrame(linkName)
        pos, quat = transformUtils.poseFromTransform(t)

        if False and linkName == footLinkNames[0]:
            pc = pydrakeik.WorldPositionConstraint(robot, robot.findLink(str(linkName)).get_body_index(), np.zeros((3,)), pos + [-10,-10,0], pos+[10,10,0], tspan)
        else:
            pc = pydrakeik.WorldPositionConstraint(robot, robot.findLink(str(linkName)).get_body_index(), np.zeros((3,)), pos, pos, tspan)

        qc = pydrakeik.WorldQuatConstraint(robot, robot.findLink(str(linkName)).get_body_index(), quat, 0.01, tspan)
        footConstraints.append(pc)
        footConstraints.append(qc)


    qsc = pydrakeik.QuasiStaticConstraint(robot, tspan)
    qsc.setActive(True)
    qsc.setShrinkFactor(1.0)
    qsc.addContact([leftFootBodyId], lfootContactPts.transpose())
    qsc.addContact([rightFootBodyId], rfootContactPts.transpose())


    #q_seed = robot.getZeroConfiguration()
    #q_seed = robotSystem.robotStateJointController.getPose('q_nom').copy()

    # todo, joint costs, and other options
    global options
    options = pydrakeik.IKoptions(robot)


    jointIndexMap = np.zeros(robotSystem.robotStateJointController.numberOfJoints)
    drakePositionNames = [robot.get_position_name(i) for i in xrange(robot.get_num_positions())]
    for i, name in enumerate(robotSystem.robotStateJointController.jointNames):
        jointIndexMap[i] = drakePositionNames.index(name)

    jointIndexMap = list(jointIndexMap)
    print jointIndexMap

    q_seed = np.zeros(robot.get_num_positions())
    q_seed[jointIndexMap] = robotSystem.robotStateJointController.getPose('q_nom').copy()

    # setup costs

    costs = np.ones(robot.get_num_positions())


    groupCosts = [
      ('Left Leg', 1e3),
      ('Right Leg', 1e3),
      ('Left Arm', 1),
      ('Right Arm', 1),
      ('Back', 1e4),
      ('Neck', 1e6),
      ]

    for groupName, costValue in groupCosts:
        names = robotSystem.ikPlanner.getJointGroup(groupName)
        inds = [drakePositionNames.index(name) for name in names]
        print costValue, names
        costs[inds] = costValue

    costs[drakePositionNames.index('base_x')] = 0
    costs[drakePositionNames.index('base_y')] = 0
    costs[drakePositionNames.index('base_z')] = 0
    costs[drakePositionNames.index('base_roll')] = 1e3
    costs[drakePositionNames.index('base_pitch')] = 1e3
    costs[drakePositionNames.index('base_yaw')] = 0


    print costs
    options.setQ(np.diag(costs))


    def solveAndDraw(constraints):

        #print q_seed.shape

        global results
        global constraintList
        constraintList = constraints
        results = pydrakeik.InverseKin(robot, q_seed, q_seed, constraints, options)
        pose = results.q_sol[0]

        #print results.info[0]

        pose = pose[jointIndexMap]

        robotSystem.robotStateJointController.setPose('pose_end', pose)


    def onFrameModified(obj):

        pos, quat = transformUtils.poseFromTransform(obj.transform)

        pc = pydrakeik.WorldPositionConstraint(robot, leftHandBodyId,
                                              np.zeros((3,)),
                                              pos,
                                              pos, tspan)

        qc = pydrakeik.WorldQuatConstraint(robot, leftHandBodyId,
                          quat, 0.01, tspan)

        solveAndDraw([qsc, pc, qc] + footConstraints)

        #solveAndDraw([pc])


    frameObj = vis.updateFrame(robotSystem.robotStateModel.getLinkFrame(ikPlanner.getHandLink('left')), 'left hand frame')
    frameObj.setProperty('Edit', True)
    frameObj.setProperty('Scale', 0.2)
    frameObj.connectFrameModified(onFrameModified)


def makeRobotSystem(view):

    '''
    args = dict()

    urdfFile = drcargs.getDirectorConfig()['urdfConfig']['ik']
    colorMode = drcargs.getDirectorConfig()['colorMode']

    robotStateModel, robotStateJointController = roboturdf.loadRobotModel('robot model', view, urdfFile=urdfFile, colorMode=colorMode)

    robotStateModel.setProperty('Color Mode', 'Textures')

    handFactory = roboturdf.HandFactory(robotStateModel)
    handModels = []

    for side in ['left', 'right']:
        if side in handFactory.defaultHandTypes:
            handModels.append(handFactory.getLoader(side))

    ikPlanner = ikplanner.IKPlanner(None, robotStateModel, robotStateJointController, handModels)

    manipPlanner = None #robotplanlistener.ManipulationPlanDriver(ikPlanner)

    return FieldContainer(
      robotStateModel=robotStateModel,
      robotStateJointController=robotStateJointController,
      directorConfig=drcargs.getDirectorConfig(),
      ikPlanner=ikPlanner,
      manipPlanner=manipPlanner,
      footstepsDriver=None,
      )
    '''

    factory = robotsystem.RobotSystemFactory()

    options = factory.getDisabledOptions()
    options.useDirectorConfig = True
    options.useRobotState = True
    options.usePlanning = True
    options.usePlayback = True
    options.useAffordances = True
    options.usePlannerPublisher = True
    options.useTeleop = True

    robotSystem = factory.construct(view=view, options=options)
    return robotSystem


app = ConsoleApp()
app.setupGlobals(globals())

view = app.createView()

robotSystem = makeRobotSystem(view)

robotSystem.ikPlanner.planningMode = 'pydrake'
robotSystem.teleopPanel.widget.show()

#testIkPlan()


#constraintSet = ikplanner.ConstraintSet(robotSystem.ikPlanner, buildConstraints(), endPoseName='user_end', startPoseName='q_nom')
#fields = constraintSet.runIk()
#print fields



#####
from director.debugVis import DebugData

def drawCenterOfMass(model):
    #stanceFrame = footstepsDriver.getFeetMidPoint(model)
    com = list(model.model.getCenterOfMass())
    com[2] = 0.0 #stanceFrame.GetPosition()[2]
    d = DebugData()
    d.addSphere(com, radius=0.015)
    obj = vis.updatePolyData(d.getPolyData(), 'COM %s' % model.getProperty('Name'), color=[1,0,0], visible=False, parent=model)

    for linkName in [robotSystem.ikPlanner.leftFootLink, robotSystem.ikPlanner.rightFootLink]:
        d = DebugData()
        contactPts = robotSystem.ikPlanner.robotModel.getLinkContactPoints(linkName)
        linkFrame = model.getLinkFrame(linkName)

        for p in contactPts:
            p = linkFrame.TransformPoint(p)
            d.addSphere(p, radius=0.01)

        obj = vis.updatePolyData(d.getPolyData(), '%s %s contact points' % (model.getProperty('Name'), linkName), color=[1,0,0], visible=False, parent=model)


def initCenterOfMassVisulization():
    for model in [robotSystem.robotStateModel, robotSystem.teleopRobotModel, robotSystem.playbackRobotModel]:
        model.connectModelChanged(drawCenterOfMass)
        drawCenterOfMass(model)

#if not robotSystem.ikPlanner.fixedBaseArm:
#    initCenterOfMassVisulization()

#####

#endPose, info = s.runIk(fields)

#print endPose
#print info


#ikplanner.RobotPoseGUIWrapper.show()

#view.show()


w = QtGui.QWidget()
l = QtGui.QGridLayout(w)
l.addWidget(view, 0, 0)
l.addWidget(robotSystem.playbackPanel.widget, 1, 0)
l.addWidget(robotSystem.teleopPanel.widget, 0, 1, 2, 1)
l.setMargin(0)
l.setSpacing(0)
w.showMaximized()
w.raise_()

from director import applogic
applogic.resetCamera(viewDirection=[-1,0,0], view=view)

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


