import os
import sys
import vtkAll as vtk
from ddapp import botpy
import math
import time
import types
import functools
import random
import numpy as np

from ddapp import transformUtils
from ddapp import lcmUtils
from ddapp.timercallback import TimerCallback
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp import applogic as app
from ddapp.debugVis import DebugData
from ddapp import ioUtils
from ddapp.simpletimer import SimpleTimer
from ddapp.utime import getUtime
from ddapp import robotstate
from ddapp import planplayback
from ddapp import segmentation

from ddapp import ik

import drc as lcmdrc

from PythonQt import QtCore, QtGui

copyFrame = transformUtils.copyFrame


class ConstraintSet(object):

    def __init__(self, ikPlanner, constraints, endPoseName, startPoseName):
        self.ikPlanner = ikPlanner
        self.constraints = constraints
        self.endPoseName = endPoseName
        self.startPoseName = startPoseName
        self.nominalPoseName = 'q_nom'

    def runIk(self):

        self.endPose, self.info = self.ikPlanner.ikServer.runIk(self.constraints, nominalPostureName=self.startPoseName, seedPostureName=self.startPoseName)
        print 'info:', self.info

        return self.endPose, self.info

    def runIkTraj(self):
        self.ikPlanner.addPose(self.endPose, self.endPoseName)
        self.plan = self.ikPlanner.runIkTraj(self.constraints, self.startPoseName, self.endPoseName, self.nominalPoseName)
        return self.plan

    def planEndPoseGoal(self):
        self.ikPlanner.addPose(self.endPose, self.endPoseName)
        self.plan = self.ikPlanner.computePostureGoal(self.startPoseName, self.endPoseName)
        return self.plan


    def onFrameModified(self, frame):
        self.runIk()


class IKPlanner(object):

    def __init__(self, ikServer, robotModel, jointController, handModels):

        self.ikServer = ikServer
        self.robotModel = robotModel
        self.jointController = jointController
        self.handModels = handModels

        self.reachingSide = 'left'

        self.additionalTimeSamples = 0
        self.useQuasiStaticConstraint = True


    def getHandModel(self, side=None):
        side = side or self.reachingSide
        assert side in ('left', 'right')
        return self.handModels[0] if side == 'left' else self.handModels[1]


    def getHandLink(self, side=None):
        return self.getHandModel(side).handLinkName


    def getPalmToHandLink(self, side=None):
        return self.getHandModel(side).palmToHandLink


    def getPalmPoint(self, side=None):
        return np.array(self.getPalmToHandLink(side).GetPosition())


    def createHandRelativePositionConstraint(self, side, targetBodyName, targetPosition):

        p = ik.RelativePositionConstraint()
        p.bodyNameA = self.getHandLink()
        p.bodyNameB = targetBodyName
        p.pointInBodyA = self.getPalmPoint()
        p.positionTarget = targetPosition
        return p


    def createQuasiStaticConstraint(self):
        return ik.QuasiStaticConstraint()


    def createFixedFootConstraints(self, startPose):

        self.jointController.setPose(startPose)
        constraints = []
        for linkName in ['l_foot', 'r_foot']:
            linkFrame = self.robotModel.getLinkFrame(linkName)
            p = ik.PositionConstraint(linkName=linkName, positionTarget=linkFrame)
            q = ik.QuatConstraint(linkName=linkName, quaternion=linkFrame)
            constraints.append(p)
            constraints.append(q)

        return constraints


    def createMovingFootConstraints(self, startPose):

        self.jointController.setPose(startPose)
        constraints = []
        for linkName in ['l_foot', 'r_foot']:
            linkFrame = self.robotModel.getLinkFrame(linkName)
            p = ik.PositionConstraint(linkName=linkName, positionTarget=linkFrame)
            p.lowerBound = [-np.inf, -np.inf, 0.0]
            p.upperBound = [np.inf, np.inf, 0.0]

            g = ik.WorldGazeDirConstraint()
            g.linkName = linkName
            g.targetFrame = vtk.vtkTransform()
            g.targetAxis = [0,0,1]
            g.bodyAxis = [0,0,1]
            g.coneThreshold = 0.0

            constraints.append(p)
            constraints.append(g)
            #constraints.append(WorldFixedBodyPoseConstraint(linkName=linkName))

        return constraints


    def createKneePostureConstraint(self, bounds):
        '''
        bounds is a size 2 vector of [lower, upper] bounds to be
        applied to both knees
        '''
        p = ik.PostureConstraint()
        p.joints = robotstate.matchJoints('.*_leg_kny')
        p.jointsLowerBound = np.tile(bounds[0], 2)
        p.jointsUpperBound = np.tile(bounds[1], 2)
        return p


    def createMovingKneePostureConstraint(self):
        return self.createKneePostureConstraint([0.0, 2.1])


    def createMovingBasePostureConstraint(self, startPostureName):
        joints = ['base_x', 'base_y', 'base_roll', 'base_pitch', 'base_yaw']
        return self.createPostureConstraint(startPostureName, joints)


    def createMovingBackPostureConstraint(self):
        p = ik.PostureConstraint()
        p.joints = ['back_bkx', 'back_bky', 'back_bkz']
        p.jointsLowerBound = [-0.2, -0.1, -0.6]
        p.jointsUpperBound = [0.2, 0.3, 0.6]
        return p


    def createPositionOrientationGraspConstraints(self, side, targetFrame, graspToHandLinkFrame=None, positionTolerance=0.0, angleToleranceInDegrees=0.0):

        graspToHandLinkFrame = graspToHandLinkFrame or self.getPalmToHandLink(side)

        p = ik.PositionConstraint()
        p.linkName = self.getHandLink(side)
        p.pointInLink = np.array(graspToHandLinkFrame.GetPosition())
        p.referenceFrame = targetFrame.transform
        p.lowerBound = np.tile(-positionTolerance, 3)
        p.upperBound = np.tile(positionTolerance, 3)
        positionConstraint = p

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(graspToHandLinkFrame.GetLinearInverse())
        t.Concatenate(targetFrame.transform)

        p = ik.QuatConstraint()
        p.linkName = self.getHandLink(side)
        p.quaternion = t
        p.angleToleranceInDegrees = angleToleranceInDegrees
        orientationConstraint = p

        return positionConstraint, orientationConstraint


    def createAxisInPlaneConstraint(self, targetFrame, graspToHandLinkFrame):
        '''
        Constrain the X axis of targetFrame to be in the XY plane of graspToHandLinkFrame in hand link.
        Returns two relative position constraints.
        '''

        def makeOffsetTransform(offset):
            t = vtk.vtkTransform()
            t.PostMultiply()
            t.Translate(offset)
            t.Concatenate(targetFrame)
            return t

        p = ik.RelativePositionConstraint()
        p.bodyNameA = 'world'
        p.bodyNameB = self.getHandLink()
        p.frameInBodyB = graspToHandLinkFrame
        p.pointInBodyA = makeOffsetTransform([0.25, 0.0, 0.0])
        p.positionTarget = np.zeros(3)
        p.lowerBound = np.array([np.nan, np.nan, 0.0])
        p.upperBound = np.array([np.nan, np.nan, 0.0])
        rollConstraint1 = p

        p = ik.RelativePositionConstraint()
        p.bodyNameA = 'world'
        p.bodyNameB = self.getHandLink()
        p.frameInBodyB = graspToHandLinkFrame
        p.pointInBodyA = makeOffsetTransform([-0.25, 0.0, 0.0])
        p.positionTarget = np.zeros(3)
        p.lowerBound = np.array([np.nan, np.nan, 0.0])
        p.upperBound = np.array([np.nan, np.nan, 0.0])
        rollConstraint2 = p

        return rollConstraint1, rollConstraint2


    def createGraspOrbitConstraints(self, targetFrame, graspToHandLinkFrame=None):

        if graspToHandLinkFrame is None:
            graspToHandLinkFrame = self.getPalmToHandLink()

        p = ik.PositionConstraint()
        p.linkName = self.getHandLink()
        p.pointInLink = np.array(graspToHandLinkFrame.GetPosition())
        p.referenceFrame = targetFrame.transform
        p.lowerBound = [0.0, 0.0, 0.0] #np.zeros(3)
        p.upperBound = [0.07, 0.0, 0.0] #np.zeros(3)
        positionConstraint = p

        rollConstraint1, rollConstraint2 = self.createAxisInPlaneConstraint(targetFrame.transform, graspToHandLinkFrame)

        return positionConstraint, rollConstraint1, rollConstraint2


    def createGazeGraspConstraint(self, side, targetFrame, graspToHandLinkFrame=None):


        graspToHandLinkFrame = graspToHandLinkFrame or self.getPalmToHandLink(side)

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(graspToHandLinkFrame.GetLinearInverse())
        t.Concatenate(targetFrame.transform)


        gazeAxis = [0.0, 1.0, 0.0]

#        g = ik.WorldGazeOrientConstraint()
#        g.linkName = self.getHandLink()
#        g.quaternion = t
#        g.axis = gazeAxis
#        g.coneThreshold = math.radians(0)
#        g.threshold = math.radians(180)

        g = ik.WorldGazeDirConstraint()
        g.linkName = self.getHandLink(side)
        g.targetFrame = t
        g.targetAxis = gazeAxis
        g.bodyAxis = gazeAxis
        g.coneThreshold = math.radians(5)
        g.tspan = [1.0, 1.0]
        return g


    def createMoveOnLineConstraints(self, startPose, targetFrame, graspToHandLinkFrame=None):

        if graspToHandLinkFrame is None:
            graspToHandLinkFrame = self.getPalmToHandLink()


        self.jointController.setPose('start_pose', startPose)
        linkFrame = self.robotModel.getLinkFrame(self.getHandLink())
        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(graspToHandLinkFrame)
        t.Concatenate(linkFrame)

        motionAxisInWorld = np.array(targetFrame.GetPosition()) - np.array(t.GetPosition())

        targetFrame = transformUtils.getTransformFromOriginAndNormal(targetFrame.GetPosition(), motionAxisInWorld)

        p = ik.PositionConstraint()
        p.linkName = self.getHandLink()
        p.pointInLink = np.array(graspToHandLinkFrame.GetPosition())
        p.referenceFrame = targetFrame
        p.lowerBound = np.zeros(3)
        p.upperBound = np.zeros(3)
        positionConstraint = p

        p = ik.QuatConstraint()
        p.linkName = self.getHandLink()
        p.quaternion = linkFrame
        p.angleToleranceInDegrees = 2
        orientationConstraint = p

        p = ik.PositionConstraint()
        p.linkName = self.getHandLink()
        p.pointInLink = np.array(graspToHandLinkFrame.GetPosition())
        p.referenceFrame = targetFrame
        p.lowerBound = [0.0, 0.0, np.nan]
        p.upperBound = [0.0, 0.0, np.nan]
        axisConstraint = p

        return positionConstraint, orientationConstraint, axisConstraint


    def createMovingBodyConstraints(self, startPoseName, lockBase=False, lockBack=False, lockLeftArm=False, lockRightArm=False):

        constraints = []
        if self.useQuasiStaticConstraint:
            constraints.append(self.createQuasiStaticConstraint())
        constraints.extend(self.createFixedFootConstraints(startPoseName))

        if lockBack:
            constraints.append(self.createLockedBackPostureConstraint(startPoseName))
        else:
            constraints.append(self.createMovingBackPostureConstraint())

        if lockBase:
            constraints.append(self.createLockedBasePostureConstraint(startPoseName))
        else:
            constraints.append(self.createMovingBasePostureConstraint(startPoseName))
            #constraints.append(self.createMovingKneePostureConstraint())

        if lockLeftArm:
            constraints.append(self.createLockedLeftArmPostureConstraint(startPoseName))
        if lockRightArm:
            constraints.append(self.createLockedRightArmPostureConstraint(startPoseName))

        return constraints


    def createMovingReachConstraints(self, startPoseName, lockBase=False, lockBack=False, lockArm=True):
        lockLeftArm = lockArm and (self.reachingSide == 'right')
        lockRightArm = lockArm and (self.reachingSide == 'left')
        return self.createMovingBodyConstraints(startPoseName, lockBase, lockBack, lockLeftArm=lockLeftArm, lockRightArm=lockRightArm)


    def getLinkFrameAtPose(self, linkName, pose):
        self.jointController.setPose('user_pose', pose)
        return self.robotModel.getLinkFrame(linkName)

    def getRobotModelAtPose(self, pose):
        self.jointController.setPose('user_pose', pose)
        return self.robotModel

    def computeNominalPose(self, startPose):

        nominalPoseName = 'q_nom'
        startPoseName = 'stand_start'
        self.jointController.addPose(startPoseName, startPose)
        self.ikServer.sendPoseToServer(startPose, startPoseName)

        constraints = []
        constraints.extend(self.createFixedFootConstraints(startPoseName))
        constraints.append(self.createMovingBasePostureConstraint(startPoseName))
        constraints.append(self.createLockedLeftArmPostureConstraint(nominalPoseName))
        constraints.append(self.createLockedRightArmPostureConstraint(nominalPoseName))
        constraints.append(self.createPostureConstraint(nominalPoseName, robotstate.matchJoints('.*_leg_kny')))
        constraints.append(self.createPostureConstraint(nominalPoseName, robotstate.matchJoints('back')))

        endPose, info = self.ikServer.runIk(constraints, seedPostureName=startPoseName)
        return endPose, info


    def computeNominalPlan(self, startPose):

        endPose, info = self.computeNominalPose(startPose)
        print 'info:', info

        return self.computePostureGoal(startPose, endPose)


    def computeStandPose(self, startPose):

        nominalPoseName = 'q_nom'
        startPoseName = 'stand_start'
        self.jointController.addPose(startPoseName, startPose)
        self.ikServer.sendPoseToServer(startPose, startPoseName)

        constraints = []
        constraints.extend(self.createFixedFootConstraints(startPoseName))
        constraints.append(self.createMovingBasePostureConstraint(startPoseName))
        constraints.append(self.createLockedLeftArmPostureConstraint(startPoseName))
        constraints.append(self.createLockedRightArmPostureConstraint(startPoseName))
        constraints.append(self.createPostureConstraint(nominalPoseName, robotstate.matchJoints('.*_leg_kny')))
        constraints.append(self.createPostureConstraint(nominalPoseName, robotstate.matchJoints('back')))

        endPose, info = self.ikServer.runIk(constraints, seedPostureName=startPoseName)
        return endPose, info


    def computeStandPlan(self, startPose):

        endPose, info = self.computeStandPose(startPose)
        print 'info:', info

        return self.computePostureGoal(startPose, endPose)


    def createPostureConstraint(self, startPostureName, jointNames):
        p = ik.PostureConstraint()
        p.postureName = startPostureName
        p.joints = jointNames
        p.jointsLowerBound = np.tile(0.0, len(p.joints))
        p.jointsUpperBound = np.tile(0.0, len(p.joints))
        return p


    def createLockedTorsoPostureConstraint(self, startPostureName):
        joints = []
        joints += robotstate.matchJoints('base_.*')
        joints += robotstate.matchJoints('back_.*')
        joints += robotstate.matchJoints('r_leg_.*')
        joints += robotstate.matchJoints('l_leg_.*')
        return self.createPostureConstraint(startPostureName, joints)


    def createLockedBasePostureConstraint(self, startPostureName):
        joints = []
        joints += robotstate.matchJoints('base_.*')
        joints += robotstate.matchJoints('r_leg_.*')
        joints += robotstate.matchJoints('l_leg_.*')
        return self.createPostureConstraint(startPostureName, joints)


    def createLockedBackPostureConstraint(self, startPostureName):
        joints = robotstate.matchJoints('back_.*')
        return self.createPostureConstraint(startPostureName, joints)


    def createLockedRightArmPostureConstraint(self, startPostureName):
        joints = robotstate.matchJoints('r_arm_.*')
        return self.createPostureConstraint(startPostureName, joints)


    def createLockedLeftArmPostureConstraint(self, startPostureName):
        joints = robotstate.matchJoints('l_arm_.*')
        return self.createPostureConstraint(startPostureName, joints)


    def createLockedArmPostureConstraint(self, startPostureName):
        if self.reachingSide == 'left':
            return self.createLockedRightArmPostureConstraint(startPostureName)
        else:
            return self.createLockedLeftArmPostureConstraint(startPostureName)


    def getMergedPostureFromDatabase(self, startPose, poseGroup, poseName, side=None):
        postureJoints = RobotPoseGUIWrapper.getPose(poseGroup, poseName, side=side)
        return self.mergePostures(startPose, postureJoints)


    def mergePostures(self, startPose, postureJoints, referencePose=None):
        '''
        postureJoints is either a dict of jointName-->jointPosition or a
        list of jointName.  If postureJoints is a list, then referencePose
        is used to lookup joint positions.
        '''
        endPose = np.array(startPose)

        if referencePose is None:

            for name, position in postureJoints.iteritems():
                jointId = robotstate.getDrakePoseJointNames().index(name)
                endPose[jointId] = position
        else:

            for name in postureJoints:
                jointId = robotstate.getDrakePoseJointNames().index(name)
                endPose[jointId] = referencePose[jointId]

        return endPose



    def planEndEffectorDelta(self, startPose, side, worldDeltaVector, constraints=None):

        self.reachingSide = side

        startPoseName = 'reach_start'
        self.jointController.addPose(startPoseName, startPose)
        self.ikServer.sendPoseToServer(startPose, startPoseName)

        if constraints is None:
            constraints = []
            constraints.extend(self.createMovingReachConstraints(startPoseName))


        graspToHandLinkFrame = self.getPalmToHandLink()

        self.jointController.setPose('start_pose', startPose)
        linkFrame = self.robotModel.getLinkFrame(self.getHandLink())

        targetFrame = vtk.vtkTransform()
        targetFrame.PostMultiply()
        targetFrame.Concatenate(graspToHandLinkFrame)
        targetFrame.Concatenate(linkFrame)
        targetFrame.Translate(worldDeltaVector)

        constraints.extend(self.createMoveOnLineConstraints(startPose, targetFrame, graspToHandLinkFrame))

        constraints[-2].tspan = [1.0, 1.0]
        constraints[-3].tspan = [1.0, 1.0]

        endPoseName = 'reach_end'
        constraintSet = ConstraintSet(self, constraints, endPoseName, startPoseName)
        return constraintSet


    def planGraspOrbitReachPlan(self, startPose, side, graspFrame, constraints=None, dist=0.0, lockTorso=False, lockArm=True):

        self.reachingSide = side

        startPoseName = 'reach_start'
        self.addPose(startPose, startPoseName)

        if constraints is None:
            constraints = []

            if lockTorso:
                constraints.append(self.createLockedTorsoPostureConstraint(startPoseName))
                constraints.append(self.createLockedArmPostureConstraint(startPoseName))
            else:
                constraints.extend(self.createMovingReachConstraints(startPoseName, lockArm=lockArm))

        graspToHandLinkFrame = self.newPalmOffsetGraspToHandFrame(side, dist)

        constraints.extend(self.createGraspOrbitConstraints(graspFrame, graspToHandLinkFrame))

        constraints[-3].tspan = [1.0, 1.0]
        constraints[-2].tspan = [1.0, 1.0]
        constraints[-1].tspan = [1.0, 1.0]

        endPoseName = 'reach_end'
        constraintSet = ConstraintSet(self, constraints, endPoseName, startPoseName)
        return constraintSet


    def addPose(self, pose, poseName):
        self.jointController.addPose(poseName, pose)
        self.ikServer.sendPoseToServer(pose, poseName)


    def newPalmOffsetGraspToHandFrame(self, side, distance):
        t = vtk.vtkTransform()
        t.Translate(0.0, distance, 0.0)
        return self.newGraspToHandFrame(side, t)


    def newGraspToHandFrame(self, side, graspToPalmFrame=None):
        '''
        Creates a grasp to hand link frame given a grasp to palm transform.
        Uses getPalmToHandLink() to complete the transform.  If the
        graspToPalmFrame is None or identity, the returned frame will
        be the palm to hand link frame.
        '''

        t = vtk.vtkTransform()
        t.PostMultiply()

        if graspToPalmFrame:
            t.Concatenate(graspToPalmFrame)
        t.Concatenate(self.getPalmToHandLink(side))
        return t


    def newGraspToWorldFrame(self, startPose, side, graspToHandFrame):

        handToWorld = self.getLinkFrameAtPose(self.getHandLink(side), startPose)
        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(graspToHandFrame)
        t.Concatenate(handToWorld)
        return t


    def newReachGoal(self, startPoseName, side, targetFrame, constraints, graspToHandLinkFrame=None, lockOrient=True):

        if graspToHandLinkFrame is None:
            graspToHandLinkFrame = self.newGraspToHandFrame(side)

        positionConstraint, orientationConstraint = self.createPositionOrientationGraspConstraints(side, targetFrame, graspToHandLinkFrame, positionTolerance=0.0, angleToleranceInDegrees=0.0)

        positionConstraint.tspan = [1.0, 1.0]
        orientationConstraint.tspan = [1.0, 1.0]

        constraints.append(positionConstraint)
        if lockOrient:
            constraints.append(orientationConstraint)

        constraintSet = ConstraintSet(self, constraints, 'reach_end', startPoseName)
        return constraintSet


    def planEndEffectorGoal(self, startPose, side, graspFrame, constraints=None, lockTorso=False, lockArm=True):

        self.reachingSide = side

        startPoseName = 'reach_start'
        self.addPose(startPose, startPoseName)

        if constraints is None:
            constraints = self.createMovingReachConstraints(startPoseName, lockBase=lockTorso, lockBack=lockTorso, lockArm=lockArm)

        return self.newReachGoal(startPoseName, side, graspFrame, constraints)


    def computeMultiPostureGoal(self, poses, feetOnGround=True):

        assert len(poses) >= 2

        constraints = []
        poseNames = []
        for i, pose in enumerate(poses):

            if isinstance(pose, str):
                poseName, pose = pose, self.jointController.getPose(pose)
            else:
                poseName = 'posture_goal_%d' % i

            self.addPose(pose, poseName)
            p = self.createPostureConstraint(poseName, robotstate.matchJoints('.*'))
            p.tspan = np.array([float(i), float(i)])
            constraints.append(p)
            poseNames.append(poseName)

        if feetOnGround:
            constraints.extend(self.createFixedFootConstraints(poseNames[0]))

        #if self.useQuasiStaticConstraint:
        #    constraints.append(self.createQuasiStaticConstraint())

        return self.runIkTraj(constraints[1:], poseNames[0], poseNames[-1])


    def computePostureGoal(self, poseStart, poseEnd, feetOnGround=True):
        return self.computeMultiPostureGoal([poseStart, poseEnd], feetOnGround)


    def getManipPlanListener(self):
        responseChannel = 'CANDIDATE_MANIP_PLAN'
        responseMessageClass = lcmdrc.robot_plan_w_keyframes_t
        return lcmUtils.MessageResponseHelper(responseChannel, responseMessageClass)


    def runIkTraj(self, constraints, poseStart, poseEnd, nominalPoseName='q_nom', timeSamples=None):

        listener = self.getManipPlanListener()

        info = self.ikServer.runIkTraj(constraints, poseStart=poseStart, poseEnd=poseEnd, nominalPose=nominalPoseName, timeSamples=timeSamples, additionalTimeSamples=self.additionalTimeSamples)
        print 'traj info:', info

        self.lastManipPlan = listener.waitForResponse()
        listener.finish()
        return self.lastManipPlan


    def computePostureCost(self, pose):

        joints = robotstate.getDrakePoseJointNames()
        nominalPose = self.jointController.getPose('q_nom')
        assert len(nominalPose) == len(joints)
        cost = np.zeros(len(joints))
        cost[[joints.index(n) for n in robotstate.matchJoints('back')]] = 100
        cost[[joints.index(n) for n in robotstate.matchJoints('arm')]] = 1

        return np.sum(np.abs(pose - nominalPose)*cost)




sys.path.append(os.path.join(app.getDRCBase(), 'software/tools/tools/scripts'))
import RobotPoseGUI as rpg


class RobotPoseGUIWrapper(object):

    initialized = False
    main = None

    @classmethod
    def init(cls):
        if cls.initialized:
            return True

        rpg.lcmWrapper = rpg.LCMWrapper()
        cls.main = rpg.MainWindow()
        cls.initialized = True

    @classmethod
    def show(cls):
        cls.init()
        cls.main.show()
        cls.main.raise_()
        cls.main.activateWindow()

    @classmethod
    def getPose(cls, groupName, poseName, side=None):

        cls.init()

        config = rpg.loadConfig(cls.main.getPoseConfigFile())
        assert groupName in config

        poses = {}
        for pose in config[groupName]:
            poses[pose['name']] = pose

        assert poseName in poses
        pose = poses[poseName]
        joints = pose['joints']

        if side is not None:
            sides = ('left', 'right')
            assert side in sides
            assert pose['nominal_handedness'] in sides

            if pose['nominal_handedness'] != side:
                joints = rpg.applyMirror(joints)

        return joints
