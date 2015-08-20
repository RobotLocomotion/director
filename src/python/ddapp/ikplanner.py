import os
import sys
import vtkAll as vtk
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
from ddapp import drcargs

from ddapp import ik
from ddapp.ikparameters import IkParameters
from ddapp import ikconstraintencoder

import drc as lcmdrc
import json

import PythonQt
from PythonQt import QtCore, QtGui

copyFrame = transformUtils.copyFrame

def getIkOptions():
    return om.findObjectByName('IK Planner Options').properties


class ConstraintSet(object):

    def __init__(self, ikPlanner, constraints, endPoseName, startPoseName):
        self.ikPlanner = ikPlanner
        self.ikParameters = IkParameters()
        self.constraints = constraints
        self.endPoseName = endPoseName
        self.startPoseName = startPoseName
        self.endPose = None
        self.seedPoseName = None
        self.nominalPoseName = None

    def runIk(self):
        seedPoseName = self.seedPoseName
        if not seedPoseName:
            seedPoseName = getIkOptions().getPropertyEnumValue('Seed pose')
        if seedPoseName == 'q_start':
            seedPoseName = self.startPoseName

        nominalPoseName = self.nominalPoseName
        if not nominalPoseName:
            nominalPoseName = getIkOptions().getPropertyEnumValue('Nominal pose')
        if nominalPoseName == 'q_start':
            nominalPoseName = self.startPoseName

        if (self.ikPlanner.pushToMatlab is False):
            self.endPose, self.info = self.ikPlanner.plannerPub.processIK(self.constraints, nominalPoseName=nominalPoseName, seedPoseName=seedPoseName)
            return self.endPose, self.info
        else:
            ikParameters = self.ikPlanner.mergeWithDefaultIkParameters(self.ikParameters)

            self.endPose, self.info = self.ikPlanner.ikServer.runIk(self.constraints, ikParameters, nominalPostureName=nominalPoseName, seedPostureName=seedPoseName)
            print 'info:', self.info
            return self.endPose, self.info

    def runIkTraj(self):
        assert self.endPose is not None
        self.ikPlanner.addPose(self.endPose, self.endPoseName)

        nominalPoseName = self.nominalPoseName
        if not nominalPoseName:
            nominalPoseName = getIkOptions().getPropertyEnumValue('Nominal pose')
        if nominalPoseName == 'q_start':
            nominalPoseName = self.startPoseName

        ikParameters = self.ikPlanner.mergeWithDefaultIkParameters(self.ikParameters)
        self.plan = self.ikPlanner.runIkTraj(self.constraints, self.startPoseName, self.endPoseName, nominalPoseName, ikParameters=ikParameters)

        return self.plan

    def planEndPoseGoal(self, feetOnGround = True):
        assert self.endPose is not None
        self.ikPlanner.addPose(self.endPose, self.endPoseName)
        ikParameters = self.ikPlanner.mergeWithDefaultIkParameters(self.ikParameters)
        self.plan = self.ikPlanner.computePostureGoal(self.startPoseName, self.endPoseName, feetOnGround, ikParameters=ikParameters)
        return self.plan

    def onFrameModified(self, frame):
        self.runIk()


class IkOptionsItem(om.ObjectModelItem):

    def __init__(self, ikServer, ikPlanner):
        om.ObjectModelItem.__init__(self, 'IK Planner Options')

        self.ikServer = ikServer
        self.ikPlanner = ikPlanner

        self.addProperty('Use pointwise', ikPlanner.defaultIkParameters.usePointwise)
        self.addProperty('Use collision', ikPlanner.defaultIkParameters.useCollision)
        self.addProperty('Collision min distance', ikPlanner.defaultIkParameters.collisionMinDistance, attributes=om.PropertyAttributes(decimals=3, minimum=0.001, maximum=9.999, singleStep=0.01 ))
        self.addProperty('Add knots', ikPlanner.defaultIkParameters.numberOfAddedKnots)
        #self.addProperty('Use quasistatic constraint', ikPlanner.useQuasiStaticConstraint)
        self.addProperty('Quasistatic shrink factor', ikPlanner.defaultIkParameters.quasiStaticShrinkFactor, attributes=om.PropertyAttributes(decimals=2, minimum=0.0, maximum=10.0, singleStep=0.1))
        self.addProperty('Max joint degrees/s', ikPlanner.defaultIkParameters.maxDegreesPerSecond, attributes=om.PropertyAttributes(decimals=0, minimum=1, maximum=100.0, singleStep=1.0))
        self.addProperty('Nominal pose', 1, attributes=om.PropertyAttributes(enumNames=['q_start', 'q_nom', 'q_end', 'q_zero']))
        self.addProperty('Seed pose', 0, attributes=om.PropertyAttributes(enumNames=['q_start', 'q_nom', 'q_end', 'q_zero']))
        self.addProperty('Major iterations limit', ikPlanner.defaultIkParameters.majorIterationsLimit)
        self.addProperty('Major feasibility tolerance', ikPlanner.defaultIkParameters.majorFeasibilityTolerance, attributes=om.PropertyAttributes(decimals=6, minimum=1e-6, maximum=1.0, singleStep=1e-5))
        self.addProperty('Major optimality tolerance', ikPlanner.defaultIkParameters.majorOptimalityTolerance, attributes=om.PropertyAttributes(decimals=6, minimum=1e-6, maximum=1.0, singleStep=1e-4))
        self.addProperty('RRT max edge length', ikPlanner.defaultIkParameters.rrtMaxEdgeLength, attributes=om.PropertyAttributes(decimals=2, minimum=1e-2, maximum=1.0, singleStep=1e-2))
        self.addProperty('RRT max vertices', ikPlanner.defaultIkParameters.rrtMaxNumVertices, attributes=om.PropertyAttributes(decimals=0, minimum=0.0, maximum=1e5, singleStep=1e1))
        self.addProperty('RRT no. of smoothing passes', ikPlanner.defaultIkParameters.rrtNSmoothingPasses, attributes=om.PropertyAttributes(decimals=0, minimum=0.0, maximum=1e2, singleStep=1e0))
        self.addProperty('RRT goal bias', ikPlanner.defaultIkParameters.rrtGoalBias, attributes=om.PropertyAttributes(decimals=2, minimum=0.0, maximum=1.0, singleStep=1e-2))
        self.addProperty('RRT hand', ikPlanner.defaultIkParameters.rrtHand, attributes=om.PropertyAttributes(enumNames=['left', 'right']))
        self.addProperty('Goal planning mode', 0, attributes=om.PropertyAttributes(enumNames=['fix end pose', 'fix goal joints']))
        #self.addProperty('Additional time samples', ikPlanner.additionalTimeSamples)

    def _onPropertyChanged(self, propertySet, propertyName):

        om.ObjectModelItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Use pointwise':
            self.ikPlanner.defaultIkParameters.usePointwise = self.getProperty(propertyName)

        if propertyName == 'Use collision':
            self.ikPlanner.defaultIkParameters.useCollision = self.getProperty(propertyName)
            if self.ikPlanner.defaultIkParameters.useCollision:
                self.setProperty('Use pointwise', False)
                self.setProperty('Add knots', 2)
                self.setProperty('Quasistatic shrink factor', 0.5)
                self.setProperty('Major iterations limit', 500)
                self.setProperty('Major optimality tolerance', 1e-3)
                self.setProperty('Major feasibility tolerance', 5e-5)
            else:
                self.setProperty('Add knots', 0)
                self.setProperty('Quasistatic shrink factor', 0.5)
                self.setProperty('Major iterations limit', 500)
                self.setProperty('Major optimality tolerance', 1e-4)
                self.setProperty('Major feasibility tolerance', 1e-6)

        if propertyName == 'Major iterations limit':
            self.ikPlanner.defaultIkParameters.majorIterationsLimit = self.getProperty(propertyName)

        if propertyName == 'Major feasibility tolerance':
            self.ikPlanner.defaultIkParameters.majorFeasibilityTolerance = self.getProperty(propertyName)

        if propertyName == 'Major optimality tolerance':
            self.ikPlanner.defaultIkParameters.majorOptimalityTolerance = self.getProperty(propertyName)

        if propertyName == 'RRT hand':
            if (self.getProperty(propertyName) == 0):
                self.ikPlanner.defaultIkParameters.rrtHand = "left"
            else:
                self.ikPlanner.defaultIkParameters.rrtHand = "right"

        if propertyName == 'RRT max edge length':
            self.ikPlanner.defaultIkParameters.rrtMaxEdgeLength = self.getProperty(propertyName)

        if propertyName == 'RRT max vertices':
            self.ikPlanner.defaultIkParameters.rrtMaxNumVertices = self.getProperty(propertyName)

        if propertyName == 'RRT no. of smoothing passes':
            self.ikPlanner.defaultIkParameters.rrtNSmoothingPasses = self.getProperty(propertyName)

        if propertyName == 'RRT goal bias':
            self.ikPlanner.defaultIkParameters.rrtGoalBias = self.getProperty(propertyName)

        if propertyName == 'Collision min distance':
            self.ikPlanner.defaultIkParameters.collisionMinDistance = self.getProperty(propertyName)

        if propertyName == 'Add knots':
            self.ikPlanner.defaultIkParameters.numberOfAddedKnots = self.getProperty(propertyName)

        elif propertyName == 'Use quasistatic constraint':
            self.ikPlanner.useQuasiStaticConstraint = self.getProperty(propertyName)

        elif propertyName == 'Quasistatic shrink factor':
            self.ikPlanner.defaultIkParameters.quasiStaticShrinkFactor = self.getProperty(propertyName)

        elif propertyName == 'Max joint degrees/s':
            self.ikPlanner.defaultIkParameters.maxDegreesPerSecond = self.getProperty(propertyName)

        elif propertyName == 'Additional time samples':
            self.ikPlanner.additionalTimeSamples = self.getProperty(propertyName)


class IKPlanner(object):
    def setPublisher(self, pub):
        self.plannerPub = pub

    def __init__(self, ikServer, robotModel, jointController, handModels):

        self.ikServer = ikServer
        self.defaultIkParameters = IkParameters()
        self.defaultIkParameters.setToDefaults()
        self.robotModel = robotModel
        self.jointController = jointController
        self.handModels = handModels
        self.plannerPub = None

        self.ikServer.handModels = self.handModels

        self.reachingSide = 'left'

        self.additionalTimeSamples = 0
        self.useQuasiStaticConstraint = True
        self.pushToMatlab = True
        # is this dodgy?
        self.ikConstraintEncoder = ikconstraintencoder.IKConstraintEncoder(self)

        # If the robot an arm on a fixed base, set true e.g. ABB or Kuka?
        self.fixedBaseArm = False
        self.robotNoFeet = False

        self.leftFootSupportEnabled  = True
        self.rightFootSupportEnabled = True
        self.leftHandSupportEnabled  = False
        self.rightHandSupportEnabled = False
        self.pelvisSupportEnabled  = False

        om.addToObjectModel(IkOptionsItem(ikServer, self), parentObj=om.getOrCreateContainer('planning'))

        self.jointGroups = drcargs.getDirectorConfig()['teleopJointGroups']
        
        if 'kneeJoints' in drcargs.getDirectorConfig():
            self.kneeJoints = drcargs.getDirectorConfig()['kneeJoints']

        # list of joints, otherwise []
        self.baseJoints      = self.getJointGroup('Base')
        self.backJoints      = self.getJointGroup('Back')
        self.neckJoints      = self.getJointGroup('Neck')
        self.leftArmJoints   = self.getJointGroup('Left Arm')
        self.rightArmJoints  = self.getJointGroup('Right Arm')
        self.leftLegJoints   = self.getJointGroup('Left Leg')
        self.rightLegJoints  = self.getJointGroup('Right Leg')

        if 'pelvisLink' in drcargs.getDirectorConfig():
            self.pelvisLink = drcargs.getDirectorConfig()['pelvisLink']
        
        if 'leftFootLink' in drcargs.getDirectorConfig():
            self.leftFootLink =  drcargs.getDirectorConfig()['leftFootLink']
            self.rightFootLink = drcargs.getDirectorConfig()['rightFootLink']
        else:
            # print "No foot links found in config, assuming fixedBaseArm=True"
            self.fixedBaseArm = True


    def getJointGroup(self, name):
        jointGroup = filter(lambda group: group['name'] == name, self.jointGroups)
        if (len(jointGroup) == 1):
            return jointGroup[0]['joints']
        else:
            return []

    def setIkParameters(self, ikParameterDict):
        originalIkParameterDict = {}
        if 'usePointwise' in ikParameterDict:
            originalIkParameterDict['usePointwise'] = self.ikServer.usePointwise
            self.ikServer.usePointwise = ikParameterDict['usePointwise']
        if 'maxDegreesPerSecond' in ikParameterDict:
            originalIkParameterDict['maxDegreesPerSecond'] = self.defaultIkParameters.maxDegreesPerSecond
            self.ikServer.maxDegreesPerSecond = ikParameterDict['maxDegreesPerSecond']
        if 'numberOfAddedKnots' in ikParameterDict:
            originalIkParameterDict['numberOfAddedKnots'] = self.ikServer.numberOfAddedKnots
            self.ikServer.numberOfAddedKnots = ikParameterDict['numberOfAddedKnots']
        if 'quasiStaticShrinkFactor' in ikParameterDict:
            originalIkParameterDict['quasiStaticShrinkFactor'] = ik.QuasiStaticConstraint.shrinkFactor
            ik.QuasiStaticConstraint.shrinkFactor = ikParameterDict['quasiStaticShrinkFactor']
        if 'fixInitialState' in ikParameterDict:
            originalIkParameterDict['fixInitialState'] = self.ikServer.fixInitialState
            self.ikServer.fixInitialState = ikParameterDict['fixInitialState']
        if 'leftFootSupportEnabled' in ikParameterDict:
            originalIkParameterDict['leftFootSupportEnabled'] = self.leftFootSupportEnabled
            self.leftFootSupportEnabled = ikParameterDict['leftFootSupportEnabled']
        if 'rightFootSupportEnabled' in ikParameterDict:
            originalIkParameterDict['rightFootSupportEnabled'] = self.rightFootSupportEnabled
            self.rightFootSupportEnabled = ikParameterDict['rightFootSupportEnabled']
        if 'pelvisSupportEnabled' in ikParameterDict:
            originalIkParameterDict['pelvisSupportEnabled'] = self.pelvisSupportEnabled
            self.pelvisSupportEnabled = ikParameterDict['pelvisSupportEnabled']

        return originalIkParameterDict

    def getHandModel(self, side=None):
        if self.fixedBaseArm:
            return self.handModels[0]
        else:
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
        p = ik.QuasiStaticConstraint(leftFootEnabled=self.leftFootSupportEnabled,
                                        rightFootEnabled=self.rightFootSupportEnabled,
                                        pelvisEnabled=self.pelvisSupportEnabled)
        p.leftFootLinkName = self.leftFootLink
        p.rightFootLinkName = self.rightFootLink
        return p


    def createFixedFootConstraints(self, startPoseName, **kwargs):

        constraints = []
        linknames = []
        if self.leftFootSupportEnabled:
            linknames.append(self.leftFootLink)
        if self.rightFootSupportEnabled:
            linknames.append(self.rightFootLink)
        for linkName in linknames:
            p = self.createFixedLinkConstraints(startPoseName, linkName, **kwargs)
            constraints.append(p)
        return constraints

    def createSixDofLinkConstraints(self, startPose, linkName, **kwargs):
        linkFrame = self.getLinkFrameAtPose(linkName, startPose)
        p = ik.PositionConstraint(linkName=linkName, referenceFrame=linkFrame, lowerBound=-0.0001*np.ones(3), upperBound=0.0001*np.ones(3),**kwargs)
        q = ik.QuatConstraint(linkName=linkName, quaternion=linkFrame, **kwargs)
        return [p, q]

    def createFixedLinkConstraints(self, startPose, linkName, **kwargs):
        p = ik.FixedLinkFromRobotPoseConstraint(linkName=linkName, poseName=startPose, **kwargs)
        return p


    def createPlanePositionConstraint(self, linkName, linkOffsetFrame, targetFrame, planeNormalAxis, bounds):

        p = ik.PositionConstraint()
        p.linkName = linkName
        p.pointInLink = np.array(linkOffsetFrame.GetPosition())
        p.referenceFrame = targetFrame

        p.lowerBound = np.tile(-np.inf, 3)
        p.upperBound = np.tile(np.inf, 3)

        p.lowerBound[planeNormalAxis] = bounds[0]
        p.upperBound[planeNormalAxis] = bounds[1]

        return p


    def createLinePositionConstraint(self, linkName, linkOffsetFrame, targetFrame, lineAxis, bounds, positionTolerance=0.0):

        p = ik.PositionConstraint()
        p.linkName = linkName
        p.pointInLink = np.array(linkOffsetFrame.GetPosition())
        p.referenceFrame = targetFrame

        p.lowerBound = np.tile(-positionTolerance, 3)
        p.upperBound = np.tile(positionTolerance, 3)

        p.lowerBound[lineAxis] = bounds[0]
        p.upperBound[lineAxis] = bounds[1]

        return p


    def createLinkGazeConstraint(self, startPose, linkName, gazeAxis):
        linkFrame = self.getLinkFrameAtPose(linkName, startPose)
        g = ik.WorldGazeDirConstraint()
        g.linkName = linkName
        g.targetFrame = linkFrame
        g.targetAxis = gazeAxis
        g.bodyAxis = gazeAxis
        g.coneThreshold = 0.0
        return g


    def createSlidingFootConstraints(self, startPose):

        constraints = []
        for linkName in [self.leftFootLink, self.rightFootLink]:

            linkFrame = self.getLinkFrameAtPose(linkName, startPose)

            p = ik.PositionConstraint(linkName=linkName, referenceFrame=linkFrame)
            p.lowerBound = [-np.inf, -np.inf, 0.0]
            p.upperBound = [np.inf, np.inf, 0.0]

            g = ik.WorldGazeDirConstraint()
            g.linkName = linkName
            g.targetFrame = linkFrame
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
        p.joints = self.kneeJoints
        p.jointsLowerBound = np.tile(bounds[0], 2)
        p.jointsUpperBound = np.tile(bounds[1], 2)
        return p


    # remove as not used and has magic numbers:
    #def createMovingKneePostureConstraint(self):
    #    return self.createKneePostureConstraint([0.0, 2.1])


    def createZMovingBasePostureConstraint(self, startPostureName):
        joints = ['base_x', 'base_y', 'base_roll', 'base_pitch', 'base_yaw']
        return self.createPostureConstraint(startPostureName, joints)


    def createXYZMovingBasePostureConstraint(self, startPostureName):
        joints = ['base_roll', 'base_pitch', 'base_yaw']
        return self.createPostureConstraint(startPostureName, joints)

    def createXYZYawMovingBasePostureConstraint(self, startPostureName):
        joints = ['base_roll', 'base_pitch']
        return self.createPostureConstraint(startPostureName, joints)


    def createMovingBasePostureConstraint(self, startPostureName):
        return self.createXYZMovingBasePostureConstraint(startPostureName)


    def createMovingBaseSafeLimitsConstraint(self):
        p = ik.PostureConstraint()
        p.joints = ['base_roll', 'base_pitch', 'base_yaw']
        p.jointsLowerBound = [-math.radians(5), -math.radians(5), -np.inf]
        p.jointsUpperBound = [math.radians(5), math.radians(15), np.inf]
        return p


    def createBaseGroundHeightConstraint(self, groundHeightZ, bounds):
        p = ik.PostureConstraint()
        p.joints = ['base_z']
        p.jointsLowerBound = [groundHeightZ + bounds[0]]
        p.jointsUpperBound = [groundHeightZ + bounds[1]]
        return p


    def createBaseZeroConstraint(self, footReferenceFrame):

        baseReferenceWorldPos = np.array(footReferenceFrame.GetPosition())
        baseReferenceWorldYaw = math.radians(footReferenceFrame.GetOrientation()[2])

        p = ik.PostureConstraint()
        p.joints = ['base_x', 'base_y', 'base_roll', 'base_pitch', 'base_yaw']
        p.jointsLowerBound = [baseReferenceWorldPos[0], baseReferenceWorldPos[1], 0.0, 0.0, baseReferenceWorldYaw]
        p.jointsUpperBound = list(p.jointsLowerBound)

        return p


    def createMovingBackPostureConstraint(self):
        p = ik.PostureConstraint()
        p.joints = self.backJoints

        p.jointsLowerBound = [-math.radians(15), -math.radians(5), -np.inf]
        p.jointsUpperBound = [math.radians(15), math.radians(25), np.inf]
        return p

    def createMovingBackLimitedPostureConstraint(self):
        p = ik.PostureConstraint()
        p.joints = self.backJoints

        p.jointsLowerBound = [-math.radians(5), -math.radians(5), -np.inf]
        p.jointsUpperBound = [math.radians(5), math.radians(5), np.inf]
        return p

    def createBackZeroPostureConstraint(self):
        p = ik.PostureConstraint()
        p.joints = ['back_bkx', 'back_bky', 'back_bkz']
        p.jointsLowerBound = np.zeros(3)
        p.jointsUpperBound = np.zeros(3)
        return p

    def createPositionOrientationGraspConstraints(self, side, targetFrame, graspToHandLinkFrame=None, positionTolerance=0.0, angleToleranceInDegrees=0.0):
        graspToHandLinkFrame = graspToHandLinkFrame or self.getPalmToHandLink(side)
        linkName = self.getHandLink(side)
        return self.createPositionOrientationConstraint(linkName, targetFrame, graspToHandLinkFrame)

    def createPositionOrientationConstraint(self, linkName, targetFrame, linkOffsetFrame, positionTolerance=0.0, angleToleranceInDegrees=0.0):

        targetFrame = targetFrame if isinstance(targetFrame, vtk.vtkTransform) else targetFrame.transform

        p = ik.PositionConstraint()
        p.linkName = linkName
        p.pointInLink = np.array(linkOffsetFrame.GetPosition())
        p.referenceFrame = targetFrame
        p.lowerBound = np.tile(-positionTolerance, 3)
        p.upperBound = np.tile(positionTolerance, 3)
        positionConstraint = p

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(linkOffsetFrame.GetLinearInverse())
        t.Concatenate(targetFrame)

        p = ik.QuatConstraint()
        p.linkName = linkName
        p.quaternion = t
        p.angleToleranceInDegrees = angleToleranceInDegrees
        orientationConstraint = p

        return positionConstraint, orientationConstraint


    def createAxisInPlaneConstraint(self, side, targetFrame, graspToHandLinkFrame):
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
        p.bodyNameB = self.getHandLink(side)
        p.frameInBodyB = graspToHandLinkFrame
        p.pointInBodyA = makeOffsetTransform([0.25, 0.0, 0.0])
        p.positionTarget = np.zeros(3)
        p.lowerBound = np.array([np.nan, np.nan, 0.0])
        p.upperBound = np.array([np.nan, np.nan, 0.0])
        rollConstraint1 = p

        p = ik.RelativePositionConstraint()
        p.bodyNameA = 'world'
        p.bodyNameB = self.getHandLink(side)
        p.frameInBodyB = graspToHandLinkFrame
        p.pointInBodyA = makeOffsetTransform([-0.25, 0.0, 0.0])
        p.positionTarget = np.zeros(3)
        p.lowerBound = np.array([np.nan, np.nan, 0.0])
        p.upperBound = np.array([np.nan, np.nan, 0.0])
        rollConstraint2 = p

        return rollConstraint1, rollConstraint2


    def createGraspOrbitConstraints(self, side, targetFrame, graspToHandLinkFrame=None):

        if graspToHandLinkFrame is None:
            graspToHandLinkFrame = self.getPalmToHandLink(side)

        p = ik.PositionConstraint()
        p.linkName = self.getHandLink(side)
        p.pointInLink = np.array(graspToHandLinkFrame.GetPosition())
        p.referenceFrame = targetFrame.transform
        p.lowerBound = [0.0, 0.0, 0.0] #np.zeros(3)
        p.upperBound = [0.07, 0.0, 0.0] #np.zeros(3)
        positionConstraint = p

        rollConstraint1, rollConstraint2 = self.createAxisInPlaneConstraint(side, targetFrame.transform, graspToHandLinkFrame)

        return positionConstraint, rollConstraint1, rollConstraint2


    def createGazeGraspConstraint(self, side, targetFrame, graspToHandLinkFrame=None, coneThresholdDegrees=0.0, targetAxis=[0.0, 1.0, 0.0], bodyAxis=[0.0, 1.0, 0.0]):

        targetFrame = targetFrame if isinstance(targetFrame, vtk.vtkTransform) else targetFrame.transform

        graspToHandLinkFrame = graspToHandLinkFrame or self.getPalmToHandLink(side)

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(graspToHandLinkFrame.GetLinearInverse())
        t.Concatenate(targetFrame)



#        g = ik.WorldGazeOrientConstraint()
#        g.linkName = self.getHandLink()
#        g.quaternion = t
#        g.axis = gazeAxis
#        g.coneThreshold = math.radians(0)
#        g.threshold = math.radians(180)

        g = ik.WorldGazeDirConstraint()
        g.linkName = self.getHandLink(side)
        g.targetFrame = t
        g.targetAxis = targetAxis
        g.bodyAxis = bodyAxis
        g.coneThreshold = math.radians(coneThresholdDegrees)
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


    def createExcludeReachTargetCollisionGroupConstraint(self, reachTargetName):
        p = ik.ExcludeCollisionGroupConstraint()
        p.tspan = [1.0, 1.0]
        p.excludedGroupName = reachTargetName
        excludeReachTargetCollisionGroupConstraint = p;

        return excludeReachTargetCollisionGroupConstraint

    def createActiveEndEffectorConstraint(self, endEffectorName, endEffectorPoint):
        p = ik.ActiveEndEffectorConstraint()
        p.endEffectorName = endEffectorName
        p.endEffectorPoint = endEffectorPoint
        excludeReachTargetCollisionGroupConstraint = p;

        return excludeReachTargetCollisionGroupConstraint


    def setArmLocked(self, side, isLocked):
        setattr(self.ikServer,side+"ArmLocked",isLocked)


    def setBaseLocked(self, isLocked):
        setattr(self.ikServer, "baseLocked", isLocked)


    def setBackLocked(self, isLocked):
        setattr(self.ikServer, "backLocked", isLocked)


    def createMovingBodyConstraints(self, startPoseName, lockBase=False, lockBack=False, lockLeftArm=False, lockRightArm=False):

        if (self.fixedBaseArm==False):

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
                #constraints.append(self.createMovingKneePostureConstraint())
                constraints.append(self.createMovingBasePostureConstraint(startPoseName))

            if lockLeftArm:
                constraints.append(self.createLockedLeftArmPostureConstraint(startPoseName))
            if lockRightArm:
                constraints.append(self.createLockedRightArmPostureConstraint(startPoseName))

        else: # Remove all except the fixed base constraint if you only have an arm:
            constraints = []
            constraints.append(self.createLockedBasePostureConstraint(startPoseName))

        return constraints


    def createMovingReachConstraints(self, startPoseName, lockBase=False, lockBack=False, lockArm=True, side=None):
        side = side or self.reachingSide
        lockLeftArm = lockArm and (side == 'right')
        lockRightArm = lockArm and (side == 'left')
        return self.createMovingBodyConstraints(startPoseName, lockBase, lockBack, lockLeftArm=lockLeftArm, lockRightArm=lockRightArm)


    def getLinkFrameAtPose(self, linkName, pose):
        if isinstance(pose, str):
            self.jointController.setPose(pose)
        else:
            self.jointController.setPose('user_pose', pose)
        return self.robotModel.getLinkFrame(linkName)


    def getRobotModelAtPose(self, pose):
        self.jointController.setPose('user_pose', pose)
        return self.robotModel

    def computeNominalPose(self, startPose, ikParameters=None):

        ikParameters = self.mergeWithDefaultIkParameters(ikParameters)

        nominalPoseName = 'q_nom'
        startPoseName = 'stand_start'
        self.addPose(startPose, startPoseName)

        constraints = []
        constraints.extend(self.createFixedFootConstraints(startPoseName))
        constraints.append(self.createMovingBaseSafeLimitsConstraint())
        constraints.append(self.createLockedLeftArmPostureConstraint(nominalPoseName))
        constraints.append(self.createLockedRightArmPostureConstraint(nominalPoseName))
        backJoints = self.backJoints
        constraints.append(self.createPostureConstraint(nominalPoseName, backJoints))

        endPose, info = self.ikServer.runIk(constraints, ikParameters, seedPostureName=startPoseName)
        return endPose, info


    def computeNominalPlan(self, startPose):

        endPose, info = self.computeNominalPose(startPose)
        print 'info:', info

        return self.computePostureGoal(startPose, endPose)


    def computeStandPose(self, startPose, ikParameters=None):

        ikParameters = self.mergeWithDefaultIkParameters(ikParameters)

        nominalPoseName = 'q_nom'
        startPoseName = 'stand_start'
        self.addPose(startPose, startPoseName)

        constraints = []
        constraints.extend(self.createFixedFootConstraints(startPoseName))
        constraints.append(self.createMovingBaseSafeLimitsConstraint())
        constraints.append(self.createLockedLeftArmPostureConstraint(startPoseName))
        constraints.append(self.createLockedRightArmPostureConstraint(startPoseName))
        backJoints = self.backJoints
        constraints.append(self.createPostureConstraint(nominalPoseName, backJoints))

        endPose, info = self.ikServer.runIk(constraints, ikParameters, seedPostureName=startPoseName)
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


    def createLockedNeckPostureConstraint(self, startPostureName):
        joints = []
        joints += self.neckJoints
        return self.createPostureConstraint(startPostureName, joints)


    def createLockedTorsoPostureConstraint(self, startPostureName):
        joints = []
        joints += self.baseJoints
        joints += self.backJoints
        joints += self.leftLegJoints
        joints += self.rightLegJoints
        return self.createPostureConstraint(startPostureName, joints)


    def createLockedBasePostureConstraint(self, startPostureName, lockLegs=True):
        joints = []
        joints += self.baseJoints
        if lockLegs:
            joints += self.rightLegJoints
            joints += self.leftLegJoints
        return self.createPostureConstraint(startPostureName, joints)


    def createLockedBackPostureConstraint(self, startPostureName):
        joints = []
        joints += self.backJoints
        return self.createPostureConstraint(startPostureName, joints)


    def createLockedRightArmPostureConstraint(self, startPostureName):
        joints = []
        joints += self.rightArmJoints
        return self.createPostureConstraint(startPostureName, joints)


    def createLockedLeftArmPostureConstraint(self, startPostureName):
        joints = []
        joints += self.leftArmJoints
        return self.createPostureConstraint(startPostureName, joints)


    def flipSide(self, side):
        assert side in ('left', 'right')
        return 'left' if side == 'right' else 'right'

    def createLockedArmsPostureConstraints(self, startPostureName):
        return [self.createLockedLeftArmPostureConstraint(startPostureName), self.createLockedRightArmPostureConstraint(startPostureName)]


    def createLockedArmPostureConstraint(self, startPostureName, side=None):

        if side is None:
            side = self.flipSide(self.reachingSide)

        if side == 'right':
            return self.createLockedRightArmPostureConstraint(startPostureName)
        else:
            return self.createLockedLeftArmPostureConstraint(startPostureName)


    def createJointPostureConstraintFromDatabase(self, postureGroup, postureName, side=None):
        postureJoints = self.getPostureJointsFromDatabase(postureGroup, postureName, side=side)
        p = ik.PostureConstraint()
        p.joints = postureJoints.keys()
        p.jointsLowerBound = postureJoints.values()
        p.jointsUpperBound = postureJoints.values()
        p.tspan = [1, 1]
        return p


    def getPostureJointsFromDatabase(self, postureGroup, postureName, side=None):
        return RobotPoseGUIWrapper.getPose(postureGroup, postureName, side=side)


    def getMergedPostureFromDatabase(self, startPose, poseGroup, poseName, side=None):
        postureJoints = self.getPostureJointsFromDatabase(poseGroup, poseName, side=side)
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



    def planEndEffectorDelta(self, startPose, side, DeltaVector, constraints=None, LocalOrWorldDelta='World'):

        self.reachingSide = side

        startPoseName = 'reach_start'
        self.addPose(startPose, startPoseName)

        if constraints is None:
            constraints = []
            constraints.extend(self.createMovingReachConstraints(startPoseName))


        graspToHandLinkFrame = self.getPalmToHandLink()

        self.jointController.setPose('start_pose', startPose)
        linkFrame = self.robotModel.getLinkFrame(self.getHandLink())

        if LocalOrWorldDelta == 'World':
            targetFrame = vtk.vtkTransform()
            targetFrame.PostMultiply()
            targetFrame.Concatenate(graspToHandLinkFrame)
            targetFrame.Concatenate(linkFrame)
            targetFrame.Translate(DeltaVector)
        else:
            targetFrame = vtk.vtkTransform()
            targetFrame.PostMultiply()
            targetFrame.Translate(DeltaVector)
            targetFrame.Concatenate(graspToHandLinkFrame)
            targetFrame.Concatenate(linkFrame)

        constraints.extend(self.createMoveOnLineConstraints(startPose, targetFrame, graspToHandLinkFrame))

        constraints[-2].tspan = [1.0, 1.0]
        constraints[-3].tspan = [1.0, 1.0]

        endPoseName = 'reach_end'
        constraintSet = ConstraintSet(self, constraints, endPoseName, startPoseName)
        return constraintSet


    def planGraspOrbitReachPlan(self, startPose, side, graspFrame, constraints=None, dist=0.0, lockBase=False, lockBack=False, lockArm=True):

        self.reachingSide = side

        startPoseName = 'reach_start'
        self.addPose(startPose, startPoseName)

        if constraints is None:
            constraints = self.createMovingReachConstraints(startPoseName, lockBase=lockBase, lockBack=lockBack, lockArm=lockArm)

        graspToHandLinkFrame = self.newPalmOffsetGraspToHandFrame(side, dist)

        constraints.extend(self.createGraspOrbitConstraints(side, graspFrame, graspToHandLinkFrame))

        constraints[-3].tspan = [1.0, 1.0]
        constraints[-2].tspan = [1.0, 1.0]
        constraints[-1].tspan = [1.0, 1.0]

        endPoseName = 'reach_end'
        constraintSet = ConstraintSet(self, constraints, endPoseName, startPoseName)
        return constraintSet


    def addPose(self, pose, poseName):
        self.jointController.addPose(poseName, pose)

        if self.pushToMatlab:
            self.ikServer.sendPoseToServer(pose, poseName)
        else:
            self.plannerPub.processAddPose(pose, poseName)


    def newPalmOffsetGraspToHandFrame(self, side, distance):
        t = vtk.vtkTransform()
        t.Translate(0.0, distance, 0.0)
        return self.newGraspToHandFrame(side, t)


    def computeFrameToHand(self, startPose, side, frameToWorld):
        '''
        Given a starting pose, a side, and a frame in world, returns the relative transform
        from the frame to the left/right hand link using forward kinematics.
        '''
        handToWorld = self.getLinkFrameAtPose(self.getHandLink(side), startPose)
        worldToHand = handToWorld.GetLinearInverse()
        return transformUtils.concatenateTransforms([frameToWorld, worldToHand])

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


    def planEndEffectorGoal(self, startPose, side, targetFrame, graspToHandLinkFrame=None, lockBase=False, lockBack=False, lockArm=True, constraints=None):

        startPoseName = 'reach_start'
        self.addPose(startPose, startPoseName)

        if constraints is None:
            constraints = self.createMovingReachConstraints(startPoseName, lockBase=lockBase, lockBack=lockBack, lockArm=lockArm, side=side)

        return self.newReachGoal(startPoseName, side, targetFrame, constraints, graspToHandLinkFrame)


    def newReachGoals(self, startPoseName, rightFrame, leftFrame, constraints, graspToHandLinkFrame=None, lockOrient=True):

        graspToHandLeftLinkFrame = self.newGraspToHandFrame('left')

        positionLeftConstraint, orientationLeftConstraint = self.createPositionOrientationGraspConstraints('left', leftFrame, graspToHandLeftLinkFrame, positionTolerance=0.0, angleToleranceInDegrees=0.0)
        positionLeftConstraint.tspan = [1.0, 1.0]
        orientationLeftConstraint.tspan = [1.0, 1.0]

        constraints.append(positionLeftConstraint)
        if lockOrient:
            constraints.append(orientationLeftConstraint)

        graspToHandRightLinkFrame = self.newGraspToHandFrame('right')

        positionRightConstraint, orientationRightConstraint = self.createPositionOrientationGraspConstraints('right', rightFrame, graspToHandRightLinkFrame, positionTolerance=0.0, angleToleranceInDegrees=0.0)
        positionRightConstraint.tspan = [1.0, 1.0]
        orientationRightConstraint.tspan = [1.0, 1.0]

        constraints.append(positionRightConstraint)
        if lockOrient:
            constraints.append(orientationRightConstraint)

        constraintSet = ConstraintSet(self, constraints, 'reach_end', startPoseName)

        return constraintSet


    def planEndEffectorGoals(self, startPose, rightFrame, leftFrame, constraints=None, lockBase=False, lockBack=False, lockArm=True):

        startPoseName = 'reach_start'
        self.addPose(startPose, startPoseName)

        if constraints is None:
            constraints = self.createMovingReachConstraints(startPoseName, lockBase=lockBase, lockBack=lockBack, lockArm=lockArm)

        return self.newReachGoals(startPoseName, rightFrame, leftFrame, constraints)

    def planEndEffectorGoalLine(self, startPose, rightGraspFrame, leftGraspFrame, constraints=None, lockBase=False, lockBack=False, lockArm=True,
                                angleToleranceInDegrees=0.0):

        startPoseName = 'reach_start'
        self.addPose(startPose, startPoseName)

        if constraints is None:
            constraints = self.createMovingReachConstraints(startPoseName, lockBase=lockBase, lockBack=lockBack, lockArm=lockArm)

        for hand in ['left', 'right']:
            graspToHandLinkFrame = self.newGraspToHandFrame(hand)
            posConstraint, quatConstraint = self.createSegmentConstraint(self.getHandLink(hand), rightGraspFrame,
                                                                         leftGraspFrame, graspToHandLinkFrame, angleToleranceInDegrees)
            posConstraint.tspan = [1.0, 1.0]
            quatConstraint.tspan = [1.0, 1.0]
            constraints.extend([posConstraint, quatConstraint])

        constraintSet = ConstraintSet(self, constraints, 'reach_end', startPoseName)
        return constraintSet

    def planAsymmetricGoal(self, startPose, rightGraspFrame, leftGraspFrame, constraints=None, lockBase=False, lockBack=False, lockArm=True):

        startPoseName = 'reach_start'
        self.addPose(startPose, startPoseName)

        if constraints is None:
            constraints = self.createMovingReachConstraints(startPoseName, lockBase=lockBase, lockBack=lockBack, lockArm=lockArm)

        graspToHandLinkFrame = self.newGraspToHandFrame('left')
        posConstraint, quatConstraint = self.createSegmentConstraint(self.getHandLink('left'), rightGraspFrame,
                                                                     leftGraspFrame, graspToHandLinkFrame)
        posConstraint.tspan = [1.0, 1.0]
        quatConstraint.tspan = [1.0, 1.0]
        constraints.extend([posConstraint, quatConstraint])

        linkName = self.getHandLink('right')
        graspToHandLinkFrame = self.newGraspToHandFrame('right')

        p = ik.PositionConstraint()
        p.linkName = linkName
        p.pointInLink = np.array(graspToHandLinkFrame.GetPosition())
        p.referenceFrame = rightGraspFrame.transform
        p.lowerBound = np.array([np.nan, np.nan, 0.0])
        p.upperBound = np.array([np.nan, np.nan, 0.0])
        positionConstraint = p

        graspToHandLinkFrame = self.newGraspToHandFrame('right')
        rollConstraint1, rollConstraint2 = self.createAxisInPlaneConstraintAsymmetric('right', rightGraspFrame, graspToHandLinkFrame)
        rollConstraint1.tspan = [1.0, 1.0]
        rollConstraint2.tspan = [1.0, 1.0]
        constraints.extend([positionConstraint, rollConstraint1, rollConstraint2])

        constraintSet = ConstraintSet(self, constraints, 'reach_end', startPoseName)
        return constraintSet

    def createSegmentConstraint(self, linkName, firstFrame, secondFrame, linkOffsetFrame, angleToleranceInDegrees=0.0):

        firstFrame = firstFrame if isinstance(firstFrame, vtk.vtkTransform) else firstFrame.transform
        secondFrame = secondFrame if isinstance(secondFrame, vtk.vtkTransform) else secondFrame.transform

        targetFrame = transformUtils.frameInterpolate(firstFrame, secondFrame, 0.5)
        vis.updateFrame(targetFrame, 'target frame', parent=None, visible=False, scale=0.2)

        distance = np.sqrt(vtk.vtkMath().Distance2BetweenPoints(firstFrame.GetPosition(), secondFrame.GetPosition()))
        distance = distance/2 * 0.9

        p = ik.PositionConstraint()
        p.linkName = linkName
        p.pointInLink = np.array(linkOffsetFrame.GetPosition())
        p.referenceFrame = targetFrame

        p.lowerBound = np.array([-distance, 0.0, 0.0])
        p.upperBound = np.array([distance, 0.0, 0.0])
        positionConstraint = p

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(linkOffsetFrame.GetLinearInverse())
        t.Concatenate(targetFrame)

        p = ik.QuatConstraint()
        p.linkName = linkName
        p.quaternion = t
        p.angleToleranceInDegrees = angleToleranceInDegrees
        orientationConstraint = p

        return positionConstraint, orientationConstraint


    def createAxisInPlaneConstraintAsymmetric(self, side, targetFrame, graspToHandLinkFrame):
        '''
        Constrain the ?? axis of targetFrame to be in the ?? plane of graspToHandLinkFrame in hand link.
        Returns two relative position constraints.
        '''

        targetFrame = targetFrame if isinstance(targetFrame, vtk.vtkTransform) else targetFrame.transform

        def makeOffsetTransform(offset):
            t = vtk.vtkTransform()
            t.PostMultiply()
            t.Translate(offset)
            t.Concatenate(targetFrame)
            return t

        p = ik.RelativePositionConstraint()
        p.bodyNameA = 'world'
        p.bodyNameB = self.getHandLink(side)
        p.frameInBodyB = graspToHandLinkFrame
        p.pointInBodyA = makeOffsetTransform([0.0, 0.0, 0.05])
        p.positionTarget = np.zeros(3)
        p.lowerBound = np.array([0.0, 0.0, np.nan])
        p.upperBound = np.array([0.0, 0.0, np.nan])
        rollConstraint1 = p

        p = ik.RelativePositionConstraint()
        p.bodyNameA = 'world'
        p.bodyNameB = self.getHandLink(side)
        p.frameInBodyB = graspToHandLinkFrame
        p.pointInBodyA = makeOffsetTransform([0.0, 0.0, -0.05])
        p.positionTarget = np.zeros(3)
        p.lowerBound = np.array([0.0, 0.0, np.nan])
        p.upperBound = np.array([0.0, 0.0, np.nan])
        rollConstraint2 = p

        return rollConstraint1, rollConstraint2



    def computeMultiPostureGoal(self, poses, feetOnGround=True, times=None, ikParameters=None):

        assert len(poses) >= 2

        constraints = []
        poseNames = []

        times = range(len(poses)) if times is None else times
        for i, pose in enumerate(poses):

            if isinstance(pose, str):
                poseName, pose = pose, self.jointController.getPose(pose)
            else:
                poseName = 'posture_goal_%d' % i

            self.addPose(pose, poseName)
            p = self.createPostureConstraint(poseName, robotstate.matchJoints('.*'))
            p.tspan = np.array([float(times[i]), float(times[i])])
            constraints.append(p)
            poseNames.append(poseName)

        if not self.fixedBaseArm and not self.robotNoFeet:
            if feetOnGround:
                constraints.extend(self.createFixedFootConstraints(poseNames[-1]))

        #if self.useQuasiStaticConstraint:
        #    constraints.append(self.createQuasiStaticConstraint())

        return self.runIkTraj(constraints[1:], poseNames[0], poseNames[-1], nominalPoseName=poseNames[0], ikParameters=ikParameters)


    def computePostureGoal(self, poseStart, poseEnd, feetOnGround=True, ikParameters=None):
        return self.computeMultiPostureGoal([poseStart, poseEnd], feetOnGround, ikParameters=ikParameters)


    def computeJointPostureGoal(self, startPose, postureJoints, ikParameters=None):

        startPoseName = 'posture_goal_start'
        self.addPose(startPose, startPoseName)

        p = ik.PostureConstraint()
        p.joints = postureJoints.keys()
        p.jointsLowerBound = postureJoints.values()
        p.jointsUpperBound = postureJoints.values()
        p.tspan = [1, 1]

        constraints = [p]
        constraints.extend(self.createFixedFootConstraints(startPoseName))
        #constraints.append(self.createMovingBasePostureConstraint(startPoseName))
        constraints.append(self.createQuasiStaticConstraint())

        constraintSet = ConstraintSet(self, constraints, 'posture_goal_end', startPoseName)
        endPose, info = constraintSet.runIk()
        return constraintSet.runIkTraj(ikParameters=ikParameters)

    def getManipPlanListener(self):
        responseChannel = 'CANDIDATE_MANIP_PLAN'
        responseMessageClass = lcmdrc.robot_plan_w_keyframes_t
        return lcmUtils.MessageResponseHelper(responseChannel, responseMessageClass)

    def getManipIKListener(self):
        responseChannel = 'CANDIDATE_MANIP_IKPLAN'
        responseMessageClass = lcmdrc.robot_plan_w_keyframes_t
        return lcmUtils.MessageResponseHelper(responseChannel, responseMessageClass)


    def onPostureGoalMessage(self, stateJointController, msg):

        goalPoseJoints = {}
        for name, position in zip(msg.joint_name, msg.joint_position):
            goalPoseJoints[name] = position

        feetOnGround = True
        for jointName in goalPoseJoints.keys():
            if 'leg' in jointName:
                feetOnGround = False


        startPose = np.array(stateJointController.q)
        goalMode = getIkOptions().getProperty('Goal planning mode')
        if goalMode == 0:
            endPose = self.mergePostures(startPose, goalPoseJoints)
            self.computePostureGoal(startPose, endPose, feetOnGround=feetOnGround)
        else:
            self.computeJointPostureGoal(startPose, goalPoseJoints)


    def addPostureGoalListener(self, stateJointController):
        lcmUtils.addSubscriber('POSTURE_GOAL', lcmdrc.joint_angles_t, functools.partial(self.onPostureGoalMessage, stateJointController))

    def mergeWithDefaultIkParameters(self, ikParameters):
        if ikParameters is None:
            ikParameters = IkParameters()
        ikParameters.fillInWith(self.defaultIkParameters)
        return ikParameters

    def runIkTraj(self, constraints, poseStart, poseEnd, nominalPoseName='q_nom', timeSamples=None, ikParameters=None):


        if (self.pushToMatlab is False):
            self.lastManipPlan, info = self.plannerPub.processTraj(constraints,endPoseName=poseEnd, nominalPoseName=nominalPoseName,seedPoseName=poseStart, additionalTimeSamples=self.additionalTimeSamples)
        else:
            ikParameters = self.mergeWithDefaultIkParameters(ikParameters)
            listener = self.getManipPlanListener()
            info = self.ikServer.runIkTraj(constraints, poseStart=poseStart, poseEnd=poseEnd, nominalPose=nominalPoseName, ikParameters=ikParameters, timeSamples=timeSamples, additionalTimeSamples=self.additionalTimeSamples)
            self.lastManipPlan = listener.waitForResponse(timeout=12000)
            listener.finish()

        print 'traj info:', info
        return self.lastManipPlan        


    def computePostureCost(self, pose):

        joints = robotstate.getDrakePoseJointNames()
        nominalPose = self.jointController.getPose('q_nom')
        assert len(nominalPose) == len(joints)
        cost = np.zeros(len(joints))

        cost[[joints.index(n) for n in self.backJoints]] = 100
        cost[[joints.index(n) for n in self.leftArmJoints]] = 1
        cost[[joints.index(n) for n in self.rightArmJoints]] = 1

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

        rpg.setDirectorConfigFile(drcargs.args().directorConfigFile)
        rpg.lcmWrapper = rpg.LCMWrapper()
        cls.main = rpg.MainWindow()
        mainWindow = [w for w in QtGui.QApplication.topLevelWidgets() if isinstance(w, PythonQt.dd.ddMainWindow)][0]
        cls.main.messageBoxWarning = functools.partial(QtGui.QMessageBox.warning, mainWindow)
        cls.main.messageBoxQuestion = functools.partial(QtGui.QMessageBox.question, mainWindow)
        cls.main.messageBoxInput = functools.partial(QtGui.QInputDialog.getText, mainWindow)
        cls.initialized = True

    @classmethod
    def initCaptureMethods(cls, robotStateJointController, teleopJointController):
        cls.init()
        panel = cls.main.capturePanel

        if panel.getCaptureMethod('Teleop state'):
            return

        def capturePose(jointController):
            return dict(zip(jointController.jointNames, jointController.q.tolist()))

        panel.captureMethods = []
        panel.addCaptureMethod('Robot state', functools.partial(capturePose, robotStateJointController))
        panel.addCaptureMethod('Teleop state', functools.partial(capturePose, teleopJointController))

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
                if 'leftFootLink' in drcargs.getDirectorConfig(): #if not self.fixedBaseArm:
                    joints = rpg.applyMirror(joints)

        return joints


# Keep this at the end of the file!
import plannerPublisher
