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


class IKPlanner(object):

    def __init__(self, ikServer, robotModel, jointController, sensorJointController, planPlaybackFunction, showPoseFunction, playbackRobotModel):

        self.ikServer = ikServer
        self.robotModel = robotModel
        self.jointController = jointController
        self.sensorJointController = sensorJointController

        self.planPlaybackFunction = planPlaybackFunction
        self.showPoseFunction = showPoseFunction
        self.playbackRobotModel = playbackRobotModel

        self.endPoses = []
        self.affordanceName = 'board'
        #self.affordanceName = 'drill'
        self.affordance = None
        self.handModels = []

        self.reachingSide = 'left'
        self.graspSample = 0
        self.additionalTimeSamples = 10
        self.useQuasiStaticConstraint = True

        #self.handToUtorso = [0.05, 0.6, 0.10]
        self.handToUtorso = [0.2, 0.7, 0.0]

        self.planFromCurrentRobotState = True

        self.tspanPreReach = [0.35, 0.35]
        self.tspanFull = [0.0, 1.0]
        self.tspanPreGrasp = [0.7, 0.7]
        self.tspanPreGraspToEnd = [0.7, 1.0]
        self.tspanStart = [0.0, 0.0]
        self.tspanEnd = [1.0, 1.0]


    def getHandModel(self):
        return self.handModels[0] if  self.reachingSide == 'left' else self.handModels[1]

    def getHandLink(self):
        return self.getHandModel().handLinkName

    def getPalmToHandLink(self):
        return self.getHandModel().palmToHandLink

    def getPalmPoint(self):
        return np.array(self.getPalmToHandLink().GetPosition())


    def computeGroundFrame(self, robotModel):
        '''
        Given a robol model, returns a vtkTransform at a position between
        the feet, on the ground, with z-axis up and x-axis aligned with the
        robot pelvis x-axis.
        '''
        t1 = robotModel.getLinkFrame('l_foot')
        t2 = robotModel.getLinkFrame('r_foot')
        pelvisT = robotModel.getLinkFrame('pelvis')

        xaxis = [1.0, 0.0, 0.0]
        pelvisT.TransformVector(xaxis, xaxis)
        xaxis = np.array(xaxis)
        zaxis = np.array([0.0, 0.0, 1.0])
        yaxis = np.cross(zaxis, xaxis)
        yaxis /= np.linalg.norm(yaxis)
        xaxis = np.cross(yaxis, zaxis)

        stancePosition = (np.array(t2.GetPosition()) + np.array(t1.GetPosition())) / 2.0

        footHeight = 0.0811

        t = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
        t.PostMultiply()
        t.Translate(stancePosition)
        t.Translate([0.0, 0.0, -footHeight])

        return t


    def randomAffordance(self, robotModel):
        aff = self.findAffordance()
        if aff:
            om.removeFromObjectModel(aff)
        self.spawnAffordance(robotModel, randomize=True)


    def spawnAffordance(self, robotModel, randomize=False):

        if randomize:

            position = [random.uniform(0.5, 0.8), random.uniform(-0.2, 0.2), random.uniform(0.5, 0.8)]
            rpy = [random.choice((random.uniform(-35, 35), random.uniform(70, 110))), random.uniform(-10, 10),  random.uniform(-5, 5)]
            zwidth = random.uniform(0.5, 1.0)

        else:

            position = [0.65, 0.0, 0.6]
            rpy = [25, 1, 0]
            zwidth = 24 * .0254

        xwidth = 3.75 * .0254
        ywidth = 1.75 * .0254
        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.computeGroundFrame(robotModel))
        xaxis = [1,0,0]
        yaxis = [0,1,0]
        zaxis = [0,0,1]
        for axis in (xaxis, yaxis, zaxis):
            t.TransformVector(axis, axis)

        affordance = segmentation.createBlockAffordance(t.GetPosition(), xaxis, yaxis, zaxis, xwidth, ywidth, zwidth, 'board', parent='affordances')
        affordance.setProperty('Color', QtGui.QColor(200, 150, 100))
        t = affordance.actor.GetUserTransform()
        affordanceFrame = vis.showFrame(t, 'board frame', parent=affordance, visible=False, scale=0.2)


    def updateHandModel(self):
        graspFrame = self.getAffordanceChild('desired grasp frame')
        handMesh = self.findAffordanceChild('desired grasp hand')
        if not handMesh:
            handMesh = self.getHandModel().newPolyData('desired grasp hand', self.robotModel.views[0], parent=self.findAffordance())
        handFrame = om.getObjectChildren(handMesh)[0]
        handFrame.copyFrame(graspFrame.transform)

    def findAffordance(self):
        self.affordance = om.findObjectByName(self.affordanceName)
        return self.affordance


    def findAffordanceChild(self, name):
        assert self.affordance
        for child in om.getObjectChildren(self.affordance):
            if child.getProperty('Name') == name:
                return child


    def getAffordanceChild(self, name):
        child = self.findAffordanceChild(name)
        if not child:
            raise Exception('Failed to locate affordance child: %s' % name)
        return child


    def getAffordanceFrame(self):
        self.findAffordance()
        assert self.affordance
        affordanceName = self.affordance.getProperty('Name')
        return self.getAffordanceChild('%s frame' % affordanceName)


    def computeGraspFrameSamples(self):

        if self.affordanceName == 'board':
            self.computeGraspFrameSamplesBoard()
        else:
            self.getAffordanceChild('sample grasp frame 0')


    def computeGraspFrameSamplesBoard(self):

        affordanceFrame = self.getAffordanceFrame()

        additionalOffset = 0.0
        yoffset = 0.5*self.affordance.params['ywidth'] + additionalOffset
        xoffset = 0.5*self.affordance.params['xwidth'] + additionalOffset

        frames = [
          [[0.0, yoffset, 0.0], [0.0, 90, 180.0]],
          [[0.0, yoffset, 0.0], [0.0, -90, 180.0]],

          [[0.0, -yoffset, 0.0], [0.0, 90, 0.0]],
          [[0.0, -yoffset, 0.0], [0.0, -90, 0.0]],

          [[xoffset, 0.0, 0.0], [-90, -90, 180.0]],
          [[xoffset, 0.0, 0.0], [90, 90, 180.0]],

          [[-xoffset, 0.0, 0.0], [90, -90, 180.0]],
          [[-xoffset, 0.0, 0.0], [-90, 90, 180.0]],
          ]

        for i, frame in enumerate(frames):
            pos, rpy = frame
            t = transformUtils.frameFromPositionAndRPY(pos, rpy)
            t.Concatenate(affordanceFrame.transform)
            name = 'sample grasp frame %d' % i
            om.removeFromObjectModel(self.findAffordanceChild(name))
            vis.showFrame(copyFrame(t), name, parent=self.affordance, visible=False, scale=0.2)


    def computeGraspFrame(self):
        frame = self.getAffordanceChild('sample grasp frame %d' % self.graspSample)
        name = 'grasp frame'
        om.removeFromObjectModel(self.findAffordanceChild(name))
        vis.showFrame(copyFrame(frame.transform), name, parent=self.affordance, visible=False, scale=0.2)


    def computeInitialState(self):

        if self.planFromCurrentRobotState:
            startPose = np.array(self.sensorJointController.q)
        else:
            startPose = np.array(self.jointController.getPose('q_nom'))

        self.ikServer.sendPoseToServer(startPose, 'q_start')
        self.jointController.addPose('q_start', startPose)


    def createPreReachConstraint(self):

        handToUtorso = np.array(self.handToUtorso)
        if self.reachingSide == 'right':
            handToUtorso[1] *= -1

        p = ik.RelativePositionConstraint()
        p.linkName = self.getHandLink()
        p.linkNameTarget = 'utorso'
        p.pointInLink = self.getPalmPoint()
        p.positionTarget = np.array(handToUtorso)
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
        return self.createKneePostureConstraint([0.4, np.inf])


    def createMovingBasePostureConstraint(self, startPostureName):
        joints = ['base_x', 'base_y', 'base_roll', 'base_pitch', 'base_yaw']
        return self.createPostureConstraint(startPostureName, joints)


    def createMovingBackPostureConstraint(self):
        p = ik.PostureConstraint()
        p.joints = ['back_bkx', 'back_bky', 'back_bkz']
        p.jointsLowerBound = [-0.2, -0.1, -0.6]
        p.jointsUpperBound = [0.2, 0.3, 0.6]
        return p


    def createReachConstraints(self, targetFrame, linkConstraintFrame=None, positionTolerance=0.0, angleToleranceInDegrees=0.0):

        if linkConstraintFrame is None:
            linkConstraintFrame = self.getPalmToHandLink()

        p = ik.PositionConstraint()
        p.linkName = self.getHandLink()
        p.pointInLink = np.array(linkConstraintFrame.GetPosition())
        p.referenceFrame = targetFrame.transform
        p.lowerBound = np.tile(-positionTolerance, 3)
        p.upperBound = np.tile(positionTolerance, 3)
        positionConstraint = p

        t = vtk.vtkTransform()
        t.Concatenate(targetFrame.transform)
        t.Concatenate(linkConstraintFrame.GetLinearInverse())

        p = ik.QuatConstraint()
        p.linkName = self.getHandLink()
        p.quaternion = t
        p.angleToleranceInDegrees = angleToleranceInDegrees
        orientationConstraint = p

        return positionConstraint, orientationConstraint


    def createSearchGraspConstraints(self):
        if self.affordanceName == 'board':
            return self.createSearchGraspConstraintsBoard()
        else:
            targetFrame = self.getAffordanceChild('grasp frame')
            return self.createReachConstraints(targetFrame, positionTolerance=0.0025, angleToleranceInDegrees=1.0)


    def createSearchGraspConstraintsBoard(self):

        targetFrame = self.getAffordanceChild('grasp frame')
        boardHalfLength = self.affordance.params['zwidth']/2.0 - 0.08

        graspPosition, graspOrientation = self.createReachConstraints(targetFrame, positionTolerance=0.0025, angleToleranceInDegrees=1.0)
        graspPosition.lowerBound = np.array([-boardHalfLength, 0.0, 0.0])
        graspPosition.upperBound = np.array([boardHalfLength, 0.0, 0.0])

        return graspPosition, graspOrientation


    def createRetractGraspConstraints(self):

        targetFrame = self.getAffordanceChild('desired grasp frame')

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(targetFrame.transform)
        t.Translate(0.0, 0.0, 0.25)
        retractFrame = vis.updateFrame(copyFrame(t), 'retract frame', scale=0.2, visible=False, parent=self.affordance)

        return self.createReachConstraints(retractFrame, positionTolerance=0.03, angleToleranceInDegrees=5.0)


    def createGraspConstraints(self):
        targetFrame = self.getAffordanceChild('desired grasp frame')
        return self.createReachConstraints(targetFrame, positionTolerance=0.005, angleToleranceInDegrees=3.0)


    def createPreGraspConstraints(self):
        targetFrame = self.getAffordanceChild('pre grasp frame')
        return self.createReachConstraints(targetFrame, positionTolerance=0.02, angleToleranceInDegrees=7.0)


    def createMovingReachConstraints(self, startPoseName, lockBack=False, lockBase=False, lockArm=True):
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
            pass

        if lockArm:
            constraints.append(self.createLockedArmPostureConstraint(startPoseName))

        return constraints


    def computeGraspEndPoseSearch(self):

        startPoseName = 'q_start'

        constraints = []
        constraints.extend(self.createSearchGraspConstraints())
        constraints.extend(self.createMovingReachConstraints(startPoseName))

        self.graspEndPose, self.graspEndPoseInfo = self.ikServer.runIk(constraints)

        self.ikServer.sendPoseToServer(self.graspEndPose, 'grasp_end_pose')
        self.jointController.setPose('grasp_end_pose', self.graspEndPose)

        print 'grasp end pose info:', self.graspEndPoseInfo


    def computeGraspEndPoseFrames(self):

        graspFrame = self.getAffordanceChild('grasp frame')
        affordanceFrame = self.getAffordanceFrame()

        self.jointController.setPose('grasp_end_pose', self.graspEndPose)
        handFrame = self.robotModel.getLinkFrame(self.getHandLink())

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(self.getPalmToHandLink())
        t.Concatenate(handFrame)
        graspEndPoseFrame = t
        vis.updateFrame(t, 'grasp frame (ik result with tolerance)', scale=0.2, visible=False, parent=self.affordance)

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(graspFrame.transform)
        t.Translate(np.array(graspEndPoseFrame.GetPosition()) - np.array(graspFrame.transform.GetPosition()))
        t.Concatenate(affordanceFrame.transform.GetLinearInverse())
        self.affordanceToGrasp = copyFrame(t)


        def updateAffordanceToGrasp(frame):
            affordanceFrame = self.getAffordanceFrame()
            t = vtk.vtkTransform()
            t.PostMultiply()
            t.Concatenate(frame.transform)
            t.Concatenate(affordanceFrame.transform.GetLinearInverse())
            self.affordanceToGrasp = copyFrame(t)
            self.updateHandModel()


        def updateGraspFrame(frame, create=False):

            graspFrame = self.findAffordanceChild('desired grasp frame')
            if not graspFrame and not create:
                frame.onTransformModifiedCallback = None
                return

            t = vtk.vtkTransform()
            t.PostMultiply()
            t.Concatenate(self.affordanceToGrasp)
            t.Concatenate(frame.transform)

            if graspFrame:
                graspFrame.onTransformModifiedCallback = None
            graspFrame = vis.updateFrame(copyFrame(t), 'desired grasp frame', scale=0.2, visible=False, parent=self.affordance)
            graspFrame.onTransformModifiedCallback = updateAffordanceToGrasp
            self.updateHandModel()
            return graspFrame

        self.lockAffordanceToHand = False

        def onRobotModelChanged(model):
            handFrame = self.playbackRobotModel.getLinkFrame(self.getHandLink())
            t = vtk.vtkTransform()
            t.PostMultiply()
            t.Concatenate(self.getPalmToHandLink())
            t.Concatenate(handFrame)
            palmFrame = vis.updateFrame(t, 'palm frame', scale=0.2, visible=False, parent=self.affordance)

            if self.lockAffordanceToHand:
                t = vtk.vtkTransform()
                t.PostMultiply()
                t.Concatenate(self.affordanceToGrasp.GetLinearInverse())
                t.Concatenate(palmFrame.transform)
                affordanceFrame = self.getAffordanceFrame()
                affordanceFrame.copyFrame(t)

        self.playbackRobotModel.modelChangedCallback = onRobotModelChanged

        graspFrame = updateGraspFrame(affordanceFrame, create=True)
        affordanceFrame.onTransformModifiedCallback = updateGraspFrame


    def computePreGraspFrame(self, preGraspDistance=0.20):

        graspFrame = self.getAffordanceChild('desired grasp frame')

        pos = [0.0, -preGraspDistance, 0.0]
        rpy = [0.0, 0.0, 0.0]
        preGraspToGrasp = transformUtils.frameFromPositionAndRPY(pos, rpy)

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(preGraspToGrasp)
        t.Concatenate(graspFrame.transform)
        vis.updateFrame(copyFrame(t), 'pre grasp frame', scale=0.2, visible=False, parent=self.affordance)


    def computeGraspEndPose(self):

        startPoseName = 'q_start'

        constraints = []
        constraints.extend(self.createMovingReachConstraints(startPoseName))
        constraints.extend(self.createGraspConstraints())

        self.graspEndPose, info = self.ikServer.runIk(constraints)

        self.ikServer.sendPoseToServer(self.graspEndPose, 'grasp_end_pose')
        self.jointController.setPose('grasp_end_pose', self.graspEndPose)

        print 'grasp end pose info:', info


    def commitState(self):
        poseTimes, poses = planplayback.PlanPlayback.getPlanPoses(self.lastManipPlan)
        self.sensorJointController.setPose('EST_ROBOT_STATE', poses[-1])


    def computePreGraspAdjustment(self):

        assert self.planFromCurrentRobotState
        startPose = np.array(self.sensorJointController.q)

        startPoseName = 'reach_start'
        self.jointController.addPose(startPoseName, startPose)
        self.computePostureGoal(startPoseName, 'pre_grasp_end_pose')


    def computeGraspReach(self):

        if self.planFromCurrentRobotState:
            startPose = np.array(self.sensorJointController.q)
        else:
            startPose = np.array(self.jointController.getPose('pre_grasp_end_pose'))

        startPoseName = 'reach_start'
        self.jointController.addPose(startPoseName, startPose)
        self.ikServer.sendPoseToServer(startPose, startPoseName)


        constraints = []
        constraints.extend(self.createGraspConstraints())
        constraints.append(self.createLockedTorsoPostureConstraint(startPoseName))
        constraints.append(self.createLockedArmPostureConstraint(startPoseName))

        endPose, info = self.ikServer.runIk(constraints, seedPostureName=startPoseName)

        print 'grasp reach info:', info

        self.jointController.addPose('reach_end', endPose)
        self.computePostureGoal(startPoseName, 'reach_end')


    def computeNominal(self):

        if self.planFromCurrentRobotState:
            startPose = np.array(self.sensorJointController.q)
        else:
            startPose = np.array(self.jointController.getPose('retract_end'))

        startPoseName = 'retract_start'
        self.jointController.addPose(startPoseName, startPose)
        self.computePostureGoal(startPoseName, 'q_nom')

        constraints = []
        constraints.extend(self.createFixedFootConstraints(startPoseName))
        constraints.append(self.createKneePostureConstraint([0.4, 0.4]))
        constraints.append(self.createMovingBasePostureConstraint(startPoseName))

        nominalJoints = []
        nominalJoints += robotstate.matchJoints('back')
        nominalJoints += robotstate.matchJoints('arm')
        constraints.append(self.createPostureConstraint('q_nom', nominalJoints))

        endPose, info = self.ikServer.runIk(constraints, seedPostureName=startPoseName)
        print 'nominal pose info:', info

        self.jointController.addPose('retract_end', endPose)
        self.computePostureGoal(startPoseName, 'retract_end')


    def computeRetractTraj(self, poseEnd='grasp_end_pose', timeSamples=None):

        if self.planFromCurrentRobotState:
            startPose = np.array(self.sensorJointController.q)
        else:
            startPose = np.array(self.jointController.getPose('grasp_end_pose'))

        startPoseName = 'retract_start'
        self.jointController.addPose(startPoseName, startPose)
        self.ikServer.sendPoseToServer(startPose, startPoseName)

        constraints = []
        constraints.extend(self.createMovingReachConstraints(startPoseName))

        graspPosition, graspOrientation = self.createRetractGraspConstraints()
        graspPosition.tspan = self.tspanEnd
        graspOrientation.tspan = self.tspanEnd

        constraints.extend([
            graspPosition,
            graspOrientation,
            ])


        endPose, info = self.ikServer.runIk(constraints, seedPostureName=startPoseName)
        print 'retract info:', info

        self.jointController.addPose('retract_end', endPose)
        self.computePostureGoal(startPoseName, 'retract_end')

        #self.runIkTraj(constraints, startPoseName, startPoseName, timeSamples)


    def computeArmExtend(self):

        if self.planFromCurrentRobotState:
            startPose = np.array(self.sensorJointController.q)
        else:
            startPose = np.array(self.jointController.getPose('grasp_end_pose'))

        startPoseName = 'retract_start'
        self.jointController.addPose(startPoseName, startPose)
        self.ikServer.sendPoseToServer(startPose, startPoseName)

        constraints = []
        constraints.extend(self.createFixedFootConstraints(startPoseName))
        constraints.append(self.createKneePostureConstraint([0.4, 0.4]))
        constraints.append(self.createMovingBasePostureConstraint(startPoseName))
        constraints.append(self.createLockedArmPostureConstraint(startPoseName))
        constraints.append(self.createPostureConstraint('q_nom', robotstate.matchJoints('back')))

        movingArmJoints = 'l_arm' if self.reachingSide == 'left' else 'r_arm'
        constraints.append(self.createPostureConstraint('q_zero', robotstate.matchJoints(movingArmJoints)))

        endPose, info = self.ikServer.runIk(constraints, seedPostureName=startPoseName)

        print 'retract info:', info

        self.jointController.addPose('retract_end', endPose)
        self.computePostureGoal(startPoseName, 'retract_end')


    def computeStand(self):

        if self.planFromCurrentRobotState:
            startPose = np.array(self.sensorJointController.q)
        else:
            startPose = np.array(self.jointController.getPose('grasp_end_pose'))

        startPoseName = 'stand_start'
        self.jointController.addPose(startPoseName, startPose)
        self.ikServer.sendPoseToServer(startPose, startPoseName)

        constraints = []
        constraints.extend(self.createFixedFootConstraints(startPoseName))
        constraints.append(self.createKneePostureConstraint([0.4, 0.4]))
        constraints.append(self.createMovingBasePostureConstraint(startPoseName))
        constraints.append(self.createLockedLeftArmPostureConstraint(startPoseName))
        constraints.append(self.createLockedRightArmPostureConstraint(startPoseName))
        constraints.append(self.createPostureConstraint(startPoseName, robotstate.matchJoints('back')))

        endPose, info = self.ikServer.runIk(constraints, seedPostureName=startPoseName)

        print 'stand info:', info

        self.jointController.addPose('stand_end', endPose)
        self.computePostureGoal(startPoseName, 'stand_end')

    def computePreGraspEndPose(self):

        constraints = []
        constraints.extend(self.createPreGraspConstraints())
        constraints.append(self.createLockedTorsoPostureConstraint('grasp_end_pose'))

        self.preGraspEndPose, self.preGraspEndPoseInfo = self.ikServer.runIk(constraints)

        self.ikServer.sendPoseToServer(self.preGraspEndPose, 'pre_grasp_end_pose')
        self.jointController.addPose('pre_grasp_end_pose', self.preGraspEndPose)

        print 'pre grasp end pose info:', self.preGraspEndPoseInfo


    def planReach(self):
        self.computePostureGoal('reach_start', 'reach_end', feetOnGround=True)


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


    def planPoseGoal(self, startPose, goalPoseJoints):

        startPoseName = 'posture_goal_start'
        self.jointController.addPose(startPoseName, startPose)
        self.ikServer.sendPoseToServer(startPose, startPoseName)

        endPose = np.array(startPose)
        for name, position in goalPoseJoints.iteritems():
            jointId = robotstate.getDrakePoseJointNames().index(name)
            endPose[jointId] = position

        endPoseName = 'posture_goal_end'
        self.jointController.addPose(endPoseName, endPose)
        self.ikServer.sendPoseToServer(endPose, endPoseName)

        postureConstraint = self.createPostureConstraint(endPoseName, robotstate.matchJoints('.*'))

        endPose, info = self.ikServer.runIk([postureConstraint], seedPostureName=endPoseName)

        print 'pose goal info:', info

        self.jointController.addPose(endPoseName, endPose)
        return self.computePostureGoal(startPoseName, endPoseName)


    def mergePostures(self, startPose, postureJoints):
        startPose = np.array(startPose)
        for name, position in postureJoints.iteritems():
            jointId = robotstate.getDrakePoseJointNames().index(name)
            startPose[jointId] = position
        return startPose


    def planPostureGoal(self, startPose, endPose, armJoints):

        startPoseName = 'posture_goal_start'

        self.jointController.addPose(startPoseName, startPose)
        self.ikServer.sendPoseToServer(startPose, startPoseName)

        constraints = []
        constraints.append(self.createLockedArmPostureConstraint(startPoseName))
        constraints.extend(self.createMovingReachConstraints(startPoseName))

        endPose = self.mergePostures(endPose, armJoints)
        endPoseName = 'posture_goal_end'
        self.jointController.addPose(endPoseName, endPose)
        self.ikServer.sendPoseToServer(endPose, endPoseName)

        constraints.append(self.createPostureConstraint(endPoseName, robotstate.matchJoints('.*')))
        constraints[-1].tspan = [1.0, 1.0]

        print 'computing traj for prereach crouch'

        return self.runIkTraj(constraints, startPoseName, endPoseName)


    def planEndEffectorGoal(self, startPose, side, graspFrame, lockTorso=False, planTraj=True):

        self.reachingSide = side

        startPoseName = 'reach_start'
        self.jointController.addPose(startPoseName, startPose)
        self.ikServer.sendPoseToServer(startPose, startPoseName)

        constraints = []

        if lockTorso:
            constraints.append(self.createLockedTorsoPostureConstraint(startPoseName))
            constraints.append(self.createLockedArmPostureConstraint(startPoseName))
        else:
            constraints.extend(self.createMovingReachConstraints(startPoseName))

        constraints.extend(self.createReachConstraints(graspFrame, positionTolerance=0.0, angleToleranceInDegrees=0.0))

        constraints[-2].tspan = [1.0, 1.0]
        constraints[-1].tspan = [1.0, 1.0]

        endPose, info = self.ikServer.runIk(constraints, seedPostureName=startPoseName)
        print 'end effector goal info:', info

        if not planTraj:
            return endPose

        endPoseName = 'reach_end'
        self.jointController.addPose(endPoseName, endPose)
        self.ikServer.sendPoseToServer(endPose, endPoseName)

        #return self.computePostureGoal(startPoseName, endPoseName)
        return self.runIkTraj(constraints, startPoseName, endPoseName)



    def newReachStartPosture(self):

        if self.planFromCurrentRobotState:
            startPose = np.array(self.sensorJointController.q)
        else:
            startPose = np.array(self.jointController.q)

        startPoseName = 'reach_start'
        self.jointController.addPose(startPoseName, startPose)
        self.ikServer.sendPoseToServer(startPose, startPoseName)

        return startPoseName


    def newReachConstraints(self, startPoseName, lockBack=False, lockBase=False, lockLeftArm=False, lockRightArm=False):

        constraints = []
        constraints.extend(self.createMovingReachConstraints(startPoseName, lockBack=lockBack, lockBase=lockBase, lockArm=False))

        if lockLeftArm:
            constraints.append(self.createLockedLeftArmPostureConstraint(startPoseName))
        if lockRightArm:
            constraints.append(self.createLockedRightArmPostureConstraint(startPoseName))

        return constraints


    def newReachGoal(self, targetFrame=None, linkConstraintFrame=None, lockTorso=True, lockOrient=True, constraints=None, runIk=True, showPoseFunction=None):

        startPoseName = self.newReachStartPosture()

        if linkConstraintFrame is None:
            linkConstraintFrame = self.getPalmToHandLink()

        if targetFrame is None:
            self.jointController.setPose(startPoseName)
            linkFrame = self.robotModel.getLinkFrame(self.getHandLink())
            t = vtk.vtkTransform()
            t.PostMultiply()
            t.Concatenate(linkConstraintFrame)
            t.Concatenate(linkFrame)
        else:
            t = targetFrame.transform

        frameName = 'reach goal %s' % self.reachingSide
        om.removeFromObjectModel(om.findObjectByName(frameName))
        targetFrame = vis.updateFrame(copyFrame(t), frameName, scale=0.2, parent=om.getOrCreateContainer('planning'))
        targetFrame.setProperty('Edit', True)

        if constraints is None:
            constraints = self.newReachConstraints(startPoseName, lockBack=lockTorso, lockBase=lockTorso,
                               lockLeftArm=self.reachingSide=='right', lockRightArm=self.reachingSide=='left')

        positionConstraint, orientationConstraint = self.createReachConstraints(targetFrame, linkConstraintFrame, positionTolerance=0.0, angleToleranceInDegrees=0.0)

        constraints.append(positionConstraint)
        if lockOrient:
            constraints.append(orientationConstraint)

        showPoseFunction = showPoseFunction or self.showPoseFunction

        def runIkCallback(frame):
            endPose, info = self.ikServer.runIk(constraints)
            self.jointController.addPose('reach_end', endPose)
            showPoseFunction(endPose)
            print 'info:', info

        if runIk:
            runIkCallback(targetFrame)
        targetFrame.onTransformModifiedCallback = runIkCallback


    def computePreGraspTraj(self):
        self.computeGraspTraj(poseStart='q_start', poseEnd='pre_grasp_end_pose', timeSamples=[0.0, 0.35, 0.7])

    def computeEndGraspTraj(self):
        self.computeGraspTraj(poseStart='pre_grasp_end_pose', poseEnd='grasp_end_pose', timeSamples=[0.7, 1.0])

    def getManipPlanListener(self):
        responseChannel = 'CANDIDATE_MANIP_PLAN'
        responseMessageClass = lcmdrc.robot_plan_w_keyframes_t
        return lcmUtils.MessageResponseHelper(responseChannel, responseMessageClass)


    def computeGraspTraj(self, poseStart='q_start', poseEnd='grasp_end_pose', timeSamples=None):

        constraints = []
        constraints.extend(self.createMovingReachConstraints(poseStart))

        movingBaseConstraint = constraints[-2]
        assert isinstance(movingBaseConstraint, ik.PostureConstraint)
        assert 'base_x' in movingBaseConstraint.joints
        movingBaseConstraint.tspan = [self.tspanStart[0], self.tspanPreGrasp[1]-0.1]

        preReachPosition = self.createPreReachConstraint()
        preReachPosition.tspan = self.tspanPreReach

        graspPosture = self.createLockedTorsoPostureConstraint('grasp_end_pose')
        graspPosture.tspan = self.tspanPreGraspToEnd

        preGraspPosition, preGraspOrientation = self.createPreGraspConstraints()
        preGraspPosition.tspan = self.tspanPreGrasp
        preGraspOrientation.tspan = self.tspanPreGrasp

        graspPosition, graspOrientation = self.createGraspConstraints()
        graspPosition.tspan = self.tspanEnd
        graspOrientation.tspan = self.tspanEnd

        constraints.extend([
            preReachPosition,
            graspPosture,
            preGraspPosition,
            preGraspOrientation,
            graspPosition,
            graspOrientation,
            ])

        if timeSamples is None:
            timeSamples=[0.0, 0.35, 0.7, 1.0]

        self.runIkTraj(constraints, poseStart, poseEnd, timeSamples)


    def computePostureGoal(self, poseStart, poseEnd, feetOnGround=False):

        if isinstance(poseStart, str):
            startPoseName = poseStart
        else:
            startPoseName = 'posture_goal_start'
            self.jointController.addPose(startPoseName, poseStart)

        if isinstance(poseEnd, str):
            endPoseName = poseEnd
        else:
            endPoseName = 'posture_goal_end'
            self.jointController.addPose(endPoseName, poseEnd)

        self.ikServer.sendPoseToServer(self.jointController.getPose(startPoseName), startPoseName)
        self.ikServer.sendPoseToServer(self.jointController.getPose(endPoseName), endPoseName)

        pStart = self.createPostureConstraint(startPoseName, robotstate.matchJoints('.*'))
        pStart.tspan = np.array([0.0, 0.0])

        pEnd = self.createPostureConstraint(endPoseName, robotstate.matchJoints('.*'))
        pEnd.tspan = np.array([1.0, 1.0])

        constraints = [pStart, pEnd]

        if feetOnGround:
            constraints.extend(self.createFixedFootConstraints(startPoseName))

        return self.runIkTraj(constraints, startPoseName, endPoseName)


    def runIkTraj(self, constraints, poseStart, poseEnd, timeSamples=None):

        listener = self.getManipPlanListener()

        info = self.ikServer.runIkTraj(constraints, poseStart=poseStart, poseEnd=poseEnd, timeSamples=timeSamples, additionalTimeSamples=self.additionalTimeSamples)
        print 'traj info:', info

        self.lastManipPlan = listener.waitForResponse()
        listener.finish()
        return self.lastManipPlan


    def playManipPlan(self):
        self.planPlaybackFunction([self.lastManipPlan])


    def showPreGraspEndPose(self):
        self.showPoseFunction(self.jointController.getPose('pre_grasp_end_pose'))


    def showGraspEndPose(self):
        self.showPoseFunction(self.jointController.getPose('grasp_end_pose'))


    def useGraspEndPoseOption(self, index):

        side, graspSample = self.endPoses[index][3]
        self.reachingSide = side
        self.graspSample = graspSample
        self.updateGraspEndPose()
        self.showGraspEndPose()


    def updateGraspEndPose(self, enableSearch=True):

        self.computeInitialState()
        self.findAffordance()

        if enableSearch:
            om.removeFromObjectModel(self.findAffordanceChild('desired grasp frame'))
            om.removeFromObjectModel(self.findAffordanceChild('desired grasp hand'))

        if not self.findAffordanceChild('desired grasp frame'):
            self.computeGraspFrameSamples()
            self.computeGraspFrame()
            self.computeGraspEndPoseSearch()
            self.computeGraspEndPoseFrames()
        else:
            self.computeGraspEndPose()

        self.computePreGraspFrame()
        self.computePreGraspEndPose()


    def computePostureCost(self, pose):

        joints = robotstate.getDrakePoseJointNames()
        nominalPose = self.jointController.getPose('q_nom')
        assert len(nominalPose) == len(joints)
        cost = np.zeros(len(joints))
        cost[[joints.index(n) for n in robotstate.matchJoints('back')]] = 100
        cost[[joints.index(n) for n in robotstate.matchJoints('arm')]] = 1

        return np.sum(np.abs(pose - nominalPose)*cost)


    def endPoseSearch(self):

        self.findAffordance()
        self.computeGraspFrameSamples()
        self.endPoses = []

        for side in ['left', 'right']:
        #for side in ['left']:

            sampleCount = 0

            while self.findAffordanceChild('sample grasp frame %d' % sampleCount):
                self.reachingSide = side
                self.graspSample = sampleCount
                sampleCount += 1

                self.updateGraspEndPose()

                if self.graspEndPoseInfo == 1 and self.preGraspEndPoseInfo == 1:
                    params = [self.reachingSide, self.graspSample]
                    score = self.computePostureCost(self.graspEndPose)
                    print 'score:', score
                    print 'params:', self.reachingSide, self.graspSample
                    self.endPoses.append((score, self.graspEndPose, self.preGraspEndPose, params))

        if not self.endPoses:
            print 'failed to find suitable grasp end pose'
            return 0

        self.endPoses.sort(key=lambda x: x[0])
        self.useGraspEndPoseOption(0)

        print '\n\nfound %d suitable end poses' % len(self.endPoses)
        return len(self.endPoses)


    def updateGraspPlan(self, enableSearch=True):

        if enableSearch:
            if self.endPoseSearch():
                self.computePreGraspTraj()
        else:
            self.updateGraspEndPose()
            self.computePreGraspTraj()
