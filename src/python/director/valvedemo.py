import os
import math
import functools
import numpy as np

from director import transformUtils
from director import objectmodel as om
from director import visualization as vis
from director import applogic as app
from director import ikconstraints
from director.ikparameters import IkParameters
from director import lcmUtils
from director import robotstate
from director import segmentation
from director.tasks.taskuserpanel import TaskUserPanel
from director.tasks.taskuserpanel import ImageBasedAffordanceFit
from director.uuidutil import newUUID
from director import ioUtils
from director import ikplanner
from director.debugVis import DebugData

import director.tasks.robottasks as rt

from PythonQt import QtCore


class ValvePlannerDemo(object):

    def __init__(self, robotModel, footstepPlanner, footstepsPanel, manipPlanner, ikPlanner,
                 lhandDriver, rhandDriver, sensorJointController):
        self.robotModel = robotModel
        self.footstepPlanner = footstepPlanner
        self.footstepsPanel = footstepsPanel
        self.manipPlanner = manipPlanner
        self.ikPlanner = ikPlanner
        self.lhandDriver = lhandDriver
        self.rhandDriver = rhandDriver
        self.sensorJointController = sensorJointController
        self.graspingObject = 'valve'
        self.turningMode = 0  # Simple Mode
        self.setGraspingHand('left')
        self.graspFrame = None
        self.stanceFrame = None

        # live operation flags
        self.planFromCurrentRobotState = False

        self.plans = []

        # IK server speed:
        self.speedLow = 10
        self.speedHigh = 60
        self.speedTurn = 100
        self.maxHandTranslationSpeed = 0.3

        # reach to center and back - for palm point
        self.graspFrameXYZ = [0.0, 0.0, -0.1]
        self.graspFrameRPY = [90, 0, 180]
        self.nominalPelvisXYZ = None
        self.useLargeValveDefaults()

        self.coaxialTol = 0.001
        self.coaxialGazeTol = 2
        self.shxMaxTorque = 40
        self.elxMaxTorque = 10
        self.elxLowerBoundDegrees = 30
        self.reachPoseName = None
        self.touchPose = None

        self.quasiStaticShrinkFactor = 0.5

        self.lockBack = True
        self.lockBase = True

        self.nominalPoseName = 'q_valve_nom'
        self.startPoseName = 'q_valve_start'



    def setGraspingHand(self, side):
        self.graspingHand = side
        self.setupStance()

    def useLargeValveDefaults(self):
        # distance above the valve axis for the hand center
        self.reachHeight = 0.0

        # distance away from valve for palm face on approach reach
        self.reachDepth = -0.1

        # distance away from valve for palm face on retraction
        self.retractDepth = -0.05

        # distance away from valve for palm face on approach reach
        self.touchDepth = 0.05
        self.openAmount = 20
        self.closedAmount = 20
        self.smallValve = False

    def useSmallValveDefaults(self):
        # distance above the valve axis for the hand center
        self.reachHeight = 0.0

        # distance away from valve for palm face on approach reach
        self.reachDepth = -0.05

        # distance away from valve for palm face on retraction
        self.retractDepth = -0.05

        # distance away from valve for palm face on approach reach
        self.touchDepth = 0.01
        self.openAmount = 0
        self.closedAmount = 50
        self.smallValve = True

    def setupStance(self):
        if (self.turningMode == 0):  # simple
            self.relativeStanceXYZInitial = [-0.9, -0.3, 0.0]
            self.relativeStanceRPYInitial = [0, 0, 0]
        else:  # human-like
            self.relativeStanceXYZInitial = [-0.5, 0.0, 0.0]
            self.relativeStanceRPYInitial = [0, 0, 0]

        self.relativeStanceXYZ = self.relativeStanceXYZInitial
        self.relativeStanceRPY = self.relativeStanceRPYInitial

        # mirror stance and rotation direction for right hand:
        if self.graspingHand == 'right':
            self.relativeStanceXYZ[1] = -self.relativeStanceXYZ[1]
            self.relativeStanceRPY[2] = -self.relativeStanceRPY[2]

    def addPlan(self, plan):
        self.plans.append(plan)

    def computeGroundFrame(self, robotModel):
        '''
        Given a robol model, returns a vtkTransform at a position between
        the feet, on the ground, with z-axis up and x-axis aligned with the
        robot pelvis x-axis.
        '''
        t1 = robotModel.getLinkFrame(self.ikPlanner.leftFootLink)
        t2 = robotModel.getLinkFrame(self.ikPlanner.rightFootLink)
        pelvisT = robotModel.getLinkFrame(self.ikPlanner.pelvisLink)

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

    def computeRobotStanceFrame(self, objectTransform, relativeStanceTransform):
        '''
        Given a robot model, determine the height of the ground using an XY and
        Yaw standoff, combined to determine the relative 6DOF standoff For a
        grasp or approach stance
        '''

        groundFrame = self.footstepPlanner.getFeetMidPoint(self.robotModel)
        groundHeight = groundFrame.GetPosition()[2]

        graspPosition = np.array(objectTransform.GetPosition())
        graspYAxis = [0.0, 1.0, 0.0]
        graspZAxis = [0.0, 0.0, 1.0]
        objectTransform.TransformVector(graspYAxis, graspYAxis)
        objectTransform.TransformVector(graspZAxis, graspZAxis)

        xaxis = graspYAxis
        zaxis = [0, 0, 1]
        yaxis = np.cross(zaxis, xaxis)
        yaxis /= np.linalg.norm(yaxis)
        xaxis = np.cross(yaxis, zaxis)

        graspGroundTransform = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
        graspGroundTransform.PostMultiply()
        graspGroundTransform.Translate(graspPosition[0], graspPosition[1], groundHeight)

        robotStance = transformUtils.copyFrame(relativeStanceTransform)
        robotStance.Concatenate(graspGroundTransform)

        return robotStance

    def updatePointcloudSnapshot(self):

        if (self.useLidar is True):
            return vis.updatePolyData(segmentation.getCurrentRevolutionData(),
                                      'pointcloud snapshot', parent='segmentation')
        else:
            return vis.updatePolyData(segmentation.getDisparityPointCloud(4),
                                      'pointcloud snapshot', parent='segmentation')

    # Valve Focused Functions ##################################################

    def onImageViewDoubleClick(self, displayPoint, modifiers, imageView):

        if modifiers != QtCore.Qt.ControlModifier:
            return

        imagePixel = imageView.getImagePixel(displayPoint)
        cameraPos, ray = imageView.getWorldPositionAndRay(imagePixel)

        polyData = self.updatePointcloudSnapshot().polyData
        pickPoint = segmentation.extractPointsAlongClickRay(cameraPos, ray,
                                                            polyData)

        om.removeFromObjectModel(om.findObjectByName('valve'))
        segmentation.segmentValveByBoundingBox(polyData, pickPoint)
        self.findAffordance()

    def getValveAffordance(self):
        return om.findObjectByName('valve')

    def computeStanceFrame(self, useIkTraj=False):
        objectTransform = transformUtils.copyFrame(self.computeGraspFrame().transform)
        if useIkTraj:
            startPose = self.getNominalPose()
            plan = self.planInsertTraj(self.speedLow, lockFeet=False, lockBase=False,
                                       resetPoses=True, startPose=startPose)
            stancePose = robotstate.convertStateMessageToDrakePose(plan.plan[0])
            stanceRobotModel = self.ikPlanner.getRobotModelAtPose(stancePose)
            self.nominalPelvisXYZ = stancePose[:3]
            robotStance = self.footstepPlanner.getFeetMidPoint(stanceRobotModel)
        else:
            robotStance = self.computeRobotStanceFrame(objectTransform,
                                                       self.computeRelativeStanceTransform())

        stanceFrame = vis.updateFrame(robotStance, 'valve grasp stance',
                                      parent=self.getValveAffordance(), visible=False, scale=0.2)
        stanceFrame.addToView(app.getDRCView())
        return stanceFrame

    def computeRelativeStanceTransform(self):
        return transformUtils.copyFrame(
            transformUtils.frameFromPositionAndRPY(self.relativeStanceXYZ, self.relativeStanceRPY))

    def computeRelativeGraspTransform(self):
        t = transformUtils.copyFrame(transformUtils.frameFromPositionAndRPY(self.graspFrameXYZ,
                                                                            self.graspFrameRPY))
        t.PostMultiply()
        t.RotateX(180)
        t.RotateY(-90)
        return t

    def computeGraspFrame(self):
        t = self.computeRelativeGraspTransform()
        t.Concatenate(self.getValveAffordance().getChildFrame().transform)
        graspFrame = vis.updateFrame(t, 'valve grasp frame',
                                     parent=self.getValveAffordance(),
                                     visible=False, scale=0.2)
        graspFrame.addToView(app.getDRCView())
        return graspFrame

    def spawnValveAffordance(self):
        radius = 0.20
        tubeRadius = 0.02
        position = [0, 0, 1.2]
        rpy = [0, 0, 0]
        t_feet_mid = self.footstepPlanner.getFeetMidPoint(self.robotModel)
        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t_grasp = self.computeRelativeGraspTransform()
        t_grasp.Concatenate(t)
        t_stance = self.computeRobotStanceFrame(t_grasp, self.computeRelativeStanceTransform())
        t_valve = t_stance.GetInverse()

        # This is necessary to get the inversion to actually happen. We don't know why.
        t_valve.GetMatrix()

        t_valve.Concatenate(t)
        t_valve.Concatenate(t_feet_mid)
        pose = transformUtils.poseFromTransform(t_valve)
        desc = dict(classname='CapsuleRingAffordanceItem', Name='valve', uuid=newUUID(), pose=pose,
                    Color=[0, 1, 0], Radius=float(radius), Segments=20)
        desc['Tube Radius'] = tubeRadius

        obj = segmentation.affordanceManager.newAffordanceFromDescription(desc)
        obj.params = dict(radius=radius)

    # End Valve Focused Functions ##############################################
    # Planning Functions #######################################################

    # These are operational conveniences:
    def planFootstepsToStance(self, **kwargs):
        f = transformUtils.copyFrame(self.computeStanceFrame(**kwargs).transform)
        self.footstepsPanel.onNewWalkingGoal(f)

    def planPreGrasp(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(
            startPose, 'General', 'arm up pregrasp', side=self.graspingHand)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def planNominal(self):
        startPose = self.getPlanningStartPose()
        endPose, info = self.ikPlanner.computeStandPose(startPose)
        endPose = self.ikPlanner.getMergedPostureFromDatabase(endPose, 'General', 'safe nominal')
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def createStaticTorqueConstraint(self):
        if self.graspingHand == 'left':
            elxJoint = 'l_arm_elx'
            shxJoint = 'l_arm_shx'
        else:
            elxJoint = 'r_arm_elx'
            shxJoint = 'r_arm_shx'
        constraint = ikconstraints.GravityCompensationTorqueConstraint()
        constraint.joints = [shxJoint, elxJoint]
        constraint.torquesLowerBound = -np.array([self.shxMaxTorque, self.elxMaxTorque])
        constraint.torquesUpperBound = np.array([self.shxMaxTorque, self.elxMaxTorque])
        return constraint

    def createElbowPostureConstraint(self):
        if self.graspingHand == 'left':
            elxJoint = 'l_arm_elx'
            elxLowerBound = np.radians(self.elxLowerBoundDegrees)
            elxUpperBound = 2.5
        else:
            elxJoint = 'r_arm_elx'
            elxLowerBound = -2.5
            elxUpperBound = np.radians(-self.elxLowerBoundDegrees)
        constraint = ikconstraints.PostureConstraint()
        constraint.joints = [elxJoint]
        constraint.jointsLowerBound = [elxLowerBound]
        constraint.jointsUpperBound = [elxUpperBound]
        return constraint

    def createWristAngleConstraint(self, wristAngleCW, planFromCurrentRobotState):
        if self.graspingHand == 'left':
            wristJoint = ['l_arm_lwy']
            wristJointLowerBound = [-np.radians(160) + wristAngleCW]
            wristJointUpperBound = [-np.radians(160) + wristAngleCW]
        else:
            wristJoint = ['r_arm_lwy']
            wristJointLowerBound = [np.radians(160) - wristAngleCW]
            wristJointUpperBound = [np.radians(160) - wristAngleCW]
        constraint = ikconstraints.PostureConstraint()
        constraint.joints = wristJoint
        constraint.jointsLowerBound = wristJointLowerBound
        constraint.jointsUpperBound = wristJointUpperBound
        if planFromCurrentRobotState:
            constraint.tspan = [1.0, 1.0]
        return constraint

    def createHandGazeConstraint(self):
        constraint = self.ikPlanner.createGazeGraspConstraint(
            self.graspingHand, self.computeGraspFrame(), coneThresholdDegrees=self.coaxialGazeTol)
        constraint.tspan = [0.0, 1.0]
        return constraint

    def createHandFixedOrientConstraint(self):
        if self.graspingHand == 'left':
            handLink = 'l_hand'
        else:
            handLink = 'r_hand'
        return ikconstraints.WorldFixedOrientConstraint(linkName=handLink)

    def createBackPostureConstraint(self):
        if self.lockBack:
            return self.ikPlanner.createLockedBackPostureConstraint(self.startPoseName)
        else:
            return self.ikPlanner.createMovingBackLimitedPostureConstraint()

    def createFootConstraints(self, lockFeet):
        if lockFeet:
            return self.ikPlanner.createFixedFootConstraints(self.startPoseName)
        else:
            constraints = []
            constraints.extend(self.ikPlanner.createSlidingFootConstraints(self.startPoseName))
            constraints.append(ikconstraints.WorldFixedBodyPoseConstraint(
                linkName='l_foot'))
            constraints.append(ikconstraints.WorldFixedBodyPoseConstraint(
                linkName='r_foot'))

            p = ikconstraints.RelativePositionConstraint()
            p.bodyNameA = self.ikPlanner.leftFootLink
            p.bodyNameB = self.ikPlanner.rightFootLink
            p.positionTarget = np.array([0, 0.3, 0])
            p.lowerBound = np.array([0, 0, -np.inf])
            p.upperBound = np.array([0, 0, np.inf])
            constraints.append(p)

            p = ikconstraints.RelativePositionConstraint()
            p.bodyNameA = self.ikPlanner.rightFootLink
            p.bodyNameB = self.ikPlanner.leftFootLink
            p.lowerBound = np.array([0, -np.inf, -np.inf])
            p.upperBound = np.array([0, np.inf, np.inf])
            constraints.append(p)

            return constraints

    def createHeadGazeConstraint(self):
        valveCenter = np.array(self.computeGraspFrame().transform.GetPosition())
        return ikconstraints.WorldGazeTargetConstraint(
            linkName='head', bodyPoint=np.zeros(3),
            worldPoint=valveCenter, coneThreshold=np.radians(20))

    def createBaseConstraints(self, resetBase, lockBase, lockFeet, yawDesired):
        constraints = []

        if lockBase is None:
            lockBase = self.lockBase

        if resetBase:
            poseName = self.nominalPoseName
        else:
            poseName = self.startPoseName

        if lockFeet:
            if lockBase:
                constraints.append(
                    self.ikPlanner.createLockedBasePostureConstraint(poseName, lockLegs=False))
            else:
                constraints.append(
                    self.ikPlanner.createXYZMovingBasePostureConstraint(poseName))
                constraints.append(
                    ikconstraints.WorldFixedBodyPoseConstraint(linkName='pelvis'))
        else:
            constraints.append(self.ikPlanner.createXYZYawMovingBasePostureConstraint(poseName))
            constraints.append(
                ikconstraints.WorldFixedBodyPoseConstraint(linkName='pelvis'))
            constraints.append(self.createHeadGazeConstraint())

            p = ikconstraints.PostureConstraint()
            p.joints = ['base_yaw']
            p.jointsLowerBound = [yawDesired - np.radians(20)]
            p.jointsUpperBound = [yawDesired + np.radians(20)]
            constraints.append(p)

        return constraints

    def getStartPoseName(self, planFromCurrentRobotState, retract, usePoses):
        if planFromCurrentRobotState:
            poseName = self.startPoseName
        else:
            if not usePoses or self.reachPoseName is None:
                poseName = self.nominalPoseName
            else:
                if retract:
                    poseName = self.touchPoseName
                else:
                    poseName = self.reachPoseName
        return poseName

    def getEndPoseName(self, retract, usePoses):
        if not usePoses or self.touchPose is None:
            return self.nominalPoseName
        else:
            if retract:
                return self.reachPoseName
            else:
                return self.touchPoseName

    def createHandPositionConstraint(self, radialTol, axialLowerBound, axialUpperBound, tspan):
        linkOffsetFrame = self.ikPlanner.getPalmToHandLink(self.graspingHand)
        constraint = ikconstraints.PositionConstraint()
        constraint.linkName = self.ikPlanner.getHandLink(self.graspingHand)
        constraint.pointInLink = np.array(linkOffsetFrame.GetPosition())
        constraint.referenceFrame = self.computeGraspFrame().transform
        constraint.lowerBound = np.array([-radialTol, axialLowerBound, -radialTol])
        constraint.upperBound = np.array([radialTol, axialUpperBound, radialTol])
        constraint.tspan = tspan
        return constraint

    def createAllHandPositionConstraints(self, radialTol, retract):
        constraints = []

        # Constrain hand to lie on the valve axis between the reach and touch
        # depths for the entire plan
        constraints.append(self.createHandPositionConstraint(radialTol, self.reachDepth,
                                                             self.touchDepth, [0.0, 1.0]))

        # Choose initial and final depths
        if retract:
            initialDepth = self.touchDepth
            finalDepth = self.reachDepth
        else:
            initialDepth = self.reachDepth
            finalDepth = self.touchDepth

        # Constrain initial position of the hand along the valve axis
        constraints.append(self.createHandPositionConstraint(np.inf, initialDepth, initialDepth,
                                                             [0.0, 0.0]))

        # Constrain final position of the hand along the valve axis
        constraints.append(self.createHandPositionConstraint(np.inf, finalDepth, finalDepth,
                                                             [1.0, 1.0]))
        return constraints

    def setReachAndTouchPoses(self, plan=None):
        if plan is None:
            self.reachPoseName = None
            self.touchPoseName = None
            self.reachPose = None
            self.touchPose = None
        else:
            self.reachPoseName = 'q_reach'
            self.touchPoseName = 'q_touch'
            self.reachPose = robotstate.convertStateMessageToDrakePose(plan.plan[0])
            self.touchPose = robotstate.convertStateMessageToDrakePose(plan.plan[-1])
            self.ikPlanner.addPose(self.reachPose, self.reachPoseName)
            self.ikPlanner.addPose(self.touchPose, self.touchPoseName)

    def planInsertTraj(self, speed, lockFeet=True, lockBase=None, resetBase=False, wristAngleCW=0,
                       startPose=None, verticalOffset=0.01, usePoses=False, resetPoses=True,
                       planFromCurrentRobotState=False, retract=False):

        ikParameters = IkParameters(usePointwise=False, maxDegreesPerSecond=speed,
                                    numberOfAddedKnots=1,
                                    quasiStaticShrinkFactor=self.quasiStaticShrinkFactor,
                                    fixInitialState=planFromCurrentRobotState)

        _, yaxis, _ = transformUtils.getAxesFromTransform(self.computeGraspFrame().transform)
        yawDesired = np.arctan2(yaxis[1], yaxis[0])

        if startPose is None:
            startPose = self.getPlanningStartPose()

        nominalPose = self.getNominalPose()

        self.ikPlanner.addPose(nominalPose, self.nominalPoseName)
        self.ikPlanner.addPose(startPose, self.startPoseName)

        self.ikPlanner.reachingSide = self.graspingHand

        constraints = []
        constraints.extend(self.createBaseConstraints(resetBase, lockBase, lockFeet, yawDesired))
        constraints.append(self.createBackPostureConstraint())
        constraints.append(self.ikPlanner.createQuasiStaticConstraint())
        constraints.extend(self.createFootConstraints(lockFeet))
        constraints.append(self.ikPlanner.createLockedArmPostureConstraint(self.startPoseName))
        constraints.append(self.ikPlanner.createKneePostureConstraint([0.7, 2.5]))
        constraints.append(self.createElbowPostureConstraint())
        constraints.append(self.createStaticTorqueConstraint())
        constraints.append(self.createHandGazeConstraint())
        constraints.append(self.createHandFixedOrientConstraint())
        constraints.append(self.createWristAngleConstraint(wristAngleCW,
                                                           planFromCurrentRobotState))
        constraints.extend(self.createAllHandPositionConstraints(self.coaxialTol, retract))

        if retract:
            startPoseName = self.getStartPoseName(planFromCurrentRobotState, True, usePoses)
            endPoseName = self.getEndPoseName(True, usePoses)
            endPose = self.ikPlanner.jointController.getPose(endPoseName)
            endPose = self.ikPlanner.mergePostures(endPose, robotstate.matchJoints('lwy'), startPose)
            endPoseName = 'q_retract'
            self.ikPlanner.addPose(endPose, endPoseName)
        else:
            startPoseName = self.getStartPoseName(planFromCurrentRobotState, retract, usePoses)
            endPoseName = self.getEndPoseName(retract, usePoses)

        plan = self.ikPlanner.runIkTraj(constraints, startPoseName, endPoseName, self.nominalPoseName, ikParameters=ikParameters)

        if resetPoses and not retract and max(plan.plan_info) <= 10 and min(plan.plan_info) >= 0:
            self.setReachAndTouchPoses(plan)

        return plan

    def planReach(self, verticalOffset=None, **kwargs):
        startPose = self.getPlanningStartPose()
        insert_plan = self.planInsertTraj(self.speedHigh, lockFeet=True, usePoses=True,
                                          resetPoses=True, **kwargs)
        info = max(insert_plan.plan_info)
        reachPose = robotstate.convertStateMessageToDrakePose(insert_plan.plan[0])
        ikParameters = IkParameters(maxDegreesPerSecond=2 * self.speedTurn,
                                    rescaleBodyNames=[self.ikPlanner.getHandLink(side=self.graspingHand)],
                                    rescaleBodyPts=list(self.ikPlanner.getPalmPoint(side=self.graspingHand)),
                                    maxBodyTranslationSpeed=self.maxHandTranslationSpeed)
        plan = self.ikPlanner.computePostureGoal(startPose, reachPose, ikParameters=ikParameters)
        plan.plan_info = [info] * len(plan.plan_info)
        lcmUtils.publish('CANDIDATE_MANIP_PLAN', plan)
        self.addPlan(plan)

    def planTouch(self, **kwargs):
        plan = self.planInsertTraj(self.speedLow, lockBase=True, lockFeet=True, usePoses=True,
                                   resetPoses=False, planFromCurrentRobotState=True, **kwargs)
        self.addPlan(plan)

    def planTurn(self, wristAngleCW=np.radians(320)):
        ikParameters = IkParameters(maxDegreesPerSecond=self.speedTurn)
        startPose = self.getPlanningStartPose()
        wristAngleCW = min(np.radians(320) - 0.01, max(-np.radians(160) + 0.01, wristAngleCW))
        if self.graspingHand == 'left':
            postureJoints = {'l_arm_lwy': -np.radians(160) + wristAngleCW}
        else:
            postureJoints = {'r_arm_lwy': np.radians(160) - wristAngleCW}

        endPose = self.ikPlanner.mergePostures(startPose, postureJoints)

        plan = self.ikPlanner.computePostureGoal(startPose, endPose, ikParameters=ikParameters)
        app.displaySnoptInfo(1)

        self.addPlan(plan)

    def planRetract(self, **kwargs):
        startPose = self.getPlanningStartPose()
        if self.graspingHand == 'left':
            jointId = robotstate.getDrakePoseJointNames().index('l_arm_lwy')
            wristAngleCW = np.radians(160) + startPose[jointId]
        else:
            jointId = robotstate.getDrakePoseJointNames().index('r_arm_lwy')
            wristAngleCW = np.radians(160) - startPose[jointId]
        plan = self.planInsertTraj(self.speedLow, retract=True, lockBase=True, lockFeet=True,
                                   usePoses=True, planFromCurrentRobotState=True, resetPoses=False,
                                   wristAngleCW=wristAngleCW, **kwargs)
        self.addPlan(plan)

    def getNominalPose(self):
        axes = transformUtils.getAxesFromTransform(self.computeGraspFrame().transform)
        yaxis = axes[1]
        yawDesired = np.arctan2(yaxis[1], yaxis[0])
        seedDistance = 1

        nominalPose = self.ikPlanner.jointController.getPose('q_nom')
        nominalPose[0] = (self.computeGraspFrame().transform.GetPosition()[0] -
                          seedDistance * yaxis[0])
        nominalPose[1] = (self.computeGraspFrame().transform.GetPosition()[1] -
                          seedDistance * yaxis[1])
        nominalPose[5] = yawDesired
        if self.scribeDirection == 1:  # Clockwise
            nominalPose = self.ikPlanner.getMergedPostureFromDatabase(nominalPose, 'valve', 'reach-nominal-cw', side=self.graspingHand)
        else:  # Counter-clockwise
            nominalPose = self.ikPlanner.getMergedPostureFromDatabase(nominalPose, 'valve', 'reach-nominal-ccw', side=self.graspingHand)
        return nominalPose


    def computeHumanTouchFrame(self, touchValve=True, reachAngle=-45):
        # 0 is top, 90 is left, -90 is right
        # positive turn is CCW
        if touchValve:
            faceDepth = 0.0
        else:
            faceDepth = self.reachDepth

        tDepth = transformUtils.frameFromPositionAndRPY([0, faceDepth, 0], [0, 0, 0])

        scribeRadius = om.findObjectByName('valve').params['radius']

        position = [0, scribeRadius * math.sin(math.radians(reachAngle)), scribeRadius * math.cos(math.radians(reachAngle))]
        # roll angle governs how much the palm points along towards the rotation axis
        # yaw ensures thumb faces the axis
        if (self.graspingObject is 'valve'):
            # valve, left and right
            rpy = [(-self.palmInAngle - 90), reachAngle, (90)]
        else:
            if (self.graspingHand is 'left'):  # lever left
                rpy = [-90, (180 + reachAngle), 90]
            else:
                rpy = [-90, reachAngle, 90]

        t = om.findObjectByName('valve').getChildFrame().transform

        t2 = transformUtils.frameFromPositionAndRPY(position, rpy)
        t3 = transformUtils.copyFrame(t)
        t3.PreMultiply()
        t3.Concatenate(t2)
        t3.Concatenate(tDepth)

        faceFrameDesired = vis.showFrame(t3, 'face frame desired', parent=om.findObjectByName('valve'), visible=False, scale=0.2)

        return faceFrameDesired

    def planHumanReach(self):
        faceFrameDesired = self.computeHumanTouchFrame(False, reachAngle=self.reachAngle)  # 0 = not in contact
        self.computeHumanTouchPlan(faceFrameDesired)

    def planHumanTouch(self):
        faceFrameDesired = self.computeHumanTouchFrame(True, reachAngle=self.reachAngle)
        self.computeHumanTouchPlan(faceFrameDesired)

    def planHumanRetract(self):
        faceFrameDesired = self.computeHumanTouchFrame(False, reachAngle=(self.reachAngle + self.turnAngle))  # 0 = not in contact
        self.computeHumanTouchPlan(faceFrameDesired)


    def computeHumanTouchPlan(self, faceFrameDesired):
        # new full 6 dof constraint:
        startPose = self.getPlanningStartPose()
        self.constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, faceFrameDesired, lockBase=self.lockBase, lockBack=self.lockBack)
        endPose, info = self.constraintSet.runIk()

        self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedLow
        self.planTrajectory()
        self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedHigh

    def planTrajectory(self):
        self.ikPlanner.ikServer.usePointwise = False
        plan = self.constraintSet.runIkTraj()
        self.addPlan(plan)

    def resetTurnPath(self):
        for obj in om.getObjects():
            if obj.getProperty('Name') == 'face frame desired':
                om.removeFromObjectModel(obj)
        for obj in om.getObjects():
            if obj.getProperty('Name') == 'face frame desired path':
                om.removeFromObjectModel(obj)

    def initConstraintSet(self, goalFrame):

        # create constraint set
        startPose = self.getPlanningStartPose()
        startPoseName = 'turn_plan_start'
        endPoseName = 'turn_plan_end'
        self.ikPlanner.addPose(startPose, startPoseName)
        self.ikPlanner.addPose(startPose, endPoseName)
        self.constraintSet = ikplanner.ConstraintSet(self.ikPlanner, [], startPoseName, endPoseName)
        self.constraintSet.endPose = startPose

        # add body constraints
        bodyConstraints = self.ikPlanner.createMovingBodyConstraints(startPoseName, lockBase=self.lockBase, lockBack=self.lockBack, lockLeftArm=self.graspingHand == 'right', lockRightArm=self.graspingHand == 'left')
        self.constraintSet.constraints.extend(bodyConstraints)

        # add gaze constraint - TODO: this gaze constraint shouldn't be necessary, fix
        self.graspToHandLinkFrame = self.ikPlanner.newGraspToHandFrame(self.graspingHand)
        gazeConstraint = self.ikPlanner.createGazeGraspConstraint(self.graspingHand, goalFrame, self.graspToHandLinkFrame, coneThresholdDegrees=5.0)
        self.constraintSet.constraints.insert(0, gazeConstraint)

    def appendPositionOrientationConstraintForTargetFrame(self, goalFrame, t):
        positionConstraint, orientationConstraint = self.ikPlanner.createPositionOrientationGraspConstraints(self.graspingHand, goalFrame, self.graspToHandLinkFrame)
        positionConstraint.tspan = [t, t]
        orientationConstraint.tspan = [t, t]
        self.constraintSet.constraints.append(positionConstraint)
        self.constraintSet.constraints.append(orientationConstraint)

    def planHumanValveTurn(self):
        initialReachAngle = self.reachAngle
        # 10deg per sample
        numberOfSamples = abs(int(round(self.turnAngle / 10.0)))
        degreeStep = float(self.turnAngle) / numberOfSamples

        facePath = []
        self.resetTurnPath()

        reachAngle = initialReachAngle
        faceFrameDesired = self.computeHumanTouchFrame(touchValve=True, reachAngle=reachAngle)
        self.initConstraintSet(faceFrameDesired)
        facePath.append(faceFrameDesired)

        for i in range(numberOfSamples):
            # reachAngle += self.scribeDirection*degreeStep
            reachAngle += degreeStep
            faceFrameDesired = self.computeHumanTouchFrame(touchValve=True, reachAngle=reachAngle)
            self.appendPositionOrientationConstraintForTargetFrame(faceFrameDesired, i + 1)
            facePath.append(faceFrameDesired)

        self.facePath = facePath
        self.drawFacePath(facePath)
        self.planTrajectory()

    def drawFacePath(self, facePath):

        valveTransform = om.findObjectByName('valve').getChildFrame().transform
        path = DebugData()
        for i in range(1, len(facePath)):
            end0 = transformUtils.copyFrame(facePath[i - 1].transform)
            end0.Concatenate(valveTransform.GetLinearInverse())
            end1 = transformUtils.copyFrame(facePath[i].transform)
            end1.Concatenate(valveTransform.GetLinearInverse())
            path.addLine(np.array(end0.GetPosition()), np.array(end1.GetPosition()), radius=0.025)

        pathMesh = path.getPolyData()
        pointerTipLinePath = vis.showPolyData(pathMesh, 'face frame desired path', color=[0.0, 0.3, 1.0], parent=om.findObjectByName('valve'), alpha=1.0)
        pointerTipLinePath.actor.SetUserTransform(om.findObjectByName('valve').getChildFrame().transform)


    # Glue Functions ###########################################################
    def moveRobotToGraspStanceFrame(self):
        self.teleportRobotToStanceFrame(om.findObjectByName('valve grasp stance').transform)

    def teleportRobotToStanceFrame(self, frame):
        self.sensorJointController.setPose('q_nom')
        stancePosition = frame.GetPosition()
        stanceOrientation = frame.GetOrientation()

        q = self.sensorJointController.q.copy()
        q[:2] = [stancePosition[0], stancePosition[1]]
        q[5] = math.radians(stanceOrientation[2])
        self.sensorJointController.setPose('EST_ROBOT_STATE', q)

    def getHandDriver(self, side):
        assert side in ('left', 'right')
        return self.lhandDriver if side == 'left' else self.rhandDriver

    def openHand(self, side):
        self.getHandDriver(side).sendCustom(0.0, 100.0, 100.0, 0)

    def openPinch(self, side):
        self.getHandDriver(side).sendCustom(20.0, 100.0, 100.0, 1)

    def closeHand(self, side):
        self.getHandDriver(side).sendCustom(100.0, 100.0, 100.0, 0)

    def sendNeckPitchLookDown(self):
        self.multisenseDriver.setNeckPitch(40)

    def sendNeckPitchLookForward(self):
        self.multisenseDriver.setNeckPitch(15)

    def waitForAtlasBehaviorAsync(self, behaviorName):
        assert behaviorName in list(self.atlasDriver.getBehaviorMap().values())
        while self.atlasDriver.getCurrentBehaviorName() != behaviorName:
            yield

    def printAsync(self, s):
        yield
        print(s)

    def optionalUserPrompt(self, message):
        if not self.optionalUserPromptEnabled:
            return

        yield
        result = input(message)
        if result != 'y':
            raise Exception('user abort.')

    def requiredUserPrompt(self, message):
        if not self.requiredUserPromptEnabled:
            return

        yield
        result = input(message)
        if result != 'y':
            raise Exception('user abort.')

    def delay(self, delayTimeInSeconds):
        yield
        t = SimpleTimer()
        while t.elapsed() < delayTimeInSeconds:
            yield

    def waitForCleanLidarSweepAsync(self):
        currentRevolution = self.multisenseDriver.displayedRevolution
        desiredRevolution = currentRevolution + 2
        while self.multisenseDriver.displayedRevolution < desiredRevolution:
            yield

    def getEstimatedRobotStatePose(self):
        return self.sensorJointController.getPose('EST_ROBOT_STATE')

    def getPlanningStartPose(self):
        if self.planFromCurrentRobotState:
            return self.sensorJointController.getPose('EST_ROBOT_STATE')
        else:
            if self.plans:
                return robotstate.convertStateMessageToDrakePose(
                    self.plans[-1].plan[-1])
            else:
                return self.getEstimatedRobotStatePose()

    def commitManipPlan(self):
        self.manipPlanner.commitManipPlan(self.plans[-1])


    def prepFromFile(self, moveRobot=True):
        filename = os.path.expanduser('~/drc-testing-data/valve/valve-pod-wall-ihmc.vtp')
        polyData = ioUtils.readPolyData(filename)
        vis.showPolyData(polyData, 'pointcloud snapshot')

        segmentation.segmentValveWallAuto(.20, mode='valve', removeGroundMethod=segmentation.removeGround)
        self.computeStanceFrame()

        if (moveRobot):
            self.moveRobotToGraspStanceFrame()


class ValveImageFitter(ImageBasedAffordanceFit):

    def __init__(self, valveDemo):
        ImageBasedAffordanceFit.__init__(self, numberOfPoints=2)
        self.valveDemo = valveDemo

    def onImageViewDoubleClick(self, displayPoint, modifiers, imageView):
        self.valveDemo.onImageViewDoubleClick(displayPoint, modifiers, imageView)

    def fit(self, polyData, points):
        om.removeFromObjectModel(om.findObjectByName('valve'))
        segmentation.segmentValveByRim(polyData, points[0], points[1])


class ValveTaskPanel(TaskUserPanel):

    def __init__(self, valveDemo):

        TaskUserPanel.__init__(self, windowTitle='Valve Task')

        self.valveDemo = valveDemo

        self.fitter = ValveImageFitter(self.valveDemo)
        self.initImageView(self.fitter.imageView)

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()

    def addButtons(self):

        self.addManualButton('Spawn Valve', self.onSpawnValveClicked)
        self.addManualSpacer()
        self.addManualButton('Footsteps', self.valveDemo.planFootstepsToStance)
        self.addManualButton('Footsteps (IK)',
                             functools.partial(self.valveDemo.planFootstepsToStance,
                                               useIkTraj=True))
        self.addManualSpacer()
        self.addManualButton('Raise arm', self.valveDemo.planPreGrasp)
        self.addManualButton('Set fingers', self.setFingers)
        self.addManualSpacer()
        self.addManualButton('Reach', self.reach)
        self.addManualButton('Touch', self.touch)
        self.addManualButton('Turn', self.turnValve)
        self.addManualButton('Retract', self.retract)
        self.addManualSpacer()
        self.addManualButton('Nominal', self.valveDemo.planNominal)
        self.addManualSpacer()
        self.addManualButton('Commit Manip', self.valveDemo.commitManipPlan)

    def onSpawnValveClicked(self):
        self.valveDemo.spawnValveAffordance()

    def setFingers(self):
        driver = self.valveDemo.getHandDriver(self.valveDemo.graspingHand)
        driver.sendClose(self.valveDemo.openAmount)

    def reach(self):
        self.valveDemo.setReachAndTouchPoses()
        self.valveDemo.planReach(wristAngleCW=self.initialWristAngleCW)

    def touch(self):
        self.valveDemo.planTouch(wristAngleCW=self.initialWristAngleCW)

    def turnValve(self):
        self.valveDemo.planTurn(wristAngleCW=self.finalWristAngleCW)

    def retract(self):
        self.valveDemo.planRetract()

    def addDefaultProperties(self):
        self.params.addProperty('Hand', ['left','right'].index(self.valveDemo.graspingHand), attributes=om.PropertyAttributes(enumNames=['Left',
                                                                                       'Right']))
        self.params.addProperty('Turn direction', 1,
                                attributes=om.PropertyAttributes(enumNames=['Clockwise',
                                                                            'Counter clockwise']))
        self.params.addProperty('Valve size', 0,
                                attributes=om.PropertyAttributes(enumNames=['Large', 'Small']))
        self.params.addProperty('Base', 0,
                                attributes=om.PropertyAttributes(enumNames=['Fixed', 'Free']))
        self.params.addProperty('Back', 1,
                                attributes=om.PropertyAttributes(enumNames=['Fixed', 'Free']))
        self.params.addProperty('Turning Mode', self.valveDemo.turningMode,
                                attributes=om.PropertyAttributes(enumNames=['Simple Turn', 'Human-like Turn']))
        self.params.addProperty('Reach Angle (Human)', -45,
                                attributes=om.PropertyAttributes(singleStep=5))
        self.params.addProperty('Turn Angle (Human)', -90,
                                attributes=om.PropertyAttributes(singleStep=5))
        self.params.addProperty('Palm In Angle (Human)', 30,
                                attributes=om.PropertyAttributes(singleStep=5))
        self._syncProperties()

    def onPropertyChanged(self, propertySet, propertyName):
        self._syncProperties()
        self.taskTree.removeAllTasks()
        self.addTasks()


    def _syncProperties(self):

        self.valveDemo.planFromCurrentRobotState = True
        self.valveDemo.turningMode = self.params.getProperty('Turning Mode')
        self.valveDemo.setGraspingHand(self.params.getPropertyEnumValue('Hand').lower())

        if self.params.getPropertyEnumValue('Turn direction') == 'Clockwise':
            self.valveDemo.scribeDirection = 1
            self.initialWristAngleCW = 0
            self.finalWristAngleCW = np.radians(320)
        else:
            self.valveDemo.scribeDirection = -1
            self.initialWristAngleCW = np.radians(320)
            self.finalWristAngleCW = 0

        if self.params.getPropertyEnumValue('Valve size') == 'Large':
            self.valveDemo.useLargeValveDefaults()
        else:
            self.valveDemo.useSmallValveDefaults()

        if self.params.getPropertyEnumValue('Base') == 'Fixed':
            self.valveDemo.lockBase = True
        else:
            self.valveDemo.lockBase = False

        if self.params.getPropertyEnumValue('Back') == 'Fixed':
            self.valveDemo.lockBack = True
        else:
            self.valveDemo.lockBack = False

        # Settings for Human Like Turning:
        self.valveDemo.reachAngle = self.params.getProperty('Reach Angle (Human)')
        self.valveDemo.turnAngle = self.params.getProperty('Turn Angle (Human)')
        # how much should the palm face the valve axis - 0 not at all, 90 entirely
        self.valveDemo.palmInAngle = self.params.getProperty('Palm In Angle (Human)')

    def addTasks(self):
        self.taskTree.removeAllTasks()

        if self.valveDemo.turningMode == 0:
            self.addSimpleTurnTasks()
        elif self.valveDemo.turningMode == 1:
            self.addHumanTurnTasks()
        else:
            return

    def addSimpleTurnTasks(self):

        # some helpers
        def addTask(task, parent=None):
            self.taskTree.onAddTask(task, copy=False, parent=parent)

        def addFunc(func, name, parent=None):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)

        def addManipulation(func, name, parent=None):
            group = self.taskTree.addGroup(name, parent=parent)
            addFunc(func, name='plan motion', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'),
                    parent=group)
            addTask(rt.UserPromptTask(name='Confirm execution has finished', message='Continue when plan finishes.'),
                    parent=group)

        def addLargeValveTurn(parent=None):
            group = self.taskTree.addGroup('Valve Turn', parent=parent)

            # valve manip actions
            addManipulation(functools.partial(v.planReach, wristAngleCW=self.initialWristAngleCW),
                            name='Reach to valve', parent=group)
            addManipulation(functools.partial(v.planTouch, wristAngleCW=self.initialWristAngleCW),
                            name='Insert hand', parent=group)
            addManipulation(functools.partial(v.planTurn, wristAngleCW=self.finalWristAngleCW),
                            name='Turn valve', parent=group)
            addManipulation(v.planRetract, name='Retract hand', parent=group)

        def addSmallValveTurn(parent=None):
            group = self.taskTree.addGroup('Valve Turn', parent=parent)
            side = 'Right' if v.graspingHand == 'right' else 'Left'

            addManipulation(functools.partial(v.planReach, wristAngleCW=self.initialWristAngleCW),
                            name='Reach to valve', parent=group)
            addManipulation(functools.partial(v.planTouch, wristAngleCW=self.initialWristAngleCW),
                            name='Insert hand', parent=group)
            addTask(rt.CloseHand(name='grasp valve', side=side, mode='Basic',
                                 amount=self.valveDemo.closedAmount),
                    parent=group)
            addManipulation(functools.partial(v.planTurn, wristAngleCW=self.finalWristAngleCW),
                            name='plan turn valve', parent=group)
            addTask(rt.CloseHand(name='release valve', side=side, mode='Basic',
                                 amount=self.valveDemo.openAmount),
                    parent=group)
            addManipulation(v.planRetract, name='plan retract', parent=group)

        v = self.valveDemo

        side = self.params.getPropertyEnumValue('Hand')

        ###############
        # add the tasks

        # prep
        prep = self.taskTree.addGroup('Preparation')
        addTask(rt.CloseHand(name='close left hand', side='Left'), parent=prep)
        addTask(rt.CloseHand(name='close right hand', side='Right'), parent=prep)

        # fit
        fit = self.taskTree.addGroup('Fitting')
        addTask(rt.UserPromptTask(name='fit valve',
                                  message='Please fit and approve valve affordance.'), parent=fit)
        addTask(rt.FindAffordance(name='check valve affordance', affordanceName='valve'),
                parent=fit)

        # walk
        walk = self.taskTree.addGroup('Approach')
        addFunc(v.planFootstepsToStance, 'plan walk to valve', parent=walk)
        addTask(rt.UserPromptTask(name='approve footsteps',
                                  message='Please approve footstep plan.'), parent=walk)
        addTask(rt.CommitFootstepPlan(name='walk to valve',
                                      planName='valve grasp stance footstep plan'), parent=walk)
        addTask(rt.SetNeckPitch(name='set neck position', angle=35), parent=walk)
        addTask(rt.WaitForWalkExecution(name='wait for walking'), parent=walk)

        # refit
        refit = self.taskTree.addGroup('Re-fitting')
        addTask(rt.UserPromptTask(name='fit valve',
                                  message='Please fit and approve valve affordance.'),
                parent=refit)

        # set fingers
        addTask(rt.CloseHand(name='set finger positions', side=side, mode='Basic',
                             amount=self.valveDemo.openAmount), parent=refit)

        # add valve turns
        if v.smallValve:
            for i in range(0, 2):
                addSmallValveTurn()

        else:
            for i in range(0, 2):
                addLargeValveTurn()


        # go to finishing posture
        prep = self.taskTree.addGroup('Prep for walking')

        addTask(rt.CloseHand(name='close left hand', side='Left'), parent=prep)
        addTask(rt.CloseHand(name='close right hand', side='Right'), parent=prep)
        addTask(rt.PlanPostureGoal(name='plan walk posture', postureGroup='General',
                                   postureName='safe nominal', side='Default'), parent=prep)
        addTask(rt.CommitManipulationPlan(name='execute manip plan',
                                          planName='safe nominal posture plan'), parent=prep)
        addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'), parent=prep)



    def addHumanTurnTasks(self):

        # some helpers
        def addTask(task, parent=None):
            self.taskTree.onAddTask(task, copy=False, parent=parent)

        def addFunc(func, name, parent=None):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)

        def addManipulation(func, name, parent=None):
            group = self.taskTree.addGroup(name, parent=parent)
            addFunc(func, name='plan motion', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'),
                    parent=group)
            addTask(rt.UserPromptTask(name='Confirm execution has finished', message='Continue when plan finishes.'),
                    parent=group)

        v = self.valveDemo

        side = self.params.getPropertyEnumValue('Hand')

        ###############
        # add the tasks

        # prep
        prep = self.taskTree.addGroup('Preparation')
        addTask(rt.CloseHand(name='close left hand', side='Left'), parent=prep)
        addTask(rt.CloseHand(name='close right hand', side='Right'), parent=prep)
        addFunc(v.prepFromFile, 'prep from file', parent=prep)

        # fit
        fit = self.taskTree.addGroup('Fitting')
        addTask(rt.UserPromptTask(name='fit valve',
                                  message='Please fit and approve valve affordance.'), parent=fit)
        addTask(rt.FindAffordance(name='check valve affordance', affordanceName='valve'),
                parent=fit)

        # walk
        walk = self.taskTree.addGroup('Approach')
        addFunc(v.planFootstepsToStance, 'plan walk to valve', parent=walk)
        addTask(rt.UserPromptTask(name='approve footsteps',
                                  message='Please approve footstep plan.'), parent=walk)
        addTask(rt.CommitFootstepPlan(name='walk to valve',
                                      planName='valve grasp stance footstep plan'), parent=walk)
        addTask(rt.SetNeckPitch(name='set neck position', angle=35), parent=walk)
        addTask(rt.WaitForWalkExecution(name='wait for walking'), parent=walk)

        # refit
        refit = self.taskTree.addGroup('Re-fitting')
        addTask(rt.UserPromptTask(name='fit valve',
                                  message='Please fit and approve valve affordance.'),
                parent=refit)

        # set fingers
        addTask(rt.CloseHand(name='set finger positions', side=side, mode='Basic',
                             amount=self.valveDemo.openAmount), parent=refit)

        # raise the arm up
        turning = self.taskTree.addGroup('Turning')

        preturn = self.taskTree.addGroup('Pre-grasp', parent=turning)
        addTask(rt.PlanPostureGoal(name='plan arm up pregrasp', postureGroup='General',
                                   postureName='arm up pregrasp', side=side), parent=preturn)
        addTask(rt.CommitManipulationPlan(name='execute manip plan',
                                          planName='arm up pregrasp posture plan'), parent=preturn)
        addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'), parent=preturn)


        addManipulation(functools.partial(v.planHumanReach),
                            name='Reach to valve', parent=turning)
        addManipulation(functools.partial(v.planHumanTouch),
                            name='Touch valve', parent=turning)
        addManipulation(functools.partial(v.planHumanValveTurn),
                            name='Turn valve', parent=turning)
        addManipulation(functools.partial(v.planHumanRetract),
                            name='Retract from valve', parent=turning)

        # go to finishing posture
        prep = self.taskTree.addGroup('Prep for walking')

        addTask(rt.CloseHand(name='close left hand', side='Left'), parent=prep)
        addTask(rt.CloseHand(name='close right hand', side='Right'), parent=prep)
        addTask(rt.PlanPostureGoal(name='plan walk posture', postureGroup='General',
                                   postureName='safe nominal', side='Default'), parent=prep)
        addTask(rt.CommitManipulationPlan(name='execute manip plan',
                                          planName='safe nominal posture plan'), parent=prep)
        addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'), parent=prep)
