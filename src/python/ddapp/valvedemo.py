import vtkAll as vtk
import math
import functools
import numpy as np

from ddapp import transformUtils
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp import applogic as app
from ddapp import ik
from ddapp import robotstate
from ddapp import segmentation
from ddapp.tasks.taskuserpanel import TaskUserPanel
from ddapp.tasks.taskuserpanel import ImageBasedAffordanceFit

import ddapp.tasks.robottasks as rt

from PythonQt import QtCore


class ValvePlannerDemo(object):

    def __init__(self, robotModel, footstepPlanner, manipPlanner, ikPlanner, lhandDriver,
                 rhandDriver, sensorJointController):
        self.robotModel = robotModel
        self.footstepPlanner = footstepPlanner
        self.manipPlanner = manipPlanner
        self.ikPlanner = ikPlanner
        self.lhandDriver = lhandDriver
        self.rhandDriver = rhandDriver
        self.sensorJointController = sensorJointController
        self.graspingObject = 'valve'
        self.graspingHand = 'left'
        self.valveAffordance = None

        # live operation flags
        self.planFromCurrentRobotState = False

        self.plans = []

        # IK server speed:
        self.speedLow = 10
        self.speedHigh = 60
        self.speedTurn = 100

        # reach to center and back - for palm point
        self.clenchFrameXYZ = [0.0, 0.0, -0.1]
        self.clenchFrameRPY = [90, 0, 180]
        self.nominalPelvisXYZ = None
        self.useLargeValveDefaults()

        self.coaxialTol = 0.001
        self.coaxialGazeTol = 2
        self.shxMaxTorque = 40
        self.elxMaxTorque = 10
        self.reachPoseName = None
        self.touchPose = None

        self.quasiStaticShrinkFactor = 0.5

        self.lockBack = True
        self.lockBase = True

        self.nominalPoseName = 'q_valve_nom'
        self.startPoseName = 'q_valve_start'

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
        self.relativeStanceXYZInitial = [-0.9, 0.3, 0.0]
        self.relativeStanceRPYInitial = [0, 0, 0.1]
        # -1 = anticlockwise (left, default) | 1 = clockwise
        if (self.graspingHand is 'left'):
            self.scribeDirection = -1
        else:
            self.scribeDirection = 1

    def addPlan(self, plan):
        self.plans.append(plan)

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

    def computeValveStanceFrame(self):
        objectTransform = transformUtils.copyFrame(self.clenchFrame.transform)
        self.relativeStanceTransform = transformUtils.copyFrame(
            transformUtils.frameFromPositionAndRPY(self.relativeStanceXYZ, self.relativeStanceRPY))
        robotStance = self.computeRobotStanceFrame(objectTransform, self.relativeStanceTransform)
        self.stanceFrame = vis.updateFrame(robotStance, 'valve grasp stance',
                                           parent=self.valveAffordance, visible=False, scale=0.2)
        self.stanceFrame.addToView(app.getDRCView())

    def spawnValveFrame(self, robotModel, height):

        position = [0.7, 0.22, height]
        rpy = [180, -90, 0]
        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.footstepPlanner.getFeetMidPoint(robotModel))
        return t

    def findAffordance(self):
        self.setupAffordanceParams()
        self.findValveAffordance()

    def setupAffordanceParams(self):
        self.setupStance()

        self.relativeStanceXYZ = self.relativeStanceXYZInitial
        self.relativeStanceRPY = self.relativeStanceRPYInitial

        # mirror stance and rotation direction for right hand:
        if (self.graspingHand is 'right'):
            self.relativeStanceXYZ[1] = -self.relativeStanceXYZ[1]
            self.relativeStanceRPY[2] = -self.relativeStanceRPY[2]

    def findValveAffordance(self):
        self.valveAffordance = om.findObjectByName('valve')
        if self.valveAffordance is None:
            return

        valveFrame = self.valveAffordance.getChildFrame()

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.RotateX(180)
        t.RotateY(-90)
        t.Concatenate(valveFrame.transform)
        self.valveFrame = t

        self.computeClenchFrame()
        self.computeValveStanceFrame()

        self.frameSync = vis.FrameSync()
        self.frameSync.addFrame(valveFrame)
        self.frameSync.addFrame(self.clenchFrame, ignoreIncoming=True)
        self.frameSync.addFrame(self.stanceFrame, ignoreIncoming=True)

    def computeClenchFrame(self):
        t = transformUtils.frameFromPositionAndRPY(self.clenchFrameXYZ,
                                                   self.clenchFrameRPY)
        t_copy = transformUtils.copyFrame(t)
        t_copy.Concatenate(self.valveFrame)
        self.clenchFrame = vis.updateFrame(t_copy, 'valve clench frame',
                                           parent=self.valveAffordance,
                                           visible=False, scale=0.2)
        self.clenchFrame.addToView(app.getDRCView())

    # End Valve Focused Functions ##############################################
    # Planning Functions #######################################################

    # These are operational conveniences:
    def planFootstepsToStance(self):
        self.planFootsteps(self.stanceFrame.transform)

    def planFootsteps(self, goalFrame):
        startPose = self.getPlanningStartPose()
        request = self.footstepPlanner.constructFootstepPlanRequest(startPose,
                                                                    goalFrame)
        self.footstepPlan = self.footstepPlanner.sendFootstepPlanRequest(
            request, waitForResponse=True)

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
        constraint = ik.GravityCompensationTorqueConstraint()
        constraint.joints = [shxJoint, elxJoint]
        constraint.torquesLowerBound = -np.array([self.shxMaxTorque, self.elxMaxTorque])
        constraint.torquesUpperBound = np.array([self.shxMaxTorque, self.elxMaxTorque])
        return constraint

    def createWristAngleConstraint(self, wristAngleCW, planFromCurrentRobotState):
        if self.graspingHand == 'left':
            wristJoint = ['l_arm_lwy']
            wristJointLowerBound = [-np.radians(160) - wristAngleCW]
            wristJointUpperBound = [-np.radians(160) - wristAngleCW]
        else:
            wristJoint = ['r_arm_lwy']
            wristJointLowerBound = [np.radians(160) - wristAngleCW]
            wristJointUpperBound = [np.radians(160) - wristAngleCW]
        constraint = ik.PostureConstraint()
        constraint.joints = wristJoint
        constraint.jointsLowerBound = wristJointLowerBound
        constraint.jointsUpperBound = wristJointUpperBound
        if planFromCurrentRobotState:
            constraint.tspan = [1.0, 1.0]
        return constraint

    def createHandGazeConstraint(self):
        constraint = self.ikPlanner.createGazeGraspConstraint(
            self.graspingHand, self.clenchFrame, coneThresholdDegrees=self.coaxialGazeTol)
        constraint.tspan = [0.0, 1.0]
        return constraint

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
            constraints.append(ik.WorldFixedBodyPoseConstraint(linkName='l_foot'))
            constraints.append(ik.WorldFixedBodyPoseConstraint(linkName='r_foot'))

            p = ik.RelativePositionConstraint()
            p.bodyNameA = 'l_foot'
            p.bodyNameB = 'r_foot'
            p.positionTarget = np.array([0, 0.3, 0])
            p.lowerBound = np.array([0, 0, -np.inf])
            p.upperBound = np.array([0, 0, np.inf])
            constraints.append(p)

            p = ik.RelativePositionConstraint()
            p.bodyNameA = 'r_foot'
            p.bodyNameB = 'l_foot'
            p.lowerBound = np.array([0, -np.inf, -np.inf])
            p.upperBound = np.array([0, np.inf, np.inf])
            constraints.append(p)

            return constraints

    def createHeadGazeConstraint(self):
        valveCenter = np.array(self.clenchFrame.transform.GetPosition())
        return ik.WorldGazeTargetConstraint(linkName='head', bodyPoint=np.zeros(3),
                                            worldPoint=valveCenter, coneThreshold=np.radians(20))

    def createBaseConstraints(self, resetBase, lockBase, lockFeet, yawDesired):
        constraints = []

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
                constraints.append(ik.WorldFixedBodyPoseConstraint(linkName='pelvis'))
        else:
            constraints.append(self.ikPlanner.createXYZYawMovingBasePostureConstraint(poseName))
            constraints.append(ik.WorldFixedBodyPoseConstraint(linkName='pelvis'))
            constraints.append(self.createHeadGazeConstraint())

            p = ik.PostureConstraint()
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
        constraint = ik.PositionConstraint()
        constraint.linkName = self.ikPlanner.getHandLink(self.graspingHand)
        constraint.pointInLink = np.array(linkOffsetFrame.GetPosition())
        constraint.referenceFrame = self.clenchFrame.transform
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

    def setReachAndTouchPoses(self, plan):
        self.reachPoseName = 'q_reach'
        self.touchPoseName = 'q_touch'
        self.reachPose = robotstate.convertStateMessageToDrakePose(plan.plan[0])
        self.touchPose = robotstate.convertStateMessageToDrakePose(plan.plan[-1])
        self.ikPlanner.addPose(self.reachPose, self.reachPoseName)
        self.ikPlanner.addPose(self.touchPose, self.touchPoseName)

    def planInsertTraj(self, lockFeet=True, lockBase=None, resetBase=False, wristAngleCW=0,
                       startPose=None, verticalOffset=0.01, usePoses=False, resetPoses=True,
                       planFromCurrentRobotState=False, retract=False):
        QuasiStaticShrinkFactorOrig = ik.QuasiStaticConstraint.shrinkFactor
        fixInitialStateOrig = self.ikPlanner.ikServer.fixInitialState
        usePointwiseOrig = self.ikPlanner.ikServer.usePointwise
        ik.QuasiStaticConstraint.shrinkFactor = self.quasiStaticShrinkFactor
        self.ikPlanner.ikServer.fixInitialState = planFromCurrentRobotState
        self.ikPlanner.ikServer.usePointwise = False

        _, _, zaxis = transformUtils.getAxesFromTransform(self.valveFrame)
        yawDesired = np.arctan2(zaxis[1], zaxis[0])

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
        constraints.append(self.createStaticTorqueConstraint())
        constraints.append(self.createHandGazeConstraint())
        constraints.append(self.createWristAngleConstraint(wristAngleCW,
                                                           planFromCurrentRobotState))
        constraints.extend(self.createAllHandPositionConstraints(self.coaxialTol, retract))

        startPose = self.getStartPoseName(planFromCurrentRobotState, retract, usePoses)
        endPose = self.getEndPoseName(retract, usePoses)

        plan = self.ikPlanner.runIkTraj(constraints, startPose, endPose, self.nominalPoseName)

        if resetPoses and not retract and max(plan.plan_info) <= 10:
            self.setReachAndTouchPoses(plan)

        ik.QuasiStaticConstraint.shrinkFactor = QuasiStaticShrinkFactorOrig
        self.ikPlanner.ikServer.fixInitialState = fixInitialStateOrig
        self.ikPlanner.ikServer.usePointwise = usePointwiseOrig
        return plan

    def planReach(self, verticalOffset=None, **kwargs):
        startPose = self.getPlanningStartPose()
        self.planInsertTraj(lockBase=False, lockFeet=True, usePoses=True, resetPoses=True,
                            **kwargs)
        plan = self.ikPlanner.computePostureGoal(startPose, self.reachPose)
        self.addPlan(plan)

    def planTouch(self, **kwargs):
        self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedLow
        plan = self.planInsertTraj(lockBase=True, lockFeet=True, usePoses=True, resetPoses=False,
                                   planFromCurrentRobotState=True, **kwargs)
        self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedHigh
        self.addPlan(plan)

    def planTurn(self, wristAngleCW=np.radians(320)):
        startPose = self.getPlanningStartPose()
        wristAngleCW = min(np.radians(320)-0.01, max(-np.radians(160)+0.01, wristAngleCW))
        if self.graspingHand == 'left':
            postureJoints = {'l_arm_lwy': -np.radians(160) + wristAngleCW}
        else:
            postureJoints = {'r_arm_lwy': np.radians(160) - wristAngleCW}

        endPose = self.ikPlanner.mergePostures(startPose, postureJoints)

        self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedTurn
        plan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedHigh
        app.displaySnoptInfo(1)

        self.addPlan(plan)

    def planRetract(self, **kwargs):
        self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedLow
        startPose = self.getPlanningStartPose()
        if self.graspingHand == 'left':
            jointId = robotstate.getDrakePoseJointNames().index('l_arm_lwy')
            wristAngleCW = np.radians(160) + startPose[jointId]
        else:
            jointId = robotstate.getDrakePoseJointNames().index('r_arm_lwy')
            wristAngleCW = np.radians(160) - startPose[jointId]
        plan = self.planInsertTraj(retract=True, lockBase=True, lockFeet=True, usePoses=True,
                                   planFromCurrentRobotState=True, resetPoses=False,
                                   wristAngleCW=wristAngleCW, **kwargs)
        self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedHigh
        self.addPlan(plan)

    def getNominalPose(self):
        axes = transformUtils.getAxesFromTransform(self.clenchFrame.transform)
        yaxis = axes[1]
        yawDesired = np.arctan2(yaxis[1], yaxis[0])
        seedDistance = 1

        nominalPose = self.ikPlanner.jointController.getPose('q_nom')
        nominalPose[0] = (self.clenchFrame.transform.GetPosition()[0] - seedDistance*yaxis[0])
        nominalPose[1] = (self.clenchFrame.transform.GetPosition()[1] - seedDistance*yaxis[1])
        nominalPose[5] = yawDesired
        return nominalPose

    def getStanceFrameCoaxial(self):
        startPose = self.getNominalPose()

        plan = self.planInsertTraj(lockFeet=False, lockBase=False, resetPoses=True,
                                   startPose=startPose)
        stancePose = robotstate.convertStateMessageToDrakePose(plan.plan[0])
        stanceRobotModel = self.ikPlanner.getRobotModelAtPose(stancePose)
        self.nominalPelvisXYZ = stancePose[:3]
        return self.footstepPlanner.getFeetMidPoint(stanceRobotModel)

    # Glue Functions ###########################################################
    def moveRobotToStanceFrame(self, frame):
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

    def openPinch(self, side):
        self.getHandDriver(side).sendCustom(20.0, 100.0, 100.0, 1)

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

    # Nominal Plans and Execution  #############################################


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

        self.addManualButton('Start', self.onStartClicked)
        self.addManualSpacer()
        self.addManualButton('Footsteps', self.valveDemo.planFootstepsToStance)
        self.addManualSpacer()
        self.addManualButton('Raise arm', self.valveDemo.planPreGrasp)
        self.addManualButton('Set fingers', self.setFingers)
        self.addManualSpacer()
        self.addManualButton('Reach', self.reach)
        self.addManualButton('Touch', self.grasp)
        self.addManualButton('Turn', self.turnValve)
        self.addManualButton('Retract', self.retract)
        self.addManualSpacer()
        self.addManualButton('Nominal', self.valveDemo.planNominal)

    def onStartClicked(self):
        self.valveDemo.findAffordance()
        if self.valveDemo.valveAffordance is not None:
            print 'Valve Demo: Start - Ready to proceed'
        else:
            print 'Valve Demo: Start - VALVE AFFORDANCE NOT FOUND'

    def setFingers(self):
        self.valveDemo.openPinch(self.valveDemo.graspingHand)

    def reach(self):
        self.valveDemo.planReach()

    def grasp(self):
        self.valveDemo.planTouch()

    def turnValve(self):
        self.valveDemo.planTurn()

    def retract(self):
        self.valveDemo.planRetract()

    def addDefaultProperties(self):
        self.params.addProperty('Hand', 1, attributes=om.PropertyAttributes(enumNames=['Left',
                                                                                       'Right']))
        self.params.addProperty('Turn direction', 0,
                                attributes=om.PropertyAttributes(enumNames=['Clockwise',
                                                                            'Counter clockwise']))
        self.params.addProperty('Touch angle (deg)', 0)
        self._syncProperties()

    def onPropertyChanged(self, propertySet, propertyName):
        self._syncProperties()

    def _syncProperties(self):

        self.valveDemo.planFromCurrentRobotState = True
        self.valveDemo.graspingHand = self.params.getPropertyEnumValue('Hand').lower()
        if self.params.getPropertyEnumValue('Turn direction') == 'Clockwise':
            self.valveDemo.scribeDirection = 1
        else:
            self.valveDemo.scribeDirection = -1

    def addTasks(self):

        # some helpers
        def addTask(task, parent=None):
            self.taskTree.onAddTask(task, copy=False, parent=parent)

        def addFunc(func, name, parent=None):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)

        def addLargeValveTurn(parent=None):
            group = self.taskTree.addGroup('Valve Turn', parent=parent)

            initialWristAngleCW = 0 if v.scribeDirection == 1 else np.radians(320)
            finalWristAngleCW = np.radians(320) if v.scribeDirection == 1 else 0

            # valve manip actions
            addFunc(functools.partial(v.planReach, wristAngleCW=initialWristAngleCW),
                    name='plan reach to valve', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'),
                    parent=group)

            addFunc(functools.partial(v.planTouch, wristAngleCW=initialWristAngleCW),
                    name='plan insert in valve', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'),
                    parent=group)

            addFunc(functools.partial(v.planTurn, wristAngleCW=finalWristAngleCW),
                    name='plan turn valve', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'),
                    parent=group)

            addFunc(v.planRetract, name='plan retract', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'),
                    parent=group)

        def addSmallValveTurn(parent=None):
            group = self.taskTree.addGroup('Valve Turn', parent=parent)
            side = 'Right' if v.graspingHand == 'right' else 'Left'

            initialWristAngleCW = 0 if v.scribeDirection == 1 else np.radians(680)
            finalWristAngleCW = np.radians(680) if v.scribeDirection == 1 else 0

            addFunc(functools.partial(v.planReach, wristAngleCW=initialWristAngleCW),
                    name='plan reach to valve', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'),
                    parent=group)

            addFunc(functools.partial(v.planTouch, wristAngleCW=initialWristAngleCW),
                    name='plan insert in valve', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'),
                    parent=group)

            addTask(rt.CloseHand(name='grasp valve', side=side, mode='Basic',
                                 amount=self.valveDemo.closedAmount),
                    parent=group)

            addFunc(functools.partial(v.planTurn, wristAngleCW=finalWristAngleCW),
                    name='plan turn valve', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'),
                    parent=group)

            addTask(rt.CloseHand(name='release valve', side=side, mode='Basic',
                                 amount=self.valveDemo.openAmount),
                    parent=group)

            addFunc(v.planRetract, name='plan retract', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'),
                    parent=group)

        v = self.valveDemo

        self.taskTree.removeAllTasks()
        side = self.params.getPropertyEnumValue('Hand')

        ###############
        # add the tasks

        # prep
        addTask(rt.CloseHand(name='close left hand', side='Left'))
        addTask(rt.CloseHand(name='close right hand', side='Right'))
        addTask(rt.SetNeckPitch(name='set neck position', angle=0))
        addTask(rt.PlanPostureGoal(name='plan walk posture', postureGroup='General',
                                   postureName='safe nominal', side='Default'))
        addTask(rt.UserPromptTask(name='approve manip plan',
                                  message='Please approve manipulation plan.'))
        addTask(rt.CommitManipulationPlan(name='execute manip plan',
                                          planName='safe nominal posture plan'))
        addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'))

        # fit
        addTask(rt.UserPromptTask(name='fit valve',
                                  message='Please fit and approve valve affordance.'))
        addTask(rt.FindAffordance(name='check valve affordance', affordanceName='valve'))
        addFunc(v.computeValveStanceFrame, name='plan stance location')

        # walk
        addTask(rt.RequestFootstepPlan(name='plan walk to valve',
                                       stanceFrameName='valve grasp stance'))
        addTask(rt.UserPromptTask(name='approve footsteps',
                                  message='Please approve footstep plan.'))
        addTask(rt.CommitFootstepPlan(name='walk to valve',
                                      planName='valve grasp stance footstep plan'))
        addTask(rt.WaitForWalkExecution(name='wait for walking'))

        # refit
        addTask(rt.SetNeckPitch(name='set neck position', angle=35))
        addTask(rt.UserPromptTask(name='fit value',
                                  message='Please fit and approve valve affordance.'))

        # set fingers
        addTask(rt.CloseHand(name='set finger positions', side=side, mode='Basic',
                             amount=self.valveDemo.openAmount))

        # add valve turns
        if v.smallValve:
            for i in range(0, 3):
                addSmallValveTurn()

        else:
            for i in range(0, 3):
                addLargeValveTurn()
