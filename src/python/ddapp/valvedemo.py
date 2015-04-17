import vtkAll as vtk
import math
import functools
import numpy as np

from ddapp import transformUtils
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp import applogic as app
from ddapp import ik
from ddapp.simpletimer import SimpleTimer
from ddapp import robotstate
from ddapp import segmentation
from ddapp.tasks.taskuserpanel import TaskUserPanel
from ddapp.tasks.taskuserpanel import ImageBasedAffordanceFit

import ddapp.tasks.robottasks as rt

from PythonQt import QtCore


class ValvePlannerDemo(object):

    def __init__(self, robotModel, footstepPlanner, manipPlanner, ikPlanner,
                 lhandDriver, rhandDriver, atlasDriver, multisenseDriver,
                 affordanceFitFunction, sensorJointController,
                 planPlaybackFunction, showPoseFunction):
        self.robotModel = robotModel
        self.footstepPlanner = footstepPlanner
        self.manipPlanner = manipPlanner
        self.ikPlanner = ikPlanner
        self.lhandDriver = lhandDriver
        self.rhandDriver = rhandDriver
        self.atlasDriver = atlasDriver
        self.multisenseDriver = multisenseDriver
        self.affordanceFitFunction = affordanceFitFunction
        self.sensorJointController = sensorJointController
        self.planPlaybackFunction = planPlaybackFunction
        self.showPoseFunction = showPoseFunction
        self.graspingObject = 'valve'
        self.graspingHand = 'left'
        self.valveAffordance = None

        # live operation flags
        self.useFootstepPlanner = True
        self.visOnly = True
        self.planFromCurrentRobotState = False
        useDevelopment = False
        if (useDevelopment):
            self.visOnly = True
            self.planFromCurrentRobotState = False

        self.constraintSet = None

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

        self.lockBack = False
        self.lockBase = True

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

        self.relativeStanceXYZInitial = [-1.05, 0.27, 0.0]
        self.relativeStanceRPYInitial = [0, 0, 0.1]

        # -1 = anticlockwise (left, default) | 1 = clockwise
        if (self.graspingHand is 'left'):
            self.scribeDirection = -1
        else:
            self.scribeDirection = 1

    def addPlan(self, plan):
        self.plans.append(plan)

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

        stancePosition = (np.array(t2.GetPosition())
                          + np.array(t1.GetPosition())) / 2.0

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

        groundFrame = self.computeGroundFrame(self.robotModel)
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

        graspGroundTransform = transformUtils.getTransformFromAxes(xaxis, yaxis,
                                                                   zaxis)
        graspGroundTransform.PostMultiply()
        graspGroundTransform.Translate(graspPosition[0], graspPosition[1],
                                       groundHeight)

        robotStance = transformUtils.copyFrame(relativeStanceTransform)
        robotStance.Concatenate(graspGroundTransform)

        return robotStance

    def updatePointcloudSnapshot(self):

        if (self.useLidar is True):
            return vis.updatePolyData(segmentation.getCurrentRevolutionData(),
                                      'pointcloud snapshot',
                                      parent='segmentation')
        else:
            return vis.updatePolyData(segmentation.getDisparityPointCloud(4),
                                      'pointcloud snapshot',
                                      parent='segmentation')

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
        # objectTransform = transformUtils.copyFrame(self.clenchFrame.transform)
        self.relativeStanceTransform = transformUtils.copyFrame(
            transformUtils.frameFromPositionAndRPY(self.relativeStanceXYZ,
                                                   self.relativeStanceRPY))
        # robotStance = self.computeRobotStanceFrame(objectTransform,
        #                                          self.relativeStanceTransform)
        robotStance = self.getStanceFrameCoaxial()
        self.stanceFrame = vis.updateFrame(robotStance, 'valve grasp stance',
                                           parent=self.valveAffordance,
                                           visible=False, scale=0.2)
        self.stanceFrame.addToView(app.getDRCView())

    def spawnValveFrame(self, robotModel, height):

        position = [0.7, 0.22, height]
        rpy = [180, -90, 0]
        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.computeGroundFrame(robotModel))
        return t

    def findAffordance(self):
        self.setupAffordanceParams()
        if (self.graspingObject is 'valve'):
            self.findValveAffordance()
        else:
            self.findValveLeverAffordance()

    def setupAffordanceParams(self):
        self.setupStance()

        self.relativeStanceXYZ = self.relativeStanceXYZInitial
        self.relativeStanceRPY = self.relativeStanceRPYInitial

        # mirror stance and rotation direction for right hand:
        if (self.graspingHand is 'right'):
            self.relativeStanceXYZ[1] = -self.relativeStanceXYZ[1]
            self.relativeStanceRPY[2] = -self.relativeStanceRPY[2]

    def updateTouchAngleVisualization(self, angle):
        if self.valveAffordance:

            obj = om.findObjectByName('valve touch angle')

            t = vtk.vtkTransform()
            t.PostMultiply()
            t.RotateX(angle)
            t.Concatenate(self.valveAffordance.getChildFrame().transform)

            if not obj:
                pose = transformUtils.poseFromTransform(t)
                length = self.valveAffordance.getProperty('Radius')*2
                desc = dict(classname='CylinderAffordanceItem',
                            Name='valve touch angle',
                            uuid=segmentation.newUUID(), pose=pose, Radius=0.01,
                            Length=length, Color=[1.0, 1.0, 0.0])

                import affordancepanel
                obj = affordancepanel.panel.affordanceFromDescription(desc)

            obj.getChildFrame().copyFrame(t)

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

    def findValveLeverAffordance(self):

        self.valveAffordance = om.findObjectByName('lever')
        self.valveFrame = om.findObjectByName('lever frame')

        # length of lever is equivalent to radius of valve
        self.scribeRadius = self.valveAffordance.params.get('length') - 0.10

        self.computeClenchFrame()
        self.computeValveStanceFrame()

        self.frameSync = vis.FrameSync()
        self.frameSync.addFrame(self.valveFrame)
        self.frameSync.addFrame(self.clenchFrame)
        self.frameSync.addFrame(self.stanceFrame)

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
        endPose = self.ikPlanner.getMergedPostureFromDatabase(endPose,
                                                              'General',
                                                              'safe nominal')
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def coaxialPlanTraj(self, lockFeet=True, lockBack=None,
                        lockBase=None, resetBase=False,  wristAngleCW=0,
                        startPose=None, verticalOffset=0.01,
                        constrainWristX=True, usePoses=False, resetPoses=True,
                        planFromCurrentRobotState=False, retract=False):
        QuasiStaticShrinkFactorOrig = ik.QuasiStaticConstraint.shrinkFactor
        fixInitialStateOrig = self.ikPlanner.ikServer.fixInitialState
        usePointwiseOrig = self.ikPlanner.ikServer.usePointwise
        additionalTimeSamplesOrig = self.ikPlanner.additionalTimeSamples
        numberOfAddedKnotsOrig = self.ikPlanner.ikServer.numberOfAddedKnots
        ik.QuasiStaticConstraint.shrinkFactor = self.quasiStaticShrinkFactor
        self.ikPlanner.ikServer.fixInitialState = planFromCurrentRobotState
        self.ikPlanner.ikServer.usePointwise = False
        self.ikPlanner.additionalTimeSamples = 5
        self.ikPlanner.ikServer.numberOfAddedKnots = 1

        _, _, zaxis = transformUtils.getAxesFromTransform(self.valveFrame)
        yawDesired = np.arctan2(zaxis[1], zaxis[0])

        if lockBase is None:
            lockBase = self.lockBase

        if lockBack is None:
            lockBack = self.lockBack

        if self.graspingHand == 'left':
            elxJoint = 'l_arm_elx'
            shxJoint = 'l_arm_shx'
            yJoints = ['l_arm_lwy']
            yJointLowerBound = [-np.radians(160) - wristAngleCW]
            yJointUpperBound = [-np.radians(160) - wristAngleCW]
        else:
            elxJoint = 'r_arm_elx'
            shxJoint = 'r_arm_shx'
            yJoints = ['r_arm_lwy']
            yJointLowerBound = [np.radians(160) - wristAngleCW]
            yJointUpperBound = [np.radians(160) - wristAngleCW]

        if startPose is None:
            startPose = self.getPlanningStartPose()

        nominalPose = self.coaxialGetNominalPose()
        nominalPoseName = 'qNomAtRobot'
        self.ikPlanner.addPose(nominalPose, nominalPoseName)

        startPoseName = 'Start'
        self.ikPlanner.addPose(startPose, startPoseName)
        self.ikPlanner.reachingSide = self.graspingHand

        constraints = []
        constraints.append(
            self.ikPlanner.createLockedArmPostureConstraint(startPoseName))

        if resetBase:
            baseConstraintRobotPoseName = nominalPoseName
        else:
            baseConstraintRobotPoseName = startPoseName

        if lockFeet:
            constraints.extend(
                self.ikPlanner.createFixedFootConstraints(startPoseName))
            if lockBase:
                constraints.append(self.ikPlanner.createLockedBasePostureConstraint(baseConstraintRobotPoseName, lockLegs=False))
            else:
                constraints.append(self.ikPlanner.createXYZMovingBasePostureConstraint(baseConstraintRobotPoseName))
                constraints.append(ik.WorldFixedBodyPoseConstraint(linkName='pelvis'))
        else:
            constraints.append(self.ikPlanner.createXYZYawMovingBasePostureConstraint(baseConstraintRobotPoseName))
            constraints.extend(self.ikPlanner.createSlidingFootConstraints(startPose))
            constraints.append(ik.WorldFixedBodyPoseConstraint(linkName='l_foot'))
            constraints.append(ik.WorldFixedBodyPoseConstraint(linkName='r_foot'))
            constraints.append(ik.WorldFixedBodyPoseConstraint(linkName='pelvis'))

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

            headGaze = ik.WorldGazeTargetConstraint(linkName='head',
                                                    bodyPoint=np.zeros(3),
                                                    worldPoint=np.array(self.clenchFrame.transform.GetPosition()),
                                                    coneThreshold=np.radians(20))
            constraints.append(headGaze)

            p = ik.PostureConstraint()
            p.joints = ['base_yaw']
            p.jointsLowerBound = [yawDesired - np.radians(20)]
            p.jointsUpperBound = [yawDesired + np.radians(20)]
            constraints.append(p)

        if lockBack:
            constraints.append(self.ikPlanner.createLockedBackPostureConstraint(startPoseName))
        else:
            constraints.append(self.ikPlanner.createMovingBackLimitedPostureConstraint())

        constraints.append(self.ikPlanner.createKneePostureConstraint([0.7, 2.5]))

        wristTol = self.coaxialTol
        gazeDegreesTol = self.coaxialGazeTol

        constraints.append(self.ikPlanner.createQuasiStaticConstraint())

        constraints.append(self.ikPlanner.createGazeGraspConstraint(self.graspingHand, self.clenchFrame, coneThresholdDegrees=gazeDegreesTol))
        constraints[-1].tspan = [0.0, 1.0]

        p = ik.PostureConstraint()
        p.joints = yJoints
        p.jointsLowerBound = yJointLowerBound
        p.jointsUpperBound = yJointUpperBound
        constraints.append(p)

        torqueConstraint = ik.GravityCompensationTorqueConstraint()
        torqueConstraint.joints = [shxJoint, elxJoint]
        torqueConstraint.torquesLowerBound = -np.array([self.shxMaxTorque, self.elxMaxTorque])
        torqueConstraint.torquesUpperBound = np.array([self.shxMaxTorque, self.elxMaxTorque])
        constraints.append(torqueConstraint)

        constraintSet = self.ikPlanner.newReachGoal(startPoseName, self.graspingHand, self.clenchFrame.transform, constraints, lockOrient=False)
        constraintSet.constraints[-1].lowerBound = np.array([-wristTol, self.reachDepth, -wristTol])
        constraintSet.constraints[-1].upperBound = np.array([wristTol, self.touchDepth, wristTol])
        constraintSet.constraints[-1].tspan = [0.0, 1.0]

        constraintSet = self.ikPlanner.newReachGoal(startPoseName, self.graspingHand, self.clenchFrame.transform, constraintSet.constraints, lockOrient=False)
        if retract:
            constraintSet.constraints[-1].lowerBound = np.array([-np.inf, self.retractDepth, -np.inf])
            constraintSet.constraints[-1].upperBound = np.array([np.inf, self.retractDepth, np.inf])
            constraintSet.constraints[-1].tspan = [1.0, 1.0]
        else:
            constraintSet.constraints[-1].lowerBound = np.array([-np.inf, self.reachDepth, -np.inf])
            constraintSet.constraints[-1].upperBound = np.array([np.inf, self.reachDepth, np.inf])
            constraintSet.constraints[-1].tspan = [0.0, 0.0]

        constraintSet = self.ikPlanner.newReachGoal(startPoseName, self.graspingHand, self.clenchFrame.transform, constraintSet.constraints, lockOrient=False)
        constraintSet.constraints[-1].lowerBound = np.array([-np.inf, self.touchDepth, -np.inf])
        constraintSet.constraints[-1].upperBound = np.array([np.inf, self.touchDepth, np.inf])
        if retract:
            constraintSet.constraints[-1].tspan = [0.0, 0.0]
        else:
            constraintSet.constraints[-1].tspan = [1.0, 1.0]

        constraintSet.nominalPoseName = nominalPoseName
        if planFromCurrentRobotState:
            constraintSet.startPoseName = startPoseName
        else:
            if not usePoses or self.reachPoseName is None:
                constraintSet.startPoseName = nominalPoseName
            else:
                if retract:
                    constraintSet.startPoseName = self.touchPoseName
                else:
                    constraintSet.startPoseName = self.reachPoseName

        if not usePoses or self.touchPose is None:
            constraintSet.endPose = nominalPose
        else:
            if retract:
                constraintSet.endPose = self.reachPose
            else:
                constraintSet.endPose = self.touchPose

        plan = constraintSet.runIkTraj()
        if resetPoses and not retract and max(plan.plan_info) <= 10:
            self.reachPoseName = 'q_reach'
            self.touchPoseName = 'q_touch'
            self.reachPose = robotstate.convertStateMessageToDrakePose(plan.plan[0])
            self.touchPose = robotstate.convertStateMessageToDrakePose(plan.plan[-1])
            self.ikPlanner.addPose(self.reachPose, self.reachPoseName)
            self.ikPlanner.addPose(self.touchPose, self.touchPoseName)

        ik.QuasiStaticConstraint.shrinkFactor = QuasiStaticShrinkFactorOrig
        self.ikPlanner.ikServer.fixInitialState = fixInitialStateOrig
        self.ikPlanner.ikServer.usePointwise = usePointwiseOrig
        self.ikPlanner.ikServer.numberOfAddedKnots = numberOfAddedKnotsOrig
        self.ikPlanner.additionalTimeSamples = additionalTimeSamplesOrig
        return plan

    def coaxialPlanReach(self, verticalOffset=None, **kwargs):
        startPose = self.getPlanningStartPose()
        self.coaxialPlanTraj(lockBase=False, lockBack=True, lockFeet=True,
                             usePoses=True, resetPoses=True, **kwargs)
        plan = self.ikPlanner.computePostureGoal(startPose, self.reachPose)
        self.addPlan(plan)

    def coaxialPlanTouch(self, **kwargs):
        self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedLow
        plan = self.coaxialPlanTraj(lockBase=True, lockBack=True, lockFeet=True,
                                    usePoses=True, resetPoses=False,
                                    planFromCurrentRobotState=True, **kwargs)
        self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedHigh
        self.addPlan(plan)

    def coaxialPlanTurn(self, wristAngleCW=np.radians(320)):
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

    def coaxialPlanRetract(self, **kwargs):
        self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedLow
        startPose = self.getPlanningStartPose()
        if self.graspingHand == 'left':
            jointId = robotstate.getDrakePoseJointNames().index('l_arm_lwy')
            wristAngleCW = np.radians(160) + startPose[jointId]
        else:
            jointId = robotstate.getDrakePoseJointNames().index('r_arm_lwy')
            wristAngleCW = np.radians(160) - startPose[jointId]
        plan = self.coaxialPlanTraj(retract=True, lockBase=True, lockBack=True,
                                    lockFeet=True,
                                    planFromCurrentRobotState=True,
                                    usePoses=True, resetPoses=False,
                                    wristAngleCW=wristAngleCW, **kwargs)
        self.ikPlanner.ikServer.maxDegreesPerSecond = self.speedHigh
        self.addPlan(plan)

    def coaxialGetNominalPose(self):
        _, yaxis, _ = transformUtils.getAxesFromTransform(self.clenchFrame.transform)
        yawDesired = np.arctan2(yaxis[1], yaxis[0])
        seedDistance = 1

        nominalPose = self.ikPlanner.jointController.getPose('q_nom')
        nominalPose[0] = self.clenchFrame.transform.GetPosition()[0] - seedDistance*yaxis[0]
        nominalPose[1] = self.clenchFrame.transform.GetPosition()[1] - seedDistance*yaxis[1]
        nominalPose[5] = yawDesired
        return nominalPose


    def getStanceFrameCoaxial(self):
        startPose = self.coaxialGetNominalPose()

        plan = self.coaxialPlanTraj(lockFeet=False,
                                    lockBase=False, lockBack=True,
                                    resetPoses=True, startPose=startPose)
        stancePose = robotstate.convertStateMessageToDrakePose(plan.plan[0])
        stanceRobotModel = self.ikPlanner.getRobotModelAtPose(stancePose)
        self.nominalPelvisXYZ = stancePose[:3]
        return self.footstepPlanner.getFeetMidPoint(stanceRobotModel)

    def getPlannedTouchAngleCoaxial(self):
        # when the pose is computed in getStanceFrameCoaxial, we could
        # store the turn angle. This method just returns the stored value.
        return 0.0

    def setDesiredTouchAngleCoaxial(self, angle):
        # this is the turn angle that the user wants.
        # this should be close to the planned touch angle, but the user may
        # adjust that value to avoid hitting the spokes.

        self.updateTouchAngleVisualization(angle)


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

    def openPinch(self,side):
        self.getHandDriver(side).sendCustom(20.0, 100.0, 100.0, 1)

    def delay(self, delayTimeInSeconds):
        yield
        t = SimpleTimer()
        while t.elapsed() < delayTimeInSeconds:
            yield

    def getPlanningStartPose(self):
        if self.planFromCurrentRobotState:
            return self.sensorJointController.getPose('EST_ROBOT_STATE')
        else:
            if self.plans:
                return robotstate.convertStateMessageToDrakePose(self.plans[-1].plan[-1])
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

      # now get the planned turn angle and show it to the user
      self.params.setProperty('Touch angle (deg)', self.valveDemo.getPlannedTouchAngleCoaxial())

    def resetTouchAngle(self):
      self.valveDemo.findAffordance()
      self.params.setProperty('Touch angle (deg)', self.valveDemo.getPlannedTouchAngleCoaxial())


    def setFingers(self):
      self.valveDemo.openPinch(self.valveDemo.graspingHand)

    def reach(self):
        self.valveDemo.coaxialPlanReach()

    def grasp(self):
        self.valveDemo.coaxialPlanTouch()

    def turnValve(self):
        self.valveDemo.coaxialPlanTurn()

    def retract(self):
        self.valveDemo.coaxialPlanRetract()

    def addDefaultProperties(self):
        self.params.addProperty('Hand', 1, attributes=om.PropertyAttributes(enumNames=['Left', 'Right']))
        self.params.addProperty('Turn direction', 0, attributes=om.PropertyAttributes(enumNames=['Clockwise', 'Counter clockwise']))
        self.params.addProperty('Touch angle (deg)', 0)
        #self.params.addProperty('Turn amount (deg)', 60)
        self._syncProperties()

    def onPropertyChanged(self, propertySet, propertyName):
        self._syncProperties()

    def _syncProperties(self):

        self.valveDemo.planFromCurrentRobotState = True
        self.valveDemo.visOnly = False
        self.valveDemo.graspingHand = self.params.getPropertyEnumValue('Hand').lower()
        self.valveDemo.scribeDirection = 1 if self.params.getPropertyEnumValue('Turn direction') == 'Clockwise' else -1
        self.valveDemo.setDesiredTouchAngleCoaxial(self.params.getProperty('Touch angle (deg)'))
        #self.valveDemo.turnAngle = self.params.getProperty('Turn amount (deg)')


    def addTasks(self):

        # some helpers
        def addTask(task, parent=None):
            self.taskTree.onAddTask(task, copy=False, parent=parent)
        def addFunc(func, name, parent=None):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)
        def addLargeValveTurn(parent=None):
            group = self.taskTree.addGroup('Valve Turn', parent=parent)

            initialWristAngleCW = 0 if v.scribeDirection == 1 else np.radians(680)
            touchWristAngleCW = np.radians(20) if v.scribeDirection == 1 else np.radians(680)-np.radians(20)
            finalWristAngleCW = np.radians(680) if v.scribeDirection == 1 else 0

            # valve manip actions
            addFunc(functools.partial(v.coaxialPlanReach,
                                      wristAngleCW=initialWristAngleCW),
                    name='plan reach to valve', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'), parent=group)

            addFunc(functools.partial(v.coaxialPlanTouch,
                                      wristAngleCW=touchWristAngleCW),
                    name='plan insert in valve', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'), parent=group)

            addFunc(functools.partial(v.coaxialPlanTurn,
                                      wristAngleCW=finalWristAngleCW),
                    name='plan turn valve', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'), parent=group)

            addFunc(v.coaxialPlanRetract, name='plan retract', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'), parent=group)

        def addSmallValveTurn(parent=None):
            group = self.taskTree.addGroup('Valve Turn', parent=parent)
            side = 'Right' if v.graspingHand == 'right' else 'Left'

            initialWristAngleCW = 0 if v.scribeDirection == 1 else np.radians(680)
            finalWristAngleCW = np.radians(680) if v.scribeDirection == 1 else 0

            addFunc(functools.partial(v.coaxialPlanReach,
                                      wristAngleCW=initialWristAngleCW),
                    name='plan reach to valve', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'), parent=group)

            addFunc(functools.partial(v.coaxialPlanTouch,
                                      wristAngleCW=initialWristAngleCW),
                    name='plan insert in valve', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'), parent=group)

            addTask(rt.CloseHand(name='grasp valve',
                                    side=side, mode='Basic',
                                    amount=self.valveDemo.closedAmount), parent=group)

            addFunc(functools.partial(v.coaxialPlanTurn,
                                      wristAngleCW=finalWristAngleCW),
                    name='plan turn valve', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'), parent=group)

            addTask(rt.CloseHand(name='release valve',
                                 side=side, mode='Basic',
                                 amount=self.valveDemo.openAmount),
                    parent=group)

            addFunc(v.coaxialPlanRetract, name='plan retract', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'), parent=group)


        v = self.valveDemo

        self.taskTree.removeAllTasks()
        side = self.params.getPropertyEnumValue('Hand')

        ###############
        # add the tasks

        # prep
        addTask(rt.CloseHand(name='close left hand', side='Left'))
        addTask(rt.CloseHand(name='close right hand', side='Right'))
        addTask(rt.SetNeckPitch(name='set neck position', angle=0))
        addTask(rt.PlanPostureGoal(name='plan walk posture', postureGroup='General', postureName='safe nominal', side='Default'))
        addTask(rt.UserPromptTask(name='approve manip plan', message='Please approve manipulation plan.'))
        addTask(rt.CommitManipulationPlan(name='execute manip plan', planName='safe nominal posture plan'))
        addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'))


        # fit
        #addTask(rt.WaitForMultisenseLidar(name='wait for lidar sweep'))
        addTask(rt.UserPromptTask(name='fit valve', message='Please fit and approve valve affordance.'))
        addTask(rt.FindAffordance(name='check valve affordance', affordanceName='valve'))
        addFunc(self.resetTouchAngle, name='plan stance location')

        # walk
        addTask(rt.RequestFootstepPlan(name='plan walk to valve', stanceFrameName='valve grasp stance'))
        addTask(rt.UserPromptTask(name='approve footsteps', message='Please approve footstep plan.'))
        addTask(rt.CommitFootstepPlan(name='walk to valve', planName='valve grasp stance footstep plan'))
        addTask(rt.WaitForWalkExecution(name='wait for walking'))

        # refit
        addTask(rt.SetNeckPitch(name='set neck position', angle=35))
        #addTask(rt.WaitForMultisenseLidar(name='wait for lidar sweep'))
        addTask(rt.UserPromptTask(name='fit value', message='Please fit and approve valve affordance.'))
        addFunc(self.resetTouchAngle, name='check valve affordance')
        #addTask(rt.UserPromptTask(name='approve spoke location', message='Please approve valve spokes and touch angle.'))

        # set fingers
        addTask(rt.CloseHand(name='set finger positions', side=side, mode='Basic', amount=self.valveDemo.openAmount))

        # add valve turns
        if v.smallValve:
            for i in range(0, 3):
                addSmallValveTurn()

        else:
            for i in range(0, 3):
                addLargeValveTurn()

