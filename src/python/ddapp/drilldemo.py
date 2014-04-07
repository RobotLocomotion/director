import os
import sys
import vtkAll as vtk
from ddapp import botpy
import math
import time
import types
import functools
import numpy as np

from ddapp import transformUtils
from ddapp import lcmUtils
from ddapp.timercallback import TimerCallback
from ddapp.asynctaskqueue import AsyncTaskQueue
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp import applogic as app
from ddapp.debugVis import DebugData
from ddapp import ikplanner
from ddapp import ioUtils
from ddapp.simpletimer import SimpleTimer
from ddapp.utime import getUtime
from ddapp import robotstate
from ddapp import robotplanlistener
from ddapp import segmentation

import drc as lcmdrc

from PythonQt import QtCore, QtGui


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
        #cls.main.show()
        cls.initialized = True

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

RobotPoseGUIWrapper.init()


class DrillPlannerDemo(object):

    def __init__(self, robotModel, footstepPlanner, manipPlanner, ikPlanner, handDriver, atlasDriver, multisenseDriver, affordanceFitFunction, sensorJointController, planPlaybackFunction, showPoseFunction):
        self.robotModel = robotModel
        self.footstepPlanner = footstepPlanner
        self.manipPlanner = manipPlanner
        self.ikPlanner = ikPlanner
        self.handDriver = handDriver
        self.atlasDriver = atlasDriver
        self.multisenseDriver = multisenseDriver
        self.affordanceFitFunction = affordanceFitFunction
        self.sensorJointController = sensorJointController
        self.planPlaybackFunction = planPlaybackFunction
        self.showPoseFunction = showPoseFunction
        self.graspingHand = 'left'
        self.planFromCurrentRobotState = False
        self.userPromptEnabled = True
        self.walkingPlan = None
        self.preGraspPlan = None
        self.graspPlan = None


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


    def computeDrillFrame(self, robotModel):

        position = [1.5, 0.0, 0.9]
        rpy = [1, 1, 1]

        # drill close to origin
        position = [0.65, 0.4, 0.9]
        rpy = [1, 1, 1]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.computeGroundFrame(robotModel))
        return t


    def computeGraspFrame(self):
        self.computeGraspFrameBarrel()


    def computeGraspFrameRotary(self):

        assert self.drillAffordance

        position = [0.0,-0.18,0.0]
        rpy = [-90,90,0]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.drillFrame.transform)

        self.graspFrame = vis.updateFrame(t, 'grasp frame', parent=self.drillAffordance, visible=False, scale=0.3)


    def computeGraspFrameBarrel(self):

        assert self.drillAffordance

        # for left_base_link
        #position = [-0.12, 0.0, 0.025]
        #rpy = [0, 90, 0]

        # for palm point
        position = [-0.04, 0.0, 0.01]
        rpy = [0, 90, -90]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.drillFrame.transform)

        self.graspFrame = vis.updateFrame(t, 'grasp frame', parent=self.drillAffordance, visible=True, scale=0.2)
        vis.updateFrame(t, 'sample grasp frame 0', parent=self.drillAffordance, visible=False, scale=0.2)

        #def updateFrame(frame):
        #    self.computeGraspFrame()
        #self.drillFrame.onTransformModifiedCallback = updateFrame


    def computeDrillTipFrame(self):

        assert self.drillAffordance

        position = [0.18, 0.0, 0.13]
        rpy = [0, 0, 0]
        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.drillFrame.transform)

        drillTipFrame = vis.updateFrame(t, 'drill tip frame', parent=self.drillAffordance, visible=False, scale=0.2)

        #def updateFrame(frame):
        #    self.computeDrillTipFrame()
        #self.graspFrame.onTransformModifiedCallback = updateFrame


    def computeStanceFrame(self):

        graspFrame = self.graspFrame.transform

        groundFrame = self.computeGroundFrame(self.robotModel)
        groundHeight = groundFrame.GetPosition()[2]

        graspPosition = np.array(graspFrame.GetPosition())
        graspYAxis = [0.0, 1.0, 0.0]
        graspZAxis = [0.0, 0.0, 1.0]
        graspFrame.TransformVector(graspYAxis, graspYAxis)
        graspFrame.TransformVector(graspZAxis, graspZAxis)

        xaxis = graspYAxis
        #xaxis = graspZAxis
        zaxis = [0, 0, 1]
        yaxis = np.cross(zaxis, xaxis)
        yaxis /= np.linalg.norm(yaxis)
        xaxis = np.cross(yaxis, zaxis)

        graspGroundFrame = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
        graspGroundFrame.PostMultiply()
        graspGroundFrame.Translate(graspPosition[0], graspPosition[1], groundHeight)

        position = [-0.67, -0.4, 0.0]
        rpy = [0, 0, 0]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(graspGroundFrame)

        self.graspStanceFrame = vis.updateFrame(t, 'grasp stance', parent=self.drillAffordance, visible=False, scale=0.2)


    def computeFootstepPlan(self):
        self.footstepPlan = self.footstepPlanner.sendFootstepPlanRequest(self.graspStanceFrame.transform, waitForResponse=True)


    def computeWalkingPlan(self):
        self.walkingPlan = self.footstepPlanner.sendWalkingPlanRequest(self.footstepPlan, waitForResponse=True)


    def computeEndPose(self):
        graspLinks = {
            'l_hand' : 'left_base_link',
            'r_hand' : 'right_base_link',
           }
        linkName = graspLinks[self.getEndEffectorLinkName()]
        startPose = self.getEstimatedRobotStatePose()
        self.endPosePlan = self.manipPlanner.sendEndPoseGoal(startPose, linkName, self.graspFrame.transform, waitForResponse=True)
        self.showEndPose()

    def showEndPose(self):
        endPose = robotstate.convertStateMessageToDrakePose(self.endPosePlan)
        self.showPoseFunction(endPose)

    def computePreGraspPose(self):

        goalPoseJoints = RobotPoseGUIWrapper.getPose('hose', '1 walking with hose', side=self.graspingHand)

        if self.planFromCurrentRobotState:
            startPose = self.getEstimatedRobotStatePose()
        else:
            planState = self.walkingPlan.plan[-1]
            startPose = robotstate.convertStateMessageToDrakePose(planState)

        graspPose = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.graspFrame, planTraj=False)

        endPose = self.ikPlanner.mergePostures(graspPose, goalPoseJoints)
        self.preGraspPlan = self.ikPlanner.computePostureGoal(startPose, endPose)

        #self.preGraspPlan = self.ikPlanner.planPostureGoal(startPose, graspPose, goalPoseJoints)


    def planPostureGoal(self, groupName, poseName, side=None):

        goalPoseJoints = RobotPoseGUIWrapper.getPose(groupName, poseName, side=side)
        startPose = self.getEstimatedRobotStatePose()
        endPose = self.ikPlanner.mergePostures(startPose, goalPoseJoints)
        self.posturePlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.planPlaybackFunction([self.posturePlan])


    def getEndEffectorLinkName(self):
        linkMap = {
                      'left' : 'l_hand',
                      'right': 'r_hand'
                  }
        return linkMap[self.graspingHand]

    def computeGraspPlan(self):

        linkName = self.getEndEffectorLinkName()

        if self.planFromCurrentRobotState:
            startPose = self.getEstimatedRobotStatePose()
        else:
            planState = self.preGraspPlan.plan[-1]
            startPose = robotstate.convertStateMessageToDrakePose(planState)

        self.graspPlan = self.ikPlanner.planEndEffectorGoal(startPose, self.graspingHand, self.graspFrame, lockTorso=True)



    def reach(self):
        self.computeGraspFrame()
        self.ikPlanner.newReachGoal(self.graspFrame, lockTorso=False)


    def moveDrillTip(self):
        self.computeDrillTipFrame()
        tipToWorld = om.findObjectByName('drill tip frame')
        handLinkToWorld = self.robotModel.getLinkFrame('l_hand')

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(tipToWorld.transform)
        t.Concatenate(handLinkToWorld.GetLinearInverse())
        tipToHandLink = ikplanner.copyFrame(t)

        self.ikPlanner.newReachGoal(linkConstraintFrame=tipToHandLink, lockTorso=False)


    def commitFootstepPlan(self):
        self.footstepPlanner.commitFootstepPlan(self.footstepPlan)

    def commitPreGraspPlan(self):
        self.manipPlanner.commitManipPlan(self.preGraspPlan)

    def commitGraspPlan(self):
        self.manipPlanner.commitManipPlan(self.graspPlan)

    def commitStandPlan(self):
        self.manipPlanner.commitManipPlan(self.standPlan)

    def sendPelvisCrouch(self):
        self.atlasDriver.sendPelvisHeightCommand(0.7)

    def sendPelvisStand(self):
        self.atlasDriver.sendPelvisHeightCommand(0.8)

    def computeStandPlan(self):
        self.standPlan = self.ikPlanner.computeStand()

    def sendOpenHand(self):
        self.handDriver.sendOpen()

    def sendCloseHand(self):
        self.handDriver.sendClose(60)

    def sendNeckPitchLookDown(self):
        self.multisenseDriver.setNeckPitch(40)

    def sendNeckPitchLookForward(self):
        self.multisenseDriver.setNeckPitch(15)


    def waitForAtlasBehaviorAsync(self, behaviorName):
        assert behaviorName in self.atlasDriver.getBehaviorMap().values()
        while self.atlasDriver.getCurrentBehaviorName() != behaviorName:
            yield


    def printAsync(self, s):
        yield
        print s


    def userPrompt(self, message):

        if not self.userPromptEnabled:
            return

        yield
        result = raw_input(message)
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


    def spawnDrillAffordance(self):

        drillFrame = self.computeDrillFrame(self.robotModel)

        folder = om.getOrCreateContainer('affordances')
        drillMesh = segmentation.getDrillBarrelMesh()
        self.drillAffordance = vis.showPolyData(drillMesh, 'drill', color=[0.0, 1.0, 0.0], cls=vis.AffordanceItem, parent=folder)
        self.drillAffordance.actor.SetUserTransform(drillFrame)
        self.drillFrame = vis.showFrame(drillFrame, 'drill frame', parent=self.drillAffordance, visible=False, scale=0.2)

        self.computeGraspFrame()
        self.computeStanceFrame()
        self.computeDrillTipFrame()

    def findDrillAffordance(self):
        self.drillAffordance = om.findObjectByName('drill')
        self.drillFrame = om.findObjectByName('drill frame')

    def getEstimatedRobotStatePose(self):
        return self.sensorJointController.getPose('EST_ROBOT_STATE')

    def cleanupFootstepPlans(self):
        om.removeFromObjectModel(om.findObjectByName('walking goal'))
        om.removeFromObjectModel(om.findObjectByName('footstep plan'))

    def playNominalPlan(self):
        plans = [self.walkingPlan, self.preGraspPlan, self.graspPlan]
        assert None not in plans
        self.planPlaybackFunction(plans)

    def playPreGraspPlan(self):
        self.planPlaybackFunction([self.preGraspPlan])

    def playGraspPlan(self):
        self.planPlaybackFunction([self.graspPlan])


    def playStandPlan(self):
        self.planPlaybackFunction([self.standPlan])

    def computeNominalPlan(self):

        self.planFromCurrentRobotState = False

        self.findDrillAffordance()
        self.computeGraspFrame()
        self.computeStanceFrame()
        self.computeFootstepPlan()
        self.computeWalkingPlan()
        self.computePreGraspPose()
        self.computeGraspPlan()
        self.playNominalPlan()

    def sendPlanWithHeightMode(self):
        self.atlasDriver.sendPlanUsingBdiHeight(True)

    def autonomousExecute(self):

        self.planFromCurrentRobotState = True

        taskQueue = AsyncTaskQueue()

        # stand and open hand
        taskQueue.addTask(self.userPrompt('stand and open hand. continue? y/n: '))
        taskQueue.addTask(self.atlasDriver.sendStandCommand)
        taskQueue.addTask(self.sendOpenHand)
        taskQueue.addTask(self.sendPlanWithHeightMode)

        # user prompt
        taskQueue.addTask(self.userPrompt('sending neck pitch forward. continue? y/n: '))

        # set neck pitch
        taskQueue.addTask(self.printAsync('neck pitch forward'))
        taskQueue.addTask(self.sendNeckPitchLookForward)
        taskQueue.addTask(self.delay(1.0))

        # user prompt
        taskQueue.addTask(self.userPrompt('perception and fitting. continue? y/n: '))

        # perception & fitting
        taskQueue.addTask(self.printAsync('waiting for clean lidar sweep'))
        taskQueue.addTask(self.waitForCleanLidarSweepAsync)

        taskQueue.addTask(self.printAsync('fitting drill affordance'))
        taskQueue.addTask(self.affordanceFitFunction)
        taskQueue.addTask(self.findDrillAffordance)

        # compute grasp & stance
        taskQueue.addTask(self.printAsync('computing grasp and stance frames'))
        taskQueue.addTask(self.computeGraspFrame)
        taskQueue.addTask(self.computeStanceFrame)

        # footstep plan
        taskQueue.addTask(self.printAsync('compute footstep plan'))
        taskQueue.addTask(self.computeFootstepPlan)

        # user prompt
        taskQueue.addTask(self.userPrompt('sending footstep plan. continue? y/n: '))

        # walk
        taskQueue.addTask(self.printAsync('walking'))
        taskQueue.addTask(self.commitFootstepPlan)
        taskQueue.addTask(self.waitForAtlasBehaviorAsync('step'))
        taskQueue.addTask(self.waitForAtlasBehaviorAsync('stand'))

        # user prompt
        taskQueue.addTask(self.userPrompt('sending neck pitch. continue? y/n: '))

        # set neck pitch
        taskQueue.addTask(self.printAsync('neck pitch down'))
        taskQueue.addTask(self.sendNeckPitchLookDown)
        taskQueue.addTask(self.delay(1.0))

        # user prompt
        #taskQueue.addTask(self.userPrompt('crouch. continue? y/n: '))

        # crouch
        #taskQueue.addTask(self.printAsync('send manip mode'))
        #taskQueue.addTask(self.atlasDriver.sendManipCommand)
        #taskQueue.addTask(self.delay(1.0))
        #taskQueue.addTask(self.printAsync('crouching'))
        #taskQueue.addTask(self.sendPelvisCrouch)
        #taskQueue.addTask(self.delay(3.0))


        # user prompt
        taskQueue.addTask(self.userPrompt('plan pre grasp. continue? y/n: '))


        # compute pre grasp plan
        taskQueue.addTask(self.printAsync('computing pre grasp plan'))
        taskQueue.addTask(self.computePreGraspPose)
        taskQueue.addTask(self.playPreGraspPlan)

        # user prompt
        taskQueue.addTask(self.userPrompt('commit manip plan. continue? y/n: '))

        # commit pre grasp plan
        taskQueue.addTask(self.atlasDriver.sendManipCommand)
        taskQueue.addTask(self.delay(1.0))
        taskQueue.addTask(self.printAsync('commit pre grasp plan'))
        taskQueue.addTask(self.commitPreGraspPlan)
        taskQueue.addTask(self.delay(10.0))


        # user prompt
        taskQueue.addTask(self.userPrompt('perception and fitting. continue? y/n: '))

        # perception & fitting
        taskQueue.addTask(self.printAsync('waiting for clean lidar sweep'))
        taskQueue.addTask(self.waitForCleanLidarSweepAsync)

        taskQueue.addTask(self.printAsync('fitting drill affordance'))
        taskQueue.addTask(self.affordanceFitFunction)
        taskQueue.addTask(self.findDrillAffordance)

        # compute grasp frame
        taskQueue.addTask(self.printAsync('computing grasp frame'))
        taskQueue.addTask(self.computeGraspFrame)


        # compute grasp plan
        taskQueue.addTask(self.printAsync('computing grasp plan'))
        taskQueue.addTask(self.computeGraspPlan)
        taskQueue.addTask(self.playGraspPlan)

        # user prompt
        taskQueue.addTask(self.userPrompt('commit manip plan. continue? y/n: '))

        # commit grasp plan
        taskQueue.addTask(self.printAsync('commit grasp plan'))
        taskQueue.addTask(self.commitGraspPlan)
        taskQueue.addTask(self.delay(10.0))

        # recompute grasp plan
        taskQueue.addTask(self.printAsync('recompute grasp plan'))
        taskQueue.addTask(self.computeGraspPlan)
        taskQueue.addTask(self.playGraspPlan)

        # user prompt
        taskQueue.addTask(self.userPrompt('commit manip plan. continue? y/n: '))

        # commit grasp plan
        taskQueue.addTask(self.printAsync('commit grasp plan'))
        taskQueue.addTask(self.commitGraspPlan)
        taskQueue.addTask(self.delay(3.0))


        # user prompt
        taskQueue.addTask(self.userPrompt('closing hand. continue? y/n: '))

        # close hand
        taskQueue.addTask(self.printAsync('close hand'))
        taskQueue.addTask(self.sendCloseHand)
        taskQueue.addTask(self.delay(3.0))


        taskQueue.addTask(self.userPrompt('send stand command. continue? y/n: '))
        taskQueue.addTask(self.atlasDriver.sendStandCommand)
        taskQueue.addTask(self.delay(5.0))
        taskQueue.addTask(self.atlasDriver.sendManipCommand)
        taskQueue.addTask(self.delay(1.0))

        '''
        # user prompt
        taskQueue.addTask(self.userPrompt('compute stand plan. continue? y/n: '))

        # stand
        taskQueue.addTask(self.computeStandPlan)
        taskQueue.addTask(self.playStandPlan)

        taskQueue.addTask(self.userPrompt('commit stand. continue? y/n: '))

        # compute pre grasp plan
        taskQueue.addTask(self.commitStandPlan)
        taskQueue.addTask(self.delay(10.0))
        '''

        # user prompt
        #taskQueue.addTask(self.userPrompt('commit manip plan. continue? y/n: '))

        # commit pre grasp plan
        #taskQueue.addTask(self.printAsync('commit pre grasp plan'))
        #taskQueue.addTask(self.commitPreGraspPlan)
        #taskQueue.addTask(self.delay(10.0))

        taskQueue.addTask(self.printAsync('done!'))

        return taskQueue


