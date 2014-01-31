import os
import vtkAll as vtk
from ddapp import botpy
import math
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
from ddapp import segmentation


import drc as lcmdrc



class PlanSequence(object):


    def __init__(self, robotModel, footstepPlanner, manipPlanner):
        self.robotModel = robotModel
        self.footstepPlanner = footstepPlanner
        self.manipPlanner = manipPlanner
        self.graspingHand = 'left'



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
        xaxis = np.cross(yaxis, zaxis)

        stancePosition = (np.array(t2.GetPosition()) + np.array(t1.GetPosition())) / 2.0

        footHeight = 0.0811

        t = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
        t.PostMultiply()
        t.Translate(stancePosition)
        t.Translate([0.0, 0.0, -footHeight])

        return t


    def computeDrillFrame(self, robotModel):

        position = [1.5, 0.0, 1.2]
        rpy = [0, 0, -90]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.computeGroundFrame(robotModel))
        return t


    def computeGraspFrame(self):

        assert self.drillAffordance

        position = [0.0,-0.13,0.0]
        rpy = [-90,90,0]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.drillFrame.transform)

        self.graspFrame = vis.updateFrame(t, 'grasp frame', parent=self.drillAffordance, visible=False, scale=0.3)


    def computeStanceFrame(self):


        graspFrame = self.graspFrame.transform

        groundFrame = self.computeGroundFrame(self.robotModel)
        groundHeight = groundFrame.GetPosition()[2]

        graspPosition = np.array(graspFrame.GetPosition())
        graspZAxis = [0.0, 0.0, 1.0]
        graspFrame.TransformVector(graspZAxis, graspZAxis)

        xaxis = graspZAxis
        zaxis = [0, 0, 1]
        yaxis = np.cross(zaxis, xaxis)
        xaxis = np.cross(yaxis, zaxis)

        graspGroundFrame = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
        graspGroundFrame.PostMultiply()
        graspGroundFrame.Translate(graspPosition[0], graspPosition[1], groundHeight)

        position = [-0.57, -0.4, 0.0]
        rpy = [0, 0, 0]

        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(graspGroundFrame)

        self.graspStanceFrame = vis.updateFrame(t, 'grasp stance', parent=self.drillAffordance, visible=False, scale=0.3)



    def computeGraspPlan(self):

        linkMap = {
                      'left' : 'l_hand',
                      'right': 'r_hand'
                   }

        linkName = linkMap[self.graspingHand]

        self.manipPlanner.sendEndEffectorGoal('l_hand', self.graspFrame.transform)


    def computeFootstepPlan(self):
        self.footstepPlanner.sendFootstepPlanRequest(self.graspStanceFrame)


    def spawnDrillAffordance(self):

        drillFrame = self.computeDrillFrame(self.robotModel)

        folder = om.getOrCreateContainer('affordances')
        drillMesh = segmentation.getDrillMesh()
        self.drillAffordance = vis.showPolyData(drillMesh, 'drill', color=[1.0, 1.0, 0.0], parent=folder)
        self.drillAffordance.actor.SetUserTransform(drillFrame)
        self.drillFrame = vis.showFrame(drillFrame, 'drill frame', parent=self.drillAffordance, visible=True, scale=0.2)

        self.computeGraspFrame()
        self.computeStanceFrame()


    def sendEstRobotState(self):
        pose = self.footstepPlanner.jc.getPose(self.footstepPlanner.jc.currentPoseName)
        msg = robotstate.drakePoseToRobotState(pose)
        lcmUtils.publish('EST_ROBOT_STATE', msg)


    def cleanupFootstepPlans(self):
        om.removeFromObjectModel(om.findObjectByName('walking goal'))
        om.removeFromObjectModel(om.findObjectByName('footstep plan'))


    def plan(self):

        while True:
            #print 'reset'
            #self.cleanupFootstepPlans()
            #self.footstepPlanner.jc.setNominalPose()
            #self.sendEstRobotState()
            #yield

            print 'computed footsteps'
            self.computeGraspFrame()
            self.computeStanceFrame()
            self.computeFootstepPlan()
            yield

            print 'computed walking plan'
            self.footstepPlanner.sendWalkingPlanRequest()
            yield

            print 'computed grasp plan'
            self.cleanupFootstepPlans()
            self.computeGraspPlan()
            yield



