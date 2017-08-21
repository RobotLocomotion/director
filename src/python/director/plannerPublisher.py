 # for proper float division

import os
import sys
import math
import time
import types
import functools
import random
import json
from collections import OrderedDict
import numpy as np

from director import ikconstraints
from director import ikconstraintencoder
from director import ikparameters
from director.fieldcontainer import FieldContainer
from director.utime import getUtime
from director import lcmUtils
from director import transformUtils


class PlannerPublisher(object):

    def __init__(self, ikPlanner, affordanceManager):

        self.ikPlanner = ikPlanner
        self.affordanceManager = affordanceManager
        self.jointNames = list(self.ikPlanner.jointController.jointNames)
        self.jointLimits = {}
        self.poses = {}

        for jointName in self.jointNames:
            self.jointLimits[jointName] = list(self.ikPlanner.robotModel.model.getJointLimits(jointName))

        self._setup()

    def _setup(self):
        pass

    def updateJointLimits(self, limitData):
        for jointName, epsilon, jointPosition in limitData:
            if epsilon < 0:
                self.jointLimits[jointName][0]=jointPosition
            else:
                self.jointLimits[jointName][1]=jointPosition


    def setupFields(self, constraints, ikParameters, positionCosts, nominalPoseName="", seedPoseName="", endPoseName=""):

        poses = ikconstraintencoder.getPlanPoses(constraints, self.ikPlanner)
        poses.update(self.poses)
        poses['q_nom'] = list(self.ikPlanner.jointController.getPose('q_nom'))

        fields = FieldContainer(
            utime = getUtime(),
            poses = poses,
            constraints = constraints,
            seedPose = seedPoseName,
            nominalPose = nominalPoseName,
            endPose = endPoseName,
            jointNames = self.jointNames,
            jointLimits = self.jointLimits,
            positionCosts = positionCosts,
            affordances = self.processAffordances(),
            options = ikParameters,
            )

        return fields

    def processIK(self, constraints, ikParameters, positionCosts, nominalPoseName="", seedPoseName=""):
        raise Exception('not implemented')

    def processTraj(self, constraints, ikParameters, positionCosts, nominalPoseName="", seedPoseName="", endPoseName=""):
        raise Exception('not implemented')

    def processAddPose(self, pose, poseName):
        self.poses[poseName] = list(pose)

    def processAffordances(self):
        affs = self.affordanceManager.getCollisionAffordances()
        s='['
        first=True
        for aff in affs:
            des=aff.getDescription()
            classname=des['classname'];
            if first:
                s+='{'
            else:
                s+='\n,{'
            first=False
            s+='"classname":"'+classname+'"'
            s+=',"name":"'+des['Name']+'"'
            s+=',"uuid":"'+des['uuid']+'"'
            s+=',"pose": {"position":{"__ndarray__":'+repr(des['pose'][0].tolist())+'},"quaternion":{"__ndarray__":'+repr(des['pose'][1].tolist())+'}}'
            if self.affordanceManager.affordanceUpdater is not None: # attached collision object / frameSync
                if des['Name'] in self.affordanceManager.affordanceUpdater.attachedAffordances:
                    s+=',"attachedTo":"'+self.affordanceManager.affordanceUpdater.attachedAffordances[des['Name']]+'"'
                else: # it's not attached
                    s+=',"attachedTo":"__world__"' # __world__ means it's a fixed collision object (sometimes called world or map - we use __world__ here)
            else: # no affordanceUpdater - so no attached collision objects either
                s+=',"attachedTo":"__world__"'
            if classname=='MeshAffordanceItem':
                if aff.getMeshManager().getFilesystemFilename(des['Filename']):
                    s+=',"filename":"'+aff.getMeshManager().getFilesystemFilename(des['Filename'])+'"'
                else:
                    if os.path.isfile(des['Filename']):
                        s+=',"filename":"'+des['Filename']+'"'
                    else:
                        raise Exception("Mesh does not exist.")
                s+=',"scale":'+repr(des['Scale'])
            if classname=='SphereAffordanceItem':
                s+=',"radius":'+repr(des['Radius'])
            if classname=='CylinderAffordanceItem' or classname=='CapsuleAffordanceItem':
                s+=',"radius":'+repr(des['Radius'])
                s+=',"length":'+repr(des['Length'])
            if classname=='BoxAffordanceItem':
                s+=',"dimensions":'+repr(des['Dimensions'])
            if classname=='CapsuleRingAffordanceItem':
                s+=',"radius":'+repr(des['Radius'])
                s+=',"tube_radius":'+repr(des['Tube Radius'])
                s+=',"segments":'+repr(des['Segments'])
            s+='}'
        s=s+']'
        return s


class DummyPlannerPublisher(PlannerPublisher):
    pass


class MatlabDrakePlannerPublisher(PlannerPublisher):


    def processAddPose(self, pose, poseName):
        super(MatlabDrakePlannerPublisher, self).processAddPose(pose, poseName)

        self.ikServer.sendPoseToServer(pose, poseName)

    def processIK(self, constraints, ikParameters, positionCosts, nominalPoseName="", seedPoseName=""):

        endPose, info = self.ikServer.runIk(constraints, ikParameters, nominalPostureName=nominalPoseName, seedPostureName=seedPoseName)

        return endPose, info

    def processTraj(self, constraints, ikParameters, positionCosts, nominalPoseName="", seedPoseName="", endPoseName=""):

        listener = self.ikPlanner.getManipPlanListener()
        info = self.ikServer.runIkTraj(constraints, poseStart=seedPoseName, poseEnd=endPoseName, nominalPose=nominalPoseName, ikParameters=ikParameters, additionalTimeSamples=self.ikPlanner.additionalTimeSamples, graspToHandLinkFrame=self.ikPlanner.newGraspToHandFrame(ikParameters.rrtHand))
        plan = listener.waitForResponse(timeout=12000)
        listener.finish()

        return plan, info


class ExoticaPlannerPublisher(PlannerPublisher):

  def setupMessage(self, fields):

    # todo, exotica should migrate to new style option names.
    # for backward compatibility, override the options field here
    # and insert jointLimits to options list
    options = OrderedDict(self.ikPlanner.getIkOptions()._properties)
    options['jointLimits'] = fields.jointLimits

    import drc as lcmdrc

    msg = lcmdrc.exotica_planner_request_t()
    msg.utime = fields.utime
    msg.poses = json.dumps(fields.poses)
    msg.constraints = ikconstraintencoder.encodeConstraints(fields.constraints)
    msg.seed_pose = fields.seedPose
    msg.nominal_pose = fields.nominalPose
    msg.end_pose = fields.endPose
    msg.joint_names = json.dumps(fields.jointNames)
    msg.affordances = fields.affordances
    msg.options = json.dumps(options)

    return msg


  def processIK(self, constraints, ikParameters, positionCosts, nominalPoseName="", seedPoseName=""):


    fields = self.setupFields(constraints, ikParameters, positionCosts, nominalPoseName, seedPoseName)
    msg = self.setupMessage(fields)

    listener = self.ikPlanner.getManipIKListener()
    lcmUtils.publish('IK_REQUEST', msg)
    ikplan = listener.waitForResponse(timeout=12000)
    listener.finish()

    endPose = [0] * self.ikPlanner.jointController.numberOfJoints
    if ikplan.num_states>0:
      endPose[len(endPose)-len(ikplan.plan[ikplan.num_states-1].joint_position):] = ikplan.plan[ikplan.num_states-1].joint_position
      info=ikplan.plan_info[ikplan.num_states-1]
    else: 
      info = -1
    self.ikPlanner.ikServer.infoFunc(info)
    return endPose, info

  def processTraj(self, constraints, ikParameters, positionCosts, nominalPoseName="", seedPoseName="", endPoseName=""):

    # Temporary fix / HACK / TODO (should be done in exotica_json)
    largestTspan = [0, 0]
    for constraintIndex, _ in enumerate(constraints):
      # Get tspan extend to normalise time-span
      if np.isfinite(constraints[constraintIndex].tspan[0]) and np.isfinite(constraints[constraintIndex].tspan[1]):
        largestTspan[0] = constraints[constraintIndex].tspan[0] if (constraints[constraintIndex].tspan[0] < largestTspan[0]) else largestTspan[0]
        largestTspan[1] = constraints[constraintIndex].tspan[1] if (constraints[constraintIndex].tspan[1] > largestTspan[1]) else largestTspan[1]

    # Temporary fix / HACK/ TODO to normalise time spans
    for constraintIndex, _ in enumerate(constraints):
      if np.isfinite(constraints[constraintIndex].tspan[0]) and np.isfinite(constraints[constraintIndex].tspan[1]):
        if largestTspan[1] != 0:
          constraints[constraintIndex].tspan[0] = constraints[constraintIndex].tspan[0] / largestTspan[1]
          constraints[constraintIndex].tspan[1] = constraints[constraintIndex].tspan[1] / largestTspan[1]

    listener = self.ikPlanner.getManipPlanListener()

    fields = self.setupFields(constraints, ikParameters, positionCosts, nominalPoseName, seedPoseName, endPoseName)
    msg = self.setupMessage(fields)

    lcmUtils.publish('PLANNER_REQUEST', msg)
    lastManipPlan = listener.waitForResponse(timeout=20000)
    listener.finish()

    self.ikPlanner.ikServer.infoFunc(lastManipPlan.plan_info[0])
    return lastManipPlan, lastManipPlan.plan_info[0]
