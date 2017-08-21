import os
from . import vtkAll as vtk
import math
import time
import numpy as np

from director import callbacks
from director import transformUtils
from director import lcmUtils
from director import objectmodel as om
from director.utime import getUtime
from director import robotstate

try:
    import robotlocomotion as lcmrl
    USE_DRC_MESSAGES = False
except ImportError:
    import drc as lcmdrc
    USE_DRC_MESSAGES = True


class ManipulationPlanDriver(object):

    PLAN_RECEIVED = 'PLAN_RECEIVED'
    PLAN_COMMITTED = 'PLAN_COMMITTED'
    USE_SUPPORTS = 'USE_SUPPORTS'

    def __init__(self, ikPlanner):

        if USE_DRC_MESSAGES:
            lcmUtils.addSubscriber('CANDIDATE_MANIP_PLAN', lcmdrc.robot_plan_w_keyframes_t, self.onManipPlan)
            lcmUtils.addSubscriber('CANDIDATE_ROBOT_PLAN_WITH_SUPPORTS',lcmdrc.robot_plan_with_supports_t, self.onManipPlan)
        else:
            lcmUtils.addSubscriber('CANDIDATE_MANIP_PLAN', lcmrl.robot_plan_t, self.onManipPlan)

        self.lastManipPlan = None
        self.committedPlans = []
        self.callbacks = callbacks.CallbackRegistry([self.PLAN_RECEIVED,
                                                     self.PLAN_COMMITTED,
                                                     self.USE_SUPPORTS])
        self.ikPlanner = ikPlanner
        self.publishPlansWithSupports = False
        self.plansWithSupportsAreQuasistatic = True
        self.leftFootSupportEnabled  = True
        self.rightFootSupportEnabled = True
        self.leftHandSupportEnabled  = False
        self.rightHandSupportEnabled = False
        self.pelvisSupportEnabled  = False

    def onManipPlan(self, msg):
        self.lastManipPlan = msg
        if USE_DRC_MESSAGES and self.ikPlanner.clipFloat32SafeJointLimits:
            self.lastManipPlan = self.ikPlanner.clipPlan(msg)
        self.callbacks.process(self.PLAN_RECEIVED, msg)


    def convertKeyframePlan(self, keyframeMsg):
        msg = lcmdrc.robot_plan_t()
        msg.utime = keyframeMsg.utime
        msg.robot_name = keyframeMsg.robot_name
        msg.num_states = keyframeMsg.num_states
        msg.plan = keyframeMsg.plan
        msg.plan_info = keyframeMsg.plan_info
        msg.num_bytes = keyframeMsg.num_bytes
        msg.matlab_data = keyframeMsg.matlab_data
        msg.num_grasp_transitions = keyframeMsg.num_grasp_transitions

        msg.left_arm_control_type = msg.NONE
        msg.right_arm_control_type = msg.NONE
        msg.left_leg_control_type = msg.NONE
        msg.right_leg_control_type = msg.NONE

        return msg

    def createPlanMessageFromPose(self, pose):
        msg = lcmdrc.robot_plan_t()
        msg.utime = 0.0
        msg.robot_name = 'robot'
        msg.num_states = 2
        state1 = robotstate.drakePoseToRobotState(pose)
        state2 = robotstate.drakePoseToRobotState(pose)
        state1.utime = 0.0
        state2.utime = 0.01
        msg.plan = [state1, state2]
        msg.plan_info = [1, 1]
        msg.num_bytes = 0
        msg.matlab_data = []
        msg.num_grasp_transitions = 0

        msg.left_arm_control_type = msg.NONE
        msg.right_arm_control_type = msg.NONE
        msg.left_leg_control_type = msg.NONE
        msg.right_leg_control_type = msg.NONE

        return msg

    def convertPlanToPlanWithSupports(self, planMsg, supports, ts, isQuasistatic):
        assert(len(supports) == len(ts))
        msg = lcmdrc.robot_plan_with_supports_t()
        msg.utime = planMsg.utime
        msg.plan = planMsg
        msg.support_sequence.utime = planMsg.utime
        msg.support_sequence.num_ts = len(ts)
        msg.support_sequence.ts = ts
        msg.support_sequence.supports = supports
        msg.is_quasistatic = isQuasistatic

        return msg


    def getSupportLCMFromListOfSupports(self, supportsList,ts):
        supports = [0]*len(ts)
        for i in range(len(ts)):
            supportElement = lcmdrc.support_element_t()
            supportElement.utime = getUtime()
            numBodies = len(supportsList[i])
            supportElement.num_bodies = numBodies
            supportBodies = []
            for j in range(numBodies):
                name = supportsList[i][j]
                if name is 'l_foot':
                    supportBodies.append(self.getFootSupportBodyMsg('left'))
                elif name is 'r_foot':
                    supportBodies.append(self.getFootSupportBodyMsg('right'))
                elif name is 'l_hand':
                    supportBodies.append(self.getHandSupportBodyMsg('left'))
                elif name is 'r_hand':
                    supportBodies.append(self.getHandSupportBodyMsg('right'))
                elif name is 'pelvis':
                    supportBodies.append(self.getPelvisSupportBodyMsg())
                else:
                    print("passed a support that isn't allowed")

            supportElement.support_bodies = supportBodies
            supports[i] = supportElement

        return supports



    def getSupports(self):
        if self.publishPlansWithSupports:
            supportElement = lcmdrc.support_element_t()
            supportElement.utime = getUtime()
            numBodies = 0
            supportBodies = []
            if self.pelvisSupportEnabled:
                numBodies += 1
                supportBodies.append(self.getPelvisSupportBodyMsg())
            if self.leftFootSupportEnabled:
                numBodies += 1
                supportBodies.append(self.getFootSupportBodyMsg('left'))
            if self.rightFootSupportEnabled:
                numBodies += 1
                supportBodies.append(self.getFootSupportBodyMsg('right'))
            if self.leftHandSupportEnabled:
                #numBodies += 2
                numBodies += 1
                supportBodies.append(self.getHandSupportBodyMsg('left', [0, 0, 1, 0]))
                #supportBodies.append(self.getHandSupportBodyMsg('left', [0, 0, -1, 0]))
            if self.rightHandSupportEnabled:
                #numBodies += 2
                numBodies += 1
                supportBodies.append(self.getHandSupportBodyMsg('right', [0, 0, 1, 0]))
                #supportBodies.append(self.getHandSupportBodyMsg('right', [0, 0, -1, 0]))
            supportElement.num_bodies = numBodies
            supportElement.support_bodies = supportBodies
            return [supportElement]

    def getPelvisSupportBodyMsg(self):
        linkname = 'pelvis'
        supportBody = lcmdrc.support_body_t()
        supportBody.utime = getUtime()
        supportBody.body_id = int(self.ikPlanner.ikServer.comm.getFloatArray('links.%s' % linkname)[0])
        supportBody.contact_pts = np.array(self.ikPlanner.ikServer.comm.getFloatArray('pelvis_pts(:)')).reshape((3,-1), order='F')
        supportBody.num_contact_pts = len(supportBody.contact_pts[0])
        supportBody.use_support_surface = False
        supportBody.override_contact_pts = True
        return supportBody

    def getFootSupportBodyMsg(self, side):
        linkname = 'l_foot' if side == 'left' else 'r_foot'
        supportBody = lcmdrc.support_body_t()
        supportBody.utime = getUtime()
        supportBody.body_id = int(self.ikPlanner.ikServer.comm.getFloatArray('links.%s' % linkname)[0])
        supportBody.contact_pts = np.array(self.ikPlanner.ikServer.comm.getFloatArray('%s_pts(:)' % linkname)).reshape((3,-1), order='F')
        supportBody.num_contact_pts = len(supportBody.contact_pts[0])
        supportBody.use_support_surface = False
        supportBody.override_contact_pts = True
        return supportBody

    def getHandSupportBodyMsg(self, side, support_surface):
        linkname = 'l_hand' if side == 'left' else 'r_hand'
        supportBody = lcmdrc.support_body_t()
        supportBody.utime = getUtime()
        supportBody.body_id = int(self.ikPlanner.ikServer.comm.getFloatArray('links.%s' % linkname)[0])
        supportBody.contact_pts = self.ikPlanner.getPalmPoint(side=side).reshape(3,1)
        supportBody.num_contact_pts = 1
        supportBody.use_support_surface = True
        supportBody.override_contact_pts = True
        supportBody.support_surface = support_surface
        return supportBody

    def setPublishPlansWithSupports(self, useSupports):
       self.publishPlansWithSupports = useSupports
       self.callbacks.process(self.USE_SUPPORTS)

    def commitManipPlan(self, manipPlan):

        for previousPlan in self.committedPlans:
            if previousPlan.utime == manipPlan.utime:
                raise Exception('Refusing to re-commit manipulation plan.')

        self.committedPlans.append(manipPlan)

        channelMap = {}

        if USE_DRC_MESSAGES:
            channelMap[lcmdrc.robot_plan_with_supports_t] = 'COMMITTED_ROBOT_PLAN_WITH_SUPPORTS'
            if isinstance(manipPlan, lcmdrc.robot_plan_w_keyframes_t):
                manipPlan = self.convertKeyframePlan(manipPlan)
                supports = self.getSupports()
                if supports is not None:
                    manipPlan = self.convertPlanToPlanWithSupports(manipPlan, supports, [0.0],
                                                                   self.plansWithSupportsAreQuasistatic)
        manipPlan.utime = getUtime()

        defaultChannel = 'COMMITTED_ROBOT_PLAN'
        channel = channelMap.get(type(manipPlan), defaultChannel)
        lcmUtils.publish(channel, manipPlan)
        self.callbacks.process(self.PLAN_COMMITTED, manipPlan)


    def sendPlanPause(self):
        msg = lcmdrc.plan_control_t()
        msg.utime = getUtime()
        msg.control = lcmdrc.plan_control_t.PAUSE
        lcmUtils.publish('COMMITTED_PLAN_PAUSE', msg)

    def connectPlanReceived(self, func):
        return self.callbacks.connect(self.PLAN_RECEIVED, func)

    def connectUseSupports(self, func):
        return self.callbacks.connect(self.USE_SUPPORTS, func)

    def disconnectPlanReceived(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def connectPlanCommitted(self, func):
        return self.callbacks.connect(self.PLAN_COMMITTED, func)

    def disconnectPlanCommitted(self, callbackId):
        self.callbacks.disconnect(callbackId)

