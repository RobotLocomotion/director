import os
import vtkAll as vtk
import math
import time
import numpy as np

from ddapp import callbacks
from ddapp import transformUtils
from ddapp import lcmUtils
from ddapp import objectmodel as om
from ddapp.utime import getUtime
from ddapp import robotstate

import drc as lcmdrc


class ManipulationPlanDriver(object):

    PLAN_RECEIVED = 'PLAN_RECEIVED'
    PLAN_COMMITTED = 'PLAN_COMMITTED'

    def __init__(self):
        lcmUtils.addSubscriber('CANDIDATE_MANIP_PLAN', lcmdrc.robot_plan_w_keyframes_t, self.onManipPlan)
        self.lastManipPlan = None
        self.committedPlans = []
        self.callbacks = callbacks.CallbackRegistry([self.PLAN_RECEIVED,
                                                     self.PLAN_COMMITTED])

    def onManipPlan(self, msg):
        self.lastManipPlan = msg
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

    def commitManipPlan(self, manipPlan):

        if manipPlan in self.committedPlans:
            raise Exception('Refusing to re-commit manipulation plan.')
        self.committedPlans.append(manipPlan)

        if isinstance(manipPlan, lcmdrc.robot_plan_w_keyframes_t):
            manipPlan = self.convertKeyframePlan(manipPlan)
        manipPlan.utime = getUtime()
        lcmUtils.publish('COMMITTED_ROBOT_PLAN', manipPlan)
        self.callbacks.process(self.PLAN_COMMITTED, manipPlan)


    def sendPlanPause(self):
        msg = lcmdrc.plan_control_t()
        msg.utime = getUtime()
        msg.control = lcmdrc.plan_control_t.PAUSE
        lcmUtils.publish('COMMITTED_PLAN_PAUSE', msg)

    def connectPlanReceived(self, func):
        return self.callbacks.connect(self.PLAN_RECEIVED, func)

    def disconnectPlanReceived(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def connectPlanCommitted(self, func):
        return self.callbacks.connect(self.PLAN_COMMITTED, func)

    def disconnectPlanCommitted(self, callbackId):
        self.callbacks.disconnect(callbackId)

