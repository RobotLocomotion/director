from ddapp import lcmUtils
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp.utime import getUtime
from ddapp import transformUtils
from ddapp import applogic as app

import numpy as np
import drc as lcmdrc


class FootstepsDriver(object):
    def __init__(self, jc):
        self.lastFootstepPlanMessage = None
        self._setupSubscriptions()
        self.jc = jc

    def _setupSubscriptions(self):
        lcmUtils.addSubscriber('FOOTSTEP_PLAN_RESPONSE', lcmdrc.footstep_plan_t, self.onFootstepPlan)

    def onFootstepPlan(self, msg):
        print "Got footstep plan with {:d} steps".format(msg.num_steps)

    def createWalkingGoal(self, model):
        distanceForward = 1.0

        t1 = model.getLinkFrame('l_foot')
        t2 = model.getLinkFrame('r_foot')
        pelvisT = model.getLinkFrame('pelvis')

        xaxis = [1.0, 0.0, 0.0]
        pelvisT.TransformVector(xaxis, xaxis)
        xaxis = np.array(xaxis)
        zaxis = np.array([0.0, 0.0, 1.0])
        yaxis = np.cross(zaxis, xaxis)
        xaxis = np.cross(yaxis, zaxis)

        stancePosition = np.array(t2.GetPosition()) + np.array(t1.GetPosition()) / 2.0

        footHeight=0.0817

        t = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
        t.PostMultiply()
        t.Translate(stancePosition)
        t.Translate([0.0, 0.0, -footHeight])
        t.Translate(xaxis*distanceForward)

        frameObj = vis.showFrame(t, 'walking goal')
        frameObj.setProperty('Edit', True)
        t.AddObserver('Modified', self.onWalkingGoalModified)
        print "created walking goal"

    def onWalkingGoalModified(self, transform, event):
        pos, wxyz = transformUtils.poseFromTransform(transform)
        print pos


    def sendFootstepPlanRequest(self):
        goalObj = om.findObjectByName('walking goal')
        assert goalObj

        msg = lcmdrc.footstep_plan_request_t()
        msg.utime = getUtime()
        pose = self.jc.getPose('q_end')
        state_msg = robotstate.drakePoseToRobotState(pose)
        msg.initial_state = state_msg

        msg.goal_pos = positionMessageFromFrame(goalObj.transform)
        msg.params = lcmdrc.footstep_plan_params_t()
        msg.params.max_num_steps = 100
        msg.params.min_num_steps = 0
        msg.params.min_step_width = 0.21
        msg.params.nom_step_width = 0.25
        msg.params.max_step_width = 0.4
        msg.params.nom_forward_step = 0.15
        msg.params.max_forward_step = 0.45
        msg.params.ignore_terrain = False
        msg.params.planning_mode = msg.MODE_AUTO
        msg.params.behavior = msg.BEHAVIOR_BDI_STEPPING
        msg.params.map_command = 2
        msg.params.leading_foot = msg.LEAD_AUTO

        msg.default_step_params = lcmdrc.footstep_params_t()
        msg.default_step_params.step_speed = 1.0
        msg.default_step_params.step_height = 0.05
        msg.default_step_params.bdi_step_duration = 2.0
        msg.default_step_params.bdi_sway_duration = 0.0
        msg.default_step_params.bdi_lift_height = 0.05
        msg.default_step_params.bdi_toe_off = 1
        msg.default_step_params.bdi_knee_nominal = 0.0
        msg.default_step_params.bdi_max_foot_vel = 0.0
        msg.default_step_params.bdi_sway_end_dist = 0.02
        msg.default_step_params.bdi_step_end_dist = 0.02
        msg.default_step_params.mu = 1.0

        lcmUtils.publish('FOOTSTEP_PLAN_REQUEST', msg)
        return msg


def init(jc):
    global driver
    driver = FootstepsDriver(jc)

