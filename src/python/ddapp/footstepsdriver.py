from ddapp import lcmUtils
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp.utime import getUtime
from ddapp import transformUtils
from ddapp.debugVis import DebugData
from ddapp import ioUtils
from ddapp import robotstate
from ddapp import applogic as app
from ddapp import vtkAll as vtk
from ddapp.simpletimer import SimpleTimer

import os
import numpy as np
import drc as lcmdrc
import functools


def loadFootMeshes():
    meshDir = os.path.join(app.getDRCBase(), 'software/models/mit_gazebo_models/mit_robot/meshes')
    meshes = []
    for foot in ['l', 'r']:
        d = DebugData()
        d.addPolyData(ioUtils.readPolyData(os.path.join(meshDir, '%s_talus.stl' % foot), computeNormals=True))
        d.addPolyData(ioUtils.readPolyData(os.path.join(meshDir, '%s_foot.stl' % foot), computeNormals=True))
        meshes.append(d.getPolyData())
    return meshes


def getLeftFootMesh():
    return getFootMeshes()[0]


def getRightFootMesh():
    return getFootMeshes()[1]


def getLeftFootColor():
    return [1.0, 1.0, 0.0]


def getRightFootColor():
    return [0.33, 1.0, 0.0]

_footMeshes = None


def getDefaultStepParams():
    default_step_params = lcmdrc.footstep_params_t()
    default_step_params.step_speed = 1.0
    default_step_params.step_height = 0.05
    default_step_params.constrain_full_foot_pose = False
    default_step_params.bdi_step_duration = 2.0
    default_step_params.bdi_sway_duration = 0.0
    default_step_params.bdi_lift_height = 0.05
    default_step_params.bdi_toe_off = 1
    default_step_params.bdi_knee_nominal = 0.0
    default_step_params.bdi_max_foot_vel = 0.0
    default_step_params.bdi_sway_end_dist = 0.02
    default_step_params.bdi_step_end_dist = 0.02
    default_step_params.mu = 1.0
    return default_step_params


def getFootMeshes():
    global _footMeshes
    if not _footMeshes:
        _footMeshes = loadFootMeshes()
    return _footMeshes


def getFootstepsFolder():
    obj = om.findObjectByName('footstep plan')
    if obj is None:
        obj = om.getOrCreateContainer('footstep plan')
        #om.collapse(obj)
    return obj


class FootstepsDriver(object):
    def __init__(self, jc):
        self.lastFootstepPlan = None
        self.lastFootstepRequest = None
        self.goalSteps = None
        self._setupSubscriptions()
        self.jointController = jc
        self.lastWalkingPlan = None
        self.walkingPlanCallback = None


    def _setupSubscriptions(self):
        lcmUtils.addSubscriber('FOOTSTEP_PLAN_RESPONSE', lcmdrc.footstep_plan_t, self.onFootstepPlan)
        lcmUtils.addSubscriber('WALKING_TRAJ_RESPONSE', lcmdrc.robot_plan_t, self.onWalkingPlan)

    def clearFootstepPlan(self):
        self.lastFootstepPlan = None
        folder = getFootstepsFolder()
        om.removeFromObjectModel(folder)

    def onWalkingPlan(self, msg):
        self.lastWalkingPlan = msg
        if self.walkingPlanCallback:
            self.walkingPlanCallback()

    def onFootstepPlan(self, msg):
        self.clearFootstepPlan()
        self.lastFootstepPlan = msg

        planFolder = getFootstepsFolder()

        allTransforms = []
        for i, footstep in enumerate(msg.footsteps):
            trans = footstep.pos.translation
            trans = [trans.x, trans.y, trans.z]
            quat = footstep.pos.rotation
            quat = [quat.w, quat.x, quat.y, quat.z]
            footstepTransform = transformUtils.transformFromPose(trans, quat)
            allTransforms.append(footstepTransform)

            if i < 2:
                continue

            if footstep.is_right_foot:
                mesh = getRightFootMesh()
                color = getRightFootColor()
            else:
                mesh = getLeftFootMesh()
                color = getLeftFootColor()
            if footstep.infeasibility > 1e-6:
                d = DebugData()
                # normal = np.array(allTransforms[i-1].GetPosition()) - np.array(footstepTransform.GetPosition())
                # normal = normal / np.linalg.norm(normal)
                start = allTransforms[i-1].GetPosition()
                end = footstepTransform.GetPosition()
                d.addArrow(start, end, 0.02, 0.005,
                           startHead=True,
                           endHead=True)
                # d.addCone(start, normal, 0.02, 0.02)
                # d.addCone(end, -normal, 0.02, 0.02)
                # d.addLine(start, end,radius=0.005)
                vis.showPolyData(d.getPolyData(), 'infeasibility %d -> %d' % (i-2, i-1), parent=planFolder, color=[1, 0.2, 0.2])


            obj = vis.showPolyData(mesh, 'step %d' % (i-1), color=color, alpha=1.0, parent=planFolder)
            frameObj = vis.showFrame(footstepTransform, 'frame', parent=obj, scale=0.3, visible=False)
            frameObj.onTransformModifiedCallback = functools.partial(self.onStepModified, i-2)
            obj.actor.SetUserTransform(footstepTransform)

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

        stancePosition = (np.array(t2.GetPosition()) + np.array(t1.GetPosition())) / 2.0

        footHeight = 0.0817

        t = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
        t.PostMultiply()
        t.Translate(stancePosition)
        t.Translate([0.0, 0.0, -footHeight])
        t.Translate(xaxis*distanceForward)

        frameObj = vis.showFrame(t, 'walking goal')
        frameObj.setProperty('Edit', True)

        frameObj.onTransformModifiedCallback = self.onWalkingGoalModified
        self.sendFootstepPlanRequest(t)

    def createGoalSteps(self, model):
        distanceForward = 1.0

        fr = model.getLinkFrame('l_foot')
        fl = model.getLinkFrame('r_foot')
        pelvisT = model.getLinkFrame('pelvis')

        xaxis = [1.0, 0.0, 0.0]
        pelvisT.TransformVector(xaxis, xaxis)
        xaxis = np.array(xaxis)
        zaxis = np.array([0.0, 0.0, 1.0])
        yaxis = np.cross(zaxis, xaxis)
        xaxis = np.cross(yaxis, zaxis)

        numGoalSteps = 3
        is_right_foot = True
        self.goalSteps = []
        for i in range(numGoalSteps):
            t = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
            t.PostMultiply()
            if is_right_foot:
                t.Translate(fr.GetPosition())
            else:
                t.Translate(fl.GetPosition())
            t.Translate(xaxis*distanceForward)
            distanceForward += 0.15
            is_right_foot = not is_right_foot
            step = lcmdrc.footstep_t()
            step.pos = transformUtils.positionMessageFromFrame(t)
            step.is_right_foot = is_right_foot
            step.params = getDefaultStepParams()
            self.goalSteps.append(step)
        request = self.constructFootstepPlanRequest()
        request.num_goal_steps = len(self.goalSteps)
        request.goal_steps = self.goalSteps
        self.lastFootstepRequest = request
        lcmUtils.publish('FOOTSTEP_PLAN_REQUEST', request)
        return request

    def onStepModified(self, ndx, frameObj):
        self.lastFootstepPlan.footsteps[ndx+2].pos = transformUtils.positionMessageFromFrame(frameObj.transform)
        self.lastFootstepPlan.footsteps[ndx+2].fixed_x = True
        self.lastFootstepPlan.footsteps[ndx+2].fixed_y = True
        self.lastFootstepPlan.footsteps[ndx+2].fixed_yaw = True
        self.sendUpdatePlanRequest()

    def sendUpdatePlanRequest(self):
        msg = self.lastFootstepRequest
        msg.num_existing_steps = self.lastFootstepPlan.num_steps
        msg.existing_steps = self.lastFootstepPlan.footsteps
        self.lastFootstepRequest = msg
        lcmUtils.publish('FOOTSTEP_PLAN_REQUEST', msg)
        return msg

    def onWalkingGoalModified(self, frameObj):
        self.sendFootstepPlanRequest(frameObj.transform)

    def constructFootstepPlanRequest(self, goalFrame=None):

        msg = lcmdrc.footstep_plan_request_t()
        msg.utime = getUtime()
        pose = self.jointController.getPose(self.jointController.currentPoseName)
        state_msg = robotstate.drakePoseToRobotState(pose)
        msg.initial_state = state_msg

        if goalFrame is None:
            goalFrame = vtk.vtkTransform()
        msg.goal_pos = transformUtils.positionMessageFromFrame(goalFrame)

        msg.params = lcmdrc.footstep_plan_params_t()
        msg.params.max_num_steps = 30
        msg.params.min_num_steps = 0
        msg.params.min_step_width = 0.21
        msg.params.nom_step_width = 0.25
        msg.params.max_step_width = 0.4
        msg.params.nom_forward_step = 0.15
        msg.params.max_forward_step = 0.45
        msg.params.ignore_terrain = True
        msg.params.planning_mode = msg.params.MODE_AUTO
        msg.params.behavior = msg.params.BEHAVIOR_BDI_STEPPING
        msg.params.map_command = 2
        msg.params.leading_foot = msg.params.LEAD_AUTO
        msg.default_step_params = getDefaultStepParams()

        return msg

    def sendFootstepPlanRequest(self, goalFrame, waitForResponse=False, waitTimeout=5000):

        request = self.constructFootstepPlanRequest(goalFrame)
        self.lastFootstepRequest = request

        requestChannel = 'FOOTSTEP_PLAN_REQUEST'
        responseChannel = 'FOOTSTEP_PLAN_RESPONSE'

        if waitForResponse:
            if waitTimeout == 0:
                helper = lcmUtils.MessageResponseHelper(responseChannel, lcmdrc.footstep_plan_t)
                lcmUtils.publish(requestChannel, request)
                return helper
            return lcmUtils.MessageResponseHelper.publishAndWait(requestChannel, request,
                                                                 responseChannel, lcmdrc.footstep_plan_t, waitTimeout)
        else:
            lcmUtils.publish(requestChannel, request)

    def sendWalkingPlanRequest(self, footstepPlan, waitForResponse=False, waitTimeout=5000):

        msg = lcmdrc.walking_plan_request_t()
        msg.utime = getUtime()
        pose = self.jointController.getPose(self.jointController.currentPoseName)
        state_msg = robotstate.drakePoseToRobotState(pose)
        msg.initial_state = state_msg
        msg.new_nominal_state = msg.initial_state
        msg.use_new_nominal_state = False
        msg.footstep_plan = footstepPlan

        requestChannel = 'WALKING_TRAJ_REQUEST'
        responseChannel = 'WALKING_TRAJ_RESPONSE'

        if waitForResponse:
            if waitTimeout == 0:
                helper = lcmUtils.MessageResponseHelper(responseChannel, lcmdrc.robot_plan_t)
                lcmUtils.publish(requestChannel, msg)
                return helper
            return lcmUtils.MessageResponseHelper.publishAndWait(requestChannel, msg,
                                                                 responseChannel, lcmdrc.robot_plan_t, waitTimeout)
        else:
            lcmUtils.publish(requestChannel, msg)

    def sendStopWalking(self):
        msg = lcmdrc.plan_control_t()
        msg.utime = getUtime()
        msg.control = lcmdrc.plan_control_t.TERMINATE
        lcmUtils.publish('STOP_WALKING', msg)

    def commitFootstepPlan(self, footstepPlan):
        footstepPlan.utime = getUtime()
        lcmUtils.publish('COMMITTED_FOOTSTEP_PLAN', footstepPlan)
