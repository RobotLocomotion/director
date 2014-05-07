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
from ddapp.shallowCopy import shallowCopy
from ddapp import roboturdf

import os
import numpy as np
from ddapp import botpy
import drc as lcmdrc
from bot_core.pose_t import pose_t
import functools

DEFAULT_PARAM_SET = 'BDI'
DEFAULT_STEP_PARAMS = {'BDI': {'Nominal Step Width': 0.27,
                               'Nominal Forward Step': 0.15,
                               'Max Forward Step': 0.40,
                               'Max Step Width': 0.4,
                               'Behavior': 0},
                       'drake': {'Nominal Step Width': 0.28,
                                 'Nominal Forward Step': 0.25,
                                 'Max Forward Step': 0.30,
                                 'Max Step Width': 0.32,
                                 'Behavior': 2}}

import ddapp.applogic as app

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
    return shallowCopy(getFootMeshes()[0])


def getRightFootMesh():
    return shallowCopy(getFootMeshes()[1])


def getLeftFootColor():
    return [1.0, 1.0, 0.0]


def getRightFootColor():
    return [0.33, 1.0, 0.0]


_footMeshes = None

def getFootMeshes():
    global _footMeshes
    if not _footMeshes:
        _footMeshes = loadFootMeshes()
    return _footMeshes


def getFootstepsFolder():
    obj = om.findObjectByName('footstep plan')
    if obj is None:
        obj = om.getOrCreateContainer('footstep plan')
        obj.setIcon(om.Icons.Feet)
        om.collapse(obj)
    return obj

def getBDIAdjustedFootstepsFolder():
    obj = om.findObjectByName('BDI adj footstep plan')
    if obj is None:
        obj = om.getOrCreateContainer('BDI adj footstep plan')
        obj.setIcon(om.Icons.Feet)
        om.collapse(obj)
    return obj

class FootstepsDriver(object):

    def __init__(self, jointController):
        self.jointController = jointController
        self.lastFootstepPlan = None
        self.lastFootstepRequest = None
        self.goalSteps = None
        self._setupSubscriptions()
        self.lastWalkingPlan = None
        self.walkingPlanCallback = None
        self.default_step_params = DEFAULT_STEP_PARAMS
        self._setupProperties()

        ### Stuff pertaining to rendering BDI-frame steps
        self.pose_bdi = None
        self.bdi_plan = None
        self.bdi_plan_adjusted = None

        view = app.getDRCView()
        self.bdiRobotModel, self.bdiJointController = roboturdf.loadRobotModel('bdi model', view, parent='bdi model', color=roboturdf.getRobotOrangeColor(), visible=False)
        self.bdiRobotModel.setProperty('Visible', False)
        self.showBDIPlan = False # hide the BDI plans when created

    def _setupProperties(self):
        self.params = om.ObjectModelItem('Footstep Params')
        self.params.addProperty('Behavior', 0, attributes=om.PropertyAttributes(enumNames=['BDI Stepping', 'BDI Walking', 'Drake Walking']))
        self.params.addProperty('Map Command', 0, attributes=om.PropertyAttributes(enumNames=['Full Heightmap', 'Flat Ground', 'Z Normals']))
        self.params.addProperty('Ignore Terrain', True)
        self.params.addProperty('Nominal Step Width', None, attributes=om.PropertyAttributes(decimals=2, minimum=0.21, maximum=0.4, singleStep=0.01))
        self.params.addProperty('Nominal Forward Step', None, attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=0.5, singleStep=0.01))
        self.params.addProperty('Max Step Width', None, attributes=om.PropertyAttributes(decimals=2, minimum=0.22, maximum=0.5, singleStep=0.01))
        self.params.addProperty('Max Forward Step', None, attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=0.5, singleStep=0.01))
        self.applyDefaults(DEFAULT_PARAM_SET)
        self.behavior_lcm_map = {
                              0: lcmdrc.footstep_plan_params_t.BEHAVIOR_BDI_STEPPING,
                              1: lcmdrc.footstep_plan_params_t.BEHAVIOR_BDI_WALKING,
                              2: lcmdrc.footstep_plan_params_t.BEHAVIOR_WALKING}

        self.map_command_lcm_map = {
                              0: lcmdrc.map_controller_command_t.FULL_HEIGHTMAP,
                              1: lcmdrc.map_controller_command_t.FLAT_GROUND,
                              2: lcmdrc.map_controller_command_t.Z_NORMALS}

    def applyDefaults(self, set_name):
        defaults = self.default_step_params[set_name]
        for k, v in defaults.iteritems():
            self.params.setProperty(k, v)

    def _setupSubscriptions(self):
        lcmUtils.addSubscriber('FOOTSTEP_PLAN_RESPONSE', lcmdrc.footstep_plan_t, self.onFootstepPlan)
        lcmUtils.addSubscriber('WALKING_TRAJ_RESPONSE', lcmdrc.robot_plan_t, self.onWalkingPlan)

        ### Related to BDI-frame adjustment:
        sub1 = lcmUtils.addSubscriber('POSE_BDI', pose_t, self.onPoseBDI)
        sub1.setSpeedLimit(60)
        sub2 = lcmUtils.addSubscriber('BDI_ADJUSTED_FOOTSTEP_PLAN', lcmdrc.footstep_plan_t, self.onBDIAdjustedFootstepPlan)
        sub2.setSpeedLimit(1) # was 5 but was slow rendering

    ##############################

    def getDefaultStepParams(self):
        default_step_params = lcmdrc.footstep_params_t()
        default_step_params.step_speed = 0.2
        default_step_params.step_height = 0.05
        default_step_params.constrain_full_foot_pose = True
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

    def onWalkingPlan(self, msg):
        self.lastWalkingPlan = msg
        if self.walkingPlanCallback:
            self.walkingPlanCallback()

    def onBDIAdjustedFootstepPlan(self, msg):
        folder = getBDIAdjustedFootstepsFolder()
        om.removeFromObjectModel(folder)
        folder = getBDIAdjustedFootstepsFolder()
        self.drawFootstepPlan(msg, folder)

    def onFootstepPlan(self, msg):
        self.clearFootstepPlan()
        self.lastFootstepPlan = msg.decode( msg.encode() ) # decode and encode ensures deepcopy

        planFolder = getFootstepsFolder()
        self.drawFootstepPlan( self.lastFootstepPlan , planFolder)
        self.transformPlanToBDIFrame( self.lastFootstepPlan )

    def clearFootstepPlan(self):
        self.lastFootstepPlan = None
        folder = getFootstepsFolder()
        om.removeFromObjectModel(folder)


    def drawFootstepPlan(self, msg, folder,left_color=None, right_color=None):

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
                if (right_color is None):
                    color = getRightFootColor()
                else:
                    color = right_color
            else:
                mesh = getLeftFootMesh()
                if (left_color is None):
                    color = getLeftFootColor()
                else:
                    color = left_color

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
                vis.showPolyData(d.getPolyData(), 'infeasibility %d -> %d' % (i-2, i-1), parent=folder, color=[1, 0.2, 0.2])

            stepName = 'step %d' % (i-1)

            # add gradual shading to steps to indicate destination
	    frac = float(i)/ float(msg.num_steps-1)
	    this_color = [0,0,0]
            this_color[0] = 0.25*color[0] + 0.75*frac*color[0]
            this_color[1] = 0.25*color[1] + 0.75*frac*color[1]
            this_color[2] = 0.25*color[2] + 0.75*frac*color[2]

            obj = vis.showPolyData(mesh, stepName, color=this_color, alpha=1.0, parent=folder)
            obj.setIcon(om.Icons.Feet)
            frameObj = vis.showFrame(footstepTransform, stepName + ' frame', parent=obj, scale=0.3, visible=False)
            obj.actor.SetUserTransform(footstepTransform)

    def getContactPts(self):
        '''
        hard coded Location of the Drake contact points relative to foot frame. this should be read from URDF
        '''
        contact_pts = np.zeros((4,3))
        contact_pts[0,:] = [-0.082,  0.0624435, -0.081119]
        contact_pts[1,:] = [-0.082, -0.0624435, -0.081119]
        contact_pts[2,:] = [0.178,   0.0624435, -0.081119]
        contact_pts[3,:] = [0.178,  -0.0624435, -0.081119]
        return contact_pts

    def getFeetMidPoint(self, model):
        contact_pts = self.getContactPts()
        contact_pts_mid = np.mean(contact_pts, axis=0) # mid point on foot relative to foot frame

        t_lf_mid = model.getLinkFrame('l_foot')
        t_lf_mid.Translate(contact_pts_mid)

        t_rf_mid = model.getLinkFrame('r_foot')
        t_rf_mid.Translate(contact_pts_mid)
        t_feet_mid = transformUtils.frameInterpolate(t_lf_mid, t_rf_mid, 0.5)
        return t_feet_mid

    def createGoalSteps(self, model, pose):
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
            step.params = self.getDefaultStepParams()
            self.goalSteps.append(step)

        request = self.constructFootstepPlanRequest(pose)
        request.num_goal_steps = len(self.goalSteps)
        request.goal_steps = self.goalSteps

        self.sendFootstepPlanRequest(request)

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
        msg = self.applyParams(msg)
        self.sendFootstepPlanRequest(msg)

    def updateRequest(self):
        if self.lastFootstepRequest is not None:
            msg = self.lastFootstepRequest
            msg = self.applyParams(msg)
            self.sendFootstepPlanRequest(msg)

    def constructFootstepPlanRequest(self, pose, goalFrame=None):

        msg = lcmdrc.footstep_plan_request_t()
        msg.utime = getUtime()
        state_msg = robotstate.drakePoseToRobotState(pose)
        msg.initial_state = state_msg

        if goalFrame is None:
            goalFrame = vtk.vtkTransform()
        msg.goal_pos = transformUtils.positionMessageFromFrame(goalFrame)

        msg = self.applyParams(msg)
        return msg

    def applyParams(self, msg):
        msg.params = lcmdrc.footstep_plan_params_t()
        msg.params.max_num_steps = 20
        msg.params.min_num_steps = 0
        msg.params.min_step_width = 0.20
        msg.params.nom_step_width = self.params.properties.nominal_step_width
        msg.params.max_step_width = self.params.properties.max_step_width
        msg.params.nom_forward_step = self.params.properties.nominal_forward_step
        msg.params.max_forward_step = self.params.properties.max_forward_step
        msg.params.nom_upward_step = 0.25;
        msg.params.nom_downward_step = 0.15;
        msg.params.ignore_terrain = self.params.properties.ignore_terrain
        msg.params.planning_mode = msg.params.MODE_AUTO
        msg.params.behavior = self.behavior_lcm_map[self.params.properties.behavior]
        msg.params.map_command = self.map_command_lcm_map[self.params.properties.map_command]
        msg.params.leading_foot = msg.params.LEAD_AUTO
        msg.default_step_params = self.getDefaultStepParams()
        return msg

    def sendFootstepPlanRequest(self, request, waitForResponse=False, waitTimeout=5000):

        assert isinstance(request, lcmdrc.footstep_plan_request_t)
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

    def sendWalkingPlanRequest(self, footstepPlan, startPose, waitForResponse=False, waitTimeout=5000, req_type='traj'):

        msg = lcmdrc.walking_plan_request_t()
        msg.utime = getUtime()
        state_msg = robotstate.drakePoseToRobotState(startPose)
        msg.initial_state = state_msg
        msg.new_nominal_state = msg.initial_state
        msg.use_new_nominal_state = True
        msg.footstep_plan = footstepPlan

        if req_type == 'traj':
            requestChannel = 'WALKING_TRAJ_REQUEST'
            responseChannel = 'WALKING_TRAJ_RESPONSE'
            response_type = lcmdrc.robot_plan_t
        elif req_type == 'controller':
            requestChannel = 'WALKING_CONTROLLER_PLAN_REQUEST'
            responseChannel = 'WALKING_CONTROLLER_PLAN_RESPONSE'
            response_type = lcmdrc.walking_plan_t
        else:
            raise ValueError("Invalid request type: {:s}".format(req_type))


        if waitForResponse:
            if waitTimeout == 0:
                helper = lcmUtils.MessageResponseHelper(responseChannel, response_type)
                lcmUtils.publish(requestChannel, msg)
                return helper
            return lcmUtils.MessageResponseHelper.publishAndWait(requestChannel, msg,
                                                                 responseChannel, response_type, waitTimeout)
        else:
            lcmUtils.publish(requestChannel, msg)

    def sendWalkingControllerRequest(self, footstepPlan, startPose, waitForResponse=False, waitTimeout=5000):
        self.sendWalkingPlanRequest(footstepPlan, startPose, waitForResponse, waitTimeout, req_type='controller')

    def sendStopWalking(self):
        msg = lcmdrc.plan_control_t()
        msg.utime = getUtime()
        msg.control = lcmdrc.plan_control_t.TERMINATE
        lcmUtils.publish('STOP_WALKING', msg)

    def commitFootstepPlan(self, footstepPlan):
        footstepPlan.utime = getUtime()
        lcmUtils.publish('COMMITTED_FOOTSTEP_PLAN', footstepPlan)


    ####################### BDI Adjustment Logic and Visualization ##################
    def onPoseBDI(self,msg):
        self.pose_bdi = msg
        # Set the xyzrpy of this pose to equal that estimated by BDI
        rpy = botpy.quat_to_roll_pitch_yaw(msg.orientation)
        pose = self.jointController.q.copy()
        pose[0:3] = msg.pos
        pose[3:6] = rpy
        self.bdiJointController.setPose("ERS BDI", pose)

    def onBDIAdjustedFootstepPlan(self,msg):
        self.bdi_plan_adjusted = msg.decode( msg.encode() ) # decode and encode ensures deepcopy
        if (self.showBDIPlan is True):
            self.drawBDIFootstepPlanAdjusted()
        #else:
        #    print "not showing adjusted bdi plan"

    def transformPlanToBDIFrame(self, plan):
        if (self.pose_bdi is None):
            print "haven't received POSE_BDI"
            return

        # TODO: This transformation should be rewritten using the LOCAL_TO_LOCAL_BDI frame
        # instead of using FK here

        t_bodybdi  = transformUtils.transformFromPose(self.pose_bdi.pos, self.pose_bdi.orientation)
        t_bodybdi.PostMultiply()

        current_pose = self.jointController.q
        t_bodymain = transformUtils.transformFromPose( current_pose[0:3]  , botpy.roll_pitch_yaw_to_quat(current_pose[3:6])   )
        t_bodymain.PostMultiply()

        # iterate and transform
        self.bdi_plan = plan.decode( plan.encode() ) # decode and encode ensures deepcopy
        for i, footstep in enumerate(self.bdi_plan.footsteps):
            step = footstep.pos

            t_step = transformUtils.frameFromPositionMessage(step)
            t_body_to_step = vtk.vtkTransform()
            t_body_to_step.DeepCopy(t_step)
            t_body_to_step.PostMultiply()
            t_body_to_step.Concatenate(t_bodymain.GetLinearInverse())

            t_stepbdi = vtk.vtkTransform()
            t_stepbdi.DeepCopy(t_body_to_step)
            t_stepbdi.PostMultiply()
            t_stepbdi.Concatenate(t_bodybdi)
            footstep.pos = transformUtils.positionMessageFromFrame(t_stepbdi)

        if (self.showBDIPlan is True):
            self.drawBDIFootstepPlan()
        #else:
        #    print "not showing bdi plan"

    def drawBDIFootstepPlan(self):
        if (self.bdi_plan is None):
            return

        folder = om.getOrCreateContainer("BDI footstep plan")
        om.removeFromObjectModel(folder)

        folder = om.getOrCreateContainer("BDI footstep plan")
        folder.setIcon(om.Icons.Feet)
        om.collapse(folder)
        self.drawFootstepPlan(self.bdi_plan, folder, [0.0, 0.0, 1.0] , [1.0, 0.0, 0.0])

    def drawBDIFootstepPlanAdjusted(self):
        if (self.bdi_plan_adjusted is None):
            return

        folder = om.getOrCreateContainer('BDI adj footstep plan')
        om.removeFromObjectModel(folder)

        folder = om.getOrCreateContainer('BDI adj footstep plan')
        folder.setIcon(om.Icons.Feet)
        om.collapse(folder)
        self.drawFootstepPlan(self.bdi_plan_adjusted, folder, [1.0, 1.0, 0.0] , [0.0, 1.0, 1.0])

