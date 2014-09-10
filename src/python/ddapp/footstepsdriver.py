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
from ddapp import filterUtils
import ddapp.vtkNumpy as vnp

import os
import numpy as np
from ddapp import botpy
import drc as lcmdrc
from bot_core.pose_t import pose_t
import functools


from PythonQt import QtGui, QtCore


DEFAULT_PARAM_SET = 'drake'
DEFAULT_STEP_PARAMS = {'BDI': {'Max Num Steps': 20,
                               'Nominal Step Width': 0.26,
                               'Nominal Forward Step': 0.15,
                               'Max Forward Step': 0.40,
                               'Max Step Width': 0.4,
                               'Behavior': 0,
                               'Leading Foot': 0,
                               'Drake Swing Speed': 0.2,
                               'Drake Instep Shift': 0.0275,
                               'Drake Min Hold Time': 2.0},
                       'drake': {'Max Num Steps': 20,
                                 'Nominal Step Width': 0.28,
                                 'Nominal Forward Step': 0.17,
                                 'Max Forward Step': 0.17,
                                 'Max Step Width': 0.32,
                                 'Behavior': 2,
                                 'Leading Foot': 0,
                                 'Drake Swing Speed': 0.15,
                                 'Drake Instep Shift': 0.015,
                                 'Drake Min Hold Time': 1.4}}


def loadFootMeshes():
    meshDir = os.path.join(app.getDRCBase(), 'software/models/mit_gazebo_models/mit_robot/meshes')
    meshes = []
    for foot in ['l', 'r']:
        d = DebugData()
        d.addPolyData(ioUtils.readPolyData(os.path.join(meshDir, '%s_talus.stl' % foot), computeNormals=True))
        d.addPolyData(ioUtils.readPolyData(os.path.join(meshDir, '%s_foot.stl' % foot), computeNormals=True))

        t = vtk.vtkTransform()
        t.Scale(0.98, 0.98, 0.98)
        pd = filterUtils.transformPolyData(d.getPolyData(), t)
        meshes.append(pd)
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
        obj = om.getOrCreateContainer('footstep plan', parentObj=om.getOrCreateContainer('planning'))
        obj.setIcon(om.Icons.Feet)
        om.collapse(obj)
    return obj

def getWalkingVolumesFolder():
    obj = om.findObjectByName('walking volumes')
    if obj is None:
        obj = om.getOrCreateContainer('walking volumes', parentObj=getFootstepsFolder())
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
        self.lastWalkingPlan = None
        self.walkingPlanCallback = None
        self.default_step_params = DEFAULT_STEP_PARAMS
        self.contact_slices = {}
        self.show_contact_slices = False
        self.toolbarWidget = None

        ### Stuff pertaining to rendering BDI-frame steps
        self.pose_bdi = None
        self.bdi_plan = None
        self.bdi_plan_adjusted = None

        view = app.getDRCView()
        self.bdiRobotModel, self.bdiJointController = roboturdf.loadRobotModel('bdi model', view, parent='bdi model', color=roboturdf.getRobotOrangeColor(), visible=False)
        self.bdiRobotModel.setProperty('Visible', False)
        self.showBDIPlan = False # hide the BDI plans when created
        self.bdiChannel = "POSE_BDI"
        self.bdiSubcribe = None

        self._setupSubscriptions()
        self._setupProperties()


    def _setupProperties(self):
        self.params = om.ObjectModelItem('Footstep Params')
        self.params.addProperty('Behavior', 0, attributes=om.PropertyAttributes(enumNames=['BDI Stepping', 'BDI Walking', 'Drake Walking']))
        self.params.addProperty('Leading Foot', 1, attributes=om.PropertyAttributes(enumNames=['Auto', 'Left', 'Right']))
        self.leading_foot_map = [lcmdrc.footstep_plan_params_t.LEAD_AUTO,
                                 lcmdrc.footstep_plan_params_t.LEAD_LEFT,
                                 lcmdrc.footstep_plan_params_t.LEAD_RIGHT]
        # self.params.addProperty('Map Command', 0, attributes=om.PropertyAttributes(enumNames=['Full Heightmap', 'Flat Ground', 'Z Normals']))
        self.params.addProperty('Map Mode', 0, attributes=om.PropertyAttributes(enumNames=['Foot Plane', 'Terrain Heights & Normals', 'Terrain Heights, Z Normals', 'Horizontal Plane']))
        self.map_mode_map = [
                             lcmdrc.footstep_plan_params_t.FOOT_PLANE,
                             lcmdrc.footstep_plan_params_t.TERRAIN_HEIGHTS_AND_NORMALS,
                             lcmdrc.footstep_plan_params_t.TERRAIN_HEIGHTS_Z_NORMALS,
                             lcmdrc.footstep_plan_params_t.HORIZONTAL_PLANE
                             ]
        # self.params.addProperty('Heights Source', attributes=om.PropertyAttributes(enumNames=['Map Data', 'Foot Plane']))
        # self.params.addProperty('Normals Source', attributes=om.PropertyAttributes(enumNames=['Map Data', 'Foot Plane']))
        self.params.addProperty('Max Num Steps', None, attributes=om.PropertyAttributes(decimals=0, minimum=1, maximum=30, singleStep=1))
        self.params.addProperty('Nominal Step Width', None, attributes=om.PropertyAttributes(decimals=2, minimum=0.21, maximum=0.4, singleStep=0.01))
        self.params.addProperty('Nominal Forward Step', None, attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=0.5, singleStep=0.01))
        self.params.addProperty('Max Step Width', None, attributes=om.PropertyAttributes(decimals=2, minimum=0.22, maximum=0.5, singleStep=0.01))
        self.params.addProperty('Max Forward Step', None, attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=0.5, singleStep=0.01))
        self.params.addProperty('Drake Swing Speed', None, attributes=om.PropertyAttributes(decimals=2, minimum=0.05, maximum=5.0, singleStep=0.05))
        self.params.addProperty('Drake Min Hold Time', None, attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=5.0, singleStep=0.05))
        self.params.addProperty('Drake Instep Shift', None, attributes=om.PropertyAttributes(decimals=4, minimum=-0.3, maximum=0.3, singleStep=0.0005))
        self.applyDefaults(DEFAULT_PARAM_SET)
        self.behavior_lcm_map = {
                              0: lcmdrc.footstep_plan_params_t.BEHAVIOR_BDI_STEPPING,
                              1: lcmdrc.footstep_plan_params_t.BEHAVIOR_BDI_WALKING,
                              2: lcmdrc.footstep_plan_params_t.BEHAVIOR_WALKING}

    def applyDefaults(self, set_name):
        defaults = self.default_step_params[set_name]
        for k, v in defaults.iteritems():
            self.params.setProperty(k, v)

    def _setupSubscriptions(self):
        lcmUtils.addSubscriber('FOOTSTEP_PLAN_RESPONSE', lcmdrc.footstep_plan_t, self.onFootstepPlan)
        lcmUtils.addSubscriber('WALKING_TRAJ_RESPONSE', lcmdrc.robot_plan_t, self.onWalkingPlan)
        lcmUtils.addSubscriber('WALKING_SIMULATION_TRAJ_RESPONSE', lcmdrc.robot_plan_t, self.onWalkingPlan)

        ### Related to BDI-frame adjustment:
        self.bdiSubcribe = lcmUtils.addSubscriber( self.bdiChannel , pose_t, self.onPoseBDI)
        self.bdiSubcribe.setSpeedLimit(60)
        sub2 = lcmUtils.addSubscriber('BDI_ADJUSTED_FOOTSTEP_PLAN', lcmdrc.footstep_plan_t, self.onBDIAdjustedFootstepPlan)
        sub2.setSpeedLimit(1) # was 5 but was slow rendering

    def changeSubscriptionBDI(self, newBDIChannel="POSE_BDI"):
        # used to monitor a different pose e.g. POSE_BODY_LOGGED in playback
        self.bdiChannel = newBDIChannel
        lcmUtils.removeSubscriber ( self.bdiSubcribe )

        self.bdiSubcribe = lcmUtils.addSubscriber( self.bdiChannel , pose_t, self.onPoseBDI)
        self.bdiSubcribe.setSpeedLimit(60)


    ##############################

    def getDefaultStepParams(self):
        default_step_params = lcmdrc.footstep_params_t()
        default_step_params.step_speed = self.params.properties.drake_swing_speed
        default_step_params.drake_min_hold_time = self.params.properties.drake_min_hold_time
        default_step_params.drake_instep_shift = self.params.properties.drake_instep_shift
        default_step_params.step_height = 0.05
        default_step_params.constrain_full_foot_pose = True
        default_step_params.bdi_step_duration = 2.0
        default_step_params.bdi_sway_duration = 0.0
        default_step_params.bdi_lift_height = 0.065
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
            self.walkingPlanCallback(self.lastWalkingPlan)

    def onBDIAdjustedFootstepPlan(self, msg):
        folder = getBDIAdjustedFootstepsFolder()
        om.removeFromObjectModel(folder)
        folder = getBDIAdjustedFootstepsFolder()
        self.drawFootstepPlan(msg, folder)

    def onFootstepPlan(self, msg):
        #self.clearFootstepPlan()
        self.lastFootstepPlan = msg

        planFolder = getFootstepsFolder()
        self.drawFootstepPlan( self.lastFootstepPlan , planFolder)
        self.transformPlanToBDIFrame( self.lastFootstepPlan )

        self.showToolbarWidget()
        self.execButton.show()


    def showToolbarWidget(self):
        if self.toolbarWidget:
            return

        w = QtGui.QWidget()
        l = QtGui.QHBoxLayout(w)

        label = QtGui.QLabel('Walk plan:')
        execButton = QtGui.QPushButton('')
        execButton.setIcon(QtGui.QApplication.style().standardIcon(QtGui.QStyle.SP_MediaPlay))
        clearButton = QtGui.QPushButton('')
        clearButton.setIcon(QtGui.QApplication.style().standardIcon(QtGui.QStyle.SP_TrashIcon))
        stopButton = QtGui.QPushButton('')
        stopButton.setIcon(QtGui.QApplication.style().standardIcon(QtGui.QStyle.SP_MediaStop))

        l.addWidget(label)
        l.addWidget(execButton)
        l.addWidget(stopButton)
        l.addWidget(clearButton)
        l.setContentsMargins(0, 0, 0, 0)

        execButton.setShortcut(QtGui.QKeySequence('Ctrl+Return'))
        execButton.connect('clicked()', self.onExecClicked)
        clearButton.connect('clicked()', self.onClearClicked)
        stopButton.connect('clicked()', self.sendStopWalking)
        stopButton.setVisible(False)

        self.execButton = execButton
        self.stopButton = stopButton
        self.toolbarWidget = app.getMainWindow().toolBar().addWidget(w)


    def onExecClicked(self):
        self.commitFootstepPlan(self.lastFootstepPlan)

        om.removeFromObjectModel(om.findObjectByName('footstep widget'))
        walkGoal = om.findObjectByName('walking goal')
        if walkGoal:
            walkGoal.setProperty('Edit', False)
        self.execButton.hide()
        self.stopButton.show()


    def onClearClicked(self):
        om.removeFromObjectModel(om.findObjectByName('walking goal'))
        om.removeFromObjectModel(om.findObjectByName('footstep widget'))
        om.removeFromObjectModel(om.findObjectByName('LCM GL'))
        self.clearFootstepPlan()

        if self.toolbarWidget:
            app.getMainWindow().toolBar().removeAction(self.toolbarWidget)
            self.toolbarWidget = None


    def clearFootstepPlan(self):
        self.lastFootstepPlan = None
        om.removeFromObjectModel(getFootstepsFolder())

    def drawFootstepPlan(self, msg, folder,left_color=None, right_color=None):

        allTransforms = []
        volFolder = getWalkingVolumesFolder()

        steps = folder.children()[1:]

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

            # add gradual shading to steps to indicate destination
            frac = float(i)/ float(msg.num_steps-1)
            this_color = [0,0,0]
            this_color[0] = 0.25*color[0] + 0.75*frac*color[0]
            this_color[1] = 0.25*color[1] + 0.75*frac*color[1]
            this_color[2] = 0.25*color[2] + 0.75*frac*color[2]


            if self.show_contact_slices:
                for zs, xy in self.contact_slices.iteritems():
                    points0 = np.vstack((xy, zs[0] + np.zeros((1,xy.shape[1]))))
                    points1 = np.vstack((xy, zs[1] + np.zeros((1,xy.shape[1]))))
                    points = np.hstack((points0, points1))
                    points = points + np.array([[0.05],[0],[-0.0811]])
                    points = points.T
                    polyData = vnp.getVtkPolyDataFromNumpyPoints(points.copy())
                    vol_mesh = filterUtils.computeDelaunay3D(polyData)
                    obj = vis.showPolyData(vol_mesh, 'walking volume', parent=volFolder, alpha=0.5, visible=self.show_contact_slices, color=color)
                    obj.actor.SetUserTransform(footstepTransform)


            renderInfeasibility = False
            if renderInfeasibility and footstep.infeasibility > 1e-6:
                d = DebugData()
                start = allTransforms[i-1].GetPosition()
                end = footstepTransform.GetPosition()
                d.addArrow(start, end, 0.02, 0.005,
                           startHead=True,
                           endHead=True)
                vis.showPolyData(d.getPolyData(), 'infeasibility %d -> %d' % (i-2, i-1), parent=folder, color=[1, 0.2, 0.2])


            stepName = 'step %d' % (i-1)

            if steps:
                obj = steps.pop(0)
                assert obj.getProperty('Name') == stepName
                frameObj = obj.children()[0]
                frameObj.copyFrame(footstepTransform)
                obj.setProperty('Visible', True)
                obj.setProperty('Color', QtGui.QColor(*[255*v for v in this_color]))
            else:
                obj = vis.showPolyData(mesh, stepName, color=this_color, alpha=1.0, parent=folder)
                obj.setIcon(om.Icons.Feet)
                frameObj = vis.showFrame(footstepTransform, stepName + ' frame', parent=obj, scale=0.3, visible=False)
                obj.actor.SetUserTransform(footstepTransform)

        for obj in steps:
            obj.setProperty('Visible', False)

    @staticmethod
    def getContactPts():
        '''
        hard coded Location of the Drake contact points relative to foot frame. this should be read from URDF
        '''
        contact_pts = np.zeros((4,3))
        contact_pts[0,:] = [-0.082,  0.0624435, -0.081119]
        contact_pts[1,:] = [-0.082, -0.0624435, -0.081119]
        contact_pts[2,:] = [0.178,   0.0624435, -0.081119]
        contact_pts[3,:] = [0.178,  -0.0624435, -0.081119]
        return contact_pts

    @staticmethod
    def getFeetMidPoint(model):
        '''
        Returns a frame in world coordinate system that is the average of the left
        and right foot reference point positions in world frame, the average of the
        left and right foot yaw in world frame, and Z axis aligned with world Z.
        The foot reference point is the average of the foot contact points in the foot frame.
        '''
        contact_pts = FootstepsDriver.getContactPts()
        contact_pts_mid = np.mean(contact_pts, axis=0) # mid point on foot relative to foot frame

        t_lf_mid = model.getLinkFrame('l_foot')
        t_lf_mid.PreMultiply()
        t_lf_mid.Translate(contact_pts_mid)

        t_rf_mid = model.getLinkFrame('r_foot')
        t_rf_mid.PreMultiply()
        t_rf_mid.Translate(contact_pts_mid)
        t_feet_mid = transformUtils.frameInterpolate(t_lf_mid, t_rf_mid, 0.5)
        return transformUtils.frameFromPositionAndRPY(t_feet_mid.GetPosition(), [0.0, 0.0, t_feet_mid.GetOrientation()[2]])

    @staticmethod
    def debugDrawFootPoints(model):
        pts = FootstepsDriver.getContactPts()
        footMidPoint = np.mean(pts, axis=0)
        d = DebugData()

        for linkName in ['l_foot', 'r_foot']:

            t = model.getLinkFrame(linkName)
            d.addFrame(t, scale=0.2)

            for p in pts.tolist() + [footMidPoint.tolist()]:
                t.TransformPoint(p, p)
                d.addSphere(p, radius=0.015)

        d.addFrame(FootstepsDriver.getFeetMidPoint(model), scale=0.2)
        vis.showPolyData(d.getPolyData(), 'foot points debug', parent='debug', colorByName='RGB255')

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
        msg = self.applySafeRegions(msg)
        return msg

    def applyParams(self, msg):
        msg.params = lcmdrc.footstep_plan_params_t()
        msg.params.max_num_steps = self.params.properties.max_num_steps
        msg.params.min_num_steps = 0
        msg.params.min_step_width = 0.20
        msg.params.nom_step_width = self.params.properties.nominal_step_width
        msg.params.max_step_width = self.params.properties.max_step_width
        msg.params.nom_forward_step = self.params.properties.nominal_forward_step
        msg.params.max_forward_step = self.params.properties.max_forward_step
        msg.params.nom_upward_step = 0.25;
        msg.params.nom_downward_step = 0.15;
        msg.params.planning_mode = msg.params.MODE_AUTO
        msg.params.behavior = self.behavior_lcm_map[self.params.properties.behavior]
        # msg.params.use_map_heights = self.params.properties.heights_source == 0
        # msg.params.use_map_normals = self.params.properties.normals_source == 0
        msg.params.map_mode = self.map_mode_map[self.params.properties.map_mode]
        # msg.params.map_command = self.map_command_lcm_map[self.params.properties.map_command]
        msg.params.leading_foot = self.leading_foot_map[self.params.properties.leading_foot]
        msg.default_step_params = self.getDefaultStepParams()
        return msg

    def applySafeRegions(self, msg):
        safe_regions_folder = om.findObjectByName('Safe terrain regions')
        safe_terrain_regions = []
        if safe_regions_folder:
            for obj in safe_regions_folder.children():
                if obj.getProperty('Enabled for Walking'):
                    safe_terrain_regions.append(obj.safe_region)
        msg.num_iris_regions = len(safe_terrain_regions)
        for r in safe_terrain_regions:
            msg.iris_regions.append(r.to_iris_region_t())
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
        elif req_type == 'simulate_drake':
            requestChannel = 'WALKING_SIMULATION_DRAKE_REQUEST'
            responseChannel = 'WALKING_SIMULATION_TRAJ_RESPONSE'
            response_type = lcmdrc.robot_plan_t
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

    def sendStopWalking(self):
        msg = lcmdrc.plan_control_t()
        msg.utime = getUtime()
        msg.control = lcmdrc.plan_control_t.TERMINATE
        lcmUtils.publish('STOP_WALKING', msg)

    def commitFootstepPlan(self, footstepPlan):
        if footstepPlan.params.behavior in (lcmdrc.footstep_plan_params_t.BEHAVIOR_BDI_STEPPING,
                                            lcmdrc.footstep_plan_params_t.BEHAVIOR_BDI_WALKING):
            self._commitFootstepPlanBDI(footstepPlan)
        elif footstepPlan.params.behavior == lcmdrc.footstep_plan_params_t.BEHAVIOR_WALKING:
            self._commitFootstepPlanDrake(footstepPlan)

    def _commitFootstepPlanDrake(self, footstepPlan):
        startPose = self.jointController.getPose('EST_ROBOT_STATE')
        self.sendWalkingPlanRequest(footstepPlan, startPose, req_type='controller')

    def _commitFootstepPlanBDI(self, footstepPlan):
        footstepPlan.utime = getUtime()
        lcmUtils.publish('COMMITTED_FOOTSTEP_PLAN', footstepPlan)

    def sendHaltSimulationDrakeRequest(self):
        msg = lcmdrc.utime_t()
        msg.utime = getUtime()
        lcmUtils.publish('HALT_DRAKE_SIMULATION', msg)


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

