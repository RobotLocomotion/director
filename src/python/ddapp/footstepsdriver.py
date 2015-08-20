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
import math
import numpy as np
from ddapp import drcargs
import drc as lcmdrc
from bot_core.pose_t import pose_t
from drc.robot_state_t import robot_state_t
import functools
import json

from PythonQt import QtGui, QtCore

_footMeshes = None
_footMeshFiles = []
_robotType = 0 # 0 - any atlas, 1 - val v1, 2 - val v2
_pelvisLink = '' # pelvis
_leftFootLink = '' # l_foot
_rightFootLink = '' # r_foot
with open(drcargs.args().directorConfigFile) as directorConfigFile:
    directorConfig = json.load(directorConfigFile)

    # dodgy use of filename to find valkyrie:
    if (directorConfigFile.name.find("valkyrie") > -1):
        _robotType = 1
    if (directorConfigFile.name.find("val_description") > -1):
        _robotType = 2

    directorConfigDirectory = os.path.dirname(os.path.abspath(directorConfigFile.name))

    if 'leftFootMeshFiles' in directorConfig:
        _footMeshFiles.append( directorConfig['leftFootMeshFiles'] )
        _footMeshFiles.append( directorConfig['rightFootMeshFiles'] )
        for j in range(0,2):
            for i in range(len(_footMeshFiles[j])):
                _footMeshFiles[j][i] = os.path.join(directorConfigDirectory, _footMeshFiles[j][i])

    if 'pelvisLink' in directorConfig:
        _pelvisLink =  directorConfig['pelvisLink']

    if 'leftFootLink' in directorConfig:
        _leftFootLink = directorConfig['leftFootLink']
        _rightFootLink = directorConfig['rightFootLink']


DEFAULT_PARAM_SET = 'Drake Nominal'
DEFAULT_STEP_PARAMS = {'BDI': {'Min Num Steps': 0,
                               'Max Num Steps': 12,
                               'Min Step Width': 0.20,
                               'Nominal Step Width': 0.26,
                               'Nominal Forward Step': 0.15,
                               'Max Forward Step': 0.40,
                               'Max Step Width': 0.4,
                               'Max Upward Step': 0.18,
                               'Max Downward Step': 0.18,
                               'Behavior': 0,
                               'Leading Foot': 0,
                               'Swing Height': 0.05,
                               'Drake Swing Speed': 0.2,
                               'Drake Instep Shift': 0.0275,
                               'Drake Min Hold Time': 2.0,
                               'Support Contact Groups': 0,
                               'Prevent Swing Undershoot': 0,
                               'Prevent Swing Overshoot': 0,
                               'Map Mode': 0},
                       'Drake Nominal': {'Min Num Steps': 0,
                                 'Max Num Steps': 16,
                                 'Min Step Width': 0.20,
                                 'Nominal Step Width': 0.26,
                                 'Nominal Forward Step': 0.26,
                                 'Max Forward Step': 0.30,
                                 'Max Step Width': 0.32,
                                 'Max Upward Step': 0.18,
                                 'Max Downward Step': 0.18,
                                 'Behavior': 2,
                                 'Leading Foot': 0,
                                 'Swing Height': 0.03,
                                 'Drake Swing Speed': 0.6,
                                 'Drake Instep Shift': 0.005,
                                 'Drake Min Hold Time': 1.0,
                                 'Support Contact Groups': 0,
                                 'Prevent Swing Undershoot': 0,
                                 'Prevent Swing Overshoot': 0,
                                 'Map Mode': 0}}
DEFAULT_STEP_PARAMS['Terrain'] = DEFAULT_STEP_PARAMS['Drake Nominal'].copy()
DEFAULT_STEP_PARAMS['Terrain'].update({'Drake Min Hold Time': 1.0,
                                       'Drake Swing Speed': 0.6,
                                       'Swing Height': 0.05,
                                       'Max Forward Step': 0.36,
                                       'Max Num Steps': 6,
                                       'Nominal Step Width': 0.22,
                                       'Map Mode': 1})
DEFAULT_STEP_PARAMS['Stairs'] = DEFAULT_STEP_PARAMS['Drake Nominal'].copy()
DEFAULT_STEP_PARAMS['Stairs'].update({'Drake Min Hold Time': 2.0,
                                      'Swing Height': 0.05,
                                      'Max Num Steps': 8,
                                      'Min Num Steps': 8,
                                      'Drake Swing Speed': 0.6,
                                      'Support Contact Groups': lcmdrc.footstep_params_t.SUPPORT_GROUPS_MIDFOOT_TOE,
                                      'Map Mode': 2})

DEFAULT_STEP_PARAMS['Polaris Platform'] = DEFAULT_STEP_PARAMS['Drake Nominal'].copy()
DEFAULT_STEP_PARAMS['Polaris Platform'].update({'Drake Min Hold Time': 2.0,
                                      'Prevent Swing Undershoot': 1,
                                      'Swing Height': 0.05,
                                      'Map Mode': 1})

DEFAULT_CONTACT_SLICES = {(0.05, 0.3): np.array([[-0.13, -0.13, 0.13, 0.13],
                                          [0.0562, -0.0562, 0.0562, -0.0562]]),
                          (0.3, .75): np.array([[-0.13, -0.13, 0.25, 0.25],
                                          [.25, -.25, .25, -.25]]),
                          (0.75, 1.05): np.array([[-0.2, -0.2, 0.25, 0.25],
                                          [.4, -.4, .4, -.4]]),
                          (1.05, 1.85): np.array([[-0.35, -0.35, 0.28, 0.28],
                                          [.4, -.4, .4, -.4]])
                          }


def loadFootMeshes():
    meshes = []
    for i in  range(0,2):
        d = DebugData()

        for footMeshFile in _footMeshFiles[i]:
          d.addPolyData(ioUtils.readPolyData( footMeshFile , computeNormals=True))

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

def getTerrainSlicesFolder():
    obj = om.findObjectByName('terrain slices')
    if obj is None:
        obj = om.getOrCreateContainer('terrain slices', parentObj=getFootstepsFolder())
        obj.setProperty('Visible', False)
        om.collapse(obj)
    return obj

def getBDIAdjustedFootstepsFolder():
    obj = om.findObjectByName('BDI adj footstep plan')
    if obj is None:
        obj = om.getOrCreateContainer('BDI adj footstep plan')
        obj.setIcon(om.Icons.Feet)
        om.collapse(obj)
    return obj


class FootstepPlanItem(om.ObjectModelItem):
    pass


class WalkingPlanItem(om.ObjectModelItem):
    pass


class FootstepsDriver(object):

    def __init__(self, jointController):
        self.jointController = jointController
        self.lastFootstepPlan = None
        self.lastFootstepRequest = None
        self.goalSteps = None
        self.lastWalkingPlan = None
        self.walkingPlanCallback = None
        self.default_step_params = DEFAULT_STEP_PARAMS
        self.contact_slices = DEFAULT_CONTACT_SLICES
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
        #enable this to used the bdi model to render a different state
        #self.bdiJointController.addLCMUpdater("EST_ROBOT_STATE_ALT")

        self._setupSubscriptions()
        self._setupProperties()

        self.showToolbarWidget()
        # If we're a consoleapp and have no main window execButton won't exist
        if hasattr(self, 'execButton'):
            self.execButton.setEnabled(False)

        self.committedPlans = []


    def _setupProperties(self):
        self.params = om.ObjectModelItem('Footstep Params')
        self.defaults_map = ['Drake Nominal', 'Terrain', 'Stairs', 'Polaris Platform']
        self.params.addProperty('Defaults', 0, attributes=om.PropertyAttributes(enumNames=self.defaults_map))
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
        self.params.addProperty('Min Num Steps', None, attributes=om.PropertyAttributes(decimals=0, minimum=0, maximum=30, singleStep=1))
        self.params.addProperty('Max Num Steps', None, attributes=om.PropertyAttributes(decimals=0, minimum=1, maximum=30, singleStep=1))
        self.params.addProperty('Min Step Width', None, attributes=om.PropertyAttributes(decimals=2, minimum=0.1, maximum=0.35, singleStep=0.01))
        self.params.addProperty('Nominal Step Width', None, attributes=om.PropertyAttributes(decimals=2, minimum=0.21, maximum=0.4, singleStep=0.01))
        self.params.addProperty('Max Step Width', None, attributes=om.PropertyAttributes(decimals=2, minimum=0.22, maximum=0.5, singleStep=0.01))
        self.params.addProperty('Nominal Forward Step', None, attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=0.5, singleStep=0.01))
        self.params.addProperty('Max Forward Step', None, attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=0.5, singleStep=0.01))
        self.params.addProperty('Swing Height', None, attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=0.5, singleStep=0.005))
        self.params.addProperty('Max Upward Step', None, attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=0.5, singleStep=0.01))
        self.params.addProperty('Max Downward Step', None, attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=0.5, singleStep=0.01))
        self.params.addProperty('Drake Swing Speed', None, attributes=om.PropertyAttributes(decimals=2, minimum=0.05, maximum=5.0, singleStep=0.05))
        self.params.addProperty('Drake Min Hold Time', None, attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=10.0, singleStep=0.05))
        self.params.addProperty('Drake Instep Shift', None, attributes=om.PropertyAttributes(decimals=4, minimum=-0.3, maximum=0.3, singleStep=0.0005))
        self.behavior_lcm_map = {
                              0: lcmdrc.footstep_plan_params_t.BEHAVIOR_BDI_STEPPING,
                              1: lcmdrc.footstep_plan_params_t.BEHAVIOR_BDI_WALKING,
                              2: lcmdrc.footstep_plan_params_t.BEHAVIOR_WALKING}
        self.params.addProperty('Planner Mode', 0, attributes=om.PropertyAttributes(enumNames=['Fast MIQP', 'Slow MISOCP']))
        self.params.addProperty('Support Contact Groups', 0, attributes=om.PropertyAttributes(enumNames=['Whole Foot', 'Front 2/3', 'Back 2/3']))
        self.params.addProperty('Prevent Swing Undershoot', 0, attributes=om.PropertyAttributes(enumNames=['False', 'True']))
        self.params.addProperty('Prevent Swing Overshoot', 0, attributes=om.PropertyAttributes(enumNames=['False', 'True']))

        self.applyDefaults(DEFAULT_PARAM_SET)

    def applyDefaults(self, set_name):
        defaults = self.default_step_params[set_name]
        for k, v in defaults.iteritems():
            self.params.setProperty(k, v)

    def _setupSubscriptions(self):

        useHistoricalLoader = False
        historicalLoader = lcmUtils.HistoricalLCMLoader('drc', 'software/drc_lcmtypes/lcmtypes', os.getenv('DRC_BASE')) if useHistoricalLoader else None

        lcmUtils.addSubscriber('FOOTSTEP_PLAN_RESPONSE', lcmdrc.footstep_plan_t, self.onFootstepPlan, historicalLoader)
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
        default_step_params.step_height = self.params.properties.swing_height
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
        default_step_params.support_contact_groups = self.params.properties.support_contact_groups
        default_step_params.prevent_swing_undershoot = self.params.properties.prevent_swing_undershoot
        default_step_params.prevent_swing_overshoot = self.params.properties.prevent_swing_overshoot
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


    def showToolbarWidget(self):

        if app.getMainWindow() is None:
            return

        if self.toolbarWidget:
            self.execButton.setEnabled(True)
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

        self.execButton = execButton
        self.stopButton = stopButton
        self.toolbarWidget = app.getMainWindow().toolBar().addWidget(w)
        self.execButton.show()

    def onExecClicked(self):
        self.commitFootstepPlan(self.lastFootstepPlan)

        om.removeFromObjectModel(om.findObjectByName('footstep widget'))
        walkGoal = om.findObjectByName('walking goal')
        if walkGoal:
            walkGoal.setProperty('Edit', False)
        self.execButton.setEnabled(False)


    def onClearClicked(self):
        om.removeFromObjectModel(om.findObjectByName('walking goal'))
        om.removeFromObjectModel(om.findObjectByName('footstep widget'))
        om.removeFromObjectModel(om.findObjectByName('LCM GL'))
        self.clearFootstepPlan()

        if self.toolbarWidget:
            self.execButton.setEnabled(False)

    def clearFootstepPlan(self):
        self.lastFootstepPlan = None
        om.removeFromObjectModel(getFootstepsFolder())

    def drawFootstepPlan(self, msg, folder, left_color=None, right_color=None, alpha=1.0):
        for step in folder.children():
            om.removeFromObjectModel(step)
        allTransforms = []
        volFolder = getWalkingVolumesFolder()
        map(om.removeFromObjectModel, volFolder.children())
        slicesFolder = getTerrainSlicesFolder()
        map(om.removeFromObjectModel, slicesFolder.children())


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
                self.drawContactVolumes(footstepTransform, color)

            contact_pts_left, contact_pts_right = FootstepsDriver.getContactPts()
            if footstep.is_right_foot:
                sole_offset = np.mean(contact_pts_right, axis=0)
            else:
                sole_offset = np.mean(contact_pts_left, axis=0)

            t_sole_prev = transformUtils.frameFromPositionMessage(msg.footsteps[i-2].pos)
            t_sole_prev.PreMultiply()
            t_sole_prev.Translate(sole_offset)
            t_sole = transformUtils.copyFrame(footstepTransform)
            t_sole.Translate(sole_offset)
            yaw = np.arctan2(t_sole.GetPosition()[1] - t_sole_prev.GetPosition()[1],
                             t_sole.GetPosition()[0] - t_sole_prev.GetPosition()[0])
            T_terrain_to_world = transformUtils.frameFromPositionAndRPY([t_sole_prev.GetPosition()[0], t_sole_prev.GetPosition()[1], 0],
                                                                        [0, 0, math.degrees(yaw)])
            path_dist = np.array(footstep.terrain_path_dist)
            height = np.array(footstep.terrain_height)
            # if np.any(height >= trans[2]):
            terrain_pts_in_local = np.vstack((path_dist, np.zeros(len(footstep.terrain_path_dist)), height))
            d = DebugData()
            for j in range(terrain_pts_in_local.shape[1]-1):
                d.addLine(terrain_pts_in_local[:,j], terrain_pts_in_local[:,j+1], radius=0.01)
            obj = vis.showPolyData(d.getPolyData(), 'terrain slice', parent=slicesFolder, visible=slicesFolder.getProperty('Visible'), color=[.8,.8,.3])
            obj.actor.SetUserTransform(T_terrain_to_world)

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

            obj = vis.showPolyData(mesh, stepName, color=this_color, alpha=alpha, parent=folder)
            obj.setIcon(om.Icons.Feet)
            frameObj = vis.showFrame(footstepTransform, stepName + ' frame', parent=obj, scale=0.3, visible=False)
            obj.actor.SetUserTransform(footstepTransform)
            obj.addProperty('Support Contact Groups', footstep.params.support_contact_groups, attributes=om.PropertyAttributes(enumNames=['Whole Foot', 'Front 2/3', 'Back 2/3']))
            obj.properties.setPropertyIndex('Support Contact Groups', 0)
            obj.footstep_index = i
            obj.footstep_property_callback = obj.properties.connectPropertyChanged(functools.partial(self.onFootstepPropertyChanged, obj))

            self.drawContactPts(obj, footstep, color=this_color)

    def drawContactVolumes(self, footstepTransform, color):
        volFolder = getWalkingVolumesFolder()
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

    def onFootstepPropertyChanged(self, obj, propertySet, propertyName):
        if propertyName == "Support Contact Groups":
            self.lastFootstepPlan.footsteps[obj.footstep_index].params.support_contact_groups = obj.properties.support_contact_groups
            self.sendUpdatePlanRequest()

    def drawContactPts(self, obj, footstep, **kwargs):
        if footstep.is_right_foot:
            _, contact_pts = FootstepsDriver.getContactPts()
        else:
            contact_pts, _ = FootstepsDriver.getContactPts()
        d = DebugData()
        for pt in contact_pts:
            d.addSphere(pt, radius=0.01)
        d_obj = vis.showPolyData(d.getPolyData(), "contact points", parent=obj, **kwargs)
        d_obj.actor.SetUserTransform(obj.actor.GetUserTransform())

    @staticmethod
    def getContactPts(support_contact_groups = lcmdrc.footstep_params_t.SUPPORT_GROUPS_HEEL_TOE):
        '''
        hard coded Location of the Drake contact points relative to foot frame. this should be read from URDF
        '''
        contact_pts_left = np.zeros((4,3))
        contact_pts_right = np.zeros((4,3))

        # atlas:
        if support_contact_groups == lcmdrc.footstep_params_t.SUPPORT_GROUPS_HEEL_TOE:
            contact_pts_left[0,:] = [-0.0876,  0.0626, -0.07645]
            contact_pts_left[1,:] = [-0.0876, -0.0626, -0.07645]
            contact_pts_left[2,:] = [0.1728,   0.0626, -0.07645]
            contact_pts_left[3,:] = [0.1728,  -0.0626, -0.07645]
        elif support_contact_groups == lcmdrc.footstep_params_t.SUPPORT_GROUPS_MIDFOOT_TOE:
            contact_pts_left[0,:] = [-0.0008,  0.0626, -0.07645]
            contact_pts_left[1,:] = [-0.0008, -0.0626, -0.07645]
            contact_pts_left[2,:] = [0.1728,   0.0626, -0.07645]
            contact_pts_left[3,:] = [0.1728,  -0.0626, -0.07645]
        elif support_contact_groups == lcmdrc.footstep_params_t.SUPPORT_GROUPS_HEEL_MIDFOOT:
            contact_pts_left[0,:] = [-0.0876,  0.0626, -0.07645]
            contact_pts_left[1,:] = [-0.0876, -0.0626, -0.07645]
            contact_pts_left[2,:] = [0.086,   0.0626, -0.07645]
            contact_pts_left[3,:] = [0.086,  -0.0626, -0.07645]
        else:
            raise ValueError("Unrecognized support contact group: {:d}".format(support_contact_groups))

        contact_pts_right = contact_pts_left.copy()

        if (_robotType == 1):
            contact_pts_left[0,:] = [0.110, 0.0624435, -0.22]
            contact_pts_left[1,:] = [0.110,-0.0624435, -0.22]
            contact_pts_left[2,:] = [0.075, 0.0624435, 0.0775]
            contact_pts_left[3,:] = [0.075,-0.0624435, 0.0775]
            contact_pts_right[0,:] = [0.075, 0.0624435, -0.0775]
            contact_pts_right[1,:] = [0.075,-0.0624435, -0.0775]
            contact_pts_right[2,:] = [0.110, 0.0624435, 0.22]
            contact_pts_right[3,:] = [0.110,-0.0624435, 0.22]
        if (_robotType == 2):
            # these are atlas values - not correct for val v2
            contact_pts_left[0,:] = [-0.0676,  0.0624435, -0.07645]
            contact_pts_left[1,:] = [-0.0676, -0.0624435, -0.07645]
            contact_pts_left[2,:] = [0.1928,   0.0624435, -0.07645]
            contact_pts_left[3,:] = [0.1928,  -0.0624435, -0.07645]
            contact_pts_right = contact_pts_left.copy()

        return contact_pts_left, contact_pts_right

    @staticmethod
    def getFeetMidPoint(model, useWorldZ=True):
        '''
        Returns a frame in world coordinate system that is the average of the left
        and right foot reference point positions in world frame, the average of the
        left and right foot yaw in world frame, and Z axis aligned with world Z.
        The foot reference point is the average of the foot contact points in the foot frame.
        '''
        contact_pts_left, contact_pts_right = FootstepsDriver.getContactPts()

        contact_pts_mid_left = np.mean(contact_pts_left, axis=0) # mid point on foot relative to foot frame
        contact_pts_mid_right = np.mean(contact_pts_right, axis=0) # mid point on foot relative to foot frame

        t_lf_mid = model.getLinkFrame(_leftFootLink)
        t_lf_mid.PreMultiply()
        t_lf_mid.Translate(contact_pts_mid_left)

        t_rf_mid = model.getLinkFrame(_rightFootLink)
        t_rf_mid.PreMultiply()
        t_rf_mid.Translate(contact_pts_mid_right)

        if (_robotType == 0): # Atlas
            t_feet_mid = transformUtils.frameInterpolate(t_lf_mid, t_rf_mid, 0.5)
        elif (_robotType == 1):
            # Valkyrie v1 Foot orientation is silly
            l_foot_sole = transformUtils.frameFromPositionAndRPY([0,0,0], [180.0, 82.5, 0])
            l_foot_sole.PostMultiply()
            l_foot_sole.Concatenate(t_lf_mid)
            r_foot_sole = transformUtils.frameFromPositionAndRPY([0,0,0], [0.0, -82.5, 0])
            r_foot_sole.PostMultiply()
            r_foot_sole.Concatenate(t_rf_mid)
            t_feet_mid = transformUtils.frameInterpolate(l_foot_sole, r_foot_sole, 0.5)
        elif (_robotType == 2):
            # Valkyrie v2 is better
            t_feet_mid = transformUtils.frameInterpolate(t_lf_mid, t_rf_mid, 0.5)


        if useWorldZ:
            rpy = [0.0, 0.0, np.degrees(transformUtils.rollPitchYawFromTransform(t_feet_mid)[2])]
            return transformUtils.frameFromPositionAndRPY(t_feet_mid.GetPosition(), rpy)
        else:
            return t_feet_mid


    @staticmethod
    def debugDrawFootPoints(model):
        pts_left, pts_right = FootstepsDriver.getContactPts()
        d = DebugData()

        for linkName in [_leftFootLink, _rightFootLink]:

            t = model.getLinkFrame(linkName)
            d.addFrame(t, scale=0.2)

            if (linkName is _leftFootLink):
                pts = pts_left
            else:
                pts = pts_right

            footMidPoint = np.mean(pts, axis=0)
            for p in pts.tolist() + [footMidPoint.tolist()]:
                t.TransformPoint(p, p)
                d.addSphere(p, radius=0.015)

        midpt = FootstepsDriver.getFeetMidPoint(model)
        d.addFrame(midpt, scale=0.2)
        vis.showPolyData(d.getPolyData(), 'foot points debug', parent='debug', colorByName='RGB255')

    def createGoalSteps(self, model, pose):
        distanceForward = 1.0

        fr = model.getLinkFrame(_leftFootLink)
        fl = model.getLinkFrame(_rightFootLink)
        pelvisT = model.getLinkFrame(_pelvisLink)

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
        msg = lcmdrc.footstep_check_request_t()
        msg.initial_state = self.lastFootstepRequest.initial_state
        msg.footstep_plan = self.lastFootstepPlan
        msg.snap_to_terrain = True
        msg.compute_infeasibility = False
        self.sendFootstepPlanCheckRequest(msg)

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
        msg.params.min_num_steps = self.params.properties.min_num_steps
        msg.params.min_step_width = self.params.properties.min_step_width
        msg.params.nom_step_width = self.params.properties.nominal_step_width
        msg.params.max_step_width = self.params.properties.max_step_width
        msg.params.nom_forward_step = self.params.properties.nominal_forward_step
        msg.params.max_forward_step = self.params.properties.max_forward_step
        msg.params.nom_upward_step = self.params.properties.max_upward_step
        msg.params.nom_downward_step = self.params.properties.max_downward_step
        msg.params.planning_mode = self.params.properties.planner_mode
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

    def sendFootstepPlanCheckRequest(self, request, waitForResponse=False, waitTimeout=5000):
        assert isinstance(request, lcmdrc.footstep_check_request_t)

        requestChannel = 'FOOTSTEP_CHECK_REQUEST'
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

        for previousPlan in self.committedPlans:
            if previousPlan.utime == footstepPlan.utime:
                raise Exception("Footstep plan was already executed. Execution of the plan is no longer allowed for safety reasons. You should request a new footstep plan.")

        self.committedPlans.append(footstepPlan)
        self.drawFootstepPlan(footstepPlan, getFootstepsFolder(), alpha=0.3)

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
        rpy = transformUtils.quaternionToRollPitchYaw(msg.orientation)
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
            # print "haven't received POSE_BDI"
            return

        # TODO: This transformation should be rewritten using the LOCAL_TO_LOCAL_BDI frame
        # instead of using FK here

        t_bodybdi  = transformUtils.transformFromPose(self.pose_bdi.pos, self.pose_bdi.orientation)
        t_bodybdi.PostMultiply()

        current_pose = self.jointController.q
        t_bodymain = transformUtils.transformFromPose( current_pose[0:3]  , transformUtils.rollPitchYawToQuaternion(current_pose[3:6])   )
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


class FootstepRequestGenerator(object):

    def __init__(self, footstepsDriver):
        self.footstepsDriver = footstepsDriver

    @staticmethod
    def getRobotStanceFrame(robotModel):
        return FootstepsDriver.getFeetMidPoint(robotModel)

    @staticmethod
    def makeStepFrames(stepFrames, relativeFrame=None, showFrames=False):

        frames = []
        for i, stepFrame in enumerate(stepFrames):

            stepFrame = transformUtils.frameFromPositionAndRPY(stepFrame, [0,0,0])
            stepFrame.PostMultiply()
            if relativeFrame:
                stepFrame.Concatenate(relativeFrame)

            if showFrames:
                obj = vis.updateFrame(stepFrame, 'step frame %d' % i, parent='step frames', scale=0.2)
                stepFrame = obj.transform

            frames.append(stepFrame)

        return frames

    def makeStepMessages(self, stepFrames, leadingFoot, snapToTerrain=False):

        assert leadingFoot in ('left', 'right')
        isRightFootOffset = 0 if leadingFoot == 'left' else 1

        footOriginToSole = -np.mean(FootstepsDriver.getContactPts(), axis=0)

        stepMessages = []
        for i, stepFrame in enumerate(stepFrames):

            t = transformUtils.copyFrame(stepFrame)
            t.PreMultiply()
            t.Translate(footOriginToSole)

            step = lcmdrc.footstep_t()
            step.pos = transformUtils.positionMessageFromFrame(t)
            step.is_right_foot = (i + isRightFootOffset) % 2
            step.params = self.footstepsDriver.getDefaultStepParams()

            step.fixed_x = True
            step.fixed_y = True
            step.fixed_z = True
            step.fixed_roll = True
            step.fixed_pitch = True
            step.fixed_yaw = True

            if snapToTerrain:
                step.fixed_z = False
                step.fixed_roll = False
                step.fixed_pitch = False

            stepMessages.append(step)

        return stepMessages

    def makeFootstepRequest(self, startPose, stepFrames, leadingFoot, numberOfFillSteps=0, snapToTerrain=False):

        stepMessages = self.makeStepMessages(stepFrames, leadingFoot, snapToTerrain=snapToTerrain)
        request = self.footstepsDriver.constructFootstepPlanRequest(startPose)
        request.num_goal_steps = len(stepMessages)
        request.goal_steps = stepMessages
        request.params.leading_foot = lcmdrc.footstep_plan_params_t.LEAD_LEFT if leadingFoot == 'left' else lcmdrc.footstep_plan_params_t.LEAD_RIGHT
        request.params.max_num_steps = len(stepMessages) + numberOfFillSteps
        return request
