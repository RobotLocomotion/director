import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
import director
from director import lcmUtils
from director import applogic as app
from director.utime import getUtime
from director.timercallback import TimerCallback
from director.lcmframe import positionMessageFromFrame

import numpy as np
import math
from time import sleep

from . import vtkAll as vtk

from time import time
from copy import deepcopy
from director import transformUtils
import director.visualization as vis
import director.objectmodel as om
from director import robotstate


from director import ioUtils as io
from director import vtkNumpy as vnp
from director import segmentation


# lcmtypes:
import drc as lcmdrc
from pronto.indexed_measurement_t import indexed_measurement_t



def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)

class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


class NavigationPanel(object):

    def __init__(self, jointController, footstepDriver):

        self.jointController = jointController
        self.footstepDriver = footstepDriver

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddNavigation.ui')
        assert uifile.open(uifile.ReadOnly)
        self.widget = loader.load(uifile)
        self.ui = WidgetDict(self.widget.children())
        self.ui.captureButton.connect("clicked()", self.onCaptureButton)
        self.ui.visualizeButton.connect("clicked()", self.onVisualizeButton)
        self.ui.planButton.connect("clicked()", self.onPlanButton)
        self.ui.reversePlanButton.connect("clicked()", self.onReversePlanButton)
        self.ui.initAtZeroButton.connect("clicked()", self.onInitAtZeroButton)
        self.ui.restartNavButton.connect("clicked()", self.onRestartNavButton)
        self.ui.showMapButton.connect("clicked()", self.onShowMapButton)
        self.ui.hideMapButton.connect("clicked()", self.onHideMapButton)
        self.ui.disableLaserButton.connect("clicked()", self.onDisableLaserButton)
        self.ui.enableLaserButton.connect("clicked()", self.onEnableLaserButton)

        self.ui.startNewMapButton.connect("clicked()", self.onStartNewMapButton)
        self.ui.useNewMapButton.connect("clicked()", self.onUseNewMapButton)

        # Data Variables:
        self.goal = dict()
        self.init_frame = None
        self.octomap_cloud = None

    ###############################

    def getSelectedGoalName(self):
        goal_name = self.ui.comboBox.currentText
        return goal_name

    def getFrameFromCombo(self):
        pose = self.goal[ self.getSelectedGoalName() ]
        #frame = transformUtils.frameFromPositionAndRPY(pose[0:3], np.degrees(pose[3:6]) )
        return pose

    def printTransform(self,t,message):
        p = t.GetPosition()
        q = t.GetOrientation()
        print("%f %f %f | %f %f %f | %s" % (p[0],p[1],p[2],q[0],q[1],q[2],message))


    def onCaptureButton(self):
        print("capture") #,self.jointController.q
        # body frame:
        #goal = self.jointController.q[0:6]

        # mid point of feet (as used by robin)
        # this assumes model publisher exists - not always true currently
        model = om.findObjectByName("robot state model")
        t_feet_mid = self.footstepDriver.getFeetMidPoint(model)
        #vis.updateFrame(t_feet_mid, "Current Goal New", parent="navigation")

        goal_name = "Goal %d" % len(self.goal)
        self.goal[goal_name] = t_feet_mid
        self.updateComboBox()

    def updateComboBox(self):
        self.ui.comboBox.clear()
        self.ui.comboBox.addItems(list(self.goal.keys()))

    def onVisualizeButton(self):
        print("viz",self.ui.comboBox.currentText)
        frame = self.getFrameFromCombo()

        #vis.showFrame(frame, self.getSelectedGoalName(), parent="navigation", scale=0.35, visible=True)
        #vis.updateFrame(frame, self.getSelectedGoalName(), parent="navigation")
        vis.updateFrame(frame, "Current Goal", parent="navigation")

    def onPlanButton(self):
        print("plan",self.ui.comboBox.currentText)

        goalFrame = self.getFrameFromCombo()
        startPose = self.jointController.q

        request = self.footstepDriver.constructFootstepPlanRequest(startPose, goalFrame)
        self.footstepDriver.sendFootstepPlanRequest(request)

    def onReversePlanButton(self):
        print("reverse me")

        #self.clearFootstepPlan()
        lastPlan = self.footstepDriver.lastFootstepPlan
        reversedPlan = lastPlan.decode( lastPlan.encode() ) # decode and encode ensures deepcopy

        #planFolder = getFootstepsFolder()
        #self.drawFootstepPlan(msg, planFolder)

        for i, footstep in enumerate(lastPlan.footsteps):
          j = lastPlan.num_steps - 1 - i
          reversedPlan.footsteps[j] = footstep

        self.footstepDriver.onFootstepPlan(reversedPlan)

    def onRestartNavButton(self):
        ready_init = lcmdrc.utime_t()
        ready_init.utime = getUtime()
        lcmUtils.publish('STATE_EST_RESTART', ready_init)


    def onInitAtZeroButton(self):
        self.sendInitAtZero()

    def sendInitAtZero(self):
        self.sendReadyMessage()

        p1 = [0,0,0.85]
        #init_frame = transformUtils.frameFromPositionAndRPY( p1 , [0,0,0] )
        #vis.updateFrame(init_frame, "init pose", parent="navigation")
        self.sendInitMessage(p1, 0)


    def pointPickerNavigationInit(self,p1, p2):
        self.sendReadyMessage()

        yaw = math.atan2( p2[1] - p1[1] , p2[0] - p1[0] )
        p1[2] = p1[2] + 0.85
        #init_frame = transformUtils.frameFromPositionAndRPY(p1, [0,0,yaw*180/math.pi])
        #vis.updateFrame(init_frame, "init pose", parent="navigation")
        self.sendInitMessage(p1, yaw)


    def sendReadyMessage(self):
        ready_init = lcmdrc.utime_t()
        ready_init.utime = getUtime()
        lcmUtils.publish('STATE_EST_READY', ready_init)
        sleep(1) # sleep needed to give SE time to restart


    def sendInitMessage(self, pos, yaw):
        init = indexed_measurement_t()
        init.utime = getUtime()
        init.state_utime = init.utime
        init.measured_dim = 4
        init.z_effective = [ pos[0], pos[1], pos[2] , yaw ]
        init.z_indices = [9, 10, 11, 8]

        init.measured_cov_dim = init.measured_dim*init.measured_dim
        init.R_effective= [0] * init.measured_cov_dim
        init.R_effective[0]  = 0.25
        init.R_effective[5]  = 0.25
        init.R_effective[10] = 0.25
        init.R_effective[15] =  math.pow( 50*math.pi/180 , 2 )

        lcmUtils.publish('MAV_STATE_EST_VIEWER_MEASUREMENT', init)


    def onShowMapButton(self):
        # reloads the map each time - in case its changed on disk:
        #if (self.octomap_cloud is None):
        filename = director.getDRCBaseDir() + "/software/build/data/octomap.pcd"
        self.octomap_cloud = io.readPolyData(filename) # c++ object called vtkPolyData

        assert (self.octomap_cloud.GetNumberOfPoints() !=0 )

        # clip point cloud to height - doesnt work very well yet. need to know robot's height
        #self.octomap_cloud = segmentation.cropToLineSegment(self.octomap_cloud, np.array([0,0,-10]), np.array([0,0,3]) )

        # access to z values
        #points= vnp.getNumpyFromVtk(self.octomap_cloud, 'Points')
        #zvalues = points[:,2]

        # remove previous map:
        folder = om.getOrCreateContainer("segmentation")
        om.removeFromObjectModel(folder)
        vis.showPolyData(self.octomap_cloud, 'prior map', alpha=1.0, color=[0,0,0.4])


    def onHideMapButton(self):
        folder = om.getOrCreateContainer("segmentation")
        om.removeFromObjectModel(folder)


    def onEnableLaserButton(self):
        msg = lcmdrc.utime_t()
        msg.utime = getUtime()
        lcmUtils.publish('STATE_EST_LASER_ENABLE', msg)

    def onDisableLaserButton(self):
        msg = lcmdrc.utime_t()
        msg.utime = getUtime()
        lcmUtils.publish('STATE_EST_LASER_DISABLE', msg)

    def onStartNewMapButton(self):
        '''
        Send a trigger to the Create Octomap process to start a new map
        '''
        msg = lcmdrc.utime_t()
        msg.utime = getUtime()
        lcmUtils.publish('STATE_EST_START_NEW_MAP', msg)


    def onUseNewMapButton(self):
        '''
        Send a trigger to the GPF to use the newly created map
        '''
        msg = lcmdrc.utime_t()
        msg.utime = getUtime()
        lcmUtils.publish('STATE_EST_USE_NEW_MAP', msg)

    def pointPickerStoredFootsteps(self,p1, p2):

        yaw = math.atan2( p2[1] - p1[1] , p2[0] - p1[0] )*180/math.pi + 90
        frame_p1 = transformUtils.frameFromPositionAndRPY(p1, [0,0,yaw])

        blockl = 0.3937
        blockh = 0.142875
        sep = 0.11

        frame_pt_to_centerline = transformUtils.frameFromPositionAndRPY( [0, -blockl/2, 0], [0,0,0])

        frame_pt_to_centerline.PostMultiply()
        frame_pt_to_centerline.Concatenate(frame_p1)

        vis.updateFrame(frame_pt_to_centerline, "corner", parent="navigation")



# up/down 1 block (Original)
#        flist = np.array( [[ blockl*-0.5 , .1  , 0      , 0 , 0 , 0] ,
#                           [ blockl*-0.5 , -.1 , 0      , 0 , 0 , 0] ,
#                           [ blockl*0.5 - 0.03 , .1  , blockh , 0 , 0 , 0] ,
#                           [ blockl*0.5 + 0.06  ,-.1  , blockh , 0 , 0 , 0] ,
#                           [ blockl*1.5        , .1  , 0      , 0 , 0 , 0] ,
#                           [ blockl*1.5 +0.03  ,-.1  , 0      , 0 , 0 , 0]])

# up/down 1 block (Newer)
#        flist = np.array( [[ blockl*-0.5       , .1  , 0      , 0 , 0 , 0] ,
#                           [ blockl*-0.5       , -.1 , 0      , 0 , 0 , 0] ,
#	                   [ blockl*0.5 - 0.03 , .1  , blockh , 0 , 0 , 0] ,
#                           [ blockl*0.5 + 0.04 ,-.1  , blockh , 0 , 0 , 0] ,
#                           [ blockl*1.5 - 0.03 , .1  , 0      , 0 , 0 , 0] ,
#                           [ blockl*1.5 + 0.00 ,-.1  , 0      , 0 , 0 , 0]])

# up 3 blocks
#        flist = np.array( [[ blockl*-0.5       , .1  , 0      , 0 , 0 , 0] ,
#                           [ blockl*-0.5       , -.1 , 0      , 0 , 0 , 0] ,
#	                   [ blockl*0.5 - 0.03 , .1  , blockh , 0 , 0 , 0] ,
#                           [ blockl*0.5 + 0.0  ,-.1  , blockh , 0 , 0 , 0] ,
#                           [ blockl*1.5 - 0.03 , .1  , 2*blockh, 0 , 0 , 0] ,
#                           [ blockl*1.5 + 0.0  ,-.1  , 2*blockh, 0 , 0 , 0],
#                           [ blockl*2.5 - 0.03 , .1  , 3*blockh, 0 , 0 , 0] ,
#                           [ blockl*2.5 + 0.0  ,-.1  , 3*blockh, 0 , 0 , 0]])

# up and down 3 blocks (original)
#        flist = np.array( [[ blockl*-0.5       , .1  , 0       , 0 , 0 , 0] ,
#                           [ blockl*-0.5       , -.1 , 0       , 0 , 0 , 0] ,
#	                   [ blockl*0.5 - 0.03 , .1  , blockh  , 0 , 0 , 0] ,
#                           [ blockl*0.5 + 0.0  ,-.1  , blockh  , 0 , 0 , 0] ,
#                           [ blockl*1.5 - 0.03 , .1  , 2*blockh, 0 , 0 , 0] ,
#                           [ blockl*1.5 + 0.0  ,-.1  , 2*blockh, 0 , 0 , 0],
#                           [ blockl*2.5 - 0.03 , .1  , 3*blockh, 0 , 0 , 0] ,
#                           [ blockl*2.5 + 0.03  ,-.1 , 3*blockh, 0 , 0 , 0],
#                           [ blockl*3.5 - 0.03 , .1  , 2*blockh, 0 , 0 , 0] ,
#                           [ blockl*3.5 + 0.03  ,-.1 , 2*blockh, 0 , 0 , 0],
#                           [ blockl*4.5 - 0.03 , .1  , 1*blockh, 0 , 0 , 0] ,
#                           [ blockl*4.5 + 0.03  ,-.1 , 1*blockh, 0 , 0 , 0],
#                           [ blockl*5.5 - 0.03 , .1  , 0       , 0 , 0 , 0] ,
#                           [ blockl*5.5 + 0.03  ,-.1 , 0       , 0 , 0 , 0]])


# up and down 3 blocks (used in video)
#        r =1
#        flist = np.array( [[ blockl*-0.5       , sep  , 0       , 0 , 0 , 0, 0],
#                          [ blockl*-0.5       , -sep , 0       , 0 , 0 , 0, r],
#                          [ blockl*0.5 - 0.03 , sep  , blockh  , 0 , 0 , 0, 0],
#                           [ blockl*0.5 + 0.0  ,-sep  , blockh  , 0 , 0 , 0, r],
#                           [ blockl*1.5 - 0.03 , sep  , 2*blockh, 0 , 0 , 0, 0],
#                           [ blockl*1.5 + 0.0  ,-sep  , 2*blockh, 0 , 0 , 0, r],
#                           [ blockl*2.5 - 0.03 , sep  , 3*blockh, 0 , 0 , 0, 0],
#                           [ blockl*2.5 + 0.03  ,-sep , 3*blockh, 0 , 0 , 0, r],
#                           [ blockl*3.5 - 0.03 , sep  , 2*blockh, 0 , 0 , 0, 0],
#                           [ blockl*3.5 + 0.03  ,-sep , 2*blockh, 0 , 0 , 0, r],
#                           [ blockl*4.5 - 0.03 , sep  , 1*blockh, 0 , 0 , 0, 0],
#                           [ blockl*4.5 + 0.03  ,-sep , 1*blockh, 0 , 0 , 0, r],
#                           [ blockl*5.5 - 0.03 , sep  , 0       , 0 , 0 , 0, 0],
#                           [ blockl*5.5 + 0.03  ,-sep , 0       , 0 , 0 , 0, r], # half
#                           [ blockl*5.5 - 0.03 , sep  , 0       , 0 , 0 , 0, 0], # extra step for planner
#                           [ blockl*4.5 + 0.03  ,-sep , 1*blockh, 0 , 0 , 0, r], # invert order
#                           [ blockl*4.5 - 0.03 , sep  , 1*blockh, 0 , 0 , 0, 0],
#                           [ blockl*3.5 + 0.03  ,-sep , 2*blockh, 0 , 0 , 0, r],
#                           [ blockl*3.5 - 0.03 , sep  , 2*blockh, 0 , 0 , 0, 0],
#                           [ blockl*2.5 + 0.03  ,-sep , 3*blockh, 0 , 0 , 0, r], # top
#                           [ blockl*2.5 - 0.06 , sep  , 3*blockh, 0 , 0 , 0, 0], # top
#                           [ blockl*1.5 + 0.04 ,-sep  , 2*blockh, 0 , 0 , 0, r],
#                           [ blockl*1.5 - 0.06 , sep  , 2*blockh, 0 , 0 , 0, 0],
#                           [ blockl*0.5 + 0.04 ,-sep  , blockh  , 0 , 0 , 0, r],
#                           [ blockl*0.5 - 0.06 , sep  , blockh  , 0 , 0 , 0, 0],
#                           [ blockl*-0.5+ 0.04 , -sep , 0       , 0 , 0 , 0, r],
#                           [ blockl*-0.5 - 0.03, sep  , 0       , 0 , 0 , 0, 0]])


# up and down 2 blocks (used in vicon april 2014)
#        r =1
#        flist = np.array( [[ blockl*-0.5       , sep  , 0       , 0 , 0 , 0, 0],
#                           [ blockl*-0.5       , -sep , 0       , 0 , 0 , 0, r], # start
#                           [ blockl*0.5 - 0.03 , sep  , blockh  , 0 , 0 , 0, 0],
#                           [ blockl*0.5 + 0.0  ,-sep  , blockh  , 0 , 0 , 0, r], # 1
#                           [ blockl*1.5 - 0.03 , sep  , 2*blockh, 0 , 0 , 0, 0],
#                           [ blockl*1.5 + 0.03 ,-sep  , 2*blockh, 0 , 0 , 0, r], # 2
#                           [ blockl*2.5 - 0.03 , sep  , 1*blockh, 0 , 0 , 0, 0],
#                           [ blockl*2.5 + 0.03  ,-sep , 1*blockh, 0 , 0 , 0, r], # 1d
#                           [ blockl*3.5 - 0.03 , sep  , 0       , 0 , 0 , 0, 0],
#                           [ blockl*3.5 + 0.03  ,-sep , 0       , 0 , 0 , 0, r], # half
#                           [ blockl*3.5 - 0.03 , sep  , 0       , 0 , 0 , 0, 0], # extra step for planner
#                           [ blockl*2.5 + 0.03  ,-sep , 1*blockh, 0 , 0 , 0, r], # invert order
#                           [ blockl*2.5 - 0.03 , sep  , 1*blockh, 0 , 0 , 0, 0],
#                           [ blockl*1.5 + 0.03  ,-sep , 2*blockh, 0 , 0 , 0, r],
#                           [ blockl*1.5 - 0.06 , sep  , 2*blockh, 0 , 0 , 0, 0],
#                           [ blockl*0.5 + 0.04 ,-sep  , blockh  , 0 , 0 , 0, r],
#                           [ blockl*0.5 - 0.06 , sep  , blockh  , 0 , 0 , 0, 0],
#                           [ blockl*-0.5+ 0.04 , -sep , 0       , 0 , 0 , 0, r],
#                           [ blockl*-0.5 - 0.03, sep  , 0       , 0 , 0 , 0, 0]])

# up 6 blocks (used in journal paper in oct 2014)
#        r =1
#        flist = np.array( [[ blockl*-0.5       , sep  , 0       , 0 , 0 , 0, 0],
#                           [ blockl*-0.5       ,-sep  , 0       , 0 , 0 , 0, r],
#                           [ blockl*0.5 - 0.03 , sep  , blockh  , 0 , 0 , 0, 0],
#                           [ blockl*0.5 - 0.02 ,-sep  , blockh  , 0 , 0 , 0, r],
#                           [ blockl*1.5 - 0.03 , sep  , 2*blockh, 0 , 0 , 0, 0],
#                           [ blockl*1.5 - 0.02 ,-sep  , 2*blockh, 0 , 0 , 0, r],
#                           [ blockl*2.5 - 0.03 , sep  , 3*blockh, 0 , 0 , 0, 0],
#                           [ blockl*2.5 - 0.02 ,-sep  , 3*blockh, 0 , 0 , 0, r],
#                           [ blockl*3.5 - 0.03 , sep  , 4*blockh, 0 , 0 , 0, 0],
#                           [ blockl*3.5 - 0.02 ,-sep  , 4*blockh, 0 , 0 , 0, r],
#                           [ blockl*4.5 - 0.03 , sep  , 5*blockh, 0 , 0 , 0, 0],
#                           [ blockl*4.5 - 0.02 ,-sep  , 5*blockh, 0 , 0 , 0, r],
#                           [ blockl*5.5 - 0.03 , sep  , 6*blockh, 0 , 0 , 0, 0],
#                           [ blockl*5.5 - 0.02 ,-sep  , 6*blockh, 0 , 0 , 0, r]])

# continuous walking - first two blocks up
        r =1
        flist = np.array( [[ blockl*-0.5  - 0.03     , sep  , 0       , 0 , 0 , 0, 0],
                           [ blockl*-0.5  + 0.03      ,-sep  , 0       , 0 , 0 , 0, r],
                           [ blockl*0.5 - 0.03 , sep  , blockh  , 0 , 0 , 0, 0],
                           [ blockl*0.5 + 0.03 ,-sep  , blockh  , 0 , 0 , 0, r]])

        contact_pts = self.footstepDriver.getContactPts()
        contact_pts_mid = np.mean(contact_pts, axis=0) # mid point on foot relative to foot frame
        foot_to_sole = transformUtils.frameFromPositionAndRPY( contact_pts_mid, [0,0,0]).GetLinearInverse()

        flist_shape = flist.shape
        self.goalSteps = []
        for i in range(flist_shape[0]):
            step_t = vtk.vtkTransform()
            step_t.PostMultiply()
            step_t.Concatenate(transformUtils.frameFromPositionAndRPY(flist[i,0:3] , flist[i,3:6]))
            step_t.Concatenate(foot_to_sole)
            step_t.Concatenate(frame_pt_to_centerline)
            step = lcmdrc.footstep_t()
            step.pos = positionMessageFromFrame(step_t)
            step.is_right_foot =  flist[i,6] # is_right_foot
            step.params = self.footstepDriver.getDefaultStepParams()
            # Visualization via triads
            #vis.updateFrame(step_t, str(i), parent="navigation")
            self.goalSteps.append(step)

        startPose = self.jointController.q
        request = self.footstepDriver.constructFootstepPlanRequest(startPose)
        request.num_goal_steps = len(self.goalSteps)
        request.goal_steps = self.goalSteps
        lcmUtils.publish('FOOTSTEP_PLAN_REQUEST', request)


def _getAction():
    return app.getToolBarActions()['ActionNavigationPanel']


def init(jointController, footstepDriver):

    global panel
    global dock

    panel = NavigationPanel(jointController, footstepDriver)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel
