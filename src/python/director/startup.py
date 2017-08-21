# This script is executed in the main console namespace so
# that all the variables defined here become console variables.



import director
from director import irisdriver

import os
import sys
import PythonQt
import json
from PythonQt import QtCore, QtGui
from time import time
import imp
import director.applogic as app
from director import drcargs
from director import vtkAll as vtk
from director import matlab
from director import jointcontrol
from director import callbacks
from director import camerabookmarks
from director import cameracontrol
from director import cameracontrolpanel
from director import bihandeddemo
from director import debrisdemo
from director import doordemo
from director import drilldemo
from director import valvedemo
from director import drivingplanner
from director import egressplanner
from director import polarisplatformplanner
from director import surprisetask
from director import continuouswalkingdemo
from director import sitstandplanner
from director import walkingtestdemo
from director import terraintask
from director import ikplanner
from director import objectmodel as om
from director import spreadsheet
from director import transformUtils
from director import tdx
from director import skybox
from director import perception
from director import segmentation
from director import cameraview
from director import colorize
from director import drakevisualizer
from director.fieldcontainer import FieldContainer
from director import robotstate
from director import roboturdf
from director import robotsystem
from director import affordancepanel
from director import filterUtils
from director import footstepsdriver
from director import footstepsdriverpanel
from director import framevisualization
from director import lcmloggerwidget
from director import lcmgl
from director import lcmoctomap
from director import lcmcollections
from director import atlasdriver
from director import atlasdriverpanel
from director import multisensepanel
from director import navigationpanel
from director import handcontrolpanel
from director import sensordatarequestpanel
from director import tasklaunchpanel
from director.jointpropagator import JointPropagator
from director import planningutils
from director import viewcolors

from director import coursemodel

from director import copmonitor
from director import robotplanlistener
from director import handdriver
from director import planplayback
from director import playbackpanel
from director import screengrabberpanel
from director import splinewidget
from director import teleoppanel
from director import motionplanningpanel
from director import vtkNumpy as vnp
from director import visualization as vis
from director import actionhandlers
from director.timercallback import TimerCallback
from director.pointpicker import PointPicker, ImagePointPicker
from director import segmentationpanel
from director import lcmUtils
from director.utime import getUtime
from director.shallowCopy import shallowCopy

from director import segmentationroutines
from director import trackers

from director import gamepad
from director import blackoutmonitor

from director.tasks import robottasks as rt
from director.tasks import taskmanagerwidget
from director.tasks.descriptions import loadTaskDescriptions
import drc as lcmdrc
import bot_core as lcmbotcore
import maps as lcmmaps
import atlas

from collections import OrderedDict
import functools
import math

import numpy as np
from director.debugVis import DebugData
from director import ioUtils as io

drcargs.requireStrict()
drcargs.args()
app.startup(globals())
om.init(app.getMainWindow().objectTree(), app.getMainWindow().propertiesPanel())
actionhandlers.init()

quit = app.quit
exit = quit
view = app.getDRCView()
camera = view.camera()
tree = app.getMainWindow().objectTree()
orbit = cameracontrol.OrbitController(view)
showPolyData = segmentation.showPolyData
updatePolyData = segmentation.updatePolyData


###############################################################################


robotSystem = robotsystem.create(view)
globals().update(dict(robotSystem))


useIk = True
useRobotState = True
usePerception = True
useGrid = True
useSpreadsheet = True
useFootsteps = True
useHands = True
usePlanning = True
useHumanoidDRCDemos = True
useAtlasDriver = True
useLCMGL = True
useOctomap = True
useCollections = True
useLightColorScheme = True
useLoggingWidget = True
useDrakeVisualizer = True
useNavigationPanel = True
useFootContactVis = False
useFallDetectorVis = True
useCameraFrustumVisualizer = True
useControllerRate = True
useForceDisplay = True
useSkybox = False
useDataFiles = True
useGamepad = True
useBlackoutText = False
useRandomWalk = True
useCOPMonitor = True
useCourseModel = False
useLimitJointsSentToPlanner = False
useFeetlessRobot = False

# Sensor Flags
useKinect = False
useMultisense = True
useOpenniDepthImage = False

poseCollection = PythonQt.dd.ddSignalMap()
costCollection = PythonQt.dd.ddSignalMap()

if 'userConfig' in drcargs.getDirectorConfig():
    if 'fixedBaseArm' in drcargs.getDirectorConfig()['userConfig']:
        ikPlanner.fixedBaseArm = True

if 'disableComponents' in drcargs.getDirectorConfig():
    for component in drcargs.getDirectorConfig()['disableComponents']:
        print("Disabling", component)
        locals()[component] = False

if 'enableComponents' in drcargs.getDirectorConfig():
    for component in drcargs.getDirectorConfig()['enableComponents']:
        print("Enabling", component)
        locals()[component] = True


if useSpreadsheet:
    spreadsheet.init(poseCollection, costCollection)


if useIk:
    def onIkStartup(ikServer, startSuccess):
        if startSuccess:
            app.getMainWindow().statusBar().showMessage('Planning server started.', 2000)
        else:
            app.showErrorMessage('Error detected while starting the matlab planning server. '
                                 'Please check the output console for more information.', title='Error starting matlab')

    ikServer.outputConsole = app.getOutputConsole()
    ikServer.infoFunc = app.displaySnoptInfo
    ikServer.connectStartupCompleted(onIkStartup)
    startIkServer()


if useAtlasDriver:
    atlasdriver.systemStatus.outputConsole = app.getOutputConsole()
    atlasdriverpanel.init(atlasDriver)
else:
    app.removeToolbarMacro('ActionAtlasDriverPanel')


if usePerception:
    segmentationpanel.init()
    cameraview.init()
    colorize.init()

    cameraview.cameraView.initImageRotations(robotStateModel)
    cameraview.cameraView.rayCallback = segmentation.extractPointsAlongClickRay

    if useMultisense:
        multisensepanel.init(perception.multisenseDriver)
    else:
        app.removeToolbarMacro('ActionMultisensePanel')

    sensordatarequestpanel.init()

    # for kintinuous, use 'CAMERA_FUSED', 'CAMERA_TSDF'
    disparityPointCloud = segmentation.DisparityPointCloudItem('stereo point cloud', 'MULTISENSE_CAMERA', 'MULTISENSE_CAMERA_LEFT', cameraview.imageManager)
    disparityPointCloud.addToView(view)
    om.addToObjectModel(disparityPointCloud, parentObj=om.findObjectByName('sensors'))

    def createPointerTracker():
        return trackers.PointerTracker(robotStateModel, disparityPointCloud)


if useOpenniDepthImage:
    openniDepthPointCloud = segmentation.DisparityPointCloudItem('openni point cloud', 'OPENNI_FRAME', 'OPENNI_FRAME_LEFT', cameraview.imageManager)
    openniDepthPointCloud.addToView(view)
    om.addToObjectModel(openniDepthPointCloud, parentObj=om.findObjectByName('sensors'))


if useGrid:
    grid = vis.showGrid(view, color=[0,0,0], alpha=0.1)
    grid.setProperty('Surface Mode', 'Surface with edges')

app.setBackgroundColor([0.3, 0.3, 0.35], [0.95,0.95,1])

viewOptions = vis.ViewOptionsItem(view)
om.addToObjectModel(viewOptions, parentObj=om.findObjectByName('sensors'))

viewBackgroundLightHandler = viewcolors.ViewBackgroundLightHandler(viewOptions, grid,
                                app.getToolsMenuActions()['ActionToggleBackgroundLight'])
if not useLightColorScheme:
    viewBackgroundLightHandler.action.trigger()

if useHands:
    handcontrolpanel.init(lHandDriver, rHandDriver, robotStateModel, robotStateJointController, view)
else:
    app.removeToolbarMacro('ActionHandControlPanel')


if useFootsteps:
    footstepsPanel = footstepsdriverpanel.init(footstepsDriver, robotStateModel, robotStateJointController, irisDriver)
else:
    app.removeToolbarMacro('ActionFootstepPanel')


if useLCMGL:
    lcmglManager = lcmgl.init(view)
    app.MenuActionToggleHelper('Tools', 'Renderer - LCM GL', lcmglManager.isEnabled, lcmglManager.setEnabled)

if useOctomap:
    octomapManager = lcmoctomap.init(view)
    app.MenuActionToggleHelper('Tools', 'Renderer - Octomap', octomapManager.isEnabled, octomapManager.setEnabled)

if useCollections:
    collectionsManager = lcmcollections.init(view)
    app.MenuActionToggleHelper('Tools', 'Renderer - Collections', collectionsManager.isEnabled, collectionsManager.setEnabled)

if useDrakeVisualizer:
    drakeVisualizer = drakevisualizer.DrakeVisualizer(view)
    app.MenuActionToggleHelper('Tools', 'Renderer - Drake', drakeVisualizer.isEnabled, drakeVisualizer.setEnabled)


if useNavigationPanel:
    navigationPanel = navigationpanel.init(robotStateJointController, footstepsDriver)
    picker = PointPicker(view, callback=navigationPanel.pointPickerStoredFootsteps, numberOfPoints=2)
    #picker.start()

if usePlanning:
    def showPose(pose):
        playbackRobotModel.setProperty('Visible', True)
        playbackJointController.setPose('show_pose', pose)

    def playPlan(plan):
        playPlans([plan])

    def playPlans(plans):
        planPlayback.stopAnimation()
        playbackRobotModel.setProperty('Visible', True)
        planPlayback.playPlans(plans, playbackJointController)

    def playManipPlan():
        playPlan(manipPlanner.lastManipPlan)

    def playWalkingPlan():
        playPlan(footstepsDriver.lastWalkingPlan)

    def plotManipPlan():
        planPlayback.plotPlan(manipPlanner.lastManipPlan)

    def planStand():
        ikPlanner.computeStandPlan(robotStateJointController.q)

    def planNominal():
        ikPlanner.computeNominalPlan(robotStateJointController.q)

    def planHomeStand():
        ''' Move the robot back to a safe posture, 1m above its feet, w/o moving the hands '''
        ikPlanner.computeHomeStandPlan(robotStateJointController.q, footstepsDriver.getFeetMidPoint(robotStateModel), 1.0167)

    def planHomeNominal():
        ''' Move the robot back to a safe posture, 1m above its feet, w/o moving the hands '''
        ikPlanner.computeHomeNominalPlan(robotStateJointController.q, footstepsDriver.getFeetMidPoint(robotStateModel), 1.0167)

    if useMultisense:
        def fitDrillMultisense():
            pd = om.findObjectByName('Multisense').model.revPolyData
            om.removeFromObjectModel(om.findObjectByName('debug'))
            segmentation.findAndFitDrillBarrel(pd)

        def refitBlocks(autoApprove=True):
            polyData = om.findObjectByName('Multisense').model.revPolyData
            segmentation.updateBlockAffordances(polyData)
            if autoApprove:
                approveRefit()

    def approveRefit():
        for obj in om.getObjects():
            if isinstance(obj, segmentation.BlockAffordanceItem):
                if 'refit' in obj.getProperty('Name'):
                    originalObj = om.findObjectByName(obj.getProperty('Name').replace(' refit', ''))
                    if originalObj:
                        originalObj.params = obj.params
                        originalObj.polyData.DeepCopy(obj.polyData)
                        originalObj.actor.GetUserTransform().SetMatrix(obj.actor.GetUserTransform().GetMatrix())
                        originalObj.actor.GetUserTransform().Modified()
                        obj.setProperty('Visible', False)


    def sendDataRequest(requestType, repeatTime=0.0):
      msg = lcmmaps.data_request_t()
      msg.type = requestType
      msg.period = int(repeatTime*10) # period is specified in tenths of a second

      msgList = lcmmaps.data_request_list_t()
      msgList.utime = getUtime()
      msgList.requests = [msg]
      msgList.num_requests = len(msgList.requests)
      lcmUtils.publish('DATA_REQUEST', msgList)

    def sendSceneHeightRequest(repeatTime=0.0):
        sendDataRequest(lcmmaps.data_request_t.HEIGHT_MAP_SCENE, repeatTime)

    def sendWorkspaceDepthRequest(repeatTime=0.0):
        sendDataRequest(lcmmaps.data_request_t.DEPTH_MAP_WORKSPACE_C, repeatTime)

    def sendSceneDepthRequest(repeatTime=0.0):
        sendDataRequest(lcmmaps.data_request_t.DEPTH_MAP_SCENE, repeatTime)

    def sendFusedDepthRequest(repeatTime=0.0):
        sendDataRequest(lcmmaps.data_request_t.FUSED_DEPTH, repeatTime)

    def sendFusedHeightRequest(repeatTime=0.0):
        sendDataRequest(lcmmaps.data_request_t.FUSED_HEIGHT, repeatTime)


    handJoints = []
    if drcargs.args().directorConfigFile.find('atlas') != -1:
        handJoints = roboturdf.getRobotiqJoints() + ['neck_ay']
    else:
        for handModel in ikPlanner.handModels:
            handJoints += handModel.handModel.model.getJointNames()
        # filter base joints out
        handJoints = [ joint for joint in handJoints if joint.find('base')==-1 ]

    teleopJointPropagator = JointPropagator(robotStateModel, teleopRobotModel, handJoints)
    playbackJointPropagator = JointPropagator(robotStateModel, playbackRobotModel, handJoints)
    def doPropagation(model=None):
        if teleopRobotModel.getProperty('Visible'):
            teleopJointPropagator.doPropagation()
        if playbackRobotModel.getProperty('Visible'):
            playbackJointPropagator.doPropagation()
    robotStateModel.connectModelChanged(doPropagation)

    #app.addToolbarMacro('scene height', sendSceneHeightRequest)
    #app.addToolbarMacro('scene depth', sendSceneDepthRequest)
    #app.addToolbarMacro('stereo height', sendFusedHeightRequest)
    #app.addToolbarMacro('stereo depth', sendFusedDepthRequest)

    if useLimitJointsSentToPlanner:
        planningUtils.clampToJointLimits = True

    jointLimitChecker = teleoppanel.JointLimitChecker(robotStateModel, robotStateJointController)
    jointLimitChecker.setupMenuAction()
    jointLimitChecker.start()

    if useMultisense:
        spindleSpinChecker =  multisensepanel.SpindleSpinChecker(spindleMonitor)
        spindleSpinChecker.setupMenuAction()

    postureShortcuts = teleoppanel.PosturePlanShortcuts(robotStateJointController, ikPlanner, planningUtils)


    if useMultisense:
        def drillTrackerOn():
            om.findObjectByName('Multisense').model.showRevolutionCallback = fitDrillMultisense

        def drillTrackerOff():
            om.findObjectByName('Multisense').model.showRevolutionCallback = None

    def fitPosts():
        segmentation.fitVerticalPosts(segmentation.getCurrentRevolutionData())
        affordancePanel.onGetRaycastTerrain()

    ikPlanner.addPostureGoalListener(robotStateJointController)

    playbackpanel.addPanelToMainWindow(playbackPanel)
    teleoppanel.addPanelToMainWindow(teleopPanel)

    motionPlanningPanel = motionplanningpanel.init(planningUtils, robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController,
                            ikPlanner, manipPlanner, affordanceManager, playbackPanel.setPlan, playbackPanel.hidePlan, footstepsDriver)

    if useGamepad:
        gamePad = gamepad.Gamepad(teleopPanel, teleopJointController, ikPlanner, view)

    if useBlackoutText:
        blackoutMonitor = blackoutmonitor.BlackoutMonitor(robotStateJointController, view, cameraview, mapServerSource)


    taskPanels = OrderedDict()

    if useHumanoidDRCDemos:
        debrisDemo = debrisdemo.DebrisPlannerDemo(robotStateModel, robotStateJointController, playbackRobotModel,
                        ikPlanner, manipPlanner, atlasdriver.driver, lHandDriver,
                        perception.multisenseDriver, refitBlocks)

        drillDemo = drilldemo.DrillPlannerDemo(robotStateModel, playbackRobotModel, teleopRobotModel, footstepsDriver, manipPlanner, ikPlanner,
                        lHandDriver, rHandDriver, atlasdriver.driver, perception.multisenseDriver,
                        fitDrillMultisense, robotStateJointController,
                        playPlans, teleopPanel.showPose, cameraview, segmentationpanel)
        drillTaskPanel = drilldemo.DrillTaskPanel(drillDemo)

        valveDemo = valvedemo.ValvePlannerDemo(robotStateModel, footstepsDriver, footstepsPanel, manipPlanner, ikPlanner,
                                          lHandDriver, rHandDriver, robotStateJointController)
        valveTaskPanel = valvedemo.ValveTaskPanel(valveDemo)

        continuouswalkingDemo = continuouswalkingdemo.ContinousWalkingDemo(robotStateModel, footstepsPanel, footstepsDriver, playbackPanel, robotStateJointController, ikPlanner,
                                                                           teleopJointController, navigationPanel, cameraview)
        continuousWalkingTaskPanel = continuouswalkingdemo.ContinuousWalkingTaskPanel(continuouswalkingDemo)

        useDrivingPlanner = drivingplanner.DrivingPlanner.isCompatibleWithConfig()
        if useDrivingPlanner:
            drivingPlannerPanel = drivingplanner.DrivingPlannerPanel(robotSystem)

        walkingDemo = walkingtestdemo.walkingTestDemo(robotStateModel, playbackRobotModel, teleopRobotModel, footstepsDriver, manipPlanner, ikPlanner,
                        lHandDriver, rHandDriver, atlasdriver.driver, perception.multisenseDriver,
                        robotStateJointController,
                        playPlans, showPose)

        bihandedDemo = bihandeddemo.BihandedPlannerDemo(robotStateModel, playbackRobotModel, teleopRobotModel, footstepsDriver, manipPlanner, ikPlanner,
                        lHandDriver, rHandDriver, atlasdriver.driver, perception.multisenseDriver,
                        fitDrillMultisense, robotStateJointController,
                        playPlans, showPose, cameraview, segmentationpanel)

        doorDemo = doordemo.DoorDemo(robotStateModel, footstepsDriver, manipPlanner, ikPlanner,
                                          lHandDriver, rHandDriver, atlasdriver.driver, perception.multisenseDriver,
                                          fitDrillMultisense, robotStateJointController,
                                          playPlans, showPose)
        doorTaskPanel = doordemo.DoorTaskPanel(doorDemo)

        terrainTaskPanel = terraintask.TerrainTaskPanel(robotSystem)
        terrainTask = terrainTaskPanel.terrainTask

        surpriseTaskPanel = surprisetask.SurpriseTaskPanel(robotSystem)
        surpriseTask = surpriseTaskPanel.planner
        egressPanel = egressplanner.EgressPanel(robotSystem)
        egressPlanner = egressPanel.egressPlanner

        if useDrivingPlanner:
            taskPanels['Driving'] = drivingPlannerPanel.widget

        taskPanels['Egress'] = egressPanel.widget
        taskPanels['Door'] = doorTaskPanel.widget
        taskPanels['Valve'] = valveTaskPanel.widget
        taskPanels['Drill'] = drillTaskPanel.widget
        taskPanels['Surprise'] = surpriseTaskPanel.widget
        taskPanels['Terrain'] = terrainTaskPanel.widget
        taskPanels['Continuous Walking'] = continuousWalkingTaskPanel.widget

    tasklaunchpanel.init(taskPanels)

    splinewidget.init(view, handFactory, robotStateModel)


    rt.robotSystem = robotSystem
    taskManagerPanel = taskmanagerwidget.init()

    for taskDescription in loadTaskDescriptions():
        taskManagerPanel.taskQueueWidget.loadTaskDescription(taskDescription[0], taskDescription[1])
    taskManagerPanel.taskQueueWidget.setCurrentQueue('Task library')

    for obj in om.getObjects():
        obj.setProperty('Deletable', False)

if useCOPMonitor and not ikPlanner.fixedBaseArm:
    copMonitor = copmonitor.COPMonitor(robotSystem, view);


if useLoggingWidget:
    w = lcmloggerwidget.LCMLoggerWidget(statusBar=app.getMainWindow().statusBar())
    app.getMainWindow().statusBar().addPermanentWidget(w.button)




if useControllerRate:

    class ControllerRateLabel(object):
        '''
        Displays a controller frequency in the status bar
        '''

        def __init__(self, atlasDriver, statusBar):
            self.atlasDriver = atlasDriver
            self.label = QtGui.QLabel('')
            statusBar.addPermanentWidget(self.label)

            self.timer = TimerCallback(targetFps=1)
            self.timer.callback = self.showRate
            self.timer.start()

        def showRate(self):
            rate = self.atlasDriver.getControllerRate()
            rate = 'unknown' if rate is None else '%d hz' % rate
            self.label.text = 'Controller rate: %s' % rate

    controllerRateLabel = ControllerRateLabel(atlasDriver, app.getMainWindow().statusBar())


if useForceDisplay:

    class LCMForceDisplay(object):
        '''
        Displays foot force sensor signals in a status bar widget or label widget
        '''


        def onRobotState(self,msg):
            self.l_foot_force_z = msg.force_torque.l_foot_force_z
            self.r_foot_force_z = msg.force_torque.r_foot_force_z

        def __init__(self, channel, statusBar=None):

            self.sub = lcmUtils.addSubscriber(channel, lcmbotcore.robot_state_t, self.onRobotState)
            self.label = QtGui.QLabel('')
            statusBar.addPermanentWidget(self.label)

            self.timer = TimerCallback(targetFps=10)
            self.timer.callback = self.showRate
            self.timer.start()

            self.l_foot_force_z = 0
            self.r_foot_force_z = 0

        def __del__(self):
            lcmUtils.removeSubscriber(self.sub)

        def showRate(self):
            global leftInContact, rightInContact
            self.label.text = '%.2f | %.2f' % (self.l_foot_force_z,self.r_foot_force_z)

    rateComputer = LCMForceDisplay('EST_ROBOT_STATE', app.getMainWindow().statusBar())


if useSkybox:

    skyboxDataDir = os.path.expanduser('~/Downloads/skybox')
    imageMap = skybox.getSkyboxImages(skyboxDataDir)
    skyboxObjs = skybox.createSkybox(imageMap, view)
    skybox.connectSkyboxCamera(view)
    #skybox.createTextureGround(os.path.join(skyboxDataDir, 'Dirt_seamless.jpg'), view)
    #view.camera().SetViewAngle(60)


class RobotLinkHighligher(object):

    def __init__(self, robotModel):
        self.robotModel = robotModel
        self.previousColors = {}

    def highlightLink(self, linkName, color):

        currentColor = self.robotModel.model.getLinkColor(linkName)
        if not currentColor.isValid():
            return

        if linkName not in self.previousColors:
            self.previousColors[linkName] = currentColor

        alpha = self.robotModel.getProperty('Alpha')
        newColor = QtGui.QColor(color[0]*255, color[1]*255, color[2]*255, alpha*255)

        self.robotModel.model.setLinkColor(linkName, newColor)

    def dehighlightLink(self, linkName):

        color = self.previousColors.pop(linkName, None)
        if color is None:
            return

        color.setAlpha(self.robotModel.getProperty('Alpha')*255)
        self.robotModel.model.setLinkColor(linkName, color)

robotHighlighter = RobotLinkHighligher(robotStateModel)

if useFootContactVis:

    class LCMContactDisplay(object):
        '''
        Displays (controller) contact state by changing foot mesh color
        '''

        def onFootContact(self, msg):
            for linkName, inContact in [[self.leftFootLink, msg.left_contact > 0.0], [self.rightFootLink, msg.right_contact > 0.0]]:
                if inContact:
                    robotHighlighter.highlightLink(linkName, [0, 0, 1])
                else:
                    robotHighlighter.dehighlightLink(linkName)

        def __init__(self, channel):

            self.leftFootLink = drcargs.getDirectorConfig()['leftFootLink']
            self.rightFootLink = drcargs.getDirectorConfig()['rightFootLink']
            footContactSub = lcmUtils.addSubscriber(channel, lcmdrc.foot_contact_estimate_t, self.onFootContact)
            footContactSub.setSpeedLimit(60)

    footContactVis = LCMContactDisplay('FOOT_CONTACT_ESTIMATE')



if useFallDetectorVis:
    def onPlanStatus(msg):
        links = ['pelvis', 'utorso']
        if msg.plan_type == lcmdrc.plan_status_t.RECOVERING:
            for link in links:
                robotHighlighter.highlightLink(link, [1,0.4,0.0])
        elif msg.plan_type == lcmdrc.plan_status_t.BRACING:
            for link in links:
                robotHighlighter.highlightLink(link, [1, 0, 0])
        else:
            for link in links:
                robotHighlighter.dehighlightLink(link)

    fallDetectorSub = lcmUtils.addSubscriber("PLAN_EXECUTION_STATUS", lcmdrc.plan_status_t, onPlanStatus)
    fallDetectorSub.setSpeedLimit(10)

if useDataFiles:

    for filename in drcargs.args().data_files:
        actionhandlers.onOpenFile(filename)

if useCameraFrustumVisualizer and cameraview.CameraFrustumVisualizer.isCompatibleWithConfig():
    cameraFrustumVisualizer = cameraview.CameraFrustumVisualizer(robotStateModel, cameraview.imageManager, 'MULTISENSE_CAMERA_LEFT')

class ImageOverlayManager(object):

    def __init__(self):
        self.viewName = 'MULTISENSE_CAMERA_LEFT'
        self.desiredWidth = 400
        self.position = [0, 0]
        self.usePicker = False
        self.imageView = None
        self.imagePicker = None
        self._prevParent = None
        self._updateAspectRatio()

    def setWidth(self, width):
        self.desiredWidth = width
        self._updateAspectRatio()
        self.hide()
        self.show()

    def _updateAspectRatio(self):
        imageExtent = cameraview.imageManager.images[self.viewName].GetExtent()
        if imageExtent[1] != -1 and imageExtent[3] != -1:
            self.imageSize = [imageExtent[1]+1, imageExtent[3]+1]
            imageAspectRatio = self.imageSize[0] / self.imageSize[1]
            self.size = [self.desiredWidth, self.desiredWidth / imageAspectRatio]

    def show(self):
        if self.imageView:
            return

        imageView = cameraview.views[self.viewName]
        self.imageView = imageView
        self._prevParent = imageView.view.parent()

        self._updateAspectRatio()

        imageView.view.hide()
        imageView.view.setParent(view)
        imageView.view.resize(self.size[0], self.size[1])
        imageView.view.move(self.position[0], self.position[1])
        imageView.view.show()

        if self.usePicker:
            self.imagePicker = ImagePointPicker(imageView)
            self.imagePicker.start()

    def hide(self):
        if self.imageView:
            self.imageView.view.hide()
            self.imageView.view.setParent(self._prevParent)
            self.imageView.view.show()
            self.imageView = None
        if self.imagePicker:
            self.imagePicker.stop()


class ToggleImageViewHandler(object):

    def __init__(self, manager):
        self.action = app.getToolsMenuActions()['ActionToggleImageView']
        self.action.connect('triggered()', self.toggle)
        self.manager = manager

    def toggle(self):
        if self.action.checked:
            self.manager.show()
        else:
            self.manager.hide()


imageOverlayManager = ImageOverlayManager()
imageWidget = cameraview.ImageWidget(cameraview.imageManager, 'MULTISENSE_CAMERA_LEFT', view, visible=False)
imageViewHandler = ToggleImageViewHandler(imageWidget)
setImageWidgetSource = imageWidget.setImageName

screengrabberpanel.init(view)
framevisualization.init(view)
affordancePanel = affordancepanel.init(view, affordanceManager, robotStateJointController, raycastDriver)
camerabookmarks.init(view)

cameraControlPanel = cameracontrolpanel.CameraControlPanel(view)
app.addWidgetToDock(cameraControlPanel.widget, action=None).hide()


def getLinkFrame(linkName, model=None):
    model = model or robotStateModel
    return model.getLinkFrame(linkName)


def getBotFrame(frameName):
    t = vtk.vtkTransform()
    t.PostMultiply()
    cameraview.imageManager.queue.getTransform(frameName, 'local', t)
    return t


def showLinkFrame(linkName, model=None):
    frame = getLinkFrame(linkName, model)
    if not frame:
        raise Exception('Link not found: ' + linkName)
    return vis.updateFrame(frame, linkName, parent='link frames')


def sendEstRobotState(pose=None):
    if pose is None:
        pose = robotStateJointController.q
    msg = robotstate.drakePoseToRobotState(pose)
    lcmUtils.publish('EST_ROBOT_STATE', msg)

estRobotStatePublisher = TimerCallback(callback=sendEstRobotState)


def enableArmEncoders():
    msg = lcmdrc.utime_t()
    msg.utime = 1
    lcmUtils.publish('ENABLE_ENCODERS', msg)


def disableArmEncoders():
    msg = lcmdrc.utime_t()
    msg.utime = -1
    lcmUtils.publish('ENABLE_ENCODERS', msg)



def sendDesiredPumpPsi(desiredPsi):
    atlasDriver.sendDesiredPumpPsi(desiredPsi)

app.setCameraTerrainModeEnabled(view, True)
app.resetCamera(viewDirection=[-1,0,0], view=view)


# Drill Demo Functions for in-image rendering:
useDrillDemo = False
if useDrillDemo:

    def spawnHandAtCurrentLocation(side='left'):
        if (side is 'left'):
            tf = transformUtils.copyFrame( getLinkFrame( 'l_hand_face') )
            handFactory.placeHandModelWithTransform( tf , app.getCurrentView(), 'left')
        else:
            tf = transformUtils.copyFrame( getLinkFrame( 'right_pointer_tip') )
            handFactory.placeHandModelWithTransform( tf , app.getCurrentView(), 'right')

    def drawFrameInCamera(t, frameName='new frame',visible=True):

        v = imageView.view
        q = cameraview.imageManager.queue
        localToCameraT = vtk.vtkTransform()
        q.getTransform('local', 'MULTISENSE_CAMERA_LEFT', localToCameraT)

        res = vis.showFrame( vtk.vtkTransform() , 'temp',view=v, visible=True, scale = 0.2)
        om.removeFromObjectModel(res)
        pd = res.polyData
        pd = filterUtils.transformPolyData(pd, t)
        pd = filterUtils.transformPolyData(pd, localToCameraT)
        q.projectPoints('MULTISENSE_CAMERA_LEFT', pd )
        vis.showPolyData(pd, ('overlay ' + frameName), view=v, colorByName='Axes',parent='camera overlay',visible=visible)

    def drawObjectInCamera(objectName,visible=True):
        v = imageView.view
        q = cameraview.imageManager.queue
        localToCameraT = vtk.vtkTransform()
        q.getTransform('local', 'MULTISENSE_CAMERA_LEFT', localToCameraT)

        obj = om.findObjectByName(objectName)
        if obj is None:
            return
        objToLocalT = transformUtils.copyFrame(obj.actor.GetUserTransform() or vtk.vtkTransform())
        objPolyDataOriginal = obj.polyData
        pd = objPolyDataOriginal
        pd = filterUtils.transformPolyData(pd, objToLocalT)
        pd = filterUtils.transformPolyData(pd, localToCameraT)
        q.projectPoints('MULTISENSE_CAMERA_LEFT', pd)
        vis.showPolyData(pd, ('overlay ' + objectName), view=v, color=[0,1,0],parent='camera overlay',visible=visible)

    def projectDrillDemoInCamera():
        q = om.findObjectByName('camera overlay')
        om.removeFromObjectModel(q)

        imageView = cameraview.views['MULTISENSE_CAMERA_LEFT']
        imageView.imageActor.SetOpacity(.2)

        drawFrameInCamera(drillDemo.drill.frame.transform, 'drill frame',visible=False)

        tf = transformUtils.copyFrame( drillDemo.drill.frame.transform )
        tf.PreMultiply()
        tf.Concatenate( drillDemo.drill.drillToButtonTransform )
        drawFrameInCamera(tf, 'drill button')


        tf2 = transformUtils.copyFrame( tf )
        tf2.PreMultiply()
        tf2.Concatenate( transformUtils.frameFromPositionAndRPY( [0,0,0] , [180,0,0] ) )
        drawFrameInCamera(tf2, 'drill button flip')

        drawObjectInCamera('drill',visible=False)

        drawObjectInCamera('sensed pointer tip')
        obj = om.findObjectByName('sensed pointer tip frame')
        if (obj is not None):
            drawFrameInCamera(obj.transform, 'sensed pointer tip frame',visible=False)

        #drawObjectInCamera('left robotiq',visible=False)
        #drawObjectInCamera('right pointer',visible=False)

        v = imageView.view
        v.render()


    showImageOverlay()
    drillDemo.pointerTracker = createPointerTracker()
    drillDemo.projectCallback = projectDrillDemoInCamera
    drillYawPreTransform = vtk.vtkTransform()
    drillYawPreTransform.PostMultiply()
    def onDrillYawSliderChanged(value):
        yawOffset = value - 180.0
        drillDemo.drillYawSliderValue = yawOffset
        drillDemo.updateDrillToHand()


    app.getMainWindow().macrosToolBar().addWidget(QtGui.QLabel('drill yaw:'))
    slider = QtGui.QSlider(QtCore.Qt.Horizontal)
    slider.setMaximum(360)
    slider.setValue(180)
    slider.setMaximumWidth(200)
    slider.connect('valueChanged(int)', onDrillYawSliderChanged)

    app.getMainWindow().macrosToolBar().addWidget(slider)


    def sendPointerPrep():
         drillDemo.planPointerPressGaze(-0.05)

    def sendPointerPress():
         drillDemo.planPointerPressGaze(0.01)

    def sendPointerPressDeep():
         drillDemo.planPointerPressGaze(0.015)

    app.addToolbarMacro('drill posture', drillDemo.planBothRaisePowerOn)
    app.addToolbarMacro('pointer prep', sendPointerPrep)
    app.addToolbarMacro('pointer press', sendPointerPress)
    app.addToolbarMacro('pointer press deep', sendPointerPressDeep)


import signal
def sendMatlabSigint():
    ikServer.comm.client.proc.send_signal(signal.SIGINT)


#app.addToolbarMacro('Ctrl+C MATLAB', sendMatlabSigint)

class AffordanceTextureUpdater(object):

    def __init__(self, affordanceManager):
        self.affordanceManager = affordanceManager
        self.timer = TimerCallback(targetFps=10)
        self.timer.callback = self.updateTextures
        self.timer.start()

    def updateTexture(self, obj):
        if obj.getProperty('Camera Texture Enabled'):
            cameraview.applyCameraTexture(obj, cameraview.imageManager)
        else:
            cameraview.disableCameraTexture(obj)
        obj._renderAllViews()

    def updateTextures(self):

        for aff in affordanceManager.getAffordances():
            self.updateTexture(aff)


affordanceTextureUpdater = AffordanceTextureUpdater(affordanceManager)


def drawCenterOfMass(model):
    stanceFrame = footstepsDriver.getFeetMidPoint(model)
    com = list(model.model.getCenterOfMass())
    com[2] = stanceFrame.GetPosition()[2]
    d = DebugData()
    d.addSphere(com, radius=0.015)
    obj = vis.updatePolyData(d.getPolyData(), 'COM %s' % model.getProperty('Name'), color=[1,0,0], visible=False, parent=model)

def initCenterOfMassVisulization():
    for model in [robotStateModel, teleopRobotModel, playbackRobotModel]:
        model.connectModelChanged(drawCenterOfMass)
        drawCenterOfMass(model)

if not ikPlanner.fixedBaseArm:
    initCenterOfMassVisulization()


class RobotMoverWidget(object):
    def __init__(self, jointController):
        self.jointController = jointController
        pos, rpy = jointController.q[:3], jointController.q[3:6]
        t = transformUtils.frameFromPositionAndRPY(pos, np.degrees(rpy))
        self.frame = vis.showFrame(t, 'mover widget', scale=0.3)
        self.frame.setProperty('Edit', True)
        self.frame.connectFrameModified(self.onFrameModified)

    def onFrameModified(self, frame):
        pos, rpy = self.frame.transform.GetPosition(), transformUtils.rollPitchYawFromTransform(self.frame.transform)
        q = self.jointController.q.copy()
        q[:3] = pos
        q[3:6] = rpy
        self.jointController.setPose('moved_pose', q)


class RobotGridUpdater(object):

    def __init__(self, gridFrame, robotModel, jointController):
        self.gridFrame = gridFrame
        self.robotModel = robotModel
        self.jointController = jointController
        self.robotModel.connectModelChanged(self.updateGrid)

    def updateGrid(self, model):
        pos = self.jointController.q[:3]

        x = int(np.round(pos[0])) / 10
        y = int(np.round(pos[1])) / 10
        z = int(np.round(pos[2] - 0.85)) / 1

        t = vtk.vtkTransform()
        t.Translate((x*10,y*10,z))
        self.gridFrame.copyFrame(t)

#gridUpdater = RobotGridUpdater(grid.getChildFrame(), robotStateModel, robotStateJointController)


class IgnoreOldStateMessagesSelector(object):

    def __init__(self, jointController):
        self.jointController = jointController
        self.action = app.addMenuAction('Tools', 'Ignore Old State Messages')
        self.action.setCheckable(True)
        self.action.setChecked(self.jointController.ignoreOldStateMessages)
        self.action.connect('triggered()', self.toggle)

    def toggle(self):
        self.jointController.ignoreOldStateMessages = bool(self.action.checked)

IgnoreOldStateMessagesSelector(robotStateJointController)


class RandomWalk(object):
    def __init__(self, max_distance_per_plan=2):
        self.subs = []
        self.max_distance_per_plan=max_distance_per_plan

    def handleStatus(self, msg):
        if msg.plan_type == msg.STANDING:
            goal = transformUtils.frameFromPositionAndRPY(
                     np.array([robotStateJointController.q[0] + 2 * self.max_distance_per_plan * (np.random.random() - 0.5),
                               robotStateJointController.q[1] + 2 * self.max_distance_per_plan * (np.random.random() - 0.5),
                               robotStateJointController.q[2] - 0.84]),
                     [0, 0, robotStateJointController.q[5] + 2 * np.degrees(np.pi) * (np.random.random() - 0.5)])
            request = footstepsDriver.constructFootstepPlanRequest(robotStateJointController.q, goal)
            request.params.max_num_steps = 18
            footstepsDriver.sendFootstepPlanRequest(request)

    def handleFootstepPlan(self, msg):
        footstepsDriver.commitFootstepPlan(msg)

    def start(self):
        sub = lcmUtils.addSubscriber('PLAN_EXECUTION_STATUS', lcmdrc.plan_status_t, self.handleStatus)
        sub.setSpeedLimit(0.2)
        self.subs.append(sub)
        self.subs.append(lcmUtils.addSubscriber('FOOTSTEP_PLAN_RESPONSE', lcmdrc.footstep_plan_t, self.handleFootstepPlan))

    def stop(self):
        for sub in self.subs:
            lcmUtils.removeSubscriber(sub)

if useRandomWalk:
    randomWalk = RandomWalk()

if useCourseModel:
    courseModel = coursemodel.CourseModel()

if useKinect:
    from . import kinectlcm
    imageOverlayManager.viewName = "KINECT_RGB"
    #kinectlcm.startButton()

if useFeetlessRobot:
    ikPlanner.robotNoFeet = True

for scriptArgs in drcargs.args().scripts:
    exec(compile(open(scriptArgs[0]).read(), scriptArgs[0], 'exec'))
