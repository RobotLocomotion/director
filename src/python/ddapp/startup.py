# This script is executed in the main console namespace so
# that all the variables defined here become console variables.

import ddapp

import os
import sys
import PythonQt
from PythonQt import QtCore, QtGui
from time import time
import imp
import ddapp.applogic as app
from ddapp import drcargs
from ddapp import botpy
from ddapp import vtkAll as vtk
from ddapp import matlab
from ddapp import jointcontrol
from ddapp import callbacks
from ddapp import cameracontrol
from ddapp import debrisdemo
from ddapp import drilldemo
from ddapp import tabledemo
from ddapp import valvedemo
from ddapp import ik
from ddapp import ikplanner
from ddapp import objectmodel as om
from ddapp import spreadsheet
from ddapp import transformUtils
from ddapp import tdx
from ddapp import skybox
from ddapp import perception
from ddapp import segmentation
from ddapp import cameraview
from ddapp import colorize
from ddapp import drakevisualizer
from ddapp import robotstate
from ddapp import roboturdf
from ddapp import filterUtils
from ddapp import footstepsdriver
from ddapp import footstepsdriverpanel
from ddapp import framevisualization
from ddapp import lcmloggerwidget
from ddapp import lcmgl
from ddapp import atlasdriver
from ddapp import atlasdriverpanel
from ddapp import multisensepanel
from ddapp import navigationpanel
from ddapp import handcontrolpanel
from ddapp import sensordatarequestpanel

from ddapp import robotplanlistener
from ddapp import handdriver
from ddapp import planplayback
from ddapp import playbackpanel
from ddapp import screengrabberpanel
from ddapp import splinewidget
from ddapp import teleoppanel
from ddapp import vtkNumpy as vnp
from ddapp import viewbehaviors
from ddapp import visualization as vis
from ddapp import actionhandlers
from ddapp.timercallback import TimerCallback
from ddapp.pointpicker import PointPicker, ImagePointPicker
from ddapp import segmentationpanel
from ddapp import lcmUtils
from ddapp.utime import getUtime
from ddapp.shallowCopy import shallowCopy

from ddapp import segmentationroutines
from ddapp import trackers

import drc as lcmdrc

import functools
import math

import numpy as np
from ddapp.debugVis import DebugData
from ddapp import ioUtils as io


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


useIk = True
useAtlasConvexHull = False
useRobotState = True
usePerception = True
useGrid = True
useSpreadsheet = True
useFootsteps = True
useActionManager = False
useHands = True
usePlanning = True
useAtlasDriver = True
useLCMGL = True
useLightColorScheme = False
useLoggingWidget = True
useDrakeVisualizer = True
useNavigationPanel = True
useFootContactVis = True
useImageWidget = False
useImageViewDemo = True
useControllerRate = True
useSkybox = False
useDataFiles = True


poseCollection = PythonQt.dd.ddSignalMap()
costCollection = PythonQt.dd.ddSignalMap()


if useSpreadsheet:
    spreadsheet.init(poseCollection, costCollection)


if useIk:

    ikRobotModel, ikJointController = roboturdf.loadRobotModel('ik model', view, parent='IK Server', color=roboturdf.getRobotOrangeColor(), visible=False)
    ikJointController.addPose('q_end', ikJointController.getPose('q_nom'))
    ikJointController.addPose('q_start', ikJointController.getPose('q_nom'))
    om.removeFromObjectModel(om.findObjectByName('IK Server'))

    ikServer = ik.AsyncIKCommunicator()
    ikServer.outputConsole = app.getOutputConsole()
    ikServer.infoFunc = app.displaySnoptInfo

    def startIkServer():
        ikServer.startServerAsync()
        ikServer.comm.writeCommandsToLogFile = True

    startIkServer()


if useAtlasDriver:
    atlasDriver = atlasdriver.init(app.getOutputConsole())
    atlasdriverpanel.init(atlasdriver.driver)


if useRobotState:
    robotStateModel, robotStateJointController = roboturdf.loadRobotModel('robot state model', view, parent='sensors', color=roboturdf.getRobotGrayColor(), visible=True)
    robotStateJointController.setPose('EST_ROBOT_STATE', robotStateJointController.getPose('q_nom'))
    roboturdf.startModelPublisherListener([(robotStateModel, robotStateJointController)])
    robotStateJointController.addLCMUpdater('EST_ROBOT_STATE')

    segmentationroutines.SegmentationContext.initWithRobot(robotStateModel)

    def getNeckPitch():
        return robotStateJointController.q[robotstate.getDrakePoseJointNames().index('neck_ay')]


if usePerception:

    multisenseDriver, mapServerSource = perception.init(view)
    segmentationpanel.init()
    cameraview.init()
    colorize.init()

    cameraview.cameraView.rayCallback = segmentation.extractPointsAlongClickRay
    multisensepanel.init(perception.multisenseDriver)
    sensordatarequestpanel.init()

    disparityPointCloud = segmentation.DisparityPointCloudItem('stereo point cloud', cameraview.imageManager)
    disparityPointCloud.addToView(view)
    om.addToObjectModel(disparityPointCloud, parentObj=om.findObjectByName('sensors'))

    def createPointerTracker():
        return trackers.PointerTracker(robotStateModel, disparityPointCloud)

    neckDriver = perception.NeckDriver(view, getNeckPitch)


if useGrid:
    vis.showGrid(view, color=[0,0,0] if useLightColorScheme else [1,1,1], useSurface=useLightColorScheme)


if useLightColorScheme:
    app.setBackgroundColor([0.3, 0.3, 0.35], [0.95,0.95,1])


if useHands:
    rHandDriver = handdriver.RobotiqHandDriver(side='right')
    lHandDriver = handdriver.RobotiqHandDriver(side='left')
    handcontrolpanel.init(lHandDriver, rHandDriver, robotStateModel)


if useFootsteps:
    footstepsDriver = footstepsdriver.FootstepsDriver(robotStateJointController)
    footstepsPanel = footstepsdriverpanel.init(footstepsDriver, robotStateModel, robotStateJointController, mapServerSource)


if useLCMGL:
    lcmgl.init(view)


if useDrakeVisualizer:
    drakeVis = drakevisualizer.DrakeVisualizer(view)


if usePlanning:

    playbackRobotModel, playbackJointController = roboturdf.loadRobotModel('playback model', view, parent='planning', color=roboturdf.getRobotOrangeColor(), visible=False)
    teleopRobotModel, teleopJointController = roboturdf.loadRobotModel('teleop model', view, parent='planning', color=roboturdf.getRobotOrangeColor(), visible=False)

    if useAtlasConvexHull:
        chullRobotModel, chullJointController = roboturdf.loadRobotModel('convex hull atlas', view, parent='planning',
            urdfFile=os.path.join(ddapp.getDRCBaseDir(), 'software/models/mit_gazebo_models/mit_robot_drake/model_convex_hull_robotiq_hands.urdf'),
            color=roboturdf.getRobotOrangeColor(), visible=False)
        playbackJointController.models.append(chullRobotModel)


    manipPlanner = robotplanlistener.ManipulationPlanDriver()
    planPlayback = planplayback.PlanPlayback()


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

      msg = lcmdrc.data_request_t()
      msg.type = requestType
      msg.period = int(repeatTime*10) # period is specified in tenths of a second

      msgList = lcmdrc.data_request_list_t()
      msgList.utime = getUtime()
      msgList.requests = [msg]
      msgList.num_requests = len(msgList.requests)
      lcmUtils.publish('DATA_REQUEST', msgList)

    def sendSceneHeightRequest(repeatTime=0.0):
        sendDataRequest(lcmdrc.data_request_t.HEIGHT_MAP_SCENE, repeatTime)

    def sendWorkspaceDepthRequest(repeatTime=0.0):
        sendDataRequest(lcmdrc.data_request_t.DEPTH_MAP_WORKSPACE_C, repeatTime)

    def sendSceneDepthRequest(repeatTime=0.0):
        sendDataRequest(lcmdrc.data_request_t.DEPTH_MAP_SCENE, repeatTime)

    def sendFusedDepthRequest(repeatTime=0.0):
        sendDataRequest(lcmdrc.data_request_t.FUSED_DEPTH, repeatTime)

    def sendFusedHeightRequest(repeatTime=0.0):
        sendDataRequest(lcmdrc.data_request_t.FUSED_HEIGHT, repeatTime)

    app.addToolbarMacro('scene height', sendSceneHeightRequest)
    #app.addToolbarMacro('scene depth', sendSceneDepthRequest)
    #app.addToolbarMacro('stereo height', sendFusedHeightRequest)
    #app.addToolbarMacro('stereo depth', sendFusedDepthRequest)


    def drillTrackerOn():
        om.findObjectByName('Multisense').model.showRevolutionCallback = fitDrillMultisense

    def drillTrackerOff():
        om.findObjectByName('Multisense').model.showRevolutionCallback = None


    handFactory = roboturdf.HandFactory(robotStateModel)
    handModels = [handFactory.getLoader(side) for side in ['left', 'right']]

    ikPlanner = ikplanner.IKPlanner(ikServer, ikRobotModel, ikJointController, handModels)
    ikPlanner.addPostureGoalListener(robotStateJointController)


    playbackPanel = playbackpanel.init(planPlayback, playbackRobotModel, playbackJointController,
                                      robotStateModel, robotStateJointController, manipPlanner)

    footstepsDriver.walkingPlanCallback = playbackPanel.setPlan
    manipPlanner.manipPlanCallback = playbackPanel.setPlan

    teleoppanel.init(robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController,
                     ikPlanner, manipPlanner, handFactory.getLoader('left'), handFactory.getLoader('right'), playbackPanel.setPlan)



    debrisDemo = debrisdemo.DebrisPlannerDemo(robotStateModel, robotStateJointController, playbackRobotModel,
                    ikPlanner, manipPlanner, atlasdriver.driver, lHandDriver,
                    perception.multisenseDriver, refitBlocks)

    tableDemo = tabledemo.TableDemo(robotStateModel, playbackRobotModel,
                    ikPlanner, manipPlanner, footstepsDriver, atlasdriver.driver, lHandDriver, rHandDriver,
                    perception.multisenseDriver, view, robotStateJointController)

    drillDemo = drilldemo.DrillPlannerDemo(robotStateModel, playbackRobotModel, teleopRobotModel, footstepsDriver, manipPlanner, ikPlanner,
                    lHandDriver, rHandDriver, atlasdriver.driver, perception.multisenseDriver,
                    fitDrillMultisense, robotStateJointController,
                    playPlans, showPose, cameraview, segmentationpanel)

    valveDemo = valvedemo.ValvePlannerDemo(robotStateModel, footstepsDriver, manipPlanner, ikPlanner,
                                      lHandDriver, atlasdriver.driver, perception.multisenseDriver,
                                      segmentation.segmentValveWallAuto, robotStateJointController,
                                      playPlans, showPose)


    splinewidget.init(view, handFactory, robotStateModel)


if useActionManager:

    from ddapp import actionmanagerpanel
    from actionmanager import actionmanager

    actionManager = actionmanager.ActionManager(objectModel = om,
                                                robotModel = robotStateModel,
                                                sensorJointController = robotStateJointController,
                                                playbackFunction = playPlans,
                                                manipPlanner = manipPlanner,
                                                ikPlanner = ikPlanner,
                                                footstepPlanner = footstepsDriver,
                                                handDriver = lHandDriver,
                                                atlasDriver = atlasdriver.driver,
                                                multisenseDriver = perception.multisenseDriver,
                                                affordanceServer = None,
                                                fsmDebug = True)

    ampanel = actionmanagerpanel.init(actionManager)


if useNavigationPanel:
    thispanel = navigationpanel.init(robotStateJointController, footstepsDriver, playbackRobotModel, playbackJointController)
    picker = PointPicker(view, callback=thispanel.pointPickerStoredFootsteps, numberOfPoints=2)
    #picker.start()


if useLoggingWidget:
    w = lcmloggerwidget.LCMLoggerWidget(statusBar=app.getMainWindow().statusBar())
    app.getMainWindow().statusBar().addPermanentWidget(w.button)
    baseDir = os.path.expanduser('~/logs/raw')
    if os.path.isdir(baseDir):
        w.manager.baseDir = baseDir


if useControllerRate:

    class LCMMessageRateDisplay(object):
        '''
        Displays an LCM message frequency in a status bar widget or label widget
        '''

        def __init__(self, channel, messageTemplate, statusBar=None):

            self.sub = lcmUtils.addSubscriber(channel)
            self.label = QtGui.QLabel('')
            statusBar.addPermanentWidget(self.label)

            self.timer = TimerCallback(targetFps=1)
            self.timer.callback = self.showRate
            self.timer.start()

        def __del__(self):
            lcmUtils.removeSubscriber(self.sub)

        def showRate(self):
            self.label.text = 'Controller rate: %.2f hz' % self.sub.getMessageRate()

    rateComputer = LCMMessageRateDisplay('ATLAS_COMMAND', 'Controller rate: %.2 hz', app.getMainWindow().statusBar())


if useSkybox:

    skyboxDataDir = os.path.expanduser('~/Downloads/skybox')
    imageMap = skybox.getSkyboxImages(skyboxDataDir)
    skyboxObjs = skybox.createSkybox(imageMap, view)
    skybox.connectSkyboxCamera(view)
    #skybox.createTextureGround(os.path.join(skyboxDataDir, 'Dirt_seamless.jpg'), view)
    #view.camera().SetViewAngle(60)


if useFootContactVis:

    def onFootContact(msg):

        leftInContact = msg.left_contact > 0.0
        rightInContact = msg.right_contact > 0.0

        contactColor = QtGui.QColor(255,0,0)
        noContactColor = QtGui.QColor(180, 180, 180)

        robotStateModel.model.setLinkColor('l_foot', contactColor if leftInContact else noContactColor)
        robotStateModel.model.setLinkColor('r_foot', contactColor if rightInContact else noContactColor)

    footContactSub = lcmUtils.addSubscriber('FOOT_CONTACT_ESTIMATE', lcmdrc.foot_contact_estimate_t, onFootContact)
    footContactSub.setSpeedLimit(60)


if useDataFiles:

    for filename in drcargs.args().data_files:
        polyData = io.readPolyData(filename)
        if polyData:
            vis.showPolyData(polyData, os.path.basename(filename))


if useImageWidget:
    imageWidget = cameraview.ImageWidget(cameraview.imageManager, 'CAMERA_LEFT', view)


if useImageViewDemo:
    imageView = cameraview.views['CAMERA_LEFT']
    #imageView = cameraview.cameraView
    imageView.rayCallback = segmentation.extractPointsAlongClickRay
    imagePicker = ImagePointPicker(imageView)

    _prevParent = imageView.view.parent()
    def showImageOverlay(size=400):
        imageView.view.hide()
        imageView.view.setParent(view)
        imageView.view.resize(size, size)
        imageView.view.move(0,0)
        imageView.view.show()
        imagePicker.start()

    def hideImageOverlay():
        imageView.view.hide()
        imageView.view.setParent(_prevParent)
        imageView.view.show()
        imagePicker.stop()

    #showImageOverlay()


screengrabberpanel.init(view)
framevisualization.init(view)


def getLinkFrame(linkName, model=None):
    model = model or robotStateModel
    return model.getLinkFrame(linkName)


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


def addCollisionObjectToWorld(obj = None, name = None):
    if obj is None:
        obj = [om.getActiveObject()]
    if type(obj) is not list:
        obj = [obj]
    if name is None:
        name = [obj_i.parent().getProperty('Name') for obj_i in obj]
    if type(name) is not list:
        name = [name]

    assert len(obj) == len(name)
    pts = []
    for obj_i in obj:
        assert obj_i and obj_i.polyData
        polyData = filterUtils.transformPolyData(obj_i.polyData, obj_i.actor.GetUserTransform())
        pts_i = vnp.getNumpyFromVtk(polyData, 'Points')
        pts.append(pts_i.transpose());

    ikServer.addCollisionObject(pts,name)


def addCollisionObjectToLink(robotModel, linkName):
    obj = om.getActiveObject()
    assert obj and obj.polyData
    pts = vnp.getNumpyFromVtk(obj.polyData, 'Points')
    pts = pts.transpose()
    t = vtk.vtkTransform()
    t.PostMultiply()
    t.Concatenate(obj.actor.GetUserTransform())
    t.Concatenate(robotModel.getLinkFrame(linkName).GetLinearInverse())
    ikServer.addCollisionObjectToLink(pts, linkName, t)


app.setCameraTerrainModeEnabled(view, True)
app.resetCamera(viewDirection=[-1,0,0], view=view)
viewBehaviors = viewbehaviors.ViewBehaviors(view)
viewbehaviors.ViewBehaviors.addRobotBehaviors(robotStateModel, handFactory, footstepsDriver, neckDriver)


# Drill Demo Functions for in-image rendering:
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
    q.getTransform('local', 'CAMERA_LEFT', localToCameraT)

    res = vis.showFrame( vtk.vtkTransform() , 'temp',view=v, visible=True, scale = 0.2)
    om.removeFromObjectModel(res)
    pd = res.polyData
    pd = filterUtils.transformPolyData(pd, t)
    pd = filterUtils.transformPolyData(pd, localToCameraT)
    q.projectPoints('CAMERA_LEFT', pd )
    vis.showPolyData(pd, ('overlay ' + frameName), view=v, colorByName='Axes',parent='camera overlay',visible=visible)

def drawObjectInCamera(objectName,visible=True):
    v = imageView.view
    q = cameraview.imageManager.queue
    localToCameraT = vtk.vtkTransform()
    q.getTransform('local', 'CAMERA_LEFT', localToCameraT)

    obj = om.findObjectByName(objectName)
    if obj is None:
        return
    objToLocalT = transformUtils.copyFrame(obj.actor.GetUserTransform() or vtk.vtkTransform())
    objPolyDataOriginal = obj.polyData
    pd = objPolyDataOriginal
    pd = filterUtils.transformPolyData(pd, objToLocalT)
    pd = filterUtils.transformPolyData(pd, localToCameraT)
    q.projectPoints('CAMERA_LEFT', pd)
    vis.showPolyData(pd, ('overlay ' + objectName), view=v, color=[0,1,0],parent='camera overlay',visible=visible)

def projectDrillDemoInCamera():
    q = om.findObjectByName('camera overlay')
    om.removeFromObjectModel(q)

    imageView = cameraview.views['CAMERA_LEFT']
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



#mapping = cameraview.KintinuousMapping()
#timerCloud = TimerCallback(targetFps=5)
#timerCloud.callback = mapping.cameraFusedCallback
#timerCloud.start()

import time
import vtkNumpy
from ddapp import qhull_2d
from ddapp import min_bounding_rect


def getRecedingTerrainRegion(polyData, linkFrame):
    ''' Find the point cloud in front of the foot frame'''

    #polyData = shallowCopy(polyData)
    points = vtkNumpy.getNumpyFromVtk(polyData, 'Points')
    #vtkNumpy.addNumpyToVtk(polyData, points[:,0].copy(), 'x')
    #vtkNumpy.addNumpyToVtk(polyData, points[:,1].copy(), 'y')
    #vtkNumpy.addNumpyToVtk(polyData, points[:,2].copy(), 'z')

    viewOrigin = linkFrame.TransformPoint([0.0, 0.0, 0.0])
    viewX = linkFrame.TransformVector([1.0, 0.0, 0.0])
    viewY = linkFrame.TransformVector([0.0, 1.0, 0.0])
    viewZ = linkFrame.TransformVector([0.0, 0.0, 1.0])
    polyData = segmentation.labelPointDistanceAlongAxis(polyData, viewX, origin=viewOrigin, resultArrayName='distance_along_foot_x')
    polyData = segmentation.labelPointDistanceAlongAxis(polyData, viewY, origin=viewOrigin, resultArrayName='distance_along_foot_y')
    polyData = segmentation.labelPointDistanceAlongAxis(polyData, viewZ, origin=viewOrigin, resultArrayName='distance_along_foot_z')

    polyData = segmentation.thresholdPoints(polyData, 'distance_along_foot_x', [0.20, 1.5])
    polyData = segmentation.thresholdPoints(polyData, 'distance_along_foot_y', [-0.4, 0.4])
    polyData = segmentation.thresholdPoints(polyData, 'distance_along_foot_z', [-0.4, 0.4])

    vis.updatePolyData( polyData, 'walking snapshot trimmed', parent='continuous', visible=True)
    return polyData


def findFarRightCorner(polyData, linkFrame):
    '''
    Within a point cloud find the point to the far right from the link
    The input is typically the 4 corners of a minimum bounding box
    '''

    diagonalTransform = transformUtils.frameFromPositionAndRPY([0,0,0], [0,0,45])
    diagonalTransform.Concatenate(linkFrame)
    vis.updateFrame(diagonalTransform, 'diagonal frame', parent='continuous', visible=False)

    #polyData = shallowCopy(polyData)
    points = vtkNumpy.getNumpyFromVtk(polyData, 'Points')
    #vtkNumpy.addNumpyToVtk(polyData, points[:,0].copy(), 'x')
    #vtkNumpy.addNumpyToVtk(polyData, points[:,1].copy(), 'y')
    #vtkNumpy.addNumpyToVtk(polyData, points[:,2].copy(), 'z')

    viewOrigin = diagonalTransform.TransformPoint([0.0, 0.0, 0.0])
    viewX = diagonalTransform.TransformVector([1.0, 0.0, 0.0])
    viewY = diagonalTransform.TransformVector([0.0, 1.0, 0.0])
    viewZ = diagonalTransform.TransformVector([0.0, 0.0, 1.0])
    #polyData = segmentation.labelPointDistanceAlongAxis(polyData, viewX, origin=viewOrigin, resultArrayName='distance_along_foot_x')
    polyData = segmentation.labelPointDistanceAlongAxis(polyData, viewY, origin=viewOrigin, resultArrayName='distance_along_foot_y')
    #polyData = segmentation.labelPointDistanceAlongAxis(polyData, viewZ, origin=viewOrigin, resultArrayName='distance_along_foot_z')

    vis.updatePolyData( polyData, 'cornerPoints', parent='continuous', visible=False)
    farRightIndex = vtkNumpy.getNumpyFromVtk(polyData, 'distance_along_foot_y').argmin()
    points = vtkNumpy.getNumpyFromVtk(polyData, 'Points')
    return points[farRightIndex,:]


def get2DAsPolyData(xy_points):
    '''
    Convert a 2D np array to a 3D polydata by appending z=0
    '''
    d = np.vstack((xy_points.T, np.zeros( xy_points.shape[0]) )).T
    d2=d.copy()
    return vtkNumpy.getVtkPolyDataFromNumpyPoints( d2 )


def findMinimumBoundingRectangle(polyData, linkFrame):
    '''
    Find minimum bounding rectangle.
    The input is assumed to be a rectangular point cloud of cinder blocks
    Returns transform of far right corner (pointing away from robot)
    '''
    # TODO: for non-z up surfaces, this needs work
    # TODO: return other parameters

    # Originally From: https://github.com/dbworth/minimum-area-bounding-rectangle
    polyData = segmentation.applyVoxelGrid(polyData, leafSize=0.02)
    #vis.updatePolyData( polyData, 'block top', parent='continuous', visible=False)
    polyDataCentroid = segmentation.computeCentroid(polyData)
    pts =vtkNumpy.getNumpyFromVtk( polyData , 'Points' )

    xy_points =  pts[:,[0,1]]
    vis.updatePolyData( get2DAsPolyData(xy_points) , 'xy_points', parent='continuous', visible=False)
    hull_points = qhull_2d.qhull2D(xy_points)
    vis.updatePolyData( get2DAsPolyData(hull_points) , 'hull_points', parent='continuous', visible=False)
    # Reverse order of points, to match output from other qhull implementations
    hull_points = hull_points[::-1]
    # print 'Convex hull points: \n', hull_points, "\n"

    # Find minimum area bounding rectangle
    (rot_angle, area, width, height, center_point, corner_points_ground) = min_bounding_rect.minBoundingRect(hull_points)
    vis.updatePolyData( get2DAsPolyData(corner_points_ground) , 'corner_points_ground', parent='continuous', visible=False)
    cornerPoints = np.vstack((corner_points_ground.T, polyDataCentroid[2]*np.ones( corner_points_ground.shape[0]) )).T
    cornerPolyData = vtkNumpy.getVtkPolyDataFromNumpyPoints(cornerPoints)

    # Create a frame at the far right point - which points away from the robot
    farRightCorner = findFarRightCorner(cornerPolyData , linkFrame)
    viewDirection = segmentation.SegmentationContext.getGlobalInstance().getViewDirection()
    robotYaw = math.atan2( viewDirection[1], viewDirection[0] )*180.0/np.pi
    blockAngle =  rot_angle*(180/math.pi)
    #print "robotYaw   ", robotYaw
    #print "blockAngle ", blockAngle
    blockAngleAll = np.array([blockAngle , blockAngle+90 , blockAngle+180, blockAngle+270])
    #print blockAngleAll
    for i in range(0,4):
        if(blockAngleAll[i]>180):
           blockAngleAll[i]=blockAngleAll[i]-360
    #print blockAngleAll
    values = abs(blockAngleAll - robotYaw)
    #print values
    min_idx = np.argmin(values)
    #print "best angle", blockAngleAll[min_idx]
    rot_angle = blockAngleAll[min_idx]*math.pi/180.0

    cornerTransform = transformUtils.frameFromPositionAndRPY( farRightCorner , [0,0, np.rad2deg(rot_angle) ] )

    #print "Minimum area bounding box:"
    #print "Rotation angle:", rot_angle, "rad  (", rot_angle*(180/math.pi), "deg )"
    #print "Width:", width, " Height:", height, "  Area:", area
    #print "Center point: \n", center_point # numpy array
    #print "Corner points: \n", cornerPoints, "\n"  # numpy array
    return cornerTransform


def extractRectanglesFromSurfaces(clusters, linkFrame):
    ''' find the corners of the minimum bounding rectangles '''
    om.removeFromObjectModel(om.findObjectByName('block corners'))
    om.removeFromObjectModel(om.findObjectByName('foot placements'))
    om.removeFromObjectModel(om.findObjectByName('steps'))
    for i, cluster in enumerate(clusters):
        cornerTransform = findMinimumBoundingRectangle( cluster, linkFrame )
        vis.updateFrame(cornerTransform, 'block corners %d' % i , parent='block corners', scale=0.2, visible=False)

        nextLeftTransform = transformUtils.frameFromPositionAndRPY([-0.27,0.29,0.08], [0,0,0])
        nextLeftTransform.Concatenate(cornerTransform)
        vis.updateFrame(nextLeftTransform, 'left foot placement %d' % i , parent='foot placements', scale=0.2, visible=False)

        leftMesh = footstepsdriver.getLeftFootMesh()
        obj = vis.showPolyData(leftMesh, 'left step %d' % i, color=[1.0,1.0,0.0], alpha=1.0, parent='steps')
        #frameObj = vis.showFrame(footstepTransform, stepName + ' frame', parent=obj, scale=0.3, visible=False)
        obj.actor.SetUserTransform(nextLeftTransform)

        nextRightTransform = transformUtils.frameFromPositionAndRPY([-0.23,0.1,0.08], [0,0,0])
        nextRightTransform.Concatenate(cornerTransform)
        vis.updateFrame(nextRightTransform, 'right foot placement %d' % i , parent='foot placements', scale=0.2, visible=False)

        rightMesh = footstepsdriver.getRightFootMesh()
        obj = vis.showPolyData(leftMesh, 'right step %d' % i, color=[0.33,1.0,0.0], alpha=1.0, parent='steps')
        #frameObj = vis.showFrame(footstepTransform, stepName + ' frame', parent=obj, scale=0.3, visible=False)
        obj.actor.SetUserTransform(nextRightTransform)


def replanFootsteps(polyData, linkName):
    vis.updatePolyData( polyData, 'walking snapshot', parent='continuous', visible=False)

    linkFrame = robotStateModel.getLinkFrame(linkName)
    vis.updateFrame(linkFrame, linkName, parent='continuous', visible=False)
    # TODO: remove the pitch and roll of this frame to support it being on uneven ground

    # Step 1: filter the data down to a box in front of the robot:
    polyData = getRecedingTerrainRegion(polyData, linkFrame)

    # Step 2: find all the surfaces in front of the robot (about 0.75sec)
    clusters = segmentation.findHorizontalSurfaces(polyData)
    if (clusters is None):
        print "No cluster found, stop walking now!"
        return

    # Step 3: find the corners of the minimum bounding rectangles
    extractRectanglesFromSurfaces(clusters, linkFrame)


lastContactState = "none"
def onFootContactContinous(msg):
    global lastContactState

    leftInContact = msg.left_contact > 0.0
    rightInContact = msg.right_contact > 0.0

    if (leftInContact and rightInContact):
        contactState="both"
    elif (leftInContact and not rightInContact):
        contactState="left"
    elif (not leftInContact and rightInContact):
        contactState="right"
    else:
        contactState="none"
        print "No foot contacts. Error!"

    if (lastContactState is "both") and (contactState is "left"):
        #print "trigger left"
        replanFootsteps(segmentation.getCurrentRevolutionData(), 'l_foot')

    if (lastContactState is "both") and (contactState is "right"):
        #print "trigger right"
        replanFootsteps(segmentation.getCurrentRevolutionData(), 'r_foot')

    lastContactState = contactState

footContactSubContinous = lcmUtils.addSubscriber('FOOT_CONTACT_ESTIMATE', lcmdrc.foot_contact_estimate_t, onFootContactContinous)
footContactSubContinous.setSpeedLimit(60)


# Test stippets:
def processSnippet():
    polyData = io.readPolyData('/home/mfallon/Desktop/continuous_walking/snippet.vtp')
    vis.updatePolyData( polyData, 'walking snapshot trimmed', parent='segmentation')

    linkFrame = robotStateModel.getLinkFrame('l_foot')
    vis.updateFrame(linkFrame, 'l_foot', parent='continuous', visible=False)

    # Step 2: find all the surfaces in front of the robot (about 0.75sec)
    clusters = segmentation.findHorizontalSurfaces(polyData)
    if (clusters is None):
        print "No cluster found, stop walking now!"
        return

    # Step 3: find the corners of the minimum bounding rectangles
    extractRectanglesFromSurfaces(clusters)


def processSingleBlock():
    if (1==1):
        polyData = io.readPolyData('/home/mfallon/Desktop/continuous_walking/block_top.vtp')

    if (1==0):
        polyData = io.readPolyData('/home/mfallon/Desktop/continuous_walking/table_top_45.vtp')

    findMinimumBoundingRectangle(polyData)