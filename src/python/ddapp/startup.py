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
from ddapp import perception
from ddapp import segmentation
from ddapp import cameraview
from ddapp import colorize
from ddapp import drakevisualizer
from ddapp import robotstate
from ddapp import roboturdf
from ddapp import footstepsdriver
from ddapp import footstepsdriverpanel
from ddapp import framevisualization
from ddapp import lcmgl
from ddapp import atlasdriver
from ddapp import atlasdriverpanel
from ddapp import multisensepanel
from ddapp import navigationpanel
from ddapp import handcontrolpanel

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


import drc as lcmdrc

import functools
import math

import numpy as np
from ddapp.debugVis import DebugData
from ddapp import ioUtils as io


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
useDrakeVisualizer = True
useNavigationPanel = True
useImageWidget = False
useImageViewDemo = True


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


if usePerception:

    multisenseDriver, mapServerSource = perception.init(view)
    segmentationpanel.init()
    cameraview.init()
    colorize.init()

    cameraview.cameraView.rayCallback = segmentation.extractPointsAlongClickRay
    multisensepanel.init(perception.multisenseDriver)


if useGrid:
    vis.showGrid(view, color=[0,0,0] if useLightColorScheme else [1,1,1])
    app.toggleCameraTerrainMode(view)


if useLightColorScheme:
    app.setBackgroundColor([0.3, 0.3, 0.35], [1,1,1])


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

    def fitDrillMultisense():
        pd = om.findObjectByName('Multisense').model.revPolyData
        om.removeFromObjectModel(om.findObjectByName('debug'))
        segmentation.findAndFitDrillBarrel(pd,  getLinkFrame('utorso'))

    def refitBlocks(autoApprove=True):
        polyData = om.findObjectByName('Multisense').model.revPolyData
        segmentation.updateBlockAffordances(polyData)
        if autoApprove:
            approveRefit()

    def approveRefit():

        for obj in om.getObjects():
            if isinstance(obj, vis.BlockAffordanceItem):
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

    app.addToolbarMacro('scene height', sendSceneHeightRequest)


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

    playbackPanel.visOnly = False

    teleoppanel.init(robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController,
                     ikPlanner, manipPlanner, handFactory.getLoader('left'), handFactory.getLoader('right'), playbackPanel.setPlan)



    debrisDemo = debrisdemo.DebrisPlannerDemo(robotStateModel, robotStateJointController, playbackRobotModel,
                    ikPlanner, manipPlanner, atlasdriver.driver, lHandDriver,
                    perception.multisenseDriver, refitBlocks)

    tableDemo = tabledemo.TableDemo(robotStateModel, playbackRobotModel,
                    ikPlanner, manipPlanner, footstepsDriver, atlasdriver.driver, lHandDriver, rHandDriver,
                    perception.multisenseDriver, view, robotStateJointController)

    drillDemo = drilldemo.DrillPlannerDemo(robotStateModel, footstepsDriver, manipPlanner, ikPlanner,
                                        lHandDriver, atlasdriver.driver, perception.multisenseDriver,
                                        fitDrillMultisense, robotStateJointController,
                                        playPlans, showPose)

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
    picker = PointPicker(view, callback=thispanel.pointPickerDemo, numberOfPoints=2)
    #picker.start()


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


app.resetCamera(viewDirection=[-1,0,0], view=view)
viewBehaviors = viewbehaviors.ViewBehaviors(view)
viewbehaviors.ViewBehaviors.addRobotBehaviors(robotStateModel, handFactory, footstepsDriver)

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


def onFootContact(msg):

    leftInContact = msg.left_contact > 0.0
    rightInContact = msg.right_contact > 0.0

    contactColor = QtGui.QColor(255,0,0)
    noContactColor = QtGui.QColor(180, 180, 180)

    robotStateModel.model.setLinkColor('l_foot', contactColor if leftInContact else noContactColor)
    robotStateModel.model.setLinkColor('r_foot', contactColor if rightInContact else noContactColor)

sub = lcmUtils.addSubscriber('FOOT_CONTACT_ESTIMATE', lcmdrc.foot_contact_estimate_t, onFootContact)
sub.setSpeedLimit(60)


if (1==1):
    polyData = io.readPolyData('/home/mfallon/Desktop/rgbd_data/multisense_20_local_sparse.pcd')
    #polyData = io.readPolyData('~/Desktop/rgbd_data/table_sparse.pcd')

    vis.showPolyData(polyData, 'depth point cloud', colorByName='rgb_colors')


    # strips color out
    polyData = segmentation.applyVoxelGrid(polyData, leafSize=0.01)
    
    polyData = segmentation.labelOutliers(polyData)
    vis.showPolyData(polyData, 'is_outlier', colorByName='is_outlier', visible=False, parent=segmentation.getDebugFolder())

    polyData = segmentation.thresholdPoints(polyData, 'is_outlier', [0.0, 0.0])    

    
    polyData = segmentation.addCoordArraysToPolyData(polyData)
    
    vis.showPolyData(polyData, 'workspace', colorByName='z', visible=True, parent=segmentation.getDebugFolder())
    
    #segmentationpanel.activateSegmentationMode(polyData)
    #segmentation.segmentValveWallAuto(.2) #,'valve')
    #segmentation.switchToView( 'DRC View')
    
    
if (1==0):
    robotStateJointController.q[5] = -0.63677# math.radians(120)
    robotStateJointController.q[0] = 0.728
    robotStateJointController.q[1] = -0.7596
    robotStateJointController.q[2] = 0.79788
    robotStateJointController.push()



    polyData = io.readPolyData('/home/mfallon/Desktop/rgbd_data/multisense_20_sparse.pcd')
    #polyData = io.readPolyData('~/Desktop/rgbd_data/table_sparse.pcd')

    #vis.showPolyData(polyData, 'depth point cloud', colorByName='rgb_colors')

    points = vnp.getNumpyFromVtk(polyData, 'Points')
    rgb = vnp.getNumpyFromVtk(polyData, 'rgb_colors')

    print points.shape
    print rgb.shape

    cameraToLocal = transformUtils.transformFromPose([ 0.92912804, -0.86441896,  1.63176906], [ 0.22322877, -0.39282596,  0.77466717, -0.44243355])

    polyData = segmentation.transformPolyData(polyData, cameraToLocal)

    # strips color out
    polyData = segmentation.applyVoxelGrid(polyData, leafSize=0.01)

    polyData_original = shallowCopy(polyData)

    vis.showPolyData(polyData, 'voxel', visible=False, parent=segmentation.getDebugFolder())

    polyData = segmentation.addCoordArraysToPolyData(polyData)

    #polyData = segmentation.thresholdPoints(polyData, 'z', [0.5, 2.0])
    polyData = segmentation.thresholdPoints(polyData, 'z', [0.0, 2.0])

    polyData = segmentation.thresholdPoints(polyData, 'x', [0.0, 2.0])
    polyData = segmentation.thresholdPoints(polyData, 'y', [-2, 0.0])

    vis.showPolyData(polyData, 'nearby', visible=False, parent=segmentation.getDebugFolder())

    #polyData = segmentation.applyEuclideanClustering(polyData)
    #polyData = segmentation.thresholdPoints(polyData, 'cluster_labels', [1.0, np.inf])


    polyData = segmentation.labelOutliers(polyData)
    vis.showPolyData(polyData, 'is_outlier', colorByName='is_outlier', visible=False, parent=segmentation.getDebugFolder())

    polyData = segmentation.thresholdPoints(polyData, 'is_outlier', [0.0, 0.0])


    vis.showPolyData(polyData, 'workspace', colorByName='z', visible=True, parent=segmentation.getDebugFolder())




    if (1==1):
        segmentation.segmentTableThenFindDrills(polyData, [1.2864902,  -0.93351376,  1.10208917])
    elif (1==0):
        segmentation.findAndFitDrillBarrel(polyData, getLinkFrame('utorso'))
    elif (1==0):
        segmentation.segmentTable(polyData, [1.2864902,  -0.93351376,  1.10208917])


# pcd file is 1024x1024

#cameraview.imageManager.updateImages()
#imageUtime = cameraview.imageManager.getUtime("CAMERA_LEFT")

#cameraToLocal = vtk.vtkTransform()
#cameraview.imageManager.queue.getTransform("CAMERA_LEFT", 'local', imageUtime, cameraToLocal)
#polyData = segmentation.transformPolyData(polyData, cameraToLocal)


#array = transformUtils.poseFromTransform(cameraToLocal)
#(array([ 0.92912804, -0.86441896,  1.63176906]), array([ 0.22322877, -0.39282596,  0.77466717, -0.44243355]))



# segmentation.findAndFitDrillBarrel(polyData, getLinkFrame('utorso'))

'''
>>> 
>>> o = om.getActiveObject()
>>> o
<ddapp.visualization.PolyDataItem object at 0x21f140d0>
>>> o
<ddapp.visualization.PolyDataItem object at 0x21f140d0>
>>> o.polyData
(vtkPolyData)0x21f078e8
>>> o.getArrayNames()
['rgb_colors', 'x', 'y', 'z', 'distance_along_robot_x', 'distance_along_robot_y', 'distance_along_robot_z']
>>> o.colorBy('x')
>>> 
>>> help(o.colorBy)
Help on method colorBy in module ddapp.visualization:

colorBy(self, arrayName, scalarRange=None, lut=None) method of ddapp.visualization.PolyDataItem instance

>>> 
>>> 
>>> o.colorBy('x', scalarRange=[0,1])
>>> 
>>> o.colorBy('x', scalarRange=[0.0,1.0])
>>> o.colorBy('distance_along_robot_x', scalarRange=[0.0,1.0])
>>> 
>>> o.colorBy(None)
>>> 
>>> o.colorBy('distance_along_robot_x', scalarRange=[0.0,1.0])
'''
