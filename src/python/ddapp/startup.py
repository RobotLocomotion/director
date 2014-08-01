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

from ddapp import segmentationroutines
from ddapp import trackers

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
useDrakeVisualizer = True
useNavigationPanel = True
useFootContactVis = True
useImageWidget = False
useImageViewDemo = True
useControllerRate = True
useSkybox = False


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


if usePerception:

    multisenseDriver, mapServerSource = perception.init(view)
    segmentationpanel.init()
    cameraview.init()
    colorize.init()

    cameraview.cameraView.rayCallback = segmentation.extractPointsAlongClickRay
    multisensepanel.init(perception.multisenseDriver)

    disparityPointCloud = segmentation.DisparityPointCloudItem('stereo point cloud', cameraview.imageManager)
    disparityPointCloud.addToView(view)
    om.addToObjectModel(disparityPointCloud, parentObj=om.findObjectByName('sensors'))


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
            urdfFile=os.path.join(ddapp.getDRCBaseDir(), 'software/drake/examples/Atlas/urdf/atlas_convex_hull.urdf'),
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

    teleoppanel.init(robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController,
                     ikPlanner, manipPlanner, handFactory.getLoader('left'), handFactory.getLoader('right'), playbackPanel.setPlan)



    debrisDemo = debrisdemo.DebrisPlannerDemo(robotStateModel, robotStateJointController, playbackRobotModel,
                    ikPlanner, manipPlanner, atlasdriver.driver, lHandDriver,
                    perception.multisenseDriver, refitBlocks)

    tableDemo = tabledemo.TableDemo(robotStateModel, playbackRobotModel,
                    ikPlanner, manipPlanner, footstepsDriver, atlasdriver.driver, lHandDriver, rHandDriver,
                    perception.multisenseDriver, view, robotStateJointController)

    def constructDrillDemo():
        imp.reload(drilldemo)
        global drillDemo, d
        drillDemo = drilldemo.DrillPlannerDemo(robotStateModel, playbackRobotModel, teleopRobotModel, footstepsDriver, manipPlanner, ikPlanner,
                                          lHandDriver, atlasdriver.driver, perception.multisenseDriver,
                                          fitDrillMultisense, robotStateJointController,
                                          playPlans, showPose)
        d = drillDemo
    constructDrillDemo()





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


def addCollisionObjectToWorld():
    obj = om.getActiveObject()
    assert obj and obj.polyData
    polyData = filterUtils.transformPolyData(obj.polyData, obj.actor.GetUserTransform())
    pts = vnp.getNumpyFromVtk(polyData, 'Points')
    pts = pts.transpose()
    ikServer.addCollisionObject(pts)


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
viewbehaviors.ViewBehaviors.addRobotBehaviors(robotStateModel, handFactory, footstepsDriver)



def prepButtonPress():

    q = np.array([  0.00000000e+00,   0.00000000e+00,   8.52500000e-01,
        0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
        0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
        -2.52936989e-01,  -1.75060872e-02,   1.72317326e+00,
        2.27700758e+00,   1.54566648e-03,   0.00000000e+00,
        5.49999997e-02,  -4.90000010e-01,   1.00000000e+00,
        -5.09999990e-01,  -5.99999987e-02,  -4.17805046e-01,
        -4.10342306e-01,   7.97343493e-01,   1.85813034e+00,
        -2.20951104e+00,   2.16407776e+00,   0.00000000e+00,
        -5.49999997e-02,  -4.90000010e-01,   1.00000000e+00,
        -5.09999990e-01,   5.99999987e-02,  -3.95531267e-01,
        0.00000000e+00])

    robotStateJointController.setPose("EST_ROBOT_STATE", q)
    d.moveDrillToHand()
    d.addDrillButtonFrame()


def reset():
    wall=  om.findObjectByName('wall')
    om.removeFromObjectModel(wall)

    drill=  om.findObjectByName('drill')
    om.removeFromObjectModel(drill)


    constructDrillDemo()
    q = np.array([ 0.    ,  0.    ,  0.8525,  0.    ,  0.    ,  0.    ,  0.    ,
        0.    ,  0.    ,  0.27  , -1.33  ,  2.1   ,  0.5   ,  0.    ,
        0.    ,  0.055 , -0.49  ,  1.    , -0.51  , -0.06  ,  0.    ,
        0.27  ,  1.33  ,  2.1   , -0.5   ,  0.    ,  0.    , -0.055 ,
       -0.49  ,  1.    , -0.51  ,  0.06  ,  0.    ,  0.    ])
    robotStateJointController.setPose("EST_ROBOT_STATE", q)


def prepDrilling():
    wall=  om.findObjectByName('wall')
    om.removeFromObjectModel(wall)

    constructDrillDemo()
    q = np.array([  6.42492209e-03,   7.70942744e-03,   8.52836686e-01,
        -8.95310355e-03,   1.00091533e-02,  -1.37427243e-03,
        -2.18423404e-04,   7.05790648e-04,  -1.00478379e-03,
         5.85000038e-01,  -1.29673553e+00,   2.54605365e+00,
         1.63658464e+00,   1.03048101e-01,   1.96849159e-03,
         5.39942160e-02,  -4.93786156e-01,   1.00414515e+00,
        -5.20112395e-01,  -4.99881543e-02,   8.58246744e-01,
         2.70000011e-01,   1.33000004e+00,   2.09999990e+00,
        -5.00000000e-01,   0.00000000e+00,   7.10978289e-04,
        -5.59350178e-02,  -4.86784428e-01,   9.89762068e-01,
        -5.12869358e-01,   6.99684396e-02,   0.00000000e+00,
         6.92082313e-06])

    robotStateJointController.setPose("EST_ROBOT_STATE", q)
    d.moveDrillToHand()
    d.addDrillGuardFrame()

    d.spawnDrillWallAffordance()
    d.getFirstCutDesired(engagedTip=False)


def printTF(tf, msg="transform pos and rpy"):
    print msg
    print tf.GetPosition()
    print tf.GetOrientation()



if (1==1):
  #polyData = io.readPolyData(os.path.expanduser('~/Desktop/table-and-door-scene.vtp'))
  #polyData = io.readPolyData(os.path.expanduser('~/Desktop/valve-lever-scene.vtp')) # different position
  polyData = io.readPolyData(os.path.expanduser('~/Desktop/drill_on_table.vtp'))


  obj = vis.showPolyData(polyData, 'scene', parent=None, alpha=0.3)
  obj.colorBy('z', scalarRange=[-0.2, 3.0])


def doIt():
  segmentationpanel.activateSegmentationMode(polyData)

  data = segmentation.segmentDrillUsingTableOrientation(viewbehaviors.lastCachedPickedPoint )

  #print "seg"
  #data = segmentation.segmentTableScene(polyData, [1.83691132,  0.04082248 , 0.63584119] )

  #vis.showClusterObjects(data.clusters + [data.table], parent='segmentation')

  ## crude use of the table frame to determine the frame of the drill on the table
  #table_xaxis, table_yaxis, table_zaxis = transformUtils.getAxesFromTransform( data.table.frame )
  #t = transformUtils.getTransformFromAxes( table_yaxis, table_xaxis,  -1*np.array( table_zaxis) )
  #t.Translate ( data.clusters[0].frame.GetPosition() )


  ##t = transformUtils.frameFromPositionAndRPY( data.clusters[0].frame.GetPosition() , data.table.frame.GetOrientation() )

  ##t.Concatenate(self.drill.frame.transform)
  ##self.drill.graspFrame =
  #vis.updateFrame(t , 'drill frame mf', visible=True, scale=0.5)


  ##segmentation.segmentValveWallAuto(.2,'valve')
  segmentation.switchToView( 'DRC View')




