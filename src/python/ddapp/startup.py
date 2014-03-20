# This script is executed in the main console namespace so
# that all the variables defined here become console variables.

import ddapp

import os
import sys
import PythonQt
from PythonQt import QtCore, QtGui
import ddapp.applogic as app
from ddapp import vtkAll as vtk
from ddapp import matlab
from ddapp import jointcontrol
from ddapp import cameracontrol
from ddapp import ik
from ddapp import ikeditor
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
from ddapp import lcmgl
from ddapp import atlasdriver
from ddapp import atlasdriverpanel
from ddapp import atlasstatuspanel
from ddapp import multisensepanel
from ddapp import handcontrolpanel
from ddapp import robotplanlistener
from ddapp import handdriver
from ddapp import plansequence
from ddapp import vtkNumpy as vnp
from ddapp import visualization as vis
from ddapp import actionhandlers
from ddapp.timercallback import TimerCallback
from ddapp import segmentationpanel
from ddapp import lcmUtils
from ddapp.shallowCopy import shallowCopy
import drc as lcmdrc

from ddapp import botpy

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


useIk = False
useRobotState = True
usePerception = True
useGrid = True
useSpreadsheet = True
useFootsteps = True
useHands = True
usePlanning = True
useAtlasDriver = True
useLCMGL = True
useDrakeVisualizer = True

poseCollection = PythonQt.dd.ddSignalMap()
costCollection = PythonQt.dd.ddSignalMap()


if useSpreadsheet:
    spreadsheet.init(poseCollection, costCollection)


if useIk:

    ikRobotModel, ikJointController = roboturdf.loadRobotModel('ik model', view, parent='IK Server', color=roboturdf.getRobotOrangeColor(), visible=False)
    ikJointController.addPose('q_end', ikJointController.getPose('q_nom'))
    ikJointController.addPose('q_start', ikJointController.getPose('q_nom'))

    def startIkServer():
        global s
        s = ik.AsyncIKCommunicator(ikJointController)
        s.outputConsole = app.getOutputConsole()
        s.infoFunc = app.displaySnoptInfo
        s.start()
        s.startServerAsync()

    startAutomatically = True
    if startAutomatically:
        startIkServer()
    else:
        s = None


if useAtlasDriver:
    atlasdriver.init(app.getOutputConsole())
    atlasdriverpanel.init(atlasdriver.driver)
    atlasstatuspanel.init(atlasdriver.driver)


if useRobotState:
    robotStateModel, robotStateJointController = roboturdf.loadRobotModel('robot state model', view, parent='sensors', color=roboturdf.getRobotGrayColor(), visible=True)
    robotStateJointController.setPose('EST_ROBOT_STATE', robotStateJointController.getPose('q_nom'))
    roboturdf.startModelPublisherListener([(robotStateModel, robotStateJointController)])
    robotStateJointController.addLCMUpdater('EST_ROBOT_STATE')


if usePerception:

    perception.init(view)
    segmentationpanel.init()
    cameraview.init()
    colorize.init()

    cameraview.cameraView.rayCallback = segmentation.extractPointsAlongClickRay
    multisensepanel.init(perception.multisenseDriver)


if useGrid:
    vis.showGrid(view)
    view.connect('computeBoundsRequest(ddQVTKWidgetView*)', vis.computeViewBoundsNoGrid)
    app.toggleCameraTerrainMode(view)


if useHands:
    rHandDriver = handdriver.RobotiqHandDriver(side='right')
    lHandDriver = handdriver.RobotiqHandDriver(side='left')
    handcontrolpanel.init(lHandDriver, rHandDriver)


if useFootsteps:
    footstepsDriver = footstepsdriver.FootstepsDriver(robotStateJointController)
    footstepsdriverpanel.init(footstepsDriver)


if useLCMGL:
    lcmgl.init(view)


if useDrakeVisualizer:
    drakeVis = drakevisualizer.DrakeVisualizer(view)


if usePlanning:

    playbackRobotModel, playbackJointController = roboturdf.loadRobotModel('playback model', view, parent='planning', color=roboturdf.getRobotOrangeColor(), visible=False)
    teleopRobotModel, teleopJointController = roboturdf.loadRobotModel('teleop model', view, parent='planning', color=roboturdf.getRobotOrangeColor(), visible=False)


    manipPlanner = robotplanlistener.ManipulationPlanDriver()
    planPlayback = robotplanlistener.RobotPlanPlayback()


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

    app.addToolbarMacro('plot plan', plotManipPlan)
    app.addToolbarMacro('play manip plan', playManipPlan)
    #app.addToolbarMacro('fit drill', fitDrillMultisense)

    def drillTrackerOn():
        om.findObjectByName('Multisense').model.showRevolutionCallback = fitDrillMultisense

    def drillTrackerOff():
        om.findObjectByName('Multisense').model.showRevolutionCallback = None


    planner = plansequence.PlanSequence(robotStateModel, footstepsDriver, manipPlanner,
                                        lHandDriver, atlasdriver.driver, perception.multisenseDriver,
                                        fitDrillMultisense, robotStateJointController,
                                        playPlans, showPose)

    #planner.userPromptEnabled = False
    #q = planner.autonomousExecute()
    #defaultJointController.setPose('EST_ROBOT_STATE', defaultJointController.getPose('q_nom'))
    #planner.spawnDrillAffordance()




app.resetCamera(viewDirection=[-1,0,0], view=view)



def affUpdaterOn():
    vis.affup.updater.on()

def affUpdaterOff():
    vis.affup.updater.off()


def getLinkFrame(linkName, model=None):
    model = model or robotStateModel
    return model.getLinkFrame(linkName)


def showLinkFrame(linkName, model=None):
    frame = getLinkFrame(linkName, model)
    if not frame:
        raise Exception('Link not found: ' + linkName)
    return vis.updateFrame(frame, linkName, parent='link frames')


def resetCameraToRobot():
    t = getLinkFrame('utorso')
    focalPoint = [0.3, 0.0, 0.3]
    position = [-4.0, -2.0, 2.0]
    t.TransformPoint(focalPoint, focalPoint)
    t.TransformPoint(position, position)
    c = view.camera()
    c.SetFocalPoint(focalPoint)
    c.SetPosition(position)
    c.SetViewUp([0.0, 0.0, 1.0])
    view.render()


def resetCameraToHeadView():

    head = getLinkFrame('head')
    utorso = getLinkFrame('utorso')

    viewDirection = np.array([1.0, 0.0, 0.0])
    utorso.TransformVector(viewDirection, viewDirection)

    cameraPosition = np.array(head.GetPosition()) + 0.10 * viewDirection

    camera = view.camera()

    focalOffset = np.array(camera.GetFocalPoint()) - np.array(camera.GetPosition())
    focalOffset /= np.linalg.norm(focalOffset)

    camera.SetPosition(cameraPosition)
    camera.SetFocalPoint(cameraPosition + focalOffset*0.03)
    camera.SetViewUp([0, 0, 1])
    camera.SetViewAngle(90)
    view.render()


def sendEstRobotState(pose=None):
    if pose is None:
        pose = robotStateJointController.q
    msg = robotstate.drakePoseToRobotState(pose)
    lcmUtils.publish('EST_ROBOT_STATE', msg)


tc = TimerCallback()
tc.targetFps = 60
tc.callback = resetCameraToHeadView


class ViewEventFilter(object):

    def __init__(self, view):
        self.view = view
        self.initEventFilter()
        self.doubleClickCallback = None


    def filterEvent(self, obj, event):
        if event.type() == QtCore.QEvent.MouseButtonDblClick:
            if self.doubleClickCallback:
                result = self.doubleClickCallback(vis.mapMousePosition(obj, event), self.view)
                self.eventFilter.setEventHandlerResult(result)

    def initEventFilter(self):
        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        qvtkwidget = self.view.vtkWidget()
        qvtkwidget.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonDblClick)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.filterEvent)


def highlightSelectedLink(displayPoint, view):

    model = robotStateModel.model

    polyData, pickedPoint = vis.pickPoint(displayPoint, view=view, pickType='cells')
    linkName = model.getLinkNameForMesh(polyData)
    if not linkName:
        return False

    colorNoHighlight = QtGui.QColor(190, 190, 190)
    colorHighlight = QtCore.Qt.red

    linkNames = [linkName]
    model.setColor(colorNoHighlight)

    for name in linkNames:
        model.setLinkColor(name, colorHighlight)

    return True


def toggleChildFrameWidget(displayPoint, view):

    pickedObj, pickedPoint = vis.findPickedObject(displayPoint, view=view)
    if not pickedObj:
        return False

    name = pickedObj.getProperty('Name')
    children = om.getObjectChildren(pickedObj)

    for child in children:
        if isinstance(child, vis.FrameItem) and child.getProperty('Name') == name + ' frame':
            edit = not child.getProperty('Edit')
            child.setProperty('Edit', edit)
            pickedObj.setProperty('Alpha', 0.5 if edit else 1.0)
            return True

    return False


    return False


def callbackSwitch(displayPoint, view):

  if toggleChildFrameWidget(displayPoint, view):
      return

  #if highlightSelectedLink(displayPoint, view):
  #    return

  if segmentationpanel.activateSegmentationMode():
        return


ef = ViewEventFilter(view)
ef.doubleClickCallback = callbackSwitch
