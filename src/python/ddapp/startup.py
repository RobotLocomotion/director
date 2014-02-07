# This script is executed in the main console namespace so
# that all the variables defined here become console variables.

import ddapp

import os
import sys
import vtk
import PythonQt
from PythonQt import QtCore, QtGui
import ddapp.applogic as app
from ddapp import matlab
from ddapp import jointcontrol
from ddapp import cameracontrol
from ddapp import ik
from ddapp import ikeditor
from ddapp import objectmodel as om
from ddapp import spreadsheet
from ddapp import tdx
from ddapp import perception
from ddapp import segmentation
from ddapp import cameraview
from ddapp import colorize
from ddapp import robotstate
from ddapp import footstepsdriver
from ddapp import footstepsdriverpanel
from ddapp import atlasdriver
from ddapp import atlasdriverpanel
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
usePerception = True
useSpreadsheet = True
useFootsteps = True
usePlanning = True
useAtlasDriver = True


poseCollection = PythonQt.dd.ddSignalMap()
costCollection = PythonQt.dd.ddSignalMap()


if useSpreadsheet:
    spreadsheet.init(poseCollection, costCollection)


if useIk:


    ikview = app.getViewManager().createView('IK View', 'VTK View')

    app.getViewManager().switchToView('IK View')

    ikFolder = om.addContainer('Drake IK')
    om.addPlaceholder('matlab server', om.Icons.Matlab, ikFolder)

    #urdfFile = os.path.join(app.getDRCBase(), 'software/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_fixedjoint_hands.urdf')
    urdfFile = os.path.join(app.getDRCBase(), 'software/models/mit_gazebo_models/mit_robot/model_LI_RR.urdf')

    model = app.loadRobotModelFromFile(urdfFile)
    obj = om.addRobotModel(model, ikFolder)
    obj.addToView(ikview)
    defaultRobotModel = obj


    useTable = False
    if useTable:
        tableModel = ikview.loadURDFModel(os.path.join(app.getDRCBase(), 'software/drake/systems/plants/test/table.urdf'))
        tableModel.setVisible(True)
        affordancesFolder = om.getOrCreateContainer('affordances')
        om.addRobotModel(tableModel, affordancesFolder)


    jc = jointcontrol.JointController([model], poseCollection)
    jc.setNominalPose(jc.loadPoseFromFile(app.getNominalPoseMatFile()))
    jc.addPose('q_end', jc.poses['q_nom'])
    jc.addPose('q_start', jc.poses['q_nom'])
    defaultJointController = jc


    def startIkServer():
        s = ik.AsyncIKCommunicator(jc)
        s.outputConsole = app.getOutputConsole()
        s.infoFunc = app.displaySnoptInfo
        s.start()
        s.startServerAsync()

        e = ikeditor.IKEditor(app.getMainWindow(), s, poseCollection, costCollection)
        e.makeFrameWidget(ikview)
        app.addWidgetToDock(e.widget)
        tdx.init(ikview, e)

    startAutomatically = False
    if startAutomatically:
        startIkServer()

    app.resetCamera(viewDirection=[-1,0,0], view=ikview)


if useAtlasDriver:
    atlasdriver.init(app.getOutputConsole())
    atlasdriverpanel.init(atlasdriver.driver)



if usePerception:


    mitRobotDir = os.path.join(app.getDRCBase(), 'software/models/mit_gazebo_models/mit_robot')
    urdfFile = os.path.join(mitRobotDir, 'model_LI_RR.urdf')

    robotStateModel = app.loadRobotModelFromFile(urdfFile)


    robotStateJointController = jointcontrol.JointController([robotStateModel])
    robotStateJointController.setNominalPose(robotStateJointController.loadPoseFromFile(app.getNominalPoseMatFile()))
    robotStateJointController.setPose('EST_ROBOT_STATE', robotStateJointController.getPose('q_zero'))
    defaultJointController = robotStateJointController


    perception.init(view, robotStateJointController)
    segmentationpanel.init()
    cameraview.init()
    colorize.init()

    sensorsFolder = om.getOrCreateContainer('sensors')
    robotStateModel = om.addRobotModel(robotStateModel, sensorsFolder)
    robotStateModel.addToView(view)
    defaultRobotModel = robotStateModel

    cameraview.cameraView.rayCallback = segmentation.extractPointsAlongClickRay

    def grabRobotState():
        poseName = 'EST_ROBOT_STATE'
        robotStatePose = robotStateJointController.poses[poseName]
        s.sendPoseToServer(robotStatePose, poseName)
        s.forcePose(poseName)


    def onRobotModel(m):

        global robotStateModel, defaultRobotModel

        model = app.loadRobotModelFromString(m.urdf_xml_string)
        sensorsFolder = om.getOrCreateContainer('sensors')
        obj = om.addRobotModel(model, sensorsFolder)
        obj.setProperty('Name', 'model publisher')
        robotStateJointController.models.append(model)
        robotStateJointController.push()

        obj.addToView(robotStateModel.views[0])
        om.removeFromObjectModel(robotStateModel)
        robotStateJointController.models.remove(robotStateModel.model)
        robotStateModel = obj
        defaultRobotModel = obj

    lcmUtils.captureMessageCallback('ROBOT_MODEL', lcmdrc.robot_urdf_t, onRobotModel)


    def setupFinger(pos):

        grabRobotState()
        p = perception._multisenseItem.model.revPolyData
        vis.showPolyData(p, 'lidar', alpha=0.3)
        t = s.grabCurrentLinkPose('l_hand')
        vis.showFrame(t, 'lhand')

        fingerTip = np.array(pos)

        handLinkPos = np.array(t.GetPosition())
        posOffset = fingerTip - handLinkPos

        ft = vtk.vtkTransform()
        ft.DeepCopy(t)

        ft.PostMultiply()
        ft.Translate(posOffset)

        vis.showFrame(ft, 'fingerTip')

        hand_to_finger = vtk.vtkTransform()
        hand_to_finger.DeepCopy(ft)
        hand_to_finger.Concatenate(t.GetLinearInverse())

        posOffset = np.array(hand_to_finger.GetPosition())

        s.setPointInLink('l_hand', posOffset)



if useFootsteps:
    footstepsDriver = footstepsdriver.FootstepsDriver(defaultJointController)
    footstepsdriverpanel.init(footstepsDriver)


if usePlanning:


    planningFolder = om.getOrCreateContainer('planning')

    urdfFile = os.path.join(app.getDRCBase(), 'software/models/mit_gazebo_models/mit_robot/model_LI_RI.urdf')

    planningModel = app.loadRobotModelFromFile(urdfFile)
    obj = om.addRobotModel(planningModel, planningFolder)
    obj.addToView(view)
    obj.setProperty('Visible', False)
    obj.setProperty('Name', 'robot model')
    obj.setProperty('Color', QtGui.QColor(255, 253, 213))
    planningRobotModel = obj

    handDriver = handdriver.IRobotHandDriver(side='left')

    planningJc = jointcontrol.JointController([planningModel], poseCollection)
    planningJc.setNominalPose(planningJc.loadPoseFromFile(app.getNominalPoseMatFile()))

    #midiController = jointcontrol.MidiJointControl(planningJc)
    #midiController.start()

    manipPlanner = robotplanlistener.ManipulationPlanDriver()
    planPlayback = robotplanlistener.RobotPlanPlayback()


    def playPlan(plan):
        playPlans([plan])

    def playPlans(plans):
        planPlayback.stopAnimation()
        planningRobotModel.setProperty('Visible', True)
        planPlayback.playPlans(plans, planningJc)

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
    app.addToolbarMacro('fit drill', fitDrillMultisense)

    def drillTrackerOn():
        om.findObjectByName('Multisense').model.showRevolutionCallback = fitDrillMultisense

    def drillTrackerOff():
        om.findObjectByName('Multisense').model.showRevolutionCallback = None


    planner = plansequence.PlanSequence(defaultRobotModel, footstepsDriver, manipPlanner,
                                        handDriver, atlasdriver.driver, perception.multisenseDriver,
                                        fitDrillMultisense, robotStateJointController,
                                        playPlans)

    planner.userPromptEnabled = False
    q = planner.autonomousExecute()


app.resetCamera(viewDirection=[-1,0,0], view=view)



def affUpdaterOn():
    vis.affup.updater.on()

def affUpdaterOff():
    vis.affup.updater.off()


def getLinkFrame(linkName, model=None):
    model = model or defaultRobotModel
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
        pose = defaultJointController.q
    msg = robotstate.drakePoseToRobotState(pose)
    lcmUtils.publish('EST_ROBOT_STATE', msg)


tc = TimerCallback()
tc.targetFps = 60
tc.callback = resetCameraToHeadView



'''
class ViewEventFilter(object):

    def __init__(self, view):
        self.view = view
        self.initEventFilter()
        self.doubleClickCallback = None


    def filterEvent(self, obj, event):
        if event.type() == QtCore.QEvent.MouseButtonDblClick:
            if self.doubleClickCallback:
                result = self.doubleClickCallback(vis.mapMousePosition(obj, event))
                self.eventFilter.setEventHandlerResult(result)

    def initEventFilter(self):
        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        qvtkwidget = self.view.vtkWidget()
        qvtkwidget.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonDblClick)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.filterEvent)


def onViewDoubleClicked(displayPoint):

    model = defaultRobotModel.model

    polyData, pickedPoint = vis.pickPoint(displayPoint, view=ikview, pickType='cells')
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

ef = ViewEventFilter(ikview)
ef.doubleClickCallback = onViewDoubleClicked
'''
