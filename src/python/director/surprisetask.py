import os
import sys
from . import vtkAll as vtk
import math
import time
import types
import functools
import numpy as np

from director import transformUtils
from director import lcmUtils
from director.timercallback import TimerCallback
from director.asynctaskqueue import AsyncTaskQueue
from director import objectmodel as om
from director import visualization as vis
from director import applogic as app
from director.debugVis import DebugData
from director import ikplanner
from director.ikparameters import IkParameters
from director import ioUtils
from director.simpletimer import SimpleTimer
from director.utime import getUtime
from director import affordanceitems
from director import robotstate
from director import robotplanlistener
from director import segmentation
from director import planplayback
from director import affordanceupdater
from director import segmentationpanel
from director import vtkNumpy as vnp
from director import switchplanner

from director.tasks.taskuserpanel import TaskUserPanel
from director.tasks.taskuserpanel import ImageBasedAffordanceFit

import director.tasks.robottasks as rt
import director.tasks.taskmanagerwidget as tmw

import drc as lcmdrc
import copy

from PythonQt import QtCore, QtGui



class SurpriseTaskPlanner(object):

    def __init__(self, robotSystem):
        self.robotSystem = robotSystem
        self.robotModel = robotSystem.robotStateModel
        self.ikPlanner = robotSystem.ikPlanner
        self.lockBackForManip = False
        self.lockBaseForManip = True
        self.side = 'right'
        self.toolTipToHandFrame = robotSystem.ikPlanner.newPalmOffsetGraspToHandFrame(self.side, 0.1)


class ImageFitter(ImageBasedAffordanceFit):

    def __init__(self, switchPlanner):
        ImageBasedAffordanceFit.__init__(self, numberOfPoints=1)
        self.switchPlanner = switchPlanner
        self.fitFunc = None
        self.pickLineRadius = 0.05
        self.pickNearestToCamera = False

        self.useLocalPlaneFit = True
        self.useVoxelGrid = True

    def fit(self, polyData, points):
        if self.fitFunc:
            self.fitFunc(polyData, points)

    def fitSwitchBox(self, polyData, points):
        boxPosition = points[0]
        wallPoint = points[1]


        # find a frame that is aligned with wall
        searchRadius = 0.2
        planePoints, normal = segmentation.applyLocalPlaneFit(polyData, points[0], searchRadius=np.linalg.norm(points[1] - points[0]), searchRadiusEnd=1.0)

        obj = vis.updatePolyData(planePoints, 'wall plane points', color=[0,1,0], visible=False)
        obj.setProperty('Point Size', 7)

        viewDirection = segmentation.SegmentationContext.getGlobalInstance().getViewDirection()
        if np.dot(normal, viewDirection) < 0:
            normal = -normal

        origin = segmentation.computeCentroid(planePoints)

        zaxis = [0,0,1]
        xaxis = normal
        yaxis = np.cross(zaxis, xaxis)
        yaxis /= np.linalg.norm(yaxis)
        zaxis = np.cross(xaxis, yaxis)
        zaxis /= np.linalg.norm(zaxis)

        t = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)

        # translate that frame to the box position
        t.PostMultiply()
        t.Translate(boxPosition)

        boxFrame = transformUtils.copyFrame(t)
        self.switchPlanner.spawnBoxAffordanceAtFrame(boxFrame)

class SurpriseTaskPanel(TaskUserPanel):

    def __init__(self, robotSystem):

        TaskUserPanel.__init__(self, windowTitle='Surprise Task')

        self.planner = SurpriseTaskPlanner(robotSystem)
        self.switchPlanner = switchplanner.SwitchPlanner(robotSystem)
        self.fitter = ImageFitter(self.switchPlanner)
        self.initImageView(self.fitter.imageView)

        self.addDefaultProperties()
        self.addButtons()
        self.addSwitchTasks()


    def test(self):
        print('test')

    def addButtons(self):

        self.addManualSpacer()
        self.addManualButton('arms prep 1', self.switchPlanner.planArmsPrep1)
        self.addManualButton('arms prep 2', self.switchPlanner.planArmsPrep2)
        self.addManualButton('fit switch box', self.fitSwitchBox)
        self.addManualButton('spawn switch box affordance', self.switchPlanner.spawnBoxAffordance)
        self.addManualButton('spawn footstep frame', self.switchPlanner.spawnFootstepFrame)
        self.addManualButton('reset reach frame', self.switchPlanner.updateReachFrame)
        # self.addManualButton('plan reach to reach frame', self.switchPlanner.planReach)
        self.addManualButton('Reach to pinch reach frame', self.onPlanPinchReach)
        self.addManualButton('Commit Manip Plan', self.switchPlanner.commitManipPlan)

    def onPlanPinchReach(self):
        self.switchPlanner.planPinchReach(maxDegreesPerSecond=self.maxDegreesPerSecond)

    def getSide(self):
        return self.params.getPropertyEnumValue('Hand').lower()

    def addDefaultProperties(self):
        self.params.addProperty('max degrees per second', 10, attributes=om.PropertyAttributes(singleStep=1, decimals=2))
        self.params.addProperty('Hand', 0, attributes=om.PropertyAttributes(enumNames=['Left', 'Right']))
        self.params.setProperty('Hand', self.planner.side.capitalize())


    def onPropertyChanged(self, propertySet, propertyName):
        if propertyName == 'Hand':
            self.planner.side = self.getSide()

        self.syncProperties()

    def syncProperties(self):
        self.maxDegreesPerSecond = self.params.getProperty('max degrees per second')

    def setParamsPreTeleop(self):
        self.params.setProperty('max degrees per second', 30)

    def setParamsTeleop(self):
        self.params.setProperty('max degrees per second', 10)

    def addTasks(self):

        # some helpers
        self.folder = None
        def addTask(task, parent=None):
            parent = parent or self.folder
            self.taskTree.onAddTask(task, copy=False, parent=parent)
        def addFunc(name, func, parent=None):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)
        def addFolder(name, parent=None):
            self.folder = self.taskTree.addGroup(name, parent=parent)
            return self.folder

        def addManipTask(name, planFunc, userPrompt=False):

            prevFolder = self.folder
            addFolder(name, prevFolder)
            addFunc('plan', planFunc)
            if not userPrompt:
                addTask(rt.CheckPlanInfo(name='check manip plan info'))
            else:
                addTask(rt.UserPromptTask(name='approve manip plan', message='Please approve manipulation plan.'))
            addFunc('execute manip plan', self.drillDemo.commitManipPlan)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'))
            self.folder = prevFolder


        self.taskTree.removeAllTasks()
        side = self.getSide()

        ###############
        # add the tasks

        # prep
        # addFolder('Prep')
        # addTask(rt.CloseHand(name='close left hand', side='Left'))
        # addTask(rt.CloseHand(name='close right hand', side='Right'))
        self.addSwitchTasks()

    def addSwitchTasks(self):

        # some helpers
        self.folder = None
        def addTask(task, parent=None):
            parent = parent or self.folder
            self.taskTree.onAddTask(task, copy=False, parent=parent)
        def addFunc(name, func, parent=None):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)
        def addFolder(name, parent=None):
            self.folder = self.taskTree.addGroup(name, parent=parent)
            return self.folder

        def addManipTask(name, planFunc, userPrompt=False):

            prevFolder = self.folder
            addFolder(name, prevFolder)
            addFunc('plan', planFunc)
            if not userPrompt:
                addTask(rt.CheckPlanInfo(name='check manip plan info'))
            else:
                addTask(rt.UserPromptTask(name='approve manip plan', message='Please approve manipulation plan.'))
            addFunc('execute manip plan', self.switchPlanner.commitManipPlan)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'))
            self.folder = prevFolder


        self.taskTree.removeAllTasks()
        side = self.getSide()

        addFolder('Fit Box Affordance')
        addFunc('fit switch box affordance', self.fitSwitchBox)
        addTask(rt.UserPromptTask(name='verify/adjust affordance', message='verify/adjust affordance.'))

        # walk to drill
        addFolder('Walk')
        addFunc('plan footstep frame', self.switchPlanner.spawnFootstepFrame)
        addTask(rt.RequestFootstepPlan(name='plan walk to drill', stanceFrameName='switch box stance frame'))
        addTask(rt.UserPromptTask(name='approve footsteps', message='Please approve footstep plan.'))
        addTask(rt.CommitFootstepPlan(name='walk to switch box', planName='switch box stance frame footstep plan'))
        addTask(rt.WaitForWalkExecution(name='wait for walking'))

        armsUp = addFolder('Arms Up')
        addManipTask('Arms Up 1', self.switchPlanner.planArmsPrep1, userPrompt=True)
        self.folder = armsUp
        addManipTask('Arms Up 2', self.switchPlanner.planArmsPrep2, userPrompt=True)
        addTask(rt.CloseHand(side='Right', mode='Pinch', name='set finger pinch'))
        reach = addFolder('Reach')

        addFunc('set degrees per second 30', self.setParamsPreTeleop)
        addFunc('update reach frame', self.switchPlanner.updateReachFrame)
        addTask(rt.UserPromptTask(name='adjust frame', message='adjust reach frame if necessary'))
        addManipTask('reach above box', self.onPlanPinchReach, userPrompt=True)

        teleop = addFolder('Teleop')
        addFunc('set degrees per second 10', self.setParamsTeleop)
        addTask(rt.UserPromptTask(name='wait for teleop', message='continue when finished with task.'))


        armsDown = addFolder('Arms Down')
        addTask(rt.UserPromptTask(name='check left hand free', message='check left hand free to close and move back'))
        addTask(rt.CloseHand(name='close left hand', side='Right'))
        addManipTask('Arms Down 1', self.switchPlanner.planArmsPrep2, userPrompt=True)
        self.folder = armsDown
        self.folder = armsDown
        addManipTask('Arms Down 2', self.switchPlanner.planArmsPrep1, userPrompt=True)
        self.folder = armsDown
        addManipTask('plan nominal', self.switchPlanner.planNominal, userPrompt=True)


    def fitSwitchBox(self):
        print('fitting switch box')
        self.fitter.imagePicker.numberOfPoints = 2
        self.fitter.pointCloudSource = 'lidar'
        self.fitter.fitFunc = self.fitter.fitSwitchBox




