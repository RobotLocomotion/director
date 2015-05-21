import os
import sys
import vtkAll as vtk
from ddapp import botpy
import math
import time
import types
import functools
import numpy as np

from ddapp import transformUtils
from ddapp import lcmUtils
from ddapp.timercallback import TimerCallback
from ddapp.asynctaskqueue import AsyncTaskQueue
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp import applogic as app
from ddapp.debugVis import DebugData
from ddapp import ikplanner
from ddapp.ikparameters import IkParameters
from ddapp import ioUtils
from ddapp.simpletimer import SimpleTimer
from ddapp.utime import getUtime
from ddapp import affordanceitems
from ddapp import robotstate
from ddapp import robotplanlistener
from ddapp import segmentation
from ddapp import planplayback
from ddapp import affordanceupdater
from ddapp import segmentationpanel
from ddapp import vtkNumpy as vnp

from ddapp.tasks.taskuserpanel import TaskUserPanel
from ddapp.tasks.taskuserpanel import ImageBasedAffordanceFit

import ddapp.tasks.robottasks as rt
import ddapp.tasks.taskmanagerwidget as tmw

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



class SurpriseTaskPanel(TaskUserPanel):

    def __init__(self, robotSystem):

        TaskUserPanel.__init__(self, windowTitle='Surprise Task')

        self.planner = SurpriseTaskPlanner(robotSystem)

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()


    def test(self):
        print 'test'

    def addButtons(self):

        self.addManualSpacer()
        self.addManualButton('test', self.test)

    def getSide(self):
        return self.params.getPropertyEnumValue('Hand').lower()

    def addDefaultProperties(self):
        self.params.addProperty('Hand', 0, attributes=om.PropertyAttributes(enumNames=['Left', 'Right']))
        self.params.setProperty('Hand', self.planner.side.capitalize())

    def onPropertyChanged(self, propertySet, propertyName):
        if propertyName == 'Hand':
            self.planner.side = self.getSide()

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
        addFolder('Prep')
        addTask(rt.CloseHand(name='close left hand', side='Left'))
        addTask(rt.CloseHand(name='close right hand', side='Right'))


