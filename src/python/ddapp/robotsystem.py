import ddapp

import os
import sys
import PythonQt
from PythonQt import QtCore, QtGui
from time import time
import imp
from ddapp import applogic
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


def create(view=None, globalsDict=None):
    s = RobotSystem()
    s.init(view, globalsDict)


class RobotSystem(object):


    def __init__(self):
        pass


    def init(self, view=None, globalsDict=None):

        view = view or applogic.getCurrentRenderView()

        useIk = True
        useRobotState = True
        usePerception = True
        useFootsteps = True
        useHands = True
        usePlanning = True
        useAtlasDriver = True


        if useIk:

            ikRobotModel, ikJointController = roboturdf.loadRobotModel('ik model', parent=None)
            om.removeFromObjectModel(ikRobotModel)

            ikJointController.addPose('q_end', ikJointController.getPose('q_nom'))
            ikJointController.addPose('q_start', ikJointController.getPose('q_nom'))

            ikServer = ik.AsyncIKCommunicator()

            def startIkServer():
                ikServer.startServerAsync()
                ikServer.comm.writeCommandsToLogFile = True

            #startIkServer()


        if useAtlasDriver:
            atlasDriver = atlasdriver.init(None)


        if useRobotState:
            robotStateModel, robotStateJointController = roboturdf.loadRobotModel('robot state model', view, parent='sensors', color=roboturdf.getRobotGrayColor(), visible=True)
            robotStateJointController.setPose('EST_ROBOT_STATE', robotStateJointController.getPose('q_nom'))
            roboturdf.startModelPublisherListener([(robotStateModel, robotStateJointController)])
            robotStateJointController.addLCMUpdater('EST_ROBOT_STATE')


        if usePerception:
            multisenseDriver, mapServerSource = perception.init(view)


        if useHands:
            rHandDriver = handdriver.RobotiqHandDriver(side='right')
            lHandDriver = handdriver.RobotiqHandDriver(side='left')


        if useFootsteps:
            footstepsDriver = footstepsdriver.FootstepsDriver(robotStateJointController)


        if usePlanning:

            manipPlanner = robotplanlistener.ManipulationPlanDriver()

            handFactory = roboturdf.HandFactory(robotStateModel)
            handModels = [handFactory.getLoader(side) for side in ['left', 'right']]

            ikPlanner = ikplanner.IKPlanner(ikServer, ikRobotModel, ikJointController, handModels)


            tableDemo = tabledemo.TableDemo(robotStateModel, None,
                            ikPlanner, manipPlanner, footstepsDriver, atlasdriver.driver, lHandDriver, rHandDriver,
                            perception.multisenseDriver, view, robotStateJointController)


        applogic.resetCamera(viewDirection=[-1,0,0], view=view)
        viewbehaviors.ViewBehaviors.addRobotBehaviors(robotStateModel, handFactory, footstepsDriver)


        if globalsDict is not None:
            globalsDict.update(locals())
            for arg in ['globalsDict', 'self']:
                del globalsDict[arg]


