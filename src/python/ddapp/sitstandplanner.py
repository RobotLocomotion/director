import ddapp
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp import objectmodel as om
from ddapp import ik

import os
import functools
import numpy as np
import scipy.io
from ddapp.tasks.taskuserpanel import TaskUserPanel

class SitStandPlanner(object):

    def __init__(self, ikServer, robotSystem):
        self.ikServer = ikServer
        self.robotSystem = robotSystem
        self.ikServer.connectStartupCompleted(self.initialize)
        self.initializedFlag = False
        self.planOptions = dict()

    def initialize(self, ikServer, success):
        if ikServer.restarted:
            return
        commands = self.getInitCommands()
        self.ikServer.taskQueue.addTask(functools.partial(self.ikServer.comm.sendCommandsAsync, commands))
        self.ikServer.taskQueue.start()
        self.initializedFlag = True

    def getInitCommands(self):
        commands = ['''
        addpath([getenv('DRC_BASE'), '/software/control/matlab/planners/chair_standup']);
        pssRobot = s.robot;
        warning('Off','KinematicPoseTrajectory:Terrain');
        pss = PlanSitStand(pssRobot);
      ''']
        return commands

    def applyParams(self):
        commands = []
        commands.append("pss.plan_options.chair_height = %s;" %self.planOptions['chair_height'])
        commands.append("pss.plan_options.speed = %s;" %self.planOptions['speed'])
        commands.append("pss.plan_options.back_gaze_bound = %s;" %self.planOptions['back_gaze_bound'])
        commands.append("pss.plan_options.min_distance = %s;" %self.planOptions['min_distance'])
        commands.append("pss.plan_options.pelvis_gaze_bound = %s;" %self.planOptions['pelvis_gaze_bound'])
        commands.append("pss.plan_options.pelvis_gaze_angle = %s;" %self.planOptions['pelvis_gaze_angle'])
        commands.append("pss.plan_options.sit_back_distance = %s;" %self.planOptions['sit_back_distance'])
        commands.append("pss.plan_options.bky_angle = %s;" %self.planOptions['bky_angle'])

        self.ikServer.taskQueue.addTask(functools.partial(self.ikServer.comm.sendCommandsAsync, commands))
        self.ikServer.taskQueue.start()        

    def plan(self,planType):
        self.applyParams()
        startPose = self.getPlanningStartPose()
        commands = []
        commands.append("pss.planSitting(%s, '%s');" % (ik.ConstraintBase.toColumnVectorString(startPose),
            planType))

        self.ikServer.taskQueue.addTask(functools.partial(self.ikServer.comm.sendCommandsAsync, commands))
        self.ikServer.taskQueue.start()

    def getPlanningStartPose(self):
        return self.robotSystem.robotStateJointController.q


class SitStandPlannerPanel(TaskUserPanel):

    def __init__(self, robotSystem):

        TaskUserPanel.__init__(self, windowTitle='Sit/Stand')

        self.robotSystem = robotSystem
        self.sitStandPlanner = SitStandPlanner(robotSystem.ikServer, robotSystem)
        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()
        

    def addButtons(self):
        self.addManualButton('Start', self.onStart)
        self.addManualButton('Sit', functools.partial(self.onPlan, 'sit'))
        self.addManualButton('Stand', functools.partial(self.onPlan, 'stand'))
        self.addManualButton('Squat', functools.partial(self.onPlan, 'squat'))
        self.addManualButton('Stand From Squat', functools.partial(self.onPlan, 'stand_from_squat'))
        self.addManualButton('Sit From Current', functools.partial(self.onPlan, 'sit_from_current'))
        self.addManualButton('Hold With Pelvis Contact', functools.partial(self.onPlan, 'hold_with_pelvis_contact'))
        self.addManualButton('Hold Without Pelvis Contact', functools.partial(self.onPlan, 'hold_without_pelvis_contact'))

    def onStart(self):
        self._syncProperties()

    def onPlan(self,planType):
        self._syncProperties()
        self.sitStandPlanner.plan(planType)

    def addDefaultProperties(self):
        self.params.addProperty('Chair Height', 0.57, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Sit Back Distance', 0.2, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Speed', 1, attributes=om.PropertyAttributes(singleStep=0.1, decimals=3))
        self.params.addProperty('Back Gaze Bound', 0.4, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Min Distance', 0.1, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Pelvis Gaze Bound', 0.1, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Pelvis Gaze Angle', 0, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Back y Angle', -0.2, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))

    def _syncProperties(self):
        self.sitStandPlanner.planOptions['chair_height'] = self.params.getProperty('Chair Height')
        self.sitStandPlanner.planOptions['sit_back_distance'] = self.params.getProperty('Sit Back Distance')
        self.sitStandPlanner.planOptions['speed'] = self.params.getProperty('Speed')
        self.sitStandPlanner.planOptions['back_gaze_bound'] = self.params.getProperty('Back Gaze Bound')
        self.sitStandPlanner.planOptions['min_distance'] = self.params.getProperty('Min Distance')
        self.sitStandPlanner.planOptions['pelvis_gaze_bound']= self.params.getProperty('Pelvis Gaze Bound')
        self.sitStandPlanner.planOptions['pelvis_gaze_angle'] = self.params.getProperty('Pelvis Gaze Angle')
        self.sitStandPlanner.planOptions['bky_angle'] = self.params.getProperty('Back y Angle')
        self.sitStandPlanner.applyParams()

    def onPropertyChanged(self, propertySet, propertyName):
        self._syncProperties()

    def addTasks(self):

        # some helpers
        self.folder = None
        def addTask(task, parent=None):
            parent = parent or self.folder
            self.taskTree.onAddTask(task, copy=False, parent=parent)
        def addFunc(func, name, parent=None):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)
        def addFolder(name, parent=None):
            self.folder = self.taskTree.addGroup(name, parent=parent)
            return self.folder