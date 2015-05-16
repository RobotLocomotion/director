import ddapp
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp import objectmodel as om
from ddapp import ik
from ddapp import polarisplatformplanner
from ddapp import sitstandplanner

import os
import functools
import numpy as np
import scipy.io
from ddapp.tasks.taskuserpanel import TaskUserPanel
import ddapp.tasks.robottasks as rt


class EgressPanel(TaskUserPanel):

    def __init__(self, robotSystem):

        TaskUserPanel.__init__(self, windowTitle='Egress')

        self.robotSystem = robotSystem
        self.sitStandPlanner = sitstandplanner.SitStandPlanner(robotSystem.ikServer, robotSystem)
        self.platformPlanner = polarisplatformplanner.PolarisPlatformPlanner(robotSystem.ikServer, robotSystem)
        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()
        

    def addButtons(self):
        #sit/stand buttons
        self.addManualButton('Start', self.onStart)
        self.addManualButton('Sit', functools.partial(self.onPlan, 'sit'))
        self.addManualButton('Stand', functools.partial(self.onPlan, 'stand'))
        self.addManualButton('Squat', functools.partial(self.onPlan, 'squat'))
        self.addManualButton('Stand From Squat', functools.partial(self.onPlan, 'stand_from_squat'))
        self.addManualButton('Sit From Current', functools.partial(self.onPlan, 'sit_from_current'))
        self.addManualButton('Hold With Pelvis Contact', functools.partial(self.onPlan, 'hold_with_pelvis_contact'))
        self.addManualButton('Hold Without Pelvis Contact', functools.partial(self.onPlan, 'hold_without_pelvis_contact'))

        # polaris step down buttons
        self.addManualButton('Fit Platform Affordance', self.platformPlanner.fitRunningBoardAtFeet)
        self.addManualButton('Spawn Ground Affordance', self.platformPlanner.spawnGroundAffordance)
        self.addManualButton('Raycast Terrain', self.platformPlanner.requestRaycastTerrain)
        self.addManualButton('Update Affordance', self.platformPlanner.updateAffordance)
        self.addManualButton('Arms Up',self.onArmsUp)
        self.addManualButton('Plan Turn', self.onPlanTurn)
        self.addManualButton('Plan Step Down', self.onPlanStepDown)
        self.addManualButton('Plan Weight Shift', self.onPlanWeightShift)
        self.addManualButton('Plan Step Off', self.onPlanStepOff)

    def addDefaultProperties(self):
        self.params.addProperty('Chair Height', 0.57, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Sit Back Distance', 0.2, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Speed', 1, attributes=om.PropertyAttributes(singleStep=0.1, decimals=3))
        self.params.addProperty('Back Gaze Bound', 0.4, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Min Distance', 0.1, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Pelvis Gaze Bound', 0.1, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Pelvis Gaze Angle', 0, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))
        self.params.addProperty('Back y Angle', -0.2, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))

        self.params.addProperty('Step Off Direction', 0, attributes=om.PropertyAttributes(enumNames=['Forwards','Sideways']))

    def _syncProperties(self):
        self.sitStandPlanner.planOptions['chair_height'] = self.params.getProperty('Chair Height')
        self.sitStandPlanner.planOptions['sit_back_distance'] = self.params.getProperty('Sit Back Distance')
        self.sitStandPlanner.planOptions['speed'] = self.params.getProperty('Speed')
        self.sitStandPlanner.planOptions['back_gaze_bound'] = self.params.getProperty('Back Gaze Bound')
        self.sitStandPlanner.planOptions['min_distance'] = self.params.getProperty('Min Distance')
        self.sitStandPlanner.planOptions['pelvis_gaze_bound']= self.params.getProperty('Pelvis Gaze Bound')
        self.sitStandPlanner.planOptions['pelvis_gaze_angle'] = self.params.getProperty('Pelvis Gaze Angle')
        self.sitStandPlanner.planOptions['bky_angle'] = self.params.getProperty('Back y Angle')
        self.stepOffDirection = self.params.getPropertyEnumValue('Step Off Direction').lower()
        self.sitStandPlanner.applyParams()

    def onStart(self):
        self._syncProperties()
        print 'Egress Planner Ready'

    def onUpdateAffordance(self):
        if not self.platformPlanner.initializedFlag:
            self.platformPlanner.initialize()

        self.platformPlanner.updateAffordance()

    def onPlan(self,planType):
        self._syncProperties()
        self.sitStandPlanner.plan(planType)

    def onPlanTurn(self):
        self._syncProperties()
        self.platformPlanner.planTurn()

    def onArmsUp(self):
        self.platformPlanner.planArmsUp(self.stepOffDirection)

    def onPropertyChanged(self, propertySet, propertyName):
        self._syncProperties()

    def onPlanStepDown(self):
        self._syncProperties()
        if self.stepOffDirection == 'forwards':
            self.platformPlanner.planStepDownForwards()
        else:
            self.platformPlanner.planStepDown()

    def onPlanWeightShift(self):
        self._syncProperties()
        if self.stepOffDirection == 'forwards':
            self.platformPlanner.planWeightShiftForwards()
        else:
            self.platformPlanner.planWeightShift()

    def onPlanStepOff(self):
        self._syncProperties()
        if self.stepOffDirection == 'forwards':
            self.platformPlanner.planStepOffForwards()
        else:
            self.platformPlanner.planStepOff()

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

        def addManipTask(name, planFunc, userPrompt=False):

            folder = addFolder(name)
            addFunc(planFunc, name='plan')
            if not userPrompt:
                addTask(rt.CheckPlanInfo(name='check manip plan info'))
            else:
                addTask(rt.UserPromptTask(name='approve manip plan', message='Please approve manipulation plan.'))
            addFunc(pp.commitManipPlan, name='execute manip plan')
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'))

        pp = self.platformPlanner

        folder = addFolder('Prep')
        addFunc(self.onStart, 'start')
        addManipTask('arms up', self.onArmsUp, userPrompt=True)
        addFunc(pp.fitRunningBoardAtFeet, 'fit running board')
        addFunc(pp.spawnGroundAffordance, 'spawn ground affordance')
        addFunc(pp.requestRaycastTerrain, 'raycast terrain')
        addTask(rt.UserPromptTask(name="set walking params", message="Please set walking params to 'Polaris Platform'"))

        folder = addFolder('Step Down')
        addFunc(pp.spawnGroundAffordance, 'spawn ground affordance')
        addFunc(pp.requestRaycastTerrain, 'raycast terrain')
        addFunc(self.onPlanStepDown, 'plan step down')
        addTask(rt.UserPromptTask(name="approve/adjust footsteps", message="Please approve footsteps, modify if necessary"))
        addTask(rt.UserPromptTask(name="execute step down", message="Please execute walking plan"))
        addTask(rt.WaitForWalkExecution(name='wait for step down'))

        folder = addFolder('Step Off')
        addFunc(pp.spawnGroundAffordance, 'spawn ground affordance')
        addFunc(pp.requestRaycastTerrain, 'raycast terrain')
        addFunc(self.onPlanStepOff, 'plan step off')
        addTask(rt.UserPromptTask(name="approve/adjust footsteps", message="Please approve footsteps, modify if necessary"))
        addTask(rt.UserPromptTask(name="execute step off", message="Please execute walking plan"))

