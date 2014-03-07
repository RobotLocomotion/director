from time import time
import numpy as np
from ddapp import robotstate
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp import segmentation

from ddapp.plansequence import RobotPoseGUIWrapper

# Declare all valid inputs/ouputs here
# Using a dictionary to possibly give them type information later
# right now the keys of this dict are the valid list of arguments
argType = {}
argType['Affordance'] = None
argType['WalkPlan'] = None
argType['WalkTarget'] = None
argType['RobotPose'] = None
argType['JointPlan'] = None
argType['Hand'] = None
argType['Constraints'] = None


# Base Class, all actions must inherit from this
# All inheritors must:
#
# In constructor, need to pass the following
# -instance name
# -name of an instance to transition to on success
# -name of an instance to transition to on failure
# -a list of arguments passed in
# -a reference to an ActionSequence object used as a data container
#  to share data between Actions
#
# Inputs:
# -a list of arguments that must be present in the args dict
#
# Outputs:
# -dictonary to store the outputs of an action
# -starts empty (keys indicate valid outputs)
# -after running, will be populated with output data
#

class Action(object):

    inputs = []
    outputs = {}

    def __init__(self, name, success, fail, args, container):
        self.name = name
        self.container = container
        self.args = args     # storage for raw input arguments
        self.parsedArgs = {} # storage for inputs after being processed
        self.successAction = success
        self.failAction = fail

    def onEnter(self):
        print "default enter"

    def onUpdate(self):
        print "default update"

    def onExit(self):
        print "default exit"

    def transition(self, val):
        self.container.fsm.transition(val)

    def success(self):
        self.transition(self.successAction)

    def fail(self):
        self.transition(self.failAction)

    def hardFail(self):
        self.transition('fail')

    def argParseAndEnter(self):
        self.parseInputs()
        self.onEnter()

    def checkInputArgs(self, args, inputs):
        # Simple arg safety checking

        # Do we have the right number of args?
        if not len(self.inputs) == len(self.args.keys()):
            return False

        # Does every arg also exist in the input list?
        valid = True
        for arg in args:
            if arg not in inputs:
                valid = False
                print "Arg:", arg, "provided but it is not a valid input for:", self.name

        return valid

    def parseInputs(self):
        # args dictionary provides all the inputs, but these can be of two forms:
        # strings: indicating an specified input
        # or
        # references to other action classes: indicating intention to get the
        # input from that classes output at execution time
        #
        # Call this function at runtime to check consitency and grab
        # the output args from the specificed classes

        if not self.checkInputArgs(self.args, self.inputs):
            self.hardFail()
            return

        for arg in self.args.keys():
            if isinstance(self.args[arg], str):
                self.parsedArgs[arg] = self.args[arg]
            elif isinstance(self.args[arg], Action):
                self.parsedArgs[arg] = self.args[arg].outputs[arg]
            else:
                print "ERROR: provided input arg is neither a string nor a class reference"
                self.parsedArgs[arg] = None
                self.hardFail()


# Below are standard success and fail actions which simply terminate
# the sequence with a success or fail message

class Goal(Action):

    inputs = []
    outputs = {}

    def __init__(self, container):
        Action.__init__(self, 'goal', None, None, {}, container)

    def onEnter(self):
        print "HOORAY! Entered the GOAL state"

        # If in vizmode, play the animation
        if self.container.vizMode:
            self.container.play()

        # Stop the FSM now that we're at a terminus
        self.container.fsm.stop()

class Fail(Action):

    def __init__(self, container):
        Action.__init__(self, 'fail', None, None, {}, container)

    def onEnter(self):
        print "NOOO! Entered the FAIL state"

        # Stop the FSM now that we're at a terminus
        self.container.fsm.stop()

# Full action objects start below:

# Generic notes:
#        state transition when success
#        state transition when fail
#        function to evaluate
#        on enter function -kick off eval function
#        on update function -wait for success fail
#        on exit function -teardown, if necessary

class ChangeMode(Action):

    inputs = ['NewMode']
    outputs = {}

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)

        self.timeOut = 10.0
        self.enterTime = 0.0

    def onEnter(self):

        # Logic for viz-mode:
        if self.container.vizMode:
            self.currentBehavior = self.parsedArgs['NewMode']

        # Logic for execute mode:
        else:
            self.enterTime = time()
            self.behaviorTarget = self.parsedArgs['NewMode']
            self.container.atlasDriver.sendBehaviorCommand(self.behaviorTarget)

    def onUpdate(self):

        # Logic for viz-mide:
        if self.container.vizMode:
            self.success()

        # Error checking on bad mode types
        else:
            if self.behaviorTarget not in self.container.atlasDriver.getBehaviorMap().values():
                print 'ERROR: desired transition does not exist, failing'
                self.fail()
            else:
                self.currentBehavior = self.container.atlasDriver.getCurrentBehaviorName()

                if self.currentBehavior == self.behaviorTarget:
                    self.success()
                else:
                    if time() > self.enterTime + self.timeOut:
                        print 'ERROR: mode change timed out'
                        self.fail()
                    else:
                        return

    def onExit(self):
        return


class WalkPlan(Action):

    inputs = ['WalkTarget']
    outputs = {'WalkPlan' : None}

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.walkPlanResponse = None

    def onEnter(self):
        graspStanceFrame = self.container.om.findObjectByName(self.parsedArgs['WalkTarget'])
        self.walkPlanResponse = self.container.footstepPlanner.sendFootstepPlanRequest(graspStanceFrame.transform, waitForResponse=True, waitTimeout=0)

    def onUpdate(self):

        response = self.walkPlanResponse.waitForResponse(timeout = 0)

        if response:
            if response.num_steps == 0:
                print 'walk plan failed'
                self.fail()
            else:
                print 'walk plan successful'
                self.container.footstepPlan = response
                self.outputs['WalkPlan'] = response
                self.success()

    def onExit(self):
        self.walkPlanResponse.finish()
        self.walkPlanResponse = None

class Walk(Action):

    inputs = ['WalkPlan']
    outputs = {}

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.walkAnimationResponse = None
        self.startTime = 0.0

    def onEnter(self):

        # Logic for viz-mode:
        if self.container.vizMode:
            # Use the footstep plan to calculate a walking plan, for animation purposes only
            self.walkAnimationResponse = self.container.footstepPlanner.sendWalkingPlanRequest(self.container.footstepPlan, waitForResponse=True, waitTimeout=0)

        # Logic for execute mode:
        else:
            self.container.footstepPlanner.commitFootstepPlan(self.container.footstepPlan)
            self.walkStart = False

    def onUpdate(self):

        # Logic for viz-mode:
        if self.container.vizMode:
            response = self.walkAnimationResponse.waitForResponse(timeout = 0)
            if response:
                # We got a successful response from the walk planner, cache it and update the plan state
                self.container.walkAnimation = response
                self.container.planPose = robotstate.convertStateMessageToDrakePose(self.container.walkAnimation.plan[-1])
                self.container.vizModeAnimation.append(self.container.walkAnimation)
                self.success()

        # Logic for execute mode:
        else:
            if self.container.atlasDriver.getCurrentBehaviorName() == 'step':
                self.walkStart = True
            if self.walkStart and self.container.atlasDriver.getCurrentBehaviorName() == 'stand':
                self.success()

    def onExit(self):

        # Logic for viz-mode:
        if self.container.vizMode:
            self.walkAnimationResponse.finish()
            self.walkAnimationResponse = None

        # Logic for execute-mode:
        else:
            self.walkStart = None

def computeGroundFrame(robotModel):
    '''
    Given a robol model, returns a vtkTransform at a position between
    the feet, on the ground, with z-axis up and x-axis aligned with the
    robot pelvis x-axis.
    '''
    t1 = robotModel.getLinkFrame('l_foot')
    t2 = robotModel.getLinkFrame('r_foot')
    pelvisT = robotModel.getLinkFrame('pelvis')

    xaxis = [1.0, 0.0, 0.0]
    pelvisT.TransformVector(xaxis, xaxis)
    xaxis = np.array(xaxis)
    zaxis = np.array([0.0, 0.0, 1.0])
    yaxis = np.cross(zaxis, xaxis)
    yaxis /= np.linalg.norm(yaxis)
    xaxis = np.cross(yaxis, zaxis)

    stancePosition = (np.array(t2.GetPosition()) + np.array(t1.GetPosition())) / 2.0

    footHeight = 0.0811

    t = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
    t.PostMultiply()
    t.Translate(stancePosition)
    t.Translate([0.0, 0.0, -footHeight])

    return t

class PoseSearch(Action):

    inputs = ['Affordance', 'Hand']
    outputs = {'TargetFrame' : None, 'WalkTarget' : None, 'Hand' : None}

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)

    def onEnter(self):

        self.targetAffordance = self.container.om.findObjectByName(self.parsedArgs['Affordance'])
        self.targetAffordanceFrame = self.container.om.findObjectByName(self.parsedArgs['Affordance'] + ' frame')

        self.outputs['TargetFrame'] = self.parsedArgs['Affordance'] + ' grasp frame'
        self.outputs['WalkTarget'] = self.parsedArgs['Affordance'] + ' stance frame'
        self.outputs['Hand'] = self.parsedArgs['Hand']

        #Calculate where to place the hand (hard coded offsets based on drill, just for testing)
        # for left_base_link
        position = [-0.12, 0.0, 0.028]
        rpy = [0, 90, 0]
        grasp = transformUtils.frameFromPositionAndRPY(position, rpy)
        grasp.Concatenate(self.targetAffordanceFrame.transform)
        self.graspFrame = vis.updateFrame(grasp, self.outputs['TargetFrame'], parent=self.targetAffordance, visible=True, scale=0.25)

        #Calculate where to place the walking target (hard coded offsets based on drill, just for testing)
        graspFrame = self.graspFrame.transform
        groundFrame = computeGroundFrame(self.container.robotModel)
        groundHeight = groundFrame.GetPosition()[2]

        graspPosition = np.array(graspFrame.GetPosition())
        graspYAxis = [0.0, 1.0, 0.0]
        graspZAxis = [0.0, 0.0, 1.0]
        graspFrame.TransformVector(graspYAxis, graspYAxis)
        graspFrame.TransformVector(graspZAxis, graspZAxis)

        xaxis = graspZAxis
        zaxis = [0, 0, 1]
        yaxis = np.cross(zaxis, xaxis)
        yaxis /= np.linalg.norm(yaxis)
        xaxis = np.cross(yaxis, zaxis)

        graspGroundFrame = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
        graspGroundFrame.PostMultiply()
        graspGroundFrame.Translate(graspPosition[0], graspPosition[1], groundHeight)

        if self.parsedArgs['Hand'] == 'right':
            position = [-0.57, 0.4, 0.0]
        else:
            position = [-0.57, -0.4, 0.0]
        rpy = [0, 0, 0]

        stance = transformUtils.frameFromPositionAndRPY(position, rpy)
        stance.Concatenate(graspGroundFrame)

        self.stanceFrame = vis.updateFrame(stance, self.outputs['WalkTarget'], parent=self.targetAffordance, visible=True, scale=0.25)

        return

    def onUpdate(self):
        self.success()

    def onExit(self):
        return

class ReachPlan(Action):

    inputs = ['TargetFrame', 'Hand', 'Constraints']
    outputs = {'JointPlan' : None}

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.manipPlanResponse = None

    def onEnter(self):
        linkMap = { 'left' : 'l_hand', 'right': 'r_hand'}

        linkName = linkMap[self.parsedArgs['Hand']]
        graspFrame = self.container.om.findObjectByName(self.parsedArgs['TargetFrame'])

        if self.container.planPose == None:
            self.container.planPose = self.container.sensorJointController.getPose('EST_ROBOT_STATE')
        self.manipPlanResponse = self.container.manipPlanner.sendEndEffectorGoal(self.container.planPose, linkName, graspFrame.transform, waitForResponse=True, waitTimeout=0)

    def onUpdate(self):
        response = self.manipPlanResponse.waitForResponse(timeout = 0)

        if response:
            if response.plan_info[-1] > 10:
                print "PLANNER REPORTS ERROR!"
                self.fail()
            else:
                print "Planner reports success!"
                self.success()

                # Viz Mode Logic:
                if self.container.vizMode:
                    self.container.vizModeAnimation.append(response)
                    self.container.planPose = robotstate.convertStateMessageToDrakePose(self.container.vizModeAnimation[-1].plan[-1])
                    self.outputs['JointPlan'] = robotstate.convertStateMessageToDrakePose(self.container.vizModeAnimation[-1].plan[-1])

    def onExit(self):
        self.manipPlanResponse.finish()
        self.manipPlanResponse = None


class Reach(Action):

    inputs = ['JointPlan']
    outputs = {}

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.counter = 10

    def onEnter(self):
        return

    def onUpdate(self):

        # Viz Mode Logic
        if self.container.vizMode:
            # Update the current state to be the end of the last step for future planning
            self.container.planPose = robotstate.convertStateMessageToDrakePose(self.container.vizModeAnimation[-1].plan[-1])
            self.success()

        # Execute Mode Logic
        else:
            self.counter -= 1
            if self.counter == 0:
                self.success()

    def onExit(self):
        self.counter = 10


class JointMovePlan(Action):

    inputs = ['PoseName', 'Group', 'Hand']
    outputs = {'JointPlan' : None}

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.jointMovePlanResponse = None

    def onEnter(self):
        # Send a planner request
        goalPoseJoints = RobotPoseGUIWrapper.getPose(self.parsedArgs['Group'], self.parsedArgs['PoseName'], self.parsedArgs['Hand'])
        if self.container.planPose == None:
            self.container.planPose = self.container.sensorJointController.getPose('EST_ROBOT_STATE')
        self.jointMovePlanResponse = self.container.manipPlanner.sendPoseGoal(self.container.planPose, goalPoseJoints, waitForResponse=True, waitTimeout=0)

    def onUpdate(self):
        # Wait for planner response
        response = self.jointMovePlanResponse.waitForResponse(timeout = 0)
        if response:
            self.outputs['JointPlan'] = response
            self.container.jointMovePlan = response
            self.container.vizModeAnimation.append(response)
            self.success()

    def onExit(self):
        return


class JointMove(Action):

    inputs = ['JointPlan']
    outputs = {'RobotPose' : None}

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.startTime = 0.0

    def onEnter(self):
        if self.container.vizMode:
            # Viz Mode Logic
            # Update the plan state to match the end of the animation
            self.container.planPose = robotstate.convertStateMessageToDrakePose(self.container.vizModeAnimation[-1].plan[-1])
        else:
            # Execute Mode Logic
            # Send the command to the robot
            self.container.manipPlanner.commitManipPlan(self.parsedArgs['JointPlan'])
            self.startTime = time()
            return

    def onUpdate(self):
        if self.container.vizMode:
            # Viz Mode Logic
            # do nothing
            self.success()
        else:
            # Execute Mode Logic
            # Wait for success
            if time() > self.startTime + 10.0:
                self.success()

                # Need logic here to see if we reached our target to within some tolerance
                # success or fail based on that
                # is there a way to see if the robot is running?

            return

    def onExit(self):
        return


class Grip(Action):

    inputs = ['Hand']
    outputs = {}

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.gripTime = 0.0
        self.gripWait = 2.0

    def onEnter(self):
        if self.container.vizMode:
            # Viz Mode Logic
            # Do nothing
            return
        else:
            # Execute Mode Logic
            self.container.handDriver.sendClose(100)
            self.gripTime = time()

    def onUpdate(self):
        if self.container.vizMode:
            # Viz Mode Logic
            self.success()
        else:
            # Execute Mode Logic
            # Wait for grip time, then send another command just to ensure
            # proper regrasp
            if time() > self.gripTime + self.gripWait:
                self.container.handDriver.sendClose(100)
                self.success()

    def onExit(self):
        return

class Release(Action):

    inputs = ['Hand']
    outputs = {}

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.gripTime = 0.0
        self.gripWait = 1.0

    def onEnter(self):
        if self.container.vizMode:
            # Viz Mode Logic
            # Do nothing
            return
        else:
            # Execute Mode Logic
            self.container.handDriver.sendOpen()
            self.gripTime = time()

    def onUpdate(self):
        if self.container.vizMode:
            # Viz Mode Logic
            self.success()
        else:
            # Execute Mode Logic
            # Wait for grip time, then send another command just to ensure
            # proper regrasp
            if time() > self.gripTime + self.gripWait:
                self.container.handDriver.sendOpen()
                self.success()

    def onExit(self):
        return

class Fit(Action):

    inputs = ['Affordance']
    outputs = {}

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)

    def onEnter(self):
        return

    def onUpdate(self):
        pd = self.container.om.findObjectByName('Multisense').model.revPolyData
        self.container.om.removeFromObjectModel(self.container.om.findObjectByName('debug'))
        segmentation.findAndFitDrillBarrel(pd, self.container.robotModel.getLinkFrame('utorso'))

        self.success()

    def onExit(self):
        return


class WaitForScan(Action):

    inputs = []
    outputs = {}

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)

    def onEnter(self):

        # Viz Mode Logic
        if self.container.vizMode:
            return

        # Execute Mode Logic
        else:
            self.currentRevolution = self.container.multisenseDriver.displayedRevolution
            self.desiredRevolution = self.currentRevolution + 2

    def onUpdate(self):

        # Viz Mode Logic
        if self.container.vizMode:
            self.success()

        # Execute Mode Logic
        else:
            if self.container.multisenseDriver.displayedRevolution >= self.desiredRevolution:
                self.success()

    def onExit(self):
        self.currentRevolution = None
        self.desiredRevolution = None

