from time import time
from copy import deepcopy, copy
from math import sqrt
import numpy as np
from time import sleep

from ddapp import robotstate
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp import lcmUtils
from ddapp import segmentation

from drc import robot_state_t, actionman_status_t, actionman_resume_t
from bot_core import rigid_transform_t
from bot_frames import update_t
from drc import pfgrasp_command_t

from vtk import vtkTransform
import botpy
import takktile

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
argType['BodyPose'] = None

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

    def __init__(self, name, success, fail, args, container):
        self.name = name
        self.container = container
        self.args = args     # storage for raw input arguments
        self.parsedArgs = {} # storage for inputs after being processed
        self.successAction = success
        self.failAction = fail
        self.animations = []
        self.inputState = None
        self.outputState = None

    def reset(self):
        self.outputs = {}
        self.animations = []
        self.inputState = None
        self.outputState = None

    def onEnter(self):
        print "default enter"

    def onUpdate(self):
        print "default update"

    def onExit(self):
        print "default exit"

    def transition(self, val):
        self.container.fsm.transition(val)

    def success(self):
        if self.container.pauseBetween:
            # Skip the pauses if it's a transition to goal
            if self.successAction is 'goal':
                self.transition(self.successAction)

            # if success called when aleady paused, that means continue
            if self.name == 'pause':
                self.transition(self.successAction)

            # Default case is transition to pause and store success transition
            else:
                self.container.actionObjects['pause'].successAction = self.successAction
                self.transition('pause')
        else:
            self.transition(self.successAction)
            self.container.executionList.append([self.name, 'success', len(self.animations)])

    def fail(self):
        self.transition(self.failAction)
        self.container.executionList.append([self.name, 'fail', len(self.animations)])

    def hardFail(self):
        self.transition('fail')
        self.container.executionList.append([self.name, 'abort', len(self.animations)])

    def argParseAndEnter(self):
        if self.container.previousAction != None and self.container.previousAction.outputState != None:
            self.inputState = self.container.previousAction.outputState
        else:
            print "ERROR: no previous state found, using estimated current state"
            self.inputState = self.container.sensorJointController.getPose('EST_ROBOT_STATE')
        self.parseInputs()
        self.onEnter()

    def storeEndStateAndExit(self):
        self.container.previousAction = self
        self.onExit()

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
                self.parsedArgs[arg] = deepcopy(self.args[arg].outputs[arg])
            else:
                print "ERROR: provided input arg is neither a string nor a class reference"
                self.parsedArgs[arg] = None
                self.hardFail()

    def animate(self, index):
        if self.animations != []:
            self.container.playbackFunction([self.animations[index]])
        else:
            print "ERROR: This action does not have an animation"

# Below are standard success and fail actions which simply terminate
# the sequence with a success or fail message

class Goal(Action):

    inputs = []

    def __init__(self, container):
        Action.__init__(self, 'goal', None, None, {}, container)
        self.outputs = {}

    def onEnter(self):
        print "HOORAY! Entered the GOAL state"

        # If in vizmode, play the animation
        if self.container.vizMode:
            self.container.play()

        # Stop the FSM now that we're at a terminus
        self.container.fsm.stop()


class Fail(Action):

    inputs = []

    def __init__(self, container):
        Action.__init__(self, 'fail', None, None, {}, container)
        self.outputs = {}

    def onEnter(self):
        print "NOOO! Entered the FAIL state"

        # Stop the FSM now that we're at a terminus
        self.container.fsm.stop()


# Below is the special Pause action which does nothing and waits
# for a special LCM message to resume

class Pause(Action):

    inputs = []

    def __init__(self, container):
        Action.__init__(self, 'pause', None, None, {}, container)
        self.outputs = {}
        self.resumeChannel = 'ACTION_MANAGER_RESUME'
        self.resumeReceived = False

    def resumeCallback(self, data):
        self.resumeReceived = True

    def onEnter(self):
        print "Entering the pause state.  Waiting for continue message via LCM topic."
        lcmUtils.captureMessageCallback(self.resumeChannel, actionman_resume_t, self.resumeCallback)

    def onUpdate(self):
        if self.resumeReceived:
            self.success()

    def onExit(self):
        # Cleanup
        self.resumeReceived = False

        # This is a dummy action, no robot motion
        self.outputState = deepcopy(self.inputState)

# Full action objects start below:
#
# Generic notes:
#        state transition when success
#        state transition when fail
#        function to evaluate
#        on enter function -kick off eval function
#        on update function -wait for success fail
#        on exit function -teardown, if necessary

class ChangeMode(Action):

    inputs = ['NewMode']

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.outputs = {}
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

        # Logic for execute mode:
        else:
            # Error checking on bad mode types
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
        # This is a state change action, no robot motion
        self.outputState = deepcopy(self.inputState)


class WalkPlan(Action):

    inputs = ['WalkTarget']

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.outputs = {'WalkPlan' : None}
        self.walkPlanResponse = None

    def onEnter(self):
        graspStanceFrame = self.container.om.findObjectByName(self.parsedArgs['WalkTarget'])
        request = self.container.footstepPlanner.constructFootstepPlanRequest(self.inputState, graspStanceFrame.transform)
        self.walkPlanResponse = self.container.footstepPlanner.sendFootstepPlanRequest(request, waitForResponse=True, waitTimeout=0)

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
        # Cleanup planner objects
        self.walkPlanResponse.finish()
        self.walkPlanResponse = None

        # This is a planning action, no robot motion
        self.outputState = deepcopy(self.inputState)


class Walk(Action):

    inputs = ['WalkPlan']

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.outputs = {}
        self.walkAnimationResponse = None

    def onEnter(self):

        # Logic for viz-mode:
        if self.container.vizMode:
            # Use the footstep plan to calculate a walking plan, for animation purposes only
            self.walkAnimationResponse = self.container.footstepPlanner.sendWalkingPlanRequest(self.container.footstepPlan, self.inputState, waitForResponse=True, waitTimeout=0)

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
                self.animations.append(response)
                self.container.vizModeAnimation.append(response)
                self.success()

        # Logic for execute mode:
        else:
            if self.container.atlasDriver.getCurrentBehaviorName() == 'step':
                self.walkStart = True
            if self.walkStart and self.container.atlasDriver.getCurrentBehaviorName() == 'stand':
                self.success()

    def onExit(self):
        # Cleanup planner code
        # Logic for viz-mode:
        if self.container.vizMode:
            self.walkAnimationResponse.finish()
            self.walkAnimationResponse = None

        # Logic for execute-mode:
        else:
            self.walkStart = None

        # This is a motion action, output should be new robot state, based on mode selection
        if self.container.vizMode:
            # Viz Mode Logic
            # (simulating a perfect execution, output state is the end of animation state)
            self.outputState = robotstate.convertStateMessageToDrakePose(self.animations[-1].plan[-1])
        else:
            # Execute Mode Logic
            # (the move is complete, so output state is current estimated robot state)
            self.outputState = self.container.sensorJointController.getPose('EST_ROBOT_STATE')
        return


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

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.outputs = {'TargetFrame' : None, 'WalkTarget' : None, 'Hand' : None}

    def onEnter(self):

        self.targetAffordance = self.container.om.findObjectByName(self.parsedArgs['Affordance'])
        self.targetAffordanceFrame = self.container.om.findObjectByName(self.parsedArgs['Affordance'] + ' frame')

        self.outputs['TargetFrame'] = self.parsedArgs['Affordance'] + ' grasp frame'
        self.outputs['WalkTarget'] = self.parsedArgs['Affordance'] + ' stance frame'
        self.outputs['Hand'] = self.parsedArgs['Hand']

        #Calculate where to place the hand (hard coded offsets based on drill, just for testing)
        # for left_base_link
        position = [-0.15, 0.0, 0.028]
        if self.parsedArgs['Hand'] == 'right':
            rpy = [0, -90, -90]
        else:
            rpy = [0, 90, -90]

        grasp = transformUtils.frameFromPositionAndRPY(position, rpy)
        grasp.Concatenate(self.targetAffordanceFrame.transform)
        self.graspFrame = vis.updateFrame(grasp, self.outputs['TargetFrame'], parent=self.targetAffordance, visible=True, scale=0.25)

        #Calculate where to place the walking target (hard coded offsets based on drill, just for testing)
        graspFrame = self.graspFrame.transform
        groundFrame = computeGroundFrame(self.container.robotModel)
        groundHeight = groundFrame.GetPosition()[2]

        graspPosition = np.array(graspFrame.GetPosition())
        graspYAxis = [0.0, 1.0, 0.0]
        graspXAxis = [1.0, 0.0, 0.0]
        graspFrame.TransformVector(graspYAxis, graspYAxis)
        graspFrame.TransformVector(graspXAxis, graspXAxis)

        xaxis = graspYAxis
        zaxis = [0, 0, 1]
        yaxis = np.cross(zaxis, xaxis)
        yaxis /= np.linalg.norm(yaxis)
        graspGroundFrame = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)
        graspGroundFrame.PostMultiply()
        graspGroundFrame.Translate(graspPosition[0], graspPosition[1], groundHeight)

        if self.parsedArgs['Hand'] == 'right':
            position = [-0.69, 0.40, 0.0]
        else:
            position = [-0.69, -0.40, 0.0]
        rpy = [0, 0, 0]

        stance = transformUtils.frameFromPositionAndRPY(position, rpy)
        stance.Concatenate(graspGroundFrame)

        self.stanceFrame = vis.updateFrame(stance, self.outputs['WalkTarget'], parent=self.targetAffordance, visible=True, scale=0.25)

        return

    def onUpdate(self):
        self.success()

    def onExit(self):
        # This is a planning action, no robot motion
        self.outputState = deepcopy(self.inputState)




class PFGraspCommand(Action):

    inputs = ['CommandType', 'Channel']

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.outputs = {'MoveCommand' : None}
        self.manipPlan = None
        self.messageReceived = False
        self.message = None

    def setMessageReceived(self, data):
        self.message = data
        self.messageReceived = True

    def onEnter(self):
        # Create a listener to wait for the response
        if self.parsedArgs['CommandType'] == '3':
            self.messageReceived = False
            self.message = None
            lcmUtils.captureMessageCallback(self.parsedArgs['Channel'], update_t, self.setMessageReceived)
            sleep(0.1)

        # Send out the message to start
        message = pfgrasp_command_t.pfgrasp_command_t()
        message.command = int(self.parsedArgs['CommandType'])
        lcmUtils.publish('PFGRASP_CMD', message)

    def onUpdate(self):
        if not self.parsedArgs['CommandType'] == '3':
            self.success()

        else:
            if self.messageReceived:
                handToWorld_XYZ = self.container.ikPlanner.getLinkFrameAtPose('l_hand_face', self.inputState).GetPosition()
                dist = sqrt( (handToWorld_XYZ[0]-self.message.trans[0])**2 +
                             (handToWorld_XYZ[1]-self.message.trans[1])**2 +
                             (handToWorld_XYZ[2]-self.message.trans[2])**2)
                print "DISTANCE TO GO:", dist
                if dist > 0.015:
                    self.outputs['MoveCommand'] = [copy(self.message.trans), copy(self.message.quat), copy(self.message.relative_to)]
                    self.fail()
                else:
                    self.success()

    def onExit(self):
        # Cleanup
        self.message = None
        self.messageReceived = False

       # This is a planning action, no robot motion
        self.outputState = deepcopy(self.inputState)


class CameraDeltaPlan(Action):

    inputs = ['TargetFrame', 'Hand', 'Style', 'MoveCommand']

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.outputs = {'JointPlan' : None}
        self.manipPlan = None

    def onEnter(self):
        self.moveCommand = self.parsedArgs['MoveCommand']

    def onUpdate(self):
        linkMap = { 'left' : 'l_hand_face', 'right': 'r_hand_face'}
        linkName = linkMap[self.parsedArgs['Hand']]

        # local means the published topic is a full transform
        if self.moveCommand[2] == 'local':
            cameraToWorld = transformUtils.frameFromPositionAndRPY([self.moveCommand[0][0],
                                                                    self.moveCommand[0][1],
                                                                    self.moveCommand[0][2]],
                                                                   np.degrees(botpy.quat_to_roll_pitch_yaw(self.moveCommand[1])))

            #peter started using bot-frames to calculate hand position so this is no longer needed
            #handToCamera = transformUtils.frameFromPositionAndRPY([0.0, 0.096, 0.02], np.degrees([1.57079, 0, 1.57079]))

            handToWorld = vtkTransform()
            handToWorld.PostMultiply()
            #handToWorld.Concatenate(handToCamera)
            handToWorld.Concatenate(cameraToWorld)

            handFrame = self.container.om.findObjectByName('cameras')
            goalFrame = vis.updateFrame(handToWorld, 'CameraAdjustFrame', parent=handFrame, visible=True, scale=0.25)
            goalFrame2 = vis.updateFrame(cameraToWorld, 'PeterFrame', parent=handFrame, visible=True, scale=0.25)

            #call the planner
            self.manipPlan = self.container.ikPlanner.planEndEffectorGoal(self.inputState, self.parsedArgs['Hand'], goalFrame, planTraj=True)

        else:
            if self.container.vizMode:
                handToWorld = self.container.ikPlanner.getLinkFrameAtPose(linkName, self.inputState)
            else:
                handToWorld = self.container.robotModel.getLinkFrame(linkName)

            delta = transformUtils.frameFromPositionAndRPY([self.moveCommand[0][0],
                                                            self.moveCommand[0][1],
                                                            self.moveCommand[0][2]],
                                                            [0, 0, 0])

            if self.parsedArgs['Style'] == 'Local':
                goalTransform = vtkTransform()
                goalTransform.PostMultiply()
                goalTransform.Concatenate(delta)
                goalTransform.Concatenate(handToWorld)
            else:
                goalTransform = vtkTransform()
                goalTransform.PostMultiply()
                goalTransform.Concatenate(handToWorld)
                goalTransform.Concatenate(delta)

            handFrame = self.container.om.findObjectByName(self.parsedArgs['TargetFrame'])
            goalFrame = vis.updateFrame(goalTransform, 'CameraAdjustFrame', parent=handFrame, visible=True, scale=0.25)

            #call the planner
            self.manipPlan = self.container.ikPlanner.planEndEffectorGoal(self.inputState, self.parsedArgs['Hand'], goalFrame, planTraj=True)

        if self.manipPlan.plan_info[-1] > 10:
            print "PLANNER REPORTS ERROR!"

            # Plan failed, save it in the animation list, but don't update output data
            self.animations.append(self.manipPlan)

            self.fail()
        else:
            print "Planner reports success!"

            # Plan was successful, save it to be animated and update output data
            self.animations.append(self.manipPlan)
            self.outputs['JointPlan'] = deepcopy(self.manipPlan)

            self.success()

    def onExit(self):
        # Planner Cleanup
        self.manipPlan = None

        # This is a planning action, no robot motion
        self.outputState = deepcopy(self.inputState)


class DeltaReachPlan(Action):

    inputs = ['TargetFrame', 'Hand', 'Style', 'Direction', 'Amount']

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.outputs = {'JointPlan' : None}
        self.manipPlan = None

    def onEnter(self):
        linkMap = { 'left' : 'l_hand_face', 'right': 'r_hand_face'}
        linkName = linkMap[self.parsedArgs['Hand']]

        if self.container.vizMode:
            handToWorld = self.container.ikPlanner.getLinkFrameAtPose(linkName, self.inputState)
        else:
            handToWorld = self.container.robotModel.getLinkFrame(linkName)

        if self.parsedArgs['Direction'] == 'X':
            delta = transformUtils.frameFromPositionAndRPY([float(self.parsedArgs['Amount']),0,0],[0,0,0])
        elif self.parsedArgs['Direction'] == 'Y':
            delta = transformUtils.frameFromPositionAndRPY([0,float(self.parsedArgs['Amount']),0],[0,0,0])
        else:
            delta = transformUtils.frameFromPositionAndRPY([0,0,float(self.parsedArgs['Amount'])],[0,0,0])

        if self.parsedArgs['Style'] == 'Local':
            goalTransform = vtkTransform()
            goalTransform.PostMultiply()
            goalTransform.Concatenate(delta)
            goalTransform.Concatenate(handToWorld)
        else:
            goalTransform = vtkTransform()
            goalTransform.PostMultiply()
            goalTransform.Concatenate(handToWorld)
            goalTransform.Concatenate(delta)

        handFrame = self.container.om.findObjectByName('left robotiq')
        goalFrame = vis.updateFrame(goalTransform, self.parsedArgs['TargetFrame'] + " " + self.name, parent=handFrame, visible=True, scale=0.25)

        self.manipPlan = self.container.ikPlanner.planEndEffectorGoal(self.inputState, self.parsedArgs['Hand'], goalFrame, planTraj=True)

    def onUpdate(self):

        if self.manipPlan.plan_info[-1] > 10:
            print "PLANNER REPORTS ERROR!"

            # Plan failed, save it in the animation list, but don't update output data
            self.animations.append(self.manipPlan)

            self.fail()
        else:
            print "Planner reports success!"

            # Plan was successful, save it to be animated and update output data
            self.animations.append(self.manipPlan)
            self.outputs['JointPlan'] = deepcopy(self.manipPlan)

            self.success()

    def onExit(self):
        # Planner Cleanup
        self.manipPlan = None

        # This is a planning action, no robot motion
        self.outputState = deepcopy(self.inputState)


class ReachPlan(Action):

    inputs = ['TargetFrame', 'Hand', 'Constraints']

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.outputs = {'JointPlan' : None, 'BodyPose' : None}
        self.manipPlan = None

    def onEnter(self):

        graspFrame = self.container.om.findObjectByName(self.parsedArgs['TargetFrame'])
        self.manipPlan = self.container.ikPlanner.planEndEffectorGoal(self.inputState, self.parsedArgs['Hand'], graspFrame, planTraj=True)

    def onUpdate(self):

        if self.manipPlan.plan_info[-1] > 10:
            print "PLANNER REPORTS ERROR!"

            # Plan failed, save it in the animation list, but don't update output data
            self.animations.append(self.manipPlan)
            self.outputs['BodyPose'] = deepcopy(self.inputState)

            self.fail()
        else:
            print "Planner reports success!"

            # Plan was successful, save it to be animated and update output data
            self.animations.append(self.manipPlan)
            self.outputs['JointPlan'] = deepcopy(self.manipPlan)
            self.outputs['BodyPose'] = deepcopy(self.manipPlan.plan[-1])

            self.success()

    def onExit(self):
        # Planner Cleanup
        self.manipPlan = None

        # This is a planning action, no robot motion
        self.outputState = deepcopy(self.inputState)

class JointMovePlan(Action):

    inputs = ['PoseName', 'Group', 'Hand', 'BodyPose']

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.outputs = {'JointPlan' : None}
        self.jointMovePlan = None

    def onEnter(self):
        # Send a planner request

        if not isinstance(self.parsedArgs['BodyPose'], robot_state_t):
            endPose = self.container.ikPlanner.getMergedPostureFromDatabase(self.inputState, self.parsedArgs['Group'], self.parsedArgs['PoseName'], self.parsedArgs['Hand'])
            self.jointMovePlan = self.container.ikPlanner.computePostureGoal(self.inputState, endPose)
        else:
            desiredBodyPose = robotstate.convertStateMessageToDrakePose(self.parsedArgs['BodyPose'])
            endPose = self.container.ikPlanner.getMergedPostureFromDatabase(desiredBodyPose, self.parsedArgs['Group'], self.parsedArgs['PoseName'], self.parsedArgs['Hand'])
            self.jointMovePlan = self.container.ikPlanner.computePostureGoal(self.inputState, endPose)

    def onUpdate(self):
        self.animations.append(self.jointMovePlan)
        self.outputs['JointPlan'] = self.jointMovePlan
        self.success()

    def onExit(self):
        # Cleanup
        self.jointMovePlan = None
        # This is a planning action, no robot motion
        self.outputState = deepcopy(self.inputState)

class JointMove(Action):

    inputs = ['JointPlan']

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.outputs = {'RobotPose' : None}
        self.startTime = 0.0

    def onEnter(self):
        # Create an animation based on the incoming plan
        self.animations.append(self.parsedArgs['JointPlan'])
        self.container.vizModeAnimation.append(self.parsedArgs['JointPlan'])

        if self.container.vizMode:
            # Viz Mode Logic
            return
        else:
            # Execute Mode Logic
            # Send the command to the robot
            self.container.manipPlanner.commitManipPlan(self.parsedArgs['JointPlan'])
            self.startTime = time()
            return

    def onUpdate(self):
        #Perform the motion
        if self.container.vizMode:
            # Viz Mode Logic
            # do nothing
            self.success()
        else:
            # Execute Mode Logic
            # Wait for success
            print "WAITING FOR MOTION to complete"
            if time() > self.startTime + 11.0:

                # Need logic here to see if we reached our target to within some tolerance
                # success or fail based on that
                # is there a way to see if the robot is running?
                self.success()

    def onExit(self):
        # This is a motion action, output should be new robot state, based on mode selection
        if self.container.vizMode:
            # Viz Mode Logic
            # (simulating a perfect execution, output state is the end of animation state)
            self.outputState = robotstate.convertStateMessageToDrakePose(self.animations[-1].plan[-1])
            self.outputs = {'RobotPose' : robotstate.convertStateMessageToDrakePose(self.animations[-1].plan[-1])}
        else:
            # Execute Mode Logic
            # (the move is complete, so output state is current estimated robot state)
            self.outputState = self.container.sensorJointController.getPose('EST_ROBOT_STATE')
            self.outputs = {'RobotPose' : self.container.sensorJointController.getPose('EST_ROBOT_STATE')}

class JointMoveGuarded(Action):

    inputs = ['JointPlan', 'Hand']

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.outputs = {'RobotPose' : None}
        self.startTime = 0.0
        self.contact = False

    def setContactTrue(self, data):
        self.contact = True

    def onEnter(self):
        #Create an animation based on the incoming plan
        self.animations.append(self.parsedArgs['JointPlan'])
        self.container.vizModeAnimation.append(self.parsedArgs['JointPlan'])
        self.contact = False

        if self.container.vizMode:
            # Viz Mode Logic
            return
        else:
            # Execute Mode Logic

            # Set up a LCM monitor on the contact detector
            channel = 'TAKKTILE_CONTACT_' + self.parsedArgs['Hand'].upper()
            lcmUtils.captureMessageCallback(channel, takktile.contact_t, self.setContactTrue)

            # Send the command to the robot
            self.container.manipPlanner.commitManipPlan(self.parsedArgs['JointPlan'])
            self.startTime = time()
            return

    def onUpdate(self):
        #Perform the motion
        if self.container.vizMode:
            # Viz Mode Logic
            # do nothing
            self.success()
        else:
            # Execute Mode Logic
            # Wait for success
            print "WAITING FOR MOTION to complete"

            if self.contact:
                print "Pausing motion because of contact!"
                self.container.manipPlanner.sendPlanPause()
                self.success()

            if time() > self.startTime + 11.0:
                print "Ending motion because of timeout, no contact found"

                # Need logic here to see if we reached our target to within some tolerance
                # success or fail based on that
                # is there a way to see if the robot is running?
                self.success()

    def onExit(self):
        # This is a motion action, output should be new robot state, based on mode selection
        if self.container.vizMode:
            # Viz Mode Logic
            # (simulating a perfect execution, output state is the end of animation state)
            self.outputState = robotstate.convertStateMessageToDrakePose(self.animations[-1].plan[-1])
            self.outputs = {'RobotPose' : robotstate.convertStateMessageToDrakePose(self.animations[-1].plan[-1])}
        else:
            # Execute Mode Logic
            # (the move is complete, so output state is current estimated robot state)
            self.outputState = self.container.sensorJointController.getPose('EST_ROBOT_STATE')
            self.outputs = {'RobotPose' : self.container.sensorJointController.getPose('EST_ROBOT_STATE')}

class Grip(Action):

    inputs = ['Hand']

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.outputs = {}
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
        # This action does not move the robot, set the output state accordingly
        # TODO: make this animate the fingers in the future
        self.outputState = deepcopy(self.inputState)


class Release(Action):

    inputs = ['Hand']

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.outputs = {}
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
        # This action does not move the robot, set the output state accordingly
        # TODO: make this animate the fingers in the future
        self.outputState = deepcopy(self.inputState)


class Fit(Action):

    inputs = ['Affordance']

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.outputs = {}

    def onEnter(self):
        return

    def onUpdate(self):
        if self.container.vizMode:
            # Viz Mode Logic
            self.success()
        else:
            # Execute Mode Logic
            pd = self.container.om.findObjectByName('Multisense').model.revPolyData
            self.container.om.removeFromObjectModel(self.container.om.findObjectByName('debug'))
            segmentation.findAndFitDrillBarrel(pd, self.container.robotModel.getLinkFrame('utorso'))
            self.success()

    def onExit(self):
        # This action does not move the robot, set the output state accordingly
        # TODO: make this animate the fingers in the future
        self.outputState = deepcopy(self.inputState)


class WaitForScan(Action):

    inputs = []

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.outputs = {}

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
        # Cleanup
        self.currentRevolution = None
        self.desiredRevolution = None

        # This action does not move the robot, set the output state accordingly
        # TODO: make this animate the fingers in the future
        self.outputState = deepcopy(self.inputState)
