from time import time
from ddapp import robotstate

from ddapp.plansequence import RobotPoseGUIWrapper

#Base Class, all actions must inherit from this
#All inheritors must:
#
# In constructor, need to pass the following
# -instance name
# -name of an instance to transition to on success
# -name of an instance to transition to on failure
# -a list of arguments passed in
# -a reference to an ActionSequence object used as a data container
#  to share data between Actions


argType = {}
argType['Affordance'] = None
argType['WalkPlan'] = None
argType['WalkTarget'] = None
argType['RobotPose'] = None
argType['JointPlan'] = None
argType['Hand'] = None
argType['Constraints'] = None

class Action(object):

    inputs = []
    outputs = []

    def __init__(self, name, success, fail, args, container):
        self.name = name
        self.container = container
        self.args = args
        self.successAction = success
        self.failAction = fail

    def onEnter(self):
        print "default enter"

    def onUpdate(self):
        print "default update"

    def onExit(self):
        print "default exit"

    def checkInputArgs(self, args, inputs):
        valid = True
        for arg in args:
            if arg not in inputs:
                valid = False
                print "Arg:", arg, "provided but it is not a valid input for:", self.name
        return valid

#Below are standard success and fail actions which simply terminate
#the sequence with a success or fail message

class Goal(Action):

    inputs = []
    outputs = []

    def __init__(self, container):
        Action.__init__(self, 'goal', None, None, None, container)

    def onEnter(self):
        print "HOORAY! Entered the GOAL state"

        #If in vizmode, play the animation
        if self.container.vizMode:
            self.container.play()

        #Stop the FSM now that we're at a terminus
        self.container.fsm.stop()

class Fail(Action):

    def __init__(self, container):
        Action.__init__(self, 'fail', None, None, None, container)

    def onEnter(self):
        print "NOOO! Entered the FAIL state"

        #Stop the FSM now that we're at a terminus
        self.container.fsm.stop()


#Generic notes:
#        state transition when success
#        state transition when fail
#        function to evaluate
#        on enter function -kick off eval function
#        on update function -wait for success fail
#        on exit function -teardown, if necessary

class WalkPlan(Action):

    inputs = ['WalkTarget']
    outputs = ['WalkPlan']

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.counter = 10
        self.walkPlanResponse = None

    def onEnter(self):
        graspStanceFrame = self.container.om.findObjectByName(self.args['WalkTarget'])
        self.walkPlanResponse = self.container.footstepPlanner.sendFootstepPlanRequest(graspStanceFrame.transform, waitForResponse=True, waitTimeout=0)

    def onUpdate(self):

        response = self.walkPlanResponse.waitForResponse(timeout = 0)

        if response:
            if response.num_steps == 0:
                print 'walk plan failed'
                self.container.fsm.transition(self.failAction)
            else:
                print 'walk plan successful'
                self.container.footstepPlan = response
                self.container.fsm.transition(self.successAction)

    def onExit(self):
        self.walkPlanResponse.finish()
        self.walkPlanResponse = None

class Walk(Action):

    inputs = ['WalkPlan']
    outputs = ['RobotPose']

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.walkAnimationResponse = None

    def onEnter(self):

        #Logic for viz-mode:
        if self.container.vizMode:
            #Use the footstep plan to calculate a walking plan, for animation purposes only
            self.walkAnimationResponse = self.container.footstepPlanner.sendWalkingPlanRequest(self.container.footstepPlan, waitForResponse=True, waitTimeout=0)

        #Logic for execute mode:
        else:
            self.container.footstepPlanner.commitFootstepPlan(self.container.footstepPlan)
            self.walkStart = False

    def onUpdate(self):

        #Logic for viz-mode:
        if self.container.vizMode:
            response = self.walkAnimationResponse.waitForResponse(timeout = 0)
            if response:
                #We got a successful response from the walk planner, cache it and update the plan state
                self.container.walkAnimation = response
                self.container.planPose = robotstate.convertStateMessageToDrakePose(self.container.walkAnimation.plan[-1])
                self.container.vizModeAnimation.append(self.container.walkAnimation)
                self.container.fsm.transition(self.successAction)

        #Logic for execute mode:
        else:
            if self.container.atlasDriver.getCurrentBehaviorName() == 'step':
                self.walkStart = True
            if self.walkStart and self.container.atlasDriver.getCurrentBehaviorName() == 'stand':
                self.container.fsm.transition(self.successAction)

    def onExit(self):

        #Logic for viz-mode:
        if self.container.vizMode:
            self.walkAnimationResponse.finish()
            self.walkAnimationResponse = None

        #Logic for execute-mode:
        else:
            self.walkStart = None

class PoseSearch(Action):

    inputs = ['Affordance', 'Hand', 'Constraints']
    outputs = ['RobotPose', 'WalkTarget', 'Hand']

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.counter = 10
        self.manipPlanResponse = None

    def onEnter(self):
        return

    def onUpdate(self):
        return

    def onExit(self):
        return

class ReachPlan(Action):

    inputs = ['TargetFrame', 'Hand', 'Constraints']
    outputs = ['JointPlan']

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.counter = 10
        self.manipPlanResponse = None

    def onEnter(self):
        linkMap = { 'left' : 'l_hand', 'right': 'r_hand'}

        if not self.checkInputArgs(self.args, self.inputs):
            print self.name, "ERROR: Input args do not match."
            self.container.fsm.transition('fail')

        linkName = linkMap[self.args['Hand']]
        graspFrame = self.container.om.findObjectByName(self.args['TargetFrame'])

        if self.container.planPose == None:
            self.container.planPose = self.container.sensorJointController.getPose('EST_ROBOT_STATE')
        self.manipPlanResponse = self.container.manipPlanner.sendEndEffectorGoal(self.container.planPose, linkName, graspFrame.transform, waitForResponse=True, waitTimeout=0)

    def onUpdate(self):
        response = self.manipPlanResponse.waitForResponse(timeout = 0)

        if response:
            self.outputVals = {'JointPose': None}
            if response.plan_info[-1] > 10:
                print "PLANNER REPORTS ERROR!"
                self.container.fsm.transition(self.failAction)
            else:
                print "Planner reports success!"
                self.container.fsm.transition(self.successAction)

                #Viz Mode Logic:
                if self.container.vizMode:
                    self.container.vizModeAnimation.append(response)
                    self.container.planPose = robotstate.convertStateMessageToDrakePose(self.container.vizModeAnimation[-1].plan[-1])
                    self.outputVals['JointPose'] = robotstate.convertStateMessageToDrakePose(self.container.vizModeAnimation[-1].plan[-1])

    def onExit(self):
        self.manipPlanResponse.finish()
        self.manipPlanResponse = None


class Reach(Action):

    inputs = ['JointPlan']
    outputs = ['RobotPose']

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.counter = 10

    def onEnter(self):
        return

    def onUpdate(self):

        #Viz Mode Logic
        if self.container.vizMode:
            #Update the current state to be the end of the last step for future planning
            self.container.planPose = robotstate.convertStateMessageToDrakePose(self.container.vizModeAnimation[-1].plan[-1])
            self.container.fsm.transition(self.successAction)

        #Execute Mode Logic
        else:
            self.counter -= 1
            if self.counter == 0:
                self.container.fsm.transition(self.successAction)

    def onExit(self):
        self.counter = 10


class JointMovePlan(Action):

    inputs = ['RobotPose']
    outputs = ['JointPlan']

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.counter = 10
        self.retractPlanResponse = None

    def onEnter(self):
        goalPoseJoints = RobotPoseGUIWrapper.getPose(self.args['Group'], self.args['RobotPose'], self.args['Hand'])
        if self.container.planPose == None:
            self.container.planPose = self.container.sensorJointController.getPose('EST_ROBOT_STATE')
        self.retractPlanResponse = self.container.manipPlanner.sendPoseGoal(self.container.planPose, goalPoseJoints, waitForResponse=True, waitTimeout=0)

    def onUpdate(self):

        response = self.retractPlanResponse.waitForResponse(timeout = 0)
        if response:
            self.container.retractPlan = response
            self.container.vizModeAnimation.append(response)
            self.container.fsm.transition(self.successAction)

    def onExit(self):
        self.counter = 10

class JointMove(Action):

    inputs = ['JointPlan']
    outputs = ['RobotPose']

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)
        self.counter = 10

    def onEnter(self):
        return

    def onUpdate(self):

        #Viz Mode Logic
        if self.container.vizMode:
            self.container.planPose = robotstate.convertStateMessageToDrakePose(self.container.vizModeAnimation[-1].plan[-1])
            self.container.fsm.transition(self.successAction)

        #Execute Mode Logic
        else:
            self.counter -= 1
            if self.counter == 0:
                self.container.fsm.transition(self.successAction)

    def onExit(self):
        self.counter = 10

class Grip(Action):

    inputs = ['Hand']
    outputs = []

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)

    def onEnter(self):
        self.container.handDriver.sendClose(60)

    def onUpdate(self):

        #Viz Mode Logic
        #Execute Mode Logic
        self.container.fsm.transition(self.successAction)

    def onExit(self):
        return

class Release(Action):

    inputs = ['Hand']
    outputs = []

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)

    def onEnter(self):
        self.container.handDriver.sendOpen()

    def onUpdate(self):

        #Viz Mode Logic
        #Execute Mode Logic
        self.container.fsm.transition(self.successAction)

    def onExit(self):
        return

class Fit(Action):

    inputs = ['Affordance']
    outputs = []

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)

    def onEnter(self):
        self.enterTime = time()

    def onUpdate(self):

        self.container.fsm.transition(self.successAction)

        #Viz Mode Logic
        if self.container.vizMode:
            self.container.fsm.transition(self.successAction)

        #Execute Mode Logic
        else:
            if self.container.affordanceServer[self.args['affordance']] > self.enterTime:
                print "New affordance fit found."
                self.container.fsm.transition(self.successAction)

    def onExit(self):
        self.waitTime = 0.0


class WaitForScan(Action):

    inputs = []
    outputs = []

    def __init__(self, name, success, fail, args, container):
        Action.__init__(self, name, success, fail, args, container)

    def onEnter(self):

        #Viz Mode Logic
        if self.container.vizMode:
            return

        #Execute Mode Logic
        else:
            self.currentRevolution = self.container.multisenseDriver.displayedRevolution
            self.desiredRevolution = self.currentRevolution + 2

    def onUpdate(self):

        #Viz Mode Logic
        if self.container.vizMode:
            self.container.fsm.transition(self.successAction)

        #Execute Mode Logic
        else:
            if self.container.multisenseDriver.displayedRevolution >= self.desiredRevolution:
                self.container.fsm.transition(self.successAction)

    def onExit(self):
        self.currentRevolution = None
        self.desiredRevolution = None

