from simplefsm import SimpleFsm
from actions import *
from ddapp.timercallback import TimerCallback

class ActionSequence(object):

    def __init__(self,
                 objectModel,
                 sensorJointController,
                 playbackFunction,
                 manipPlanner,
                 footstepPlanner,
                 handDriver,
                 atlasDriver,
                 multisenseDriver,
                 affordanceServer):

        #Planner objects
        self.om = objectModel
        self.manipPlanner = manipPlanner
        self.footstepPlanner = footstepPlanner
        self.sensorJointController = sensorJointController
        self.playbackFunction = playbackFunction

        #Hardware Drivers
        self.handDriver = handDriver
        self.atlasDriver = atlasDriver
        self.multisenseDriver = multisenseDriver
        self.affordanceServer = affordanceServer

        #Shared storage for actions
        self.planPose = None
        self.vizMode = True
        self.vizModeAnimation = []

        self.fsm = None
        self.fsmDebug = False

    def populate(self, sequence, initial):

        #Create and store all the actions, create and populate the FSM
        self.fsm = SimpleFsm(debug = self.fsmDebug)
        self.actionObjects = []

        #All FSMs need a default goal and a fail action
        self.actionObjects.append(Goal(self))
        self.actionObjects.append(Fail(self))

        #Populate the FSM with all action objects
        for name in sequence.keys():

            actionClass = sequence[name][0]

            #Create an instance
            actionPtr = actionClass(name,
                                    sequence[name][1],
                                    sequence[name][2],
                                    sequence[name][3],
                                    self)

            #Store the instance
            self.actionObjects.append(actionPtr)

            #Use the instance to populate the FSM with success/fail transitions
            self.fsm.addTransition(name, sequence[name][1])
            self.fsm.addTransition(name, sequence[name][2])

            #Setup the special transition to the init state
            if name == initial:
                self.fsm.addTransition('init', name)
                self.fsm.setInitTransition(name)
                self.fsm.onUpdate['init'] = self.fsm.initTransition

        #Populate the fsm with all appropriate function pointers
        for actionPtr in self.actionObjects:
            self.fsm.onEnter[actionPtr.name] = actionPtr.onEnter
            self.fsm.onUpdate[actionPtr.name] = actionPtr.onUpdate
            self.fsm.onExit[actionPtr.name] = actionPtr.onExit

    def reset(self):
        self.fsm.reset()
        self.planPose = None
        self.vizMode = True
        self.vizModeAnimation = []

    def clear(self):
        self.reset()
        self.fsm = None


#if __name__ == '__main__':

#states = {WalkPlan    : [Walk, Fail,        {'target':'grasp stance'} ],
#          Walk        : [WaitForScan, Fail, [None] ],
#          WaitForScan : [Fit, Fail,         [None] ],
#          Fit         : [ReachPlan, Fail,   [None] ],
#          ReachPlan   : [Reach, WalkPlan,   {'target':'grasp frame', 'hand':'left'} ],
#          Reach       : [Grip, Fail,        [None] ],
#          Grip        : [RetractPlan, Fail, [None] ],
#          RetractPlan : [Retract, Fail,     {'side':'left'} ],
#          Retract     : [Goal, Fail,        [None] ]}


#reach = ActionSequence(sequence = states,
#                       initial = WaitForScan,
#                       objectModel = om,
#                       manipPlanner = manipPlanner,
#                       footstepPlanner = footstepsDriver,
#                       sensorJointController = robotStateJointController)

#timer = TimerCallback()
#timer.callback = reach.fsm.update
#timer.targetFps = 3
#reach.fsm.start()

#timer.start()
#self.footstepPlan = self.footstepPlanner.sendFootstepPlanRequest(self.graspStanceFrame.transform, waitForResponse=True)
