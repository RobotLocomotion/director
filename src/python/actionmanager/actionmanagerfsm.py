from simplefsm import SimpleFsm
from actions import *
from ddapp.timercallback import TimerCallback

states = [  'idle',
            'walk_planning',
            'walking',
            'scanning',
            'refitting',
            'reach_planning',
            'reaching',
            'retracting']

layout = {  'plan_received': ['idle', 'walk_planning'],
            'not_reachable': ['walk_planning', 'walking'],
            'reachable': ['walk_planning', 'reach_planning'],
            'walk_done': ['walking', 'scanning'],
            'scan_done': ['scanning', 'refitting'],
            'fit_done': ['refitting', 'reach_planning'],
            'reach_planned': ['reach_planning', 'reaching'],
            'reach_done': ['reaching', 'retracting'],
            'retract_done': ['retracting', 'idle']}


class ActionSequence(object):

    def __init__(self, actionSequence, initial, objectModel, manipPlanner, footstepPlanner, sensorJointController, playbackFunction):

        #local variables
        self.fsm = SimpleFsm()

        self.om = objectModel
        self.manipPlanner = manipPlanner
        self.footstepPlanner = footstepPlanner
        self.sensorJointController = sensorJointController
        self.playbackFunction = playbackFunction

        self.vizMode = True

        #Store all the action objects
        self.action_objects = []

        #All FSMs need a default goal and a fail action
        self.action_objects.append(Goal(self))
        self.action_objects.append(Fail(self))

        #Populate the FSM with all action objects
        for key in actionSequence.keys():
            #Create an instance
            action_ptr = key(actionSequence[key][0],
                             actionSequence[key][1],
                             actionSequence[key][2],
                             self)

            #Store the instance
            self.action_objects.append(action_ptr)

            #Use the instance to populate the FSM with success/fail transitions
            self.fsm.addTransition(action_ptr.name, actionSequence[key][0].name)
            self.fsm.addTransition(action_ptr.name, actionSequence[key][1].name)

            #Setup the special transition to the init state
            if key.name == initial.name:
                self.fsm.addTransition('init', key.name)
                self.fsm.setInitTransition(key.name)
                self.fsm.onUpdate['init'] = self.fsm.initTransition

        #Populate the fsm with all appropriate function pointers
        for action_ptr in self.action_objects:
            self.fsm.onEnter[action_ptr.name] = action_ptr.onEnter
            self.fsm.onUpdate[action_ptr.name] = action_ptr.onUpdate
            self.fsm.onExit[action_ptr.name] = action_ptr.onExit


#if __name__ == '__main__':

states = {WalkPlan    : [Walk, Fail,        {'target':'grasp stance'} ],
          Walk        : [WaitForScan, Fail, [None] ],
          WaitForScan : [Fit, Fail,         [None] ],
          Fit         : [ReachPlan, Fail,   [None] ],
          ReachPlan   : [Reach, WalkPlan,   {'target':'grasp frame', 'hand':'left'} ],
          Reach       : [Grip, Fail,        [None] ],
          Grip        : [RetractPlan, Fail, [None] ],
          RetractPlan : [Retract, Fail,     [None] ],
          Retract     : [Goal, Fail,        [None] ] }


#reach = ActionSequence(actionSequence = states,
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
