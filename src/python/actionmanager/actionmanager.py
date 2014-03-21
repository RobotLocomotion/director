from simplefsm import SimpleFsm
from ddapp.timercallback import TimerCallback

from copy import deepcopy
from collections import OrderedDict
from sets import Set

import actions

def checkArgsPopulated(sequence):

    noUserArgs = True

    for action in sequence.keys():
        args = sequence[action][3]
        if 'USER' in args.values():
            noUserArgs = False

    return noUserArgs

def checkValidity(sequence):

    sequenceValid = True

    actionList = sequence.keys()
    validActions = actionList + ['goal', 'fail']

    for action in actionList:
        #Make sure the first arg is a class by seeing if it's type is a class (i.e. it is 'type')
        className = sequence[action][0]
        if not type(className) == type:
            print "Specified action argument is not an action class (or any class at all)"
            sequenceValid = False
        #If it is a class, make sure it's from the actions module
        elif className.__name__ not in dir(actions):
            print "Specified action class:", className.__name__, "is not recognized"
            sequenceValid = False

        #Make sure the second and third arguments are valid action instance names
        success = sequence[action][1]
        if success not in validActions:
            print "Action:", action, "requesting success transition to unknown state:", success
            sequenceValid = False
        fail = sequence[action][2]
        if fail not in validActions:
            print "Action:", action, "requesting failure transition to unknown state:", success
            sequenceValid = False

        #Make sure the fourth argument is a dictionary of valid argument types and argument
        #sources (other actions or 'USER')
        args = sequence[action][3]
        for arg in args.keys():
            #First, do the args exist
            if not arg in actions.argType.keys():
                print arg, "specified as arg for action", action, "but it is not valid."
                sequenceValid = False
                break
            #Second, is the specified producer of this argument valid
            producer = args[arg]
            if not producer in actionList + ['USER']:
                print producer, "specified as source of arg", arg, "is not a valid argument source."
                sequenceValid = False
                break
            #Third, does the producer actually produce that argument
            if producer == 'USER':
                continue
            else:
                producerType = sequence[producer][0]
                if arg not in producerType.outputs.keys():
                    print action, "needs", arg, "and designated it as coming from", producer, ", which is of type", producerType.__name__, "which does not make", arg
                    sequenceValid = False
                    break
    return sequenceValid

def generateUserArgDict(sequence):

    userArgs = Set([])

    for action in sequence:
        arguments = sequence[action][3]
        for argument in arguments.keys():
            if arguments[argument] == 'USER':
                userArgs.add(argument)

    return OrderedDict([(arg, 'USER') for arg in list(userArgs)])

def generateUserArgList(sequence):
    return generateUserArgDict(sequence).keys()

class ActionManager(object):

    def __init__(self,
                 objectModel,
                 robotModel,
                 sensorJointController,
                 playbackFunction,
                 manipPlanner,
                 footstepPlanner,
                 handDriver,
                 atlasDriver,
                 multisenseDriver,
                 affordanceServer,
                 fsmDebug = False):

        #Planner objects
        self.om = objectModel
        self.robotModel = robotModel
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
        self.vizMode = True
        self.vizModeAnimation = []

        self.fsm = None
        self.fsmDebug = fsmDebug
        self.name = 'None'
        self.sequence = None
        self.actionObjects = {}
        self.previousAction = None
        self.executionList = []

    def populate(self, name, sequence, initial):

        #Create and store all the actions, create and populate the FSM
        self.fsm = SimpleFsm(debug = self.fsmDebug)
        self.timerObject  = TimerCallback()
        self.timerObject.targetFps = 10
        self.timerObject.callback = self.fsm.update

        #Create and store data related to this sequence
        self.name = name
        self.sequence = deepcopy(sequence)
        self.actionObjects = {}
        self.previousAction = None
        self.executionList = []

        #All FSMs need a default goal and a fail action
        self.actionObjects['goal'] = actions.Goal(self)
        self.actionObjects['fail'] = actions.Fail(self)

        #Populate the FSM with all action objects
        for name in self.sequence.keys():

            actionClass = self.sequence[name][0]

            #Create an instance
            actionPtr = actionClass(name,              #Name
                                    self.sequence[name][1], #Success Transition
                                    self.sequence[name][2], #Fail Transition
                                    self.sequence[name][3], #Arg List
                                    self)              #Sequence Object as data container

            #Store the instance
            self.actionObjects[name] = actionPtr

            #Use the instance to populate the FSM with success/fail transitions
            self.fsm.addTransition(name, self.sequence[name][1])
            self.fsm.addTransition(name, self.sequence[name][2])

            #all states need a transition to fail to catch errors
            if self.sequence[name][2] != 'fail':
                self.fsm.addTransition(name, 'fail')

            #Setup the special transition to the init state
            if name == initial:
                self.fsm.addTransition('init', name)
                self.fsm.setInitTransition(name)
                self.fsm.onUpdate['init'] = self.fsm.initTransition

        #Populate the fsm with all appropriate function pointers
        for actionPtr in self.actionObjects.values():
            self.fsm.onEnter[actionPtr.name] = actionPtr.argParseAndEnter
            self.fsm.onUpdate[actionPtr.name] = actionPtr.onUpdate
            self.fsm.onExit[actionPtr.name] = actionPtr.storeEndStateAndExit

        #Now that all of the actions are created, the argument list needs
        #to be modified to have action references instead of names

        #Look at each action object...
        for actionRef in self.actionObjects.values():
            #Now look at each argument in the argument list...
            for argName, argVal in actionRef.args.iteritems():
                #If the argument's data is an action instance name, replace with a ref
                if argVal in self.actionObjects.keys():
                    actionRef.args[argName] = self.actionObjects[argVal]

    def getActionByName(self, name):
        if name in self.actionObjects.keys():
            return self.actionObjects[name]
        return None

    def reset(self):
        self.fsm.stop()
        self.fsm.reset()
        self.fsm.debug = self.fsmDebug
        for action in self.actionObjects.values():
            action.reset()
        self.vizModeAnimation = []
        self.executionList = []

    def start(self, vizMode = True):
        self.vizMode = vizMode
        self.timerObject.start()
        self.fsm.start()

    def stop(self):
        self.fsm.stop()
        self.timerObject.stop()

    def clear(self):
        self.reset()
        self.name = 'None'
        self.sequence = {}
        self.actionObjects = {}

    def play(self):
        if self.vizModeAnimation != []:
            self.playbackFunction(self.vizModeAnimation)
