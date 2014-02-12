from sets import Set

class SimpleFsm(object):

    def __init__(self):

        self.started = False
        self.onEnter = {}
        self.onUpdate = {}
        self.onExit = {}
        self.validTransitions = []
        self.transitionDict = {}

        #initialize state set
        self.states = Set([])

        #Add default states using addState method
        self.addState('goal')
        self.addState('fail')
        self.addState('init')

        #Set state to an initial value
        self.current = 'init'
        self.initTrans = 'init'

    def addState(self, state):
        print "adding state:", state
        self.states.add(state)
        if not state in self.onEnter:
            self.onEnter[state] = self.noop
        if not state in self.onUpdate:
            self.onUpdate[state] = self.noop
        if not state in self.onExit:
            self.onExit[state] = self.noop

    def addTransition(self, start, end, name = None):
        #Add the states if they didn't exist
        if not start in self.states:
            self.addState(start)
        if not end in self.states:
            self.addState(end)

        #Add a transition to the list
        self.validTransitions.append([start, end])

        #Add a name to the lookup dictionary if provided
        if not name:
            self.transitionDict[start+'_to_'+end] = start+'_to_'+end
        else:
            self.transitionDict[start+'_to_'+end] = name

    def start(self):
        self.started = True

    def stop(self):
        self.started = False

    def reset(self):
        self.current = 'init'

    def noop(self):
        return

    def setInitTransition(self, initTrans):
        self.initTrans = initTrans

    def initTransition(self):
        self.transition(self.initTrans)

    def transition(self, newState):
        if not newState in self.states:
            print "Desired new state does not exist"
            return

        if not [self.current, newState] in self.validTransitions:
            print "Desired transition is not valid"
            return

        #Do the transition
        print "Leaving: ", self.current
        self.onExit[self.current]()
        transitionString = self.current+'_to_'+newState
        print "Transition: ", self.transitionDict[transitionString]
        print "Entering: ", newState
        self.onEnter[newState]()
        self.current = newState

    def update(self):
        if self.started:
            print "Updating in state:", self.current
            self.onUpdate[self.current]()

