
class Action(object):

    def __init__(self, success, fail, args, fsm, sequence):
        self.fsm = fsm
        self.sequence = sequence
        self.args = args
        self.success_action = success
        self.fail_action = fail

    def onEnter(self):
        print "default enter"

    def onUpdate(self):
        print "default update"

    def onExit(self):
        print "default exit"


class Goal(Action):

    name = 'goal'

    def __init__(self, fsm):
        Action.__init__(self, None, None, None, None, None)

    def onEnter(self):
        print "HOORAY! Entered the GOAL state"


class Fail(Action):

    name = 'fail'

    def __init__(self, fsm):
        Action.__init__(self, None, None, None, None, None)

    def onEnter(self):
        print "NOOO! Entered the FAIL state"



#Generic notes:
#        state transition when success
#        state transition when fail
#        function to evaluate
#        on enter function -kick off eval function
#        on update function -wait for success fail
#        on exit function -teardown, if necessary


class ReachPlan(Action):

    name = 'reach_plan'

    def __init__(self, success, fail, args, fsm, sequence):
        Action.__init__(self, success, fail, args, fsm, sequence)
        self.counter = 10
        self.manipPlanResponse = False

    def onEnter(self):
        print "Entered the REACH_PLAN state"

        linkMap = {
                      'left' : 'l_hand',
                      'right': 'r_hand'
                  }
        linkName = linkMap[self.args['hand']]

        graspFrame = self.sequence.om.findObjectByName(self.args['target'])

        startPose = self.sequence.sensorJointController.getPose('EST_ROBOT_STATE')

        self.manipPlanResponse = self.sequence.manipPlanner.sendEndEffectorGoal(startPose, linkName, graspFrame.transform, waitForResponse=True, waitTimeout=0)

    def onUpdate(self):

        response = self.manipPlanResponse.waitForResponse(timeout = 0)

        if response:
            print "done planning, time to go to", self.success_action.name
            self.manipPlanResponse.finish()

            print response

            if response.plan_info[-1] > 10:
                print "PLANNER REPORTS ERROR!"
                self.fsm.transition(self.fail_action.name)
            else:
                print "Planner reports success!"
                self.fsm.transition(self.success_action.name)


    def onExit(self):
        self.manipPlanResponse = None

class Reach(Action):

    name = 'reach'

    def __init__(self, success, fail, args, fsm, sequence):
        Action.__init__(self, success, fail, args, fsm, sequence)
        self.counter = 10

    def onEnter(self):
        print "Entered the REACH state"

    def onUpdate(self):
        self.counter -= 1
        if self.counter == 0:
            print "done reaching, time to go to", self.success_action.name
            self.fsm.transition(self.success_action.name)

class WalkPlan(Action):

    name = 'walk_plan'

    def __init__(self, success, fail, args, fsm, sequence):
        Action.__init__(self, success, fail, args, fsm, sequence)
        self.counter = 10

    def onEnter(self):
        print "Entered the WALK_PLAN state"

    def onUpdate(self):
        self.counter -= 1
        if self.counter == 0:
            print "done planning, time to go to", self.success_action.name
            self.fsm.transition(self.success_action.name)

class Walk(Action):

    name = 'walk'

    def __init__(self, success, fail, args, fsm, sequence):
        Action.__init__(self, success, fail, args, fsm, sequence)
        self.counter = 10

    def onEnter(self):
        print "Entered the WALK state"

    def onUpdate(self):
        self.counter -= 1
        if self.counter == 0:
            print "done walking, time to go to", self.success_action.name
            self.fsm.transition(self.success_action.name)

class RetractPlan(Action):

    name = 'retract_plan'

    def __init__(self, success, fail, args, fsm, sequence):
        Action.__init__(self, success, fail, args, fsm, sequence)
        self.counter = 10

    def onEnter(self):
        print "Entered the RETRACT_PLAN state"

    def onUpdate(self):
        self.counter -= 1
        if self.counter == 0:
            print "done planning, time to go to", self.success_action.name
            self.fsm.transition(self.success_action.name)

class Retract(Action):

    name = 'retract'

    def __init__(self, success, fail, args, fsm, sequence):
        Action.__init__(self, success, fail, args, fsm, sequence)
        self.counter = 10

    def onEnter(self):
        print "Entered the RETRACT state"

    def onUpdate(self):
        self.counter -= 1
        if self.counter == 0:
            print "done retracting, time to go to", self.success_action.name
            self.fsm.transition(self.success_action.name)

    def onExit(self):
        self.counter = 10

class Grip(Action):

    name = 'grip'

    def __init__(self, success, fail, args, fsm, sequence):
        Action.__init__(self, success, fail, args, fsm, sequence)
        self.counter = 10

    def onEnter(self):
        print "Entered the GRIP state"

    def onUpdate(self):
        self.counter -= 1
        if self.counter == 0:
            print "done gripping, time to go to", self.success_action.name
            self.fsm.transition(self.success_action.name)

    def onExit(self):
        self.counter = 10


class Fit(Action):

    name = 'fit'

    def __init__(self, success, fail, args, fsm, sequence):
        Action.__init__(self, success, fail, args, fsm, sequence)
        self.counter = 10

    def onEnter(self):
        print "Entered the FIT state"

    def onUpdate(self):
        self.counter -= 1
        if self.counter == 0:
            print "done fitping, time to go to", self.success_action.name
            self.fsm.transition(self.success_action.name)

    def onExit(self):
        self.counter = 10


class WaitForScan(Action):

    name = 'waitforscan'

    def __init__(self, success, fail, args, fsm, sequence):
        Action.__init__(self, success, fail, args, fsm, sequence)
        self.counter = 10

    def onEnter(self):
        print "Entered the WAITFORSCAN state"

    def onUpdate(self):
        self.counter -= 1
        if self.counter == 0:
            print "done waiting for scan, time to go to", self.success_action.name
            self.fsm.transition(self.success_action.name)

    def onExit(self):
        self.counter = 10
