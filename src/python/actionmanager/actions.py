
class Action(object):

    def __init__(self, success, fail, args, container):
        self.container = container
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

    def __init__(self, container):
        Action.__init__(self, None, None, None, container)

    def onEnter(self):
        print "HOORAY! Entered the GOAL state"


class Fail(Action):

    name = 'fail'

    def __init__(self, container):
        Action.__init__(self, None, None, None, container)

    def onEnter(self):
        print "NOOO! Entered the FAIL state"



#Generic notes:
#        state transition when success
#        state transition when fail
#        function to evaluate
#        on enter function -kick off eval function
#        on update function -wait for success fail
#        on exit function -teardown, if necessary

class WalkPlan(Action):

    name = 'walk_plan'

    def __init__(self, success, fail, args, container):
        Action.__init__(self, success, fail, args, container)
        self.counter = 10
        self.walkPlanResponse = None

    def onEnter(self):
        print "Entered the WALK_PLAN state"
        graspStanceFrame = self.container.om.findObjectByName(self.args['target'])
        self.walkPlanResponse = self.container.footstepPlanner.sendFootstepPlanRequest(graspStanceFrame.transform, waitForResponse=True, waitTimeout=0)

    def onUpdate(self):

        response = self.walkPlanResponse.waitForResponse(timeout = 0)

        if response:

            if response.num_steps == 0:
                print 'walk plan failed'
                self.container.fsm.transition(self.fail_action.name)
            else:
                print 'walk plan successful'
                self.container.walkPlan = response
                print "done planning, time to go to", self.success_action.name
                self.container.fsm.transition(self.success_action.name)

    def onExit(self):
        self.walkPlanResponse.finish()
        self.walkPlanResponse = None

class Walk(Action):

    name = 'walk'

    def __init__(self, success, fail, args, container):
        Action.__init__(self, success, fail, args, container)
        self.counter = 10

    def onEnter(self):
        print "Entered the WALK state"
        self.container.playbackFunction([self.container.walkPlan])

    def onUpdate(self):
        self.counter -= 1
        if self.counter == 0:
            print "done walking, time to go to", self.success_action.name
            self.container.fsm.transition(self.success_action.name)

    def onExit(self):
        self.counter = 10

class ReachPlan(Action):

    name = 'reach_plan'

    def __init__(self, success, fail, args, container):
        Action.__init__(self, success, fail, args, container)
        self.counter = 10
        self.manipPlanResponse = None

    def onEnter(self):
        print "Entered the REACH_PLAN state"
        linkMap = { 'left' : 'l_hand', 'right': 'r_hand'}
        linkName = linkMap[self.args['hand']]
        graspFrame = self.container.om.findObjectByName(self.args['target'])
        startPose = self.container.sensorJointController.getPose('EST_ROBOT_STATE')
        self.manipPlanResponse = self.container.manipPlanner.sendEndEffectorGoal(startPose, linkName, graspFrame.transform, waitForResponse=True, waitTimeout=0)

    def onUpdate(self):
        response = self.manipPlanResponse.waitForResponse(timeout = 0)

        if response:
            if response.plan_info[-1] > 10:
                print "PLANNER REPORTS ERROR!"
                self.container.fsm.transition(self.fail_action.name)
            else:
                print "Planner reports success!"
                self.container.fsm.transition(self.success_action.name)

    def onExit(self):
        self.manipPlanResponse.finish()
        self.manipPlanResponse = None


class Reach(Action):

    name = 'reach'

    def __init__(self, success, fail, args, container):
        Action.__init__(self, success, fail, args, container)
        self.counter = 10

    def onEnter(self):
        print "Entered the REACH state"

    def onUpdate(self):
        self.counter -= 1
        if self.counter == 0:
            print "done reaching, time to go to", self.success_action.name
            self.container.fsm.transition(self.success_action.name)

    def onExit(self):
        self.counter = 10


class RetractPlan(Action):

    name = 'retract_plan'

    def __init__(self, success, fail, args, container):
        Action.__init__(self, success, fail, args, container)
        self.counter = 10

    def onEnter(self):
        print "Entered the RETRACT_PLAN state"

    def onUpdate(self):
        self.counter -= 1
        if self.counter == 0:
            print "done planning, time to go to", self.success_action.name
            self.container.fsm.transition(self.success_action.name)

    def onExit(self):
        self.counter = 10

class Retract(Action):

    name = 'retract'

    def __init__(self, success, fail, args, container):
        Action.__init__(self, success, fail, args, container)
        self.counter = 10

    def onEnter(self):
        print "Entered the RETRACT state"

    def onUpdate(self):
        self.counter -= 1
        if self.counter == 0:
            print "done retracting, time to go to", self.success_action.name
            self.container.fsm.transition(self.success_action.name)

    def onExit(self):
        self.counter = 10

class Grip(Action):

    name = 'grip'

    def __init__(self, success, fail, args, container):
        Action.__init__(self, success, fail, args, container)
        self.counter = 10

    def onEnter(self):
        print "Entered the GRIP state"

    def onUpdate(self):
        self.counter -= 1
        if self.counter == 0:
            print "done gripping, time to go to", self.success_action.name
            self.container.fsm.transition(self.success_action.name)

    def onExit(self):
        self.counter = 10


class Fit(Action):

    name = 'fit'

    def __init__(self, success, fail, args, container):
        Action.__init__(self, success, fail, args, container)
        self.counter = 10

    def onEnter(self):
        print "Entered the FIT state"

    def onUpdate(self):
        self.counter -= 1
        if self.counter == 0:
            print "done fitping, time to go to", self.success_action.name
            self.container.fsm.transition(self.success_action.name)

    def onExit(self):
        self.counter = 10


class WaitForScan(Action):

    name = 'waitforscan'

    def __init__(self, success, fail, args, container):
        Action.__init__(self, success, fail, args, container)
        self.counter = 10

    def onEnter(self):
        print "Entered the WAITFORSCAN state"

    def onUpdate(self):
        self.counter -= 1
        if self.counter == 0:
            print "done waiting for scan, time to go to", self.success_action.name
            self.container.fsm.transition(self.success_action.name)

    def onExit(self):
        self.counter = 10
