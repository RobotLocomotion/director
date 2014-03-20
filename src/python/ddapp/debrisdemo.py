import ddapp.objectmodel as om
from ddapp.asynctaskqueue import AsyncTaskQueue


class DebrisPlannerDemo(object):

    def __init__(self, robotStateModel, playbackRobotModel, ikPlanner, manipPlanner, atlasDriver, handDriver, multisenseDriver, refitFunction):

        self.robotStateModel = robotStateModel
        self.playbackRobotModel = playbackRobotModel
        self.ikPlanner = ikPlanner
        self.manipPlanner = manipPlanner
        self.atlasDriver = atlasDriver
        self.handDriver = handDriver
        self.multisenseDriver = multisenseDriver
        self.refitFunction = refitFunction

        self.userPromptEnabled = True
        self.visOnly = True

    def reset(self):
        self.ikPlanner.lockAffordanceToHand = False
        self.ikPlanner.randomAffordance(self.robotStateModel)

    def playManipPlan(self):
        self.ikPlanner.playManipPlan()
        self.robotStateModel.setProperty('Alpha', 0.1)

    def search(self):
        self.ikPlanner.endPoseSearch()
        self.robotStateModel.setProperty('Alpha', 0.1)

    def preGrasp(self):
        self.ikPlanner.updateGraspEndPose(enableSearch=False)
        self.ikPlanner.computePreGraspTraj()
        self.playManipPlan()

    def adjustPreGrasp(self):
        self.ikPlanner.updateGraspEndPose(enableSearch=False)
        self.ikPlanner.computePreGraspAdjustment()
        self.playManipPlan()

    def grasp(self):
        self.ikPlanner.computeGraspReach()
        self.playManipPlan()

    def retract(self):
        self.ikPlanner.lockAffordanceToHand = True
        self.ikPlanner.computeRetractTraj()
        self.playManipPlan()

    def stand(self):
        self.ikPlanner.lockAffordanceToHand = True
        self.ikPlanner.computeStand()
        self.playManipPlan()

    def extendArm(self):
        self.ikPlanner.lockAffordanceToHand = True
        self.ikPlanner.computeArmExtend()
        self.playManipPlan()

    def drop(self):
        self.ikPlanner.lockAffordanceToHand = False
        om.removeFromObjectModel(self.ikPlanner.affordance)
        self.ikPlanner.computeNominal()
        self.playManipPlan()

    def nominal(self):
        self.ikPlanner.computeNominal()
        self.playManipPlan()

    def newReachGoal(self):
        self.ikPlanner.newReachGoal()

    def toggleAffordanceEdit(self):
        aff = self.ikPlanner.findAffordance()
        frame = self.ikPlanner.getAffordanceFrame()
        edit = not frame.getProperty('Edit')
        frame.setProperty('Edit', edit)
        aff.setProperty('Alpha', 0.5 if edit else 1.0)

    def planReach(self):
        self.ikPlanner.planReach()

    def commit(self):
        if self.visOnly:
            self.ikPlanner.commitState()
            self.clearPlan()
        else:
            self.manipPlanner.commitManipPlan(self.ikPlanner.lastManipPlan)
            self.clearPlan()

    def clearPlan(self):
        self.ikPlanner.lastManipPlan = None
        self.robotStateModel.setProperty('Alpha', 1.0)
        self.playbackRobotModel.setProperty('Visible', False)

    def useEndPose(self, index):
        self.ikPlanner.useGraspEndPoseOption(index)

    def sendHeightMode(self):
        self.atlasDriver.sendPlanUsingBdiHeight(True)

    def openHand(self):
        self.handDriver.sendOpen()

    def closeHand(self):
        self.handDriver.sendClose()

    def sendPelvisCrouch(self):
        self.atlasDriver.sendPelvisHeightCommand(0.7)

    def sendPelvisStand(self):
        self.atlasDriver.sendPelvisHeightCommand(0.8)

    def sendNeckPitchLookDown(self):
        self.multisenseDriver.setNeckPitch(40)

    def spinLidar(self):
        self.multisenseDriver.setLidarRevolutionTime(10)

    def sendNeckPitchLookForward(self):
        self.multisenseDriver.setNeckPitch(15)


    def waitForAtlasBehaviorAsync(self, behaviorName):
        assert behaviorName in self.atlasDriver.getBehaviorMap().values()
        while self.atlasDriver.getCurrentBehaviorName() != behaviorName:
            yield


    def printAsync(self, s):
        yield
        print s


    def userPrompt(self, message):

        if not self.userPromptEnabled:
            return

        yield
        result = raw_input(message)
        if result != 'y':
            raise Exception('user abort.')


    def delay(self, delayTimeInSeconds):
        yield
        t = SimpleTimer()
        while t.elapsed() < delayTimeInSeconds:
            yield


    def pauseQueue(self):
        raise AsyncTaskQueue.PauseException()


    def waitForCleanLidarSweepAsync(self):
        currentRevolution = self.multisenseDriver.displayedRevolution
        desiredRevolution = currentRevolution + 2
        while self.multisenseDriver.displayedRevolution < desiredRevolution:
            yield


    def autonomousExecute(self):

        self.ikPlanner.planFromCurrentRobotState = True

        taskQueue = AsyncTaskQueue()

        # require affordance at start
        taskQueue.addTask(self.printAsync('get affordance'))
        taskQueue.addTask(self.ikPlanner.getAffordanceFrame)

        # stand, open hand, manip
        taskQueue.addTask(self.printAsync('send behavior start commands'))
        taskQueue.addTask(self.userPrompt('stand and open hand. continue? y/n: '))
        taskQueue.addTask(self.atlasDriver.sendManipCommand)
        taskQueue.addTask(self.openHand)
        taskQueue.addTask(self.delay(3.0))
        taskQueue.addTask(self.sendHeightMode)
        taskQueue.addTask(self.atlasDriver.sendManipCommand)

        # user prompt
        taskQueue.addTask(self.userPrompt('sending neck pitch down. continue? y/n: '))

        # set neck pitch
        taskQueue.addTask(self.printAsync('neck pitch down'))
        taskQueue.addTask(self.sendNeckPitchLookDown)
        taskQueue.addTask(self.delay(1.0))

        # user prompt
        taskQueue.addTask(self.userPrompt('perception and fitting. continue? y/n: '))

        # perception & fitting
        #taskQueue.addTask(self.printAsync('waiting for clean lidar sweep'))
        #taskQueue.addTask(self.waitForCleanLidarSweepAsync)
        #taskQueue.addTask(self.printAsync('user fit affordance'))
        #taskQueue.addTask(self.pauseQueue)

        # compute grasp & stance
        taskQueue.addTask(self.printAsync('grasp search'))
        taskQueue.addTask(self.search)

        # user select end pose
        taskQueue.addTask(self.printAsync('user select end pose'))
        #taskQueue.addTask(self.pauseQueue)

        # compute pre grasp plan
        taskQueue.addTask(self.preGrasp)

        # user prompt
        taskQueue.addTask(self.userPrompt('commit manip plan. continue? y/n: '))

        # commit pre grasp plan
        taskQueue.addTask(self.printAsync('commit pre grasp plan'))
        taskQueue.addTask(self.commit)
        taskQueue.addTask(self.delay(10.0))


        # perception & fitting
        taskQueue.addTask(self.printAsync('user fit affordance'))
        taskQueue.addTask(self.toggleAffordanceEdit)
        taskQueue.addTask(self.pauseQueue)

        #taskQueue.addTask(self.printAsync('waiting for clean lidar sweep'))
        #taskQueue.addTask(self.waitForCleanLidarSweepAsync)
        #taskQueue.addTask(self.printAsync('refit affordance'))
        #taskQueue.addTask(self.refitFunction)


        # compute pre grasp plan
        taskQueue.addTask(self.grasp)

        # user prompt
        taskQueue.addTask(self.userPrompt('commit manip plan. continue? y/n: '))

        # commit pre grasp plan
        taskQueue.addTask(self.printAsync('commit grasp plan'))
        taskQueue.addTask(self.commit)
        taskQueue.addTask(self.delay(10.0))

        # user prompt
        taskQueue.addTask(self.userPrompt('closing hand. continue? y/n: '))

        # close hand
        taskQueue.addTask(self.printAsync('close hand'))
        taskQueue.addTask(self.closeHand)
        taskQueue.addTask(self.delay(2.0))
        taskQueue.addTask(self.closeHand)

        # compute retract plan
        taskQueue.addTask(self.retract)

        # user prompt
        taskQueue.addTask(self.userPrompt('commit manip plan. continue? y/n: '))

        # commit pre grasp plan
        taskQueue.addTask(self.printAsync('commit retract plan'))
        taskQueue.addTask(self.commit)
        taskQueue.addTask(self.delay(0.1))
        taskQueue.addTask(self.closeHand)
        taskQueue.addTask(self.delay(0.1))
        taskQueue.addTask(self.closeHand)
        taskQueue.addTask(self.delay(0.2))
        taskQueue.addTask(self.closeHand)
        taskQueue.addTask(self.delay(0.2))
        taskQueue.addTask(self.closeHand)
        taskQueue.addTask(self.delay(0.2))
        taskQueue.addTask(self.closeHand)
        taskQueue.addTask(self.delay(0.2))
        taskQueue.addTask(self.closeHand)
        taskQueue.addTask(self.delay(1.0))
        taskQueue.addTask(self.closeHand)
        taskQueue.addTask(self.delay(1.0))
        taskQueue.addTask(self.closeHand)
        taskQueue.addTask(self.delay(1.0))
        taskQueue.addTask(self.closeHand)
        taskQueue.addTask(self.delay(6.0))


        # compute extend arm plan
        taskQueue.addTask(self.extendArm)

        # user prompt
        taskQueue.addTask(self.userPrompt('commit manip plan. continue? y/n: '))

        # commit pre grasp plan
        taskQueue.addTask(self.printAsync('commit extend arm plan'))
        taskQueue.addTask(self.commit)
        taskQueue.addTask(self.delay(10.0))

        # user prompt
        taskQueue.addTask(self.userPrompt('opening hand. continue? y/n: '))

        # open hand
        taskQueue.addTask(self.printAsync('open hand'))
        taskQueue.addTask(self.openHand)
        taskQueue.addTask(self.delay(2.0))

        # compute nominal pose plan
        taskQueue.addTask(self.drop)

        # user prompt
        taskQueue.addTask(self.userPrompt('commit manip plan. continue? y/n: '))

        # commit pre grasp plan
        taskQueue.addTask(self.printAsync('commit nominal pose plan'))
        taskQueue.addTask(self.commit)
        taskQueue.addTask(self.delay(10.0))


        taskQueue.addTask(self.printAsync('done!'))

        return taskQueue
