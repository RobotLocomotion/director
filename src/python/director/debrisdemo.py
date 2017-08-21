import director.objectmodel as om
from director.asynctaskqueue import AsyncTaskQueue


class GraspSearchPlanner(object):

    def __init__(self, ikPlanner, robotModel, jointController, sensorJointController, planPlaybackFunction, showPoseFunction, playbackRobotModel):

        self.ikPlanner = ikServer
        self.robotModel = robotModel
        self.jointController = jointController
        self.sensorJointController = sensorJointController

        self.planPlaybackFunction = planPlaybackFunction
        self.showPoseFunction = showPoseFunction
        self.playbackRobotModel = playbackRobotModel

        self.endPoses = []
        self.affordanceName = 'board'
        self.affordance = None
        self.handModels = []

        self.reachingSide = 'left'
        self.graspSample = 0

        self.handToUtorso = [0.2, 0.7, 0.0]

        self.planFromCurrentRobotState = True

        self.tspanPreReach = [0.35, 0.35]
        self.tspanFull = [0.0, 1.0]
        self.tspanPreGrasp = [0.7, 0.7]
        self.tspanPreGraspToEnd = [0.7, 1.0]
        self.tspanStart = [0.0, 0.0]
        self.tspanEnd = [1.0, 1.0]


    def playManipPlan(self):
        self.planPlaybackFunction([self.lastManipPlan])


    def showPreGraspEndPose(self):
        self.showPoseFunction(self.jointController.getPose('pre_grasp_end_pose'))


    def showGraspEndPose(self):
        self.showPoseFunction(self.jointController.getPose('grasp_end_pose'))


    def computePreGraspTraj(self):
        self.computeGraspTraj(poseStart='q_start', poseEnd='pre_grasp_end_pose', timeSamples=[0.0, 0.35, 0.7])


    def computeEndGraspTraj(self):
        self.computeGraspTraj(poseStart='pre_grasp_end_pose', poseEnd='grasp_end_pose', timeSamples=[0.7, 1.0])


    def computeGroundFrame(self, robotModel):
        '''
        Given a robol model, returns a vtkTransform at a position between
        the feet, on the ground, with z-axis up and x-axis aligned with the
        robot pelvis x-axis.
        '''
        t1 = robotModel.getLinkFrame( self.ikPlanner.leftFootLink )
        t2 = robotModel.getLinkFrame( self.ikPlanner.rightFootLink )
        pelvisT = robotModel.getLinkFrame( self.ikPlanner.pelvisLink )

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


    def randomAffordance(self, robotModel):
        aff = self.findAffordance()
        if aff:
            om.removeFromObjectModel(aff)
        self.spawnAffordance(robotModel, randomize=True)


    def spawnAffordance(self, robotModel, randomize=False):

        if randomize:

            position = [random.uniform(0.5, 0.8), random.uniform(-0.2, 0.2), random.uniform(0.5, 0.8)]
            rpy = [random.choice((random.uniform(-35, 35), random.uniform(70, 110))), random.uniform(-10, 10),  random.uniform(-5, 5)]
            zwidth = random.uniform(0.5, 1.0)

        else:

            position = [0.65, 0.0, 0.6]
            rpy = [25, 1, 0]
            zwidth = 24 * .0254

        xwidth = 3.75 * .0254
        ywidth = 1.75 * .0254
        t = transformUtils.frameFromPositionAndRPY(position, rpy)
        t.Concatenate(self.computeGroundFrame(robotModel))
        xaxis = [1,0,0]
        yaxis = [0,1,0]
        zaxis = [0,0,1]
        for axis in (xaxis, yaxis, zaxis):
            t.TransformVector(axis, axis)

        affordance = segmentation.createBlockAffordance(t.GetPosition(), xaxis, yaxis, zaxis, xwidth, ywidth, zwidth, 'board', parent='affordances')
        affordance.setProperty('Color', QtGui.QColor(200, 150, 100))
        t = affordance.actor.GetUserTransform()
        affordanceFrame = vis.showFrame(t, 'board frame', parent=affordance, visible=False, scale=0.2)


    def updateHandModel(self):
        graspFrame = self.getAffordanceChild('desired grasp frame')
        handMesh = self.findAffordanceChild('desired grasp hand')
        if not handMesh:
            handMesh = self.getHandModel().newPolyData('desired grasp hand', self.robotModel.views[0], parent=self.findAffordance())
        handFrame = handMesh.children()[0]
        handFrame.copyFrame(graspFrame.transform)

    def findAffordance(self):
        self.affordance = om.findObjectByName(self.affordanceName)
        return self.affordance


    def findAffordanceChild(self, name):
        assert self.affordance
        return self.affordance.findChild(name)


    def getAffordanceChild(self, name):
        child = self.findAffordanceChild(name)
        if not child:
            raise Exception('Failed to locate affordance child: %s' % name)
        return child


    def getAffordanceFrame(self):
        self.findAffordance()
        assert self.affordance
        affordanceName = self.affordance.getProperty('Name')
        return self.getAffordanceChild('%s frame' % affordanceName)


    def computeGraspFrameSamples(self):

        if self.affordanceName == 'board':
            self.computeGraspFrameSamplesBoard()
        else:
            self.getAffordanceChild('sample grasp frame 0')


    def computeGraspFrameSamplesBoard(self):

        affordanceFrame = self.getAffordanceFrame()

        additionalOffset = 0.0
        yoffset = 0.5*self.affordance.params['ywidth'] + additionalOffset
        xoffset = 0.5*self.affordance.params['xwidth'] + additionalOffset

        frames = [
          [[0.0, yoffset, 0.0], [0.0, 90, 180.0]],
          [[0.0, yoffset, 0.0], [0.0, -90, 180.0]],

          [[0.0, -yoffset, 0.0], [0.0, 90, 0.0]],
          [[0.0, -yoffset, 0.0], [0.0, -90, 0.0]],

          [[xoffset, 0.0, 0.0], [-90, -90, 180.0]],
          [[xoffset, 0.0, 0.0], [90, 90, 180.0]],

          [[-xoffset, 0.0, 0.0], [90, -90, 180.0]],
          [[-xoffset, 0.0, 0.0], [-90, 90, 180.0]],
          ]

        for i, frame in enumerate(frames):
            pos, rpy = frame
            t = transformUtils.frameFromPositionAndRPY(pos, rpy)
            t.Concatenate(affordanceFrame.transform)
            name = 'sample grasp frame %d' % i
            om.removeFromObjectModel(self.findAffordanceChild(name))
            vis.showFrame(copyFrame(t), name, parent=self.affordance, visible=False, scale=0.2)


    def computeGraspFrame(self):
        frame = self.getAffordanceChild('sample grasp frame %d' % self.graspSample)
        name = 'grasp frame'
        om.removeFromObjectModel(self.findAffordanceChild(name))
        vis.showFrame(copyFrame(frame.transform), name, parent=self.affordance, visible=False, scale=0.2)


    def createSearchGraspConstraints(self):
        if self.affordanceName == 'board':
            return self.createSearchGraspConstraintsBoard()
        else:
            targetFrame = self.getAffordanceChild('grasp frame')
            return self.createPositionOrientationGraspConstraints(self.reachingSide, targetFrame, positionTolerance=0.0025, angleToleranceInDegrees=1.0)


    def createSearchGraspConstraintsBoard(self):

        targetFrame = self.getAffordanceChild('grasp frame')
        boardHalfLength = self.affordance.params['zwidth']/2.0 - 0.08

        graspPosition, graspOrientation = self.createPositionOrientationGraspConstraints(self.reachingSide, targetFrame, positionTolerance=0.0025, angleToleranceInDegrees=1.0)
        graspPosition.lowerBound = np.array([-boardHalfLength, 0.0, 0.0])
        graspPosition.upperBound = np.array([boardHalfLength, 0.0, 0.0])

        return graspPosition, graspOrientation


    def createRetractGraspConstraints(self):

        targetFrame = self.getAffordanceChild('desired grasp frame')

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(targetFrame.transform)
        t.Translate(0.0, 0.0, 0.25)
        retractFrame = vis.updateFrame(copyFrame(t), 'retract frame', scale=0.2, visible=False, parent=self.affordance)

        return self.createPositionOrientationGraspConstraints(self.reachingSide, retractFrame, positionTolerance=0.03, angleToleranceInDegrees=5.0)


    def createGraspConstraints(self):
        targetFrame = self.getAffordanceChild('desired grasp frame')
        return self.createPositionOrientationGraspConstraints(self.reachingSide, targetFrame, positionTolerance=0.005, angleToleranceInDegrees=3.0)


    def createPreGraspConstraints(self):
        targetFrame = self.getAffordanceChild('pre grasp frame')
        return self.createPositionOrientationGraspConstraints(self.reachingSide, targetFrame, positionTolerance=0.02, angleToleranceInDegrees=7.0)


    def createPreReachConstraint(self):
        handToUtorso = np.array(self.handToUtorso)
        if self.reachingSide == 'right':
            handToUtorso[1] *= -1
        return self.createHandRelativePositionConstraint(self, self.reachSide, 'utorso', handToUtorso)


    def computeGraspEndPoseSearch(self):

        startPoseName = 'q_start'

        constraints = []
        constraints.extend(self.createSearchGraspConstraints())
        constraints.extend(self.createMovingReachConstraints(startPoseName))

        self.graspEndPose, self.graspEndPoseInfo = self.ikServer.runIk(constraints)

        self.ikServer.sendPoseToServer(self.graspEndPose, 'grasp_end_pose')
        self.jointController.setPose('grasp_end_pose', self.graspEndPose)

        print('grasp end pose info:', self.graspEndPoseInfo)

    def computeGraspEndPoseFrames(self):

        graspFrame = self.getAffordanceChild('grasp frame')
        affordanceFrame = self.getAffordanceFrame()

        self.jointController.setPose('grasp_end_pose', self.graspEndPose)
        handFrame = self.robotModel.getLinkFrame(self.getHandLink())

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(self.getPalmToHandLink())
        t.Concatenate(handFrame)
        graspEndPoseFrame = t
        vis.updateFrame(t, 'grasp frame (ik result with tolerance)', scale=0.2, visible=False, parent=self.affordance)

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(graspFrame.transform)
        t.Translate(np.array(graspEndPoseFrame.GetPosition()) - np.array(graspFrame.transform.GetPosition()))
        t.Concatenate(affordanceFrame.transform.GetLinearInverse())
        self.affordanceToGrasp = copyFrame(t)


        def updateAffordanceToGrasp(frame):
            affordanceFrame = self.getAffordanceFrame()
            t = vtk.vtkTransform()
            t.PostMultiply()
            t.Concatenate(frame.transform)
            t.Concatenate(affordanceFrame.transform.GetLinearInverse())
            self.affordanceToGrasp = copyFrame(t)
            self.updateHandModel()


        def updateGraspFrame(frame, create=False):

            graspFrame = self.findAffordanceChild('desired grasp frame')
            if not graspFrame and not create:
                frame.onTransformModifiedCallback = None
                return

            t = vtk.vtkTransform()
            t.PostMultiply()
            t.Concatenate(self.affordanceToGrasp)
            t.Concatenate(frame.transform)

            if graspFrame:
                graspFrame.onTransformModifiedCallback = None
            graspFrame = vis.updateFrame(copyFrame(t), 'desired grasp frame', scale=0.2, visible=False, parent=self.affordance)
            graspFrame.onTransformModifiedCallback = updateAffordanceToGrasp
            self.updateHandModel()
            return graspFrame

        self.lockAffordanceToHand = False

        def onRobotModelChanged(model):
            handFrame = self.playbackRobotModel.getLinkFrame(self.getHandLink())
            t = vtk.vtkTransform()
            t.PostMultiply()
            t.Concatenate(self.getPalmToHandLink())
            t.Concatenate(handFrame)
            palmFrame = vis.updateFrame(t, 'palm frame', scale=0.2, visible=False, parent=self.affordance)

            if self.lockAffordanceToHand:
                t = vtk.vtkTransform()
                t.PostMultiply()
                t.Concatenate(self.affordanceToGrasp.GetLinearInverse())
                t.Concatenate(palmFrame.transform)
                affordanceFrame = self.getAffordanceFrame()
                affordanceFrame.copyFrame(t)

        self.playbackRobotModel.connectModelChanged(onRobotModelChanged)

        graspFrame = updateGraspFrame(affordanceFrame, create=True)
        affordanceFrame.onTransformModifiedCallback = updateGraspFrame


    def computePreGraspFrame(self, preGraspDistance=0.20):

        graspFrame = self.getAffordanceChild('desired grasp frame')

        pos = [0.0, -preGraspDistance, 0.0]
        rpy = [0.0, 0.0, 0.0]
        preGraspToGrasp = transformUtils.frameFromPositionAndRPY(pos, rpy)

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(preGraspToGrasp)
        t.Concatenate(graspFrame.transform)
        vis.updateFrame(copyFrame(t), 'pre grasp frame', scale=0.2, visible=False, parent=self.affordance)


    def computeGraspEndPose(self):

        startPoseName = 'q_start'

        constraints = []
        constraints.extend(self.createMovingReachConstraints(startPoseName))
        constraints.extend(self.createGraspConstraints())

        self.graspEndPose, info = self.ikServer.runIk(constraints)

        self.ikServer.sendPoseToServer(self.graspEndPose, 'grasp_end_pose')
        self.jointController.setPose('grasp_end_pose', self.graspEndPose)

        print('grasp end pose info:', info)


    def commitState(self):
        poseTimes, poses = planplayback.PlanPlayback.getPlanPoses(self.lastManipPlan)
        self.sensorJointController.setPose('EST_ROBOT_STATE', poses[-1])


    def computePreGraspAdjustment(self):

        assert self.planFromCurrentRobotState
        startPose = np.array(self.sensorJointController.q)

        startPoseName = 'reach_start'
        self.jointController.addPose(startPoseName, startPose)
        self.computePostureGoal(startPoseName, 'pre_grasp_end_pose')


    def computeGraspReach(self):

        if self.planFromCurrentRobotState:
            startPose = np.array(self.sensorJointController.q)
        else:
            startPose = np.array(self.jointController.getPose('pre_grasp_end_pose'))

        startPoseName = 'reach_start'
        self.jointController.addPose(startPoseName, startPose)
        self.ikServer.sendPoseToServer(startPose, startPoseName)


        constraints = []
        constraints.extend(self.createGraspConstraints())
        constraints.append(self.createLockedTorsoPostureConstraint(startPoseName))
        constraints.append(self.createLockedArmPostureConstraint(startPoseName))

        endPose, info = self.ikServer.runIk(constraints, seedPostureName=startPoseName)

        print('grasp reach info:', info)

        self.jointController.addPose('reach_end', endPose)
        self.computePostureGoal(startPoseName, 'reach_end')


    def computeRetractTraj(self):

        if self.planFromCurrentRobotState:
            startPose = np.array(self.sensorJointController.q)
        else:
            startPose = np.array(self.jointController.getPose('grasp_end_pose'))

        startPoseName = 'retract_start'
        self.jointController.addPose(startPoseName, startPose)
        self.ikServer.sendPoseToServer(startPose, startPoseName)

        constraints = []
        constraints.extend(self.createMovingReachConstraints(startPoseName))

        graspPosition, graspOrientation = self.createRetractGraspConstraints()
        graspPosition.tspan = self.tspanEnd
        graspOrientation.tspan = self.tspanEnd

        constraints.extend([
            graspPosition,
            graspOrientation,
            ])


        endPose, info = self.ikServer.runIk(constraints, seedPostureName=startPoseName)
        print('retract info:', info)

        self.jointController.addPose('retract_end', endPose)
        self.computePostureGoal(startPoseName, 'retract_end')

        #self.runIkTraj(constraints, startPoseName, startPoseName, timeSamples)


    def computeArmExtend(self):

        if self.planFromCurrentRobotState:
            startPose = np.array(self.sensorJointController.q)
        else:
            startPose = np.array(self.jointController.getPose('grasp_end_pose'))

        startPoseName = 'retract_start'
        self.jointController.addPose(startPoseName, startPose)
        self.ikServer.sendPoseToServer(startPose, startPoseName)

        constraints = []
        constraints.extend(self.createFixedFootConstraints(startPoseName))
        constraints.append(self.createKneePostureConstraint([0.4, 0.4]))
        constraints.append(self.createMovingBasePostureConstraint(startPoseName))
        constraints.append(self.createLockedArmPostureConstraint(startPoseName))
        constraints.append(self.createPostureConstraint('q_nom', robotstate.matchJoints('back')))

        movingArmJoints = 'l_arm' if self.reachingSide == 'left' else 'r_arm'
        constraints.append(self.createPostureConstraint('q_zero', robotstate.matchJoints(movingArmJoints)))

        endPose, info = self.ikServer.runIk(constraints, seedPostureName=startPoseName)

        print('retract info:', info)

        self.jointController.addPose('retract_end', endPose)
        self.computePostureGoal(startPoseName, 'retract_end')


    def computePreGraspEndPose(self):

        constraints = []
        constraints.extend(self.createPreGraspConstraints())
        constraints.extend(self.createMovingReachConstraints('grasp_end_pose', lockBack=True, lockBase=True, lockArm=True))

        self.preGraspEndPose, self.preGraspEndPoseInfo = self.ikServer.runIk(constraints)

        self.ikServer.sendPoseToServer(self.preGraspEndPose, 'pre_grasp_end_pose')
        self.jointController.addPose('pre_grasp_end_pose', self.preGraspEndPose)

        print('pre grasp end pose info:', self.preGraspEndPoseInfo)


    def computeGraspTraj(self, poseStart='q_start', poseEnd='grasp_end_pose', timeSamples=None):

        constraints = []
        constraints.extend(self.createMovingReachConstraints(poseStart))

        movingBaseConstraint = constraints[-2]
        assert isinstance(movingBaseConstraint, ik.PostureConstraint)
        assert 'base_x' in movingBaseConstraint.joints
        movingBaseConstraint.tspan = [self.tspanStart[0], self.tspanPreGrasp[1]]

        preReachPosition = self.createPreReachConstraint()
        preReachPosition.tspan = self.tspanPreReach

        graspPosture = self.createLockedTorsoPostureConstraint('grasp_end_pose')
        graspPosture.tspan = self.tspanPreGraspToEnd

        preGraspPosition, preGraspOrientation = self.createPreGraspConstraints()
        preGraspPosition.tspan = self.tspanPreGrasp
        preGraspOrientation.tspan = self.tspanPreGrasp

        graspPosition, graspOrientation = self.createGraspConstraints()
        graspPosition.tspan = self.tspanEnd
        graspOrientation.tspan = self.tspanEnd

        constraints.extend([
            preReachPosition,
            graspPosture,
            preGraspPosition,
            preGraspOrientation,
            graspPosition,
            graspOrientation,
            ])

        if timeSamples is None:
            timeSamples=[0.0, 0.35, 0.7, 1.0]

        self.runIkTraj(constraints, poseStart, poseEnd, timeSamples)

    def useGraspEndPoseOption(self, index):

        side, graspSample = self.endPoses[index][3]
        self.reachingSide = side
        self.graspSample = graspSample
        self.updateGraspEndPose()
        self.showGraspEndPose()


    def computeInitialState(self):

        if self.planFromCurrentRobotState:
            startPose = np.array(self.sensorJointController.q)
        else:
            startPose = np.array(self.jointController.getPose('q_nom'))

        self.ikServer.sendPoseToServer(startPose, 'q_start')
        self.jointController.addPose('q_start', startPose)


    def updateGraspEndPose(self, enableSearch=True):

        self.computeInitialState()
        self.findAffordance()

        if enableSearch:
            om.removeFromObjectModel(self.findAffordanceChild('desired grasp frame'))
            om.removeFromObjectModel(self.findAffordanceChild('desired grasp hand'))

        if not self.findAffordanceChild('desired grasp frame'):
            self.computeGraspFrameSamples()
            self.computeGraspFrame()
            self.computeGraspEndPoseSearch()
            self.computeGraspEndPoseFrames()
        else:
            self.computeGraspEndPose()

        self.computePreGraspFrame()
        self.computePreGraspEndPose()


    def endPoseSearch(self):

        self.findAffordance()
        self.computeGraspFrameSamples()
        self.endPoses = []

        for side in ['left', 'right']:
        #for side in ['left']:

            sampleCount = 0

            while self.findAffordanceChild('sample grasp frame %d' % sampleCount):
                self.reachingSide = side
                self.graspSample = sampleCount
                sampleCount += 1

                self.updateGraspEndPose()

                if self.graspEndPoseInfo == 1 and self.preGraspEndPoseInfo == 1:
                    params = [self.reachingSide, self.graspSample]
                    score = self.computePostureCost(self.graspEndPose)
                    print('score:', score)
                    print('params:', self.reachingSide, self.graspSample)
                    self.endPoses.append((score, self.graspEndPose, self.preGraspEndPose, params))

        if not self.endPoses:
            print('failed to find suitable grasp end pose')
            return 0

        self.endPoses.sort(key=lambda x: x[0])
        self.useGraspEndPoseOption(0)

        print('\n\nfound %d suitable end poses' % len(self.endPoses))
        return len(self.endPoses)



class DebrisPlannerDemo(object):

    def __init__(self, robotStateModel, sensorJointController, playbackRobotModel, ikPlanner, manipPlanner, atlasDriver, handDriver, multisenseDriver, refitFunction):

        self.robotStateModel = robotStateModel
        self.sensorJointController = sensorJointController
        self.playbackRobotModel = playbackRobotModel
        self.ikPlanner = ikPlanner
        self.manipPlanner = manipPlanner
        self.atlasDriver = atlasDriver
        self.handDriver = handDriver
        self.multisenseDriver = multisenseDriver
        self.refitFunction = refitFunction

        self.planFromCurrentRobotState = True
        self.userPromptEnabled = True
        self.visOnly = True

    def reset(self):
        self.ikPlanner.lockAffordanceToHand = False
        self.ikPlanner.randomAffordance(self.robotStateModel)

    def playManipPlan(self):
        self.playManipPlan()
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
        startPose = self.getPlanningStartPose()
        self.ikPlanner.computeStandPlan(startPose)
        self.playManipPlan()

    def extendArm(self):
        self.ikPlanner.lockAffordanceToHand = True
        self.ikPlanner.computeArmExtend()
        self.playManipPlan()

    def drop(self):
        self.ikPlanner.lockAffordanceToHand = False
        om.removeFromObjectModel(self.ikPlanner.affordance)
        self.nominal()

    def nominal(self):
        startPose = self.getPlanningStartPose()
        self.ikPlanner.computeNominalPlan(startPose)
        self.playManipPlan()

    def toggleAffordanceEdit(self):
        aff = self.ikPlanner.findAffordance()
        frame = self.ikPlanner.getAffordanceFrame()
        edit = not frame.getProperty('Edit')
        frame.setProperty('Edit', edit)
        aff.setProperty('Alpha', 0.5 if edit else 1.0)

    def getEstimatedRobotStatePose(self):
        return self.sensorJointController.getPose('EST_ROBOT_STATE')

    def getPlanningStartPose(self):
        if self.planFromCurrentRobotState:
            return self.getEstimatedRobotStatePose()
        else:
            assert False

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
        assert behaviorName in list(self.atlasDriver.getBehaviorMap().values())
        while self.atlasDriver.getCurrentBehaviorName() != behaviorName:
            yield


    def printAsync(self, s):
        yield
        print(s)


    def userPrompt(self, message):

        if not self.userPromptEnabled:
            return

        yield
        result = input(message)
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
