import ddapp
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp import objectmodel as om
from ddapp import ik

import os
import functools
import numpy as np
import scipy.io


class KinematicPosePlanner(object):

    def __init__(self, ikServer):
        self.ikServer = ikServer

    def getInitCommands(self):

      commands = ['''
        addpath([getenv('DRC_BASE'), '/software/control/matlab/planners/prone']);
        kptRobot = s.robot;
        kptRobot = kptRobot.setTerrain(RigidBodyFlatTerrain());
        kptRobot = kptRobot.removeFromIgnoredListOfCollisionFilterGroup({'r_uleg'},'l_uleg');
        kptRobot = kptRobot.removeFromIgnoredListOfCollisionFilterGroup({'l_uleg'},'r_uleg');
        kptRobot = compile(kptRobot);
        kpt = KinematicPoseTrajectory(kptRobot);
      ''']

      return commands

    def init(self):
        commands = self.getInitCommands()
        self.ikServer.taskQueue.addTask(functools.partial(self.ikServer.comm.sendCommandsAsync, commands))
        self.ikServer.taskQueue.start()

    def findPose(self, contacts, positionCosts, nominalPose):

        commands = []
        commands.append('contacts = {%s};' % ', '.join(["'%s'" % c for c in contacts]))
        commands.append("position_cost = struct('name',{},'position',{}, 'scale', {});")

        for i, positionCost in enumerate(positionCosts):

            linkName, position, scale = positionCost
            commands.append("position_cost(%d).name = '%s';" % (i+1, linkName))
            commands.append("position_cost(%d).position = %s;" % (i+1, ik.ConstraintBase.toColumnVectorString(position)))
            commands.append("position_cost(%d).scale = %r;" % (i+1, float(scale)))

        commands.append('kpt_q_nom = %s;' % ik.ConstraintBase.toColumnVectorString(nominalPose))
        commands.append('\n')
        commands.append('clear kpt_q_end;')
        commands.append('clear F;')
        commands.append('clear info;')
        commands.append('clear infeasible_constraint;')
        commands.append('\n')
        commands.append('[kpt_q_end,F,info,infeasible_constraint] = kpt.findPose(contacts, position_cost, kpt_q_nom);');
        commands.append('\n')
        commands.append('if (info > 10) display(infeasibleConstraintMsg(infeasible_constraint)); end;')

        self.ikServer.taskQueue.addTask(functools.partial(self.ikServer.comm.sendCommandsAsync, commands))
        self.ikServer.taskQueue.start()


class KinematicPosePlannerPanel(object):

    def __init__(self, kinematicPosePlanner, ikServer, teleopJointController):
        self.kinematicPosePlanner = kinematicPosePlanner
        self.ikServer = ikServer
        self.teleopJointController = teleopJointController
        self._findPoseQueued = False
        self._findPosePending = False

    def testDemo(self):

        postureFile = os.path.join(ddapp.getDRCBaseDir(), 'software/control/matlab/planners/prone/data_KPT.mat')
        self.nominalPosture = scipy.io.loadmat(postureFile)['q_sol'][:,0]
        self.contacts = ['l_toe', 'l_knee', 'r_toe', 'r_knee', 'l_hand']
        self.positionCosts = [
          ['l_hand', [0.061, 0.703, 0], 1.0],
          ['r_hand', [-0.0295, -1.09, 0], 4.0],
            ]

        self.makeGoalFrames()
        self.findPose()

    def onGoalFrameModified(self, frame):
        self.findPose()

    def makeGoalFrames(self):

        for linkName, positionGoal, _ in self.positionCosts:
            orientationGoal = [0.0, 0.0, 0.0]
            t = transformUtils.frameFromPositionAndRPY(positionGoal, orientationGoal)
            f = vis.showFrame(t, '%s position goal' % linkName)
            f.connectFrameModified(self.onGoalFrameModified)


    def updatePositionCostsFromGoalFrames(self):

        for i in xrange(len(self.positionCosts)):
            linkName = self.positionCosts[i][0]
            self.positionCosts[i][1] = om.findObjectByName('%s position goal' % linkName).transform.GetPosition()

    def findPose(self):

        if self._findPosePending:
            if not self._findPoseQueued:
                self._findPoseQueued = True
                self.ikServer.taskQueue.addTask(self.findPose)
                self.ikServer.taskQueue.start()
            return

        self._findPosePending = True

        self.updatePositionCostsFromGoalFrames()
        self.kinematicPosePlanner.findPose(self.contacts, self.positionCosts, self.nominalPosture)

        def onPose():

            pose = self.ikServer.comm.getFloatArray('kpt_q_end')
            self.teleopJointController.setPose('kpt_q_end', pose)
            self._findPosePending = False
            self._findPoseQueued = False

        self.ikServer.taskQueue.addTask(onPose)
        self.ikServer.taskQueue.start()
