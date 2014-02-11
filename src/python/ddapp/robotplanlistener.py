import os
import vtkAll as vtk
from ddapp import botpy
import math
import time
import numpy as np

from ddapp import transformUtils
from ddapp import lcmUtils
from ddapp.timercallback import TimerCallback
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp import applogic as app
from ddapp.debugVis import DebugData
from ddapp import ioUtils
from ddapp.simpletimer import SimpleTimer
from ddapp.utime import getUtime
from ddapp import robotstate
from ddapp import botpy

import drc as lcmdrc

import pickle

import scipy.interpolate


class ManipulationPlanDriver(object):

    def __init__(self):
        lcmUtils.addSubscriber('CANDIDATE_MANIP_PLAN', lcmdrc.robot_plan_w_keyframes_t, self.onManipPlan)
        lcmUtils.addSubscriber('CANDIDATE_ROBOT_ENDPOSE', lcmdrc.robot_state_t, self.onRobotEndPose)
        self.lastManipPlan = None
        self.lastRobotEndPose = None
        self.manipPlanCallback = None
        self.endPoseCallback = None

    def onManipPlan(self, msg):
        self.lastManipPlan = msg
        if self.manipPlanCallback:
            self.manipPlanCallback()

    def onRobotEndPose(self, msg):
        self.lastRobotEndPose = msg
        if self.endPoseCallback:
            self.endPoseCallback()

    def convertKeyframePlan(self, keyframeMsg):
        msg = lcmdrc.robot_plan_t()
        msg.utime = keyframeMsg.utime
        msg.robot_name = keyframeMsg.robot_name
        msg.num_states = keyframeMsg.num_states
        msg.plan = keyframeMsg.plan
        msg.plan_info = keyframeMsg.plan_info
        msg.num_bytes = keyframeMsg.num_bytes
        msg.matlab_data = keyframeMsg.matlab_data
        msg.num_grasp_transitions = keyframeMsg.num_grasp_transitions

        msg.left_arm_control_type = msg.NONE
        msg.right_arm_control_type = msg.NONE
        msg.left_leg_control_type = msg.NONE
        msg.right_leg_control_type = msg.NONE

        return msg


    def commitManipPlan(self, manipPlan):

        if isinstance(manipPlan, lcmdrc.robot_plan_w_keyframes_t):
            manipPlan = self.convertKeyframePlan(manipPlan)
        manipPlan.utime = getUtime()
        lcmUtils.publish('COMMITTED_ROBOT_PLAN', manipPlan)


    def sendPlannerModeControl(self, mode='fixed_joints'):
        msg = lcmdrc.grasp_opt_mode_t()
        msg.utime = getUtime()
        msg.mode = {'fixed_joints' : 3}[mode]
        lcmUtils.publish('MANIP_PLANNER_MODE_CONTROL', msg)


    def sendJointSpeedLimit(self, speedLimit=30):

        assert speedLimit > 0 and speedLimit < 50
        msg = lcmdrc.plan_execution_speed_t()
        msg.speed = math.radians(speedLimit)
        msg.utime = getUtime()
        lcmUtils.publish('DESIRED_JOINT_SPEED', msg)


    def sendEEArcSpeedLimit(self, speedLimit=0.30):

        assert speedLimit > 0 and speedLimit < 0.50
        msg = lcmdrc.plan_execution_speed_t()
        msg.utime = getUtime()
        msg.speed = speedLimit
        lcmUtils.publish('DESIRED_EE_ARC_SPEED', msg)


    def clearEndEffectorGoals(self):
        msg = lcmdrc.ee_goal_t()
        msg.ee_goal_pos = lcmdrc.position_3d_t()
        msg.ee_goal_pos.translation = lcmdrc.vector_3d_t()
        msg.ee_goal_pos.rotation = lcmdrc.quaternion_t()
        msg.ee_goal_twist = lcmdrc.twist_t()
        msg.ee_goal_twist.linear_velocity = lcmdrc.vector_3d_t()
        msg.ee_goal_twist.angular_velocity = lcmdrc.vector_3d_t()

        lcmUtils.publish('LEFT_PALM_GOAL_CLEAR', msg)
        lcmUtils.publish('RIGHT_PALM_GOAL_CLEAR', msg)


    def sendPlannerSettings(self, initialPose):
        '''
        This will be removed when the planner lcm messages are updated to
        take parameters and inputs.
        '''

        stateMessage = robotstate.drakePoseToRobotState(initialPose)
        lcmUtils.publish('EST_ROBOT_STATE_REACHING_PLANNER', stateMessage)
        time.sleep(0.1)

        self.clearEndEffectorGoals()
        self.sendPlannerModeControl()
        self.sendJointSpeedLimit()
        self.sendEEArcSpeedLimit()
        time.sleep(0.05)


    def sendPoseGoal(self, startPose, goalPoseJoints, waitForResponse=False, waitTimeout=5000):

        msg = lcmdrc.joint_angles_t()
        msg.utime = getUtime()
        for name, position in goalPoseJoints.iteritems():
            msg.joint_name.append(name)
            msg.joint_position.append(position)
        msg.num_joints = len(msg.joint_name)

        requestChannel='POSTURE_GOAL'
        responseChannel = 'CANDIDATE_MANIP_PLAN'

        self.sendPlannerSettings(startPose)

        if waitForResponse:
            if waitTimeout == 0:
                helper = lcmUtils.MessageResponseHelper(responseChannel, lcmdrc.robot_plan_w_keyframes_t)
                lcmUtils.publish(requestChannel, msg)
                return helper
            return lcmUtils.MessageResponseHelper.publishAndWait(requestChannel, msg,
                                    responseChannel, lcmdrc.robot_plan_w_keyframes_t, waitTimeout)
        else:
            lcmUtils.publish(requestChannel, msg)


    def sendEndPoseGoal(self, startPose, linkName, goalInWorldFrame, waitForResponse=False, waitTimeout=5000):

        msg = lcmdrc.traj_opt_constraint_t()
        msg.utime = getUtime()

        msg.num_joints = 0
        msg.joint_name = []
        msg.joint_position = []
        msg.joint_timestamps = []

        msg.num_links = 1
        msg.link_name = [linkName]
        msg.link_timestamps = [0]
        msg.link_origin_position = [transformUtils.positionMessageFromFrame(goalInWorldFrame)]

        requestChannel='POSE_GOAL'
        responseChannel = 'CANDIDATE_ROBOT_ENDPOSE'

        self.sendPlannerSettings(startPose)

        if waitForResponse:
            return lcmUtils.MessageResponseHelper.publishAndWait(requestChannel, msg,
                                    responseChannel, lcmdrc.robot_state_t, waitTimeout)
        else:
            lcmUtils.publish(requestChannel, msg)


    def sendEndEffectorGoal(self, startPose, linkName, goalInWorldFrame, waitForResponse=False, waitTimeout=5000):

        msg = lcmdrc.ee_goal_t()
        msg.utime = getUtime()
        msg.ee_goal_pos = transformUtils.positionMessageFromFrame(goalInWorldFrame)
        msg.ee_goal_twist = lcmdrc.twist_t()
        msg.ee_goal_twist.linear_velocity = lcmdrc.vector_3d_t()
        msg.ee_goal_twist.angular_velocity = lcmdrc.vector_3d_t()

        channels = {
                    'l_hand' : 'LEFT_PALM_GOAL',
                    'r_hand' : 'RIGHT_PALM_GOAL',
                    'l_foot' : 'LEFT_FOOT_GOAL',
                    'l_right' : 'RIGHT_FOOT_GOAL',
                   }

        requestChannel = channels[linkName]
        responseChannel = 'CANDIDATE_MANIP_PLAN'

        self.sendPlannerSettings(startPose)

        if waitForResponse:
            return lcmUtils.MessageResponseHelper.publishAndWait(requestChannel, msg,
                                    responseChannel, lcmdrc.robot_plan_w_keyframes_t, waitTimeout)
        else:
            lcmUtils.publish(requestChannel, msg)



class RobotPlanPlayback(object):

    def __init__(self):
        self.animationCallback = None
        self.animationTimer = None
        self.interpolationMethod = 'cubic'
        self.playbackSpeed = 1.0


    def getPlanPoses(self, msg):

        poses = []
        poseTimes = []
        for plan in msg.plan:
            pose = robotstate.convertStateMessageToDrakePose(plan)
            poseTimes.append(plan.utime / 1e6)
            poses.append(pose)
        return np.array(poseTimes), poses


    def getPlanElapsedTime(self, msg):

        startTime = msg.plan[0].utime
        endTime = msg.plan[-1].utime
        return (endTime - startTime) / 1e6


    def stopAnimation(self):
        if self.animationTimer:
            self.animationTimer.stop()


    def setInterpolationMethod(method):
        self.interpolationMethod = method


    def playPlan(self, msg, jointController):

        self.playPlans(poseTimes, [msg], jointController)


    def playPlans(self, messages, jointController):

        assert len(messages)

        allPoseTimes, allPoses = self.getPlanPoses(messages[0])

        for msg in messages[1:]:
            poseTimes, poses = self.getPlanPoses(msg)
            poseTimes += allPoseTimes[-1]
            allPoseTimes = np.hstack((allPoseTimes, poseTimes[1:]))
            allPoses += poses[1:]

        self.playPoses(allPoseTimes, allPoses, jointController)


    def playPoses(self, poseTimes, poses, jointController):

        if self.interpolationMethod in ['slinear', 'quadratic', 'cubic']:
            f = scipy.interpolate.interp1d(poseTimes, poses, axis=0, kind=self.interpolationMethod)
        elif self.interpolationMethod == 'pchip':
            f = scipy.interpolate.pchip(poseTimes, poses, axis=0)

        timer = SimpleTimer()

        def updateAnimation():

            tNow = timer.elapsed() * self.playbackSpeed

            if tNow > poseTimes[-1]:
                pose = poses[-1]
                jointController.addPose('plan_playback', pose)
                jointController.setPose('plan_playback')

                if self.animationCallback:
                    self.animationCallback()

                return False

            pose = f(tNow)
            jointController.addPose('plan_playback', pose)
            jointController.setPose('plan_playback')

            if self.animationCallback:
                self.animationCallback()

        self.animationTimer = TimerCallback()
        self.animationTimer.targetFps = 60
        self.animationTimer.callback = updateAnimation
        self.animationTimer.start()



    def picklePlan(self, filename, msg):
        poseTimes, poses = self.getPlanPoses(msg)
        pickle.dump((poseTimes, poses), open(filename, 'w'))


    def plotPlan(self, msg):

        import matplotlib.pyplot as plt

        poseTimes, poses = self.getPlanPoses(msg)

        poses = np.array(poses)
        diffs = np.diff(poses, axis=0)

        movingJointIds = np.unique(np.where(diffs != 0.0)[1])
        movingJointNames = [robotstate.getDrakePoseJointNames()[jointId] for jointId in movingJointIds]
        movingJointTrajectories = [poses[:,jointId] for jointId in movingJointIds]

        seriesNames = []

        sampleResolutionInSeconds = 0.01
        numberOfSamples = (poseTimes[-1] - poseTimes[0]) / sampleResolutionInSeconds
        xnew = np.linspace(poseTimes[0], poseTimes[-1], numberOfSamples)



        for jointId, jointName, jointTrajectory in zip(movingJointIds, movingJointNames, movingJointTrajectories):

            x = poseTimes
            y = jointTrajectory

            y = np.rad2deg(y)

            if self.interpolationMethod in ['slinear', 'quadratic', 'cubic']:
                f = scipy.interpolate.interp1d(x, y, kind=self.interpolationMethod)
            elif self.interpolationMethod == 'pchip':
                f = scipy.interpolate.pchip(x, y)

            plt.plot(x, y, 'ko')
            seriesNames.append(jointName + ' points')

            plt.plot(xnew, f(xnew), '-')
            seriesNames.append(jointName + ' ' + self.interpolationMethod)


        plt.legend(seriesNames, loc='upper right')
        plt.xlabel('time (s)')
        plt.ylabel('joint angle (deg)')
        plt.show()

