import os
from . import vtkAll as vtk
import math
import time
import re
import numpy as np

from director.timercallback import TimerCallback
from director import objectmodel as om
from director.simpletimer import SimpleTimer
from director.utime import getUtime
from director import robotstate

import pickle
import scipy.interpolate


def asRobotPlan(msg):
    '''
    If the given message is a robot_plan_with_supports_t then this function returns
    the plan message contained within it.  For any other message type, this function
    just returns its input argument.
    '''
    try:
        import drc as lcmdrc
    except ImportError:
        pass
    else:
        if isinstance(msg, lcmdrc.robot_plan_with_supports_t):
            return msg.plan
    return msg


class PlanPlayback(object):

    def __init__(self):
        self.animationCallback = None
        self.animationTimer = None
        self.interpolationMethod = 'slinear'
        self.playbackSpeed = 1.0
        self.jointNameRegex = ''

    @staticmethod
    def getPlanPoses(msgOrList):

        if isinstance(msgOrList, list):
            messages = msgOrList
            allPoseTimes, allPoses = PlanPlayback.getPlanPoses(messages[0])

            for msg in messages[1:]:
                poseTimes, poses = PlanPlayback.getPlanPoses(msg)
                poseTimes += allPoseTimes[-1]
                allPoseTimes = np.hstack((allPoseTimes, poseTimes[1:]))
                allPoses += poses[1:]
            return allPoseTimes, allPoses

        else:
            msg = asRobotPlan(msgOrList)

            poses = []
            poseTimes = []
            for plan in msg.plan:
                pose = robotstate.convertStateMessageToDrakePose(plan)
                poseTimes.append(plan.utime / 1e6)
                poses.append(pose)
            return np.array(poseTimes), poses

    @staticmethod
    def getPlanElapsedTime(msg):
        msg = asRobotPlan(msg)
        startTime = msg.plan[0].utime
        endTime = msg.plan[-1].utime
        return (endTime - startTime) / 1e6


    def stopAnimation(self):
        if self.animationTimer:
            self.animationTimer.stop()


    def setInterpolationMethod(method):
        self.interpolationMethod = method


    def playPlan(self, msg, jointController):
        self.playPlans([msg], jointController)


    def playPlans(self, messages, jointController):

        assert len(messages)

        poseTimes, poses = self.getPlanPoses(messages)
        self.playPoses(poseTimes, poses, jointController)


    def getPoseInterpolatorFromPlan(self, message):
        poseTimes, poses = self.getPlanPoses(message)
        return self.getPoseInterpolator(poseTimes, poses)


    def getPoseInterpolator(self, poseTimes, poses, unwrap_rpy=True):
        if unwrap_rpy:
            poses = np.array(poses, copy=True)
            poses[:,3:6] = np.unwrap(poses[:,3:6],axis=0)

        if self.interpolationMethod in ['slinear', 'quadratic', 'cubic']:
            f = scipy.interpolate.interp1d(poseTimes, poses, axis=0, kind=self.interpolationMethod)
        elif self.interpolationMethod == 'pchip':
            f = scipy.interpolate.PchipInterpolator(poseTimes, poses, axis=0)
        return f


    def getPlanPoseMeshes(self, messages, jointController, robotModel, numberOfSamples):

        poseTimes, poses = self.getPlanPoses(messages)
        f = self.getPoseInterpolator(poseTimes, poses)
        sampleTimes = np.linspace(poseTimes[0], poseTimes[-1], numberOfSamples)
        meshes = []

        for sampleTime in sampleTimes:

            pose = f(sampleTime)
            jointController.setPose('plan_playback', pose)
            polyData = vtk.vtkPolyData()
            robotModel.model.getModelMesh(polyData)
            meshes.append(polyData)

        return meshes


    def showPoseAtTime(self, time, jointController, poseInterpolator):
        pose = poseInterpolator(time)
        jointController.setPose('plan_playback', pose)


    def playPoses(self, poseTimes, poses, jointController):

        f = self.getPoseInterpolator(poseTimes, poses)

        timer = SimpleTimer()

        def updateAnimation():

            tNow = timer.elapsed() * self.playbackSpeed

            if tNow > poseTimes[-1]:
                pose = poses[-1]
                jointController.setPose('plan_playback', pose)

                if self.animationCallback:
                    self.animationCallback()

                return False

            pose = f(tNow)
            jointController.setPose('plan_playback', pose)

            if self.animationCallback:
                self.animationCallback()

        self.animationTimer = TimerCallback()
        self.animationTimer.targetFps = 60
        self.animationTimer.callback = updateAnimation
        self.animationTimer.start()
        updateAnimation()


    def picklePlan(self, filename, msg):
        poseTimes, poses = self.getPlanPoses(msg)
        pickle.dump((poseTimes, poses), open(filename, 'w'))


    def getMovingJointNames(self, msg):
        poseTimes, poses = self.getPlanPoses(msg)
        diffs = np.diff(poses, axis=0)
        jointIds =  np.unique(np.where(diffs != 0.0)[1])
        jointNames = [robotstate.getDrakePoseJointNames()[jointId] for jointId in jointIds]
        return jointNames


    def plotPlan(self, msg):

        poseTimes, poses = self.getPlanPoses(msg)
        self.plotPoses(poseTimes, poses)


    def plotPoses(self, poseTimes, poses):

        import matplotlib.pyplot as plt

        poses = np.array(poses)

        if self.jointNameRegex:
            jointIds = list(range(poses.shape[1]))
        else:
            diffs = np.diff(poses, axis=0)
            jointIds = np.unique(np.where(diffs != 0.0)[1])

        jointNames = [robotstate.getDrakePoseJointNames()[jointId] for jointId in jointIds]
        jointTrajectories = [poses[:,jointId] for jointId in jointIds]

        seriesNames = []

        sampleResolutionInSeconds = 0.01
        numberOfSamples = (poseTimes[-1] - poseTimes[0]) / sampleResolutionInSeconds
        xnew = np.linspace(poseTimes[0], poseTimes[-1], numberOfSamples)

        fig = plt.figure()
        ax = fig.add_subplot(111)


        for jointId, jointName, jointTrajectory in zip(jointIds, jointNames, jointTrajectories):

            if self.jointNameRegex and not re.match(self.jointNameRegex, jointName):
                continue

            x = poseTimes
            y = jointTrajectory

            y = np.rad2deg(y)

            if self.interpolationMethod in ['slinear', 'quadratic', 'cubic']:
                f = scipy.interpolate.interp1d(x, y, kind=self.interpolationMethod)
            elif self.interpolationMethod == 'pchip':
                f = scipy.interpolate.PchipInterpolator(x, y)

            ax.plot(x, y, 'ko')
            seriesNames.append(jointName + ' points')

            ax.plot(xnew, f(xnew), '-')
            seriesNames.append(jointName + ' ' + self.interpolationMethod)


        ax.legend(seriesNames, loc='upper right').draggable()
        ax.set_xlabel('time (s)')
        ax.set_ylabel('joint angle (deg)')
        ax.set_title('joint trajectories')
        plt.show()
