
from ddapp.consoleapp import ConsoleApp
from ddapp import robotsystem
from ddapp import transformUtils
from ddapp import vtkAll as vtk
from ddapp.utime import getUtime
from ddapp import lcmUtils
from ddapp.timercallback import TimerCallback

import numpy as np
import math

import drc as lcmdrc
import bot_core as lcmbot



def angleBetweenVectors(a, b):
    return np.arccos(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) ))


def concat(*transforms):
    t = vtk.vtkTransform()
    t.PostMultiply()
    for transform in transforms:
        t.Concatenate(transform)
    return transformUtils.copyFrame(t)


class FootLockEstimator(object):

    def __init__(self):
        self.footBias = 0.5
        self.timer = TimerCallback(targetFps=1.0)
        self.timer.callback = self.publishCorrection
        self.clear()


    def capture(self):

        self.pelvisFrame = robotStateModel.getLinkFrame('pelvis')
        self.lfootFrame = robotStateModel.getLinkFrame('l_foot')
        self.rfootFrame = robotStateModel.getLinkFrame('r_foot')

    def clear(self):

        self.lfootFrame = None
        self.rfootFrame = None
        self.pelvisFrame = None

    def printCaptured(self):

        print '--------------------'
        print 'l_foot yaw:', self.lfootFrame.GetOrientation()[2]
        print 'r_foot yaw:', self.rfootFrame.GetOrientation()[2]
        print 'pelvis yaw:', self.pelvisFrame.GetOrientation()[2]

    def getPelvisEstimateFromFoot(self, pelvisFrame, footFrame, previousFootFrame):

        pelvisToWorld = pelvisFrame
        footToWorld = footFrame
        oldFootToWorld = previousFootFrame
        worldToFoot = footToWorld.GetLinearInverse()
        pelvisToFoot = concat(pelvisToWorld, worldToFoot)

        return concat(pelvisToFoot, oldFootToWorld)


    def getPelvisEstimate(self):

        p = robotStateModel.getLinkFrame('pelvis')
        lf = robotStateModel.getLinkFrame('l_foot')
        rf = robotStateModel.getLinkFrame('r_foot')

        pelvisLeft = self.getPelvisEstimateFromFoot(p, lf, self.lfootFrame)
        pelvisRight = self.getPelvisEstimateFromFoot(p, rf, self.rfootFrame)

        return transformUtils.frameInterpolate(pelvisLeft, pelvisRight, self.footBias)

    def onRobotStateModelChanged(self, model=None):

        if self.lfootFrame is None:
            return

        newPelvisToWorld = self.getPelvisEstimate()

        q = robotStateJointController.q.copy()
        q[0:3] = newPelvisToWorld.GetPosition()
        q[3:6] = transformUtils.rollPitchYawFromTransform(newPelvisToWorld)
        playbackJointController.setPose('lock_pose', q)

    def initVisualization(self):
        robotStateModel.connectModelChanged(self.onRobotStateModelChanged)
        playbackRobotModel.setProperty('Visible', True)

    def publishCorrection(self, channel='POSE_YAW_LOCK'):

        pelvisToWorld = self.getPelvisEstimate()

        position, quat = transformUtils.poseFromTransform(pelvisToWorld)

        msg = lcmbot.pose_t()
        msg.utime = robotStateJointController.lastRobotStateMessage.utime
        msg.pos = [0.0, 0.0, 0.0]
        msg.orientation = quat.tolist()

        lcmUtils.publish(channel, msg)

    def enable(self):
        self.capture()
        self.timer.start()

    def disable(self):
        self.timer.stop()
        self.clear()




#---------------------------------------

app = ConsoleApp()
app.setupGlobals(globals())
app.showPythonConsole()
view = app.createView()
view.show()
robotsystem.create(view, globals())

estimator = FootLockEstimator()

app.start()
