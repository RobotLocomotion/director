__author__ = 'manuelli'
import numpy as np
from director.timercallback import TimerCallback
from director import robotstate

class JointStreamScript:

    def __init__(self, initialPose, jointName='rightHipPitch', frequency=0.25, amplitude=20, type='sin'):
        self.timer = TimerCallback(targetFps=30)

        if type=="sin":
            self.timer.callback = self.sineWave
        elif type=="step":
            self.timer.callback = self.step
        else:
            raise ValueError('type must be sin or step')


        self.initialPose = np.copy(initialPose)
        self.currentPose = np.copy(initialPose)
        self.drakePoseJointNames = robotstate.getDrakePoseJointNames()
        self.jointIdx = self.drakePoseJointNames.index(jointName)

        self.frequency = frequency # frequency in Hz
        self.jointName = jointName
        self.amplitude = np.deg2rad(amplitude)


    def start(self):
        self.timer.start()

    def sineWave(self):
        elapsed = self.timer.getElapsedTime()
        jointDelta = self.amplitude*np.sin(elapsed*2*np.pi*self.frequency)
        jointValue = self.initialPose[self.jointIdx] + jointDelta
        self.currentPose[self.jointIdx] = jointValue


    def step(self):
        self.currentPose[self.jointIdx] = self.initialPose[self.jointIdx] + self.amplitude



