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
import irobothand as lcmirobot




class IRobotHandDriver(object):

    MOTOR_FINGER_0 = 0
    MOTOR_FINGER_1 = 1
    MOTOR_THUMB = 2

    def __init__(self, side):

        assert side in ('left', 'right')
        self.side = side
        self.enabledMotors = [True, True, True, False]
        pass


    def sendCalibrate(withJig=True):
        msg = lcmirobot.calibrate_t()
        msg.utime = getUtime()
        msg.in_jig = withJig
        channel = 'IROBOT_%s_CALIBRATE' % self.side.upper()
        lcmUtils.publish(channel, msg)

    def sendOpen(self, percentage=100.0):
        self.sendClose(100.0 - percentage)

    def sendClose(self, percentage=100.0):
        assert 0.0 <= percentage <= 100.0
        msg = lcmirobot.position_control_close_t()
        msg.utime = getUtime()
        msg.valid = [True, True, True, False]
        msg.close_fraction = percentage / 100.0

        channel = 'IROBOT_%s_POSITION_CONTROL_CLOSE' % self.side.upper()
        lcmUtils.publish(channel, msg)
