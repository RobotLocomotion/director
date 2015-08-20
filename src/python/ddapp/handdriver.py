import os
import vtkAll as vtk
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

import drc as lcmdrc
import irobothand as lcmirobot
import robotiqhand as lcmrobotiq



class IRobotHandDriver(object):

    MOTOR_FINGER_0 = 0
    MOTOR_FINGER_1 = 1
    MOTOR_THUMB = 2

    def __init__(self, side):

        assert side in ('left', 'right')
        self.side = side
        self.enabledMotors = [True, True, True, False]
        pass


    def sendCalibrate(self, withJig=True):
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


class RobotiqHandDriver(object):

    def __init__(self, side):

        assert side in ('left', 'right')
        self.side = side
        pass


    def sendCalibrate(self):
        msg = lcmrobotiq.command_t()
        msg.utime = getUtime()
        msg.activate = 0
        msg.emergency_release = 0
        msg.do_move = 0
        msg.mode = 0
        msg.position = 0
        msg.force = 0
        msg.velocity = 0
        channel = 'ROBOTIQ_%s_COMMAND' % self.side.upper()
        lcmUtils.publish(channel, msg)

        time.sleep(0.1)
        msg.activate = 1
        lcmUtils.publish(channel, msg)

    def sendDrop(self):
        msg = lcmrobotiq.command_t()
        msg.utime = getUtime()
        msg.activate = 1
        msg.emergency_release = 1
        msg.do_move = 0
        msg.mode = 0
        msg.position = 0
        msg.force = 0
        msg.velocity = 0
        channel = 'ROBOTIQ_%s_COMMAND' % self.side.upper()
        lcmUtils.publish(channel, msg)

    def sendOpen(self, percentage=100.0):
        self.sendClose(100.0 - percentage)

    def sendClose(self, percentage=100.0):
        assert 0.0 <= percentage <= 100.0

        msg = lcmrobotiq.command_t()
        msg.utime = getUtime()
        msg.activate = 1
        msg.emergency_release = 0
        msg.do_move = 1
        msg.position = int(254 * (percentage/100.0))
        msg.force = 254
        msg.velocity = 254
        channel = 'ROBOTIQ_%s_COMMAND' % self.side.upper()
        lcmUtils.publish(channel, msg)

    def sendCustom(self, position, force, velocity, mode):
        assert 0.0 <= position <= 100.0
        assert 0.0 <= force <= 100.0
        assert 0.0 <= velocity <= 100.0
        assert 0 <= int(mode) <= 4

        msg = lcmrobotiq.command_t()
        msg.utime = getUtime()
        msg.activate = 1
        msg.emergency_release = 0
        msg.do_move = 1
        msg.position = int(254 * (position/100.0))
        msg.force = int(254 * (force/100.0))
        msg.velocity = int(254 * (velocity/100.0))
        msg.mode = int(mode)
        channel = 'ROBOTIQ_%s_COMMAND' % self.side.upper()
        lcmUtils.publish(channel, msg)

    def sendRegrasp(self, position, force, velocity, mode):

        channel = 'ROBOTIQ_%s_STATUS' % self.side.upper()
        statusMsg = lcmUtils.captureMessage(channel, lcmrobotiq.status_t)

        avgPosition = (statusMsg.positionA +
                       statusMsg.positionB +
                       statusMsg.positionC)/3.0
        print avgPosition

        newPosition = avgPosition/254.0  * 100.0
        print newPosition
        if newPosition > 5.0:
            self.sendCustom(newPosition-5.0, force, velocity, mode)
        else:
            self.sendCustom(newPosition, force, velocity, mode)

        time.sleep(0.3)
        self.sendCustom(position, force, velocity, mode)

    def sendFingerControl(self, positionA, positionB, positionC, force, velocity, scissor, mode):
        assert 0.0 <= positionA <= 254.0
        assert 0.0 <= positionB <= 254.0
        assert 0.0 <= positionC <= 254.0
        assert 0.0 <= force <= 100.0
        assert 0.0 <= velocity <= 100.0
        assert 0 <= int(mode) <= 4

        if not scissor is None:
            assert 0.0 <= scissor <= 254.0

        msg = lcmrobotiq.command_t()
        msg.utime = getUtime()
        msg.activate = 1
        msg.emergency_release = 0
        msg.do_move = 1
        msg.ifc = 1
        msg.positionA = int(positionA)
        msg.positionB = int(positionB)
        msg.positionC = int(positionC)
        msg.force = int(254 * (force/100.0))
        msg.velocity = int(254 * (velocity/100.0))
        msg.mode = int(mode)

        if not scissor is None:
            msg.isc = 1
            msg.positionS = int(scissor)

        channel = 'ROBOTIQ_%s_COMMAND' % self.side.upper()
        lcmUtils.publish(channel, msg)

    def setMode(self, mode):
        assert 0 <= int(mode) <= 4

        msg = lcmrobotiq.command_t()
        msg.utime = getUtime()
        msg.activate = 1
        msg.emergency_release = 0
        msg.do_move = 0
        msg.position = 0
        msg.force = 0
        msg.velocity = 0
        msg.mode = int(mode)
        channel = 'ROBOTIQ_%s_COMMAND' % self.side.upper()
        lcmUtils.publish(channel, msg)
