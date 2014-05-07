import os
import vtkAll as vtk
from ddapp import botpy
import math
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

import drc as lcmdrc



class SystemStatusListener(object):

    def __init__(self, outputConsole):
        self.outputConsole = outputConsole
        lcmUtils.addSubscriber('SYSTEM_STATUS', lcmdrc.system_status_t, self.onSystemStatus)

    def onSystemStatus(self, message):
        message = 'SYSTEM_STATUS: ' + message.value
        if self.outputConsole is not None:
            self.outputConsole.append(message)
        else:
            print message


class AtlasDriver(object):

    def __init__(self):

        self.lastAtlasStatusMessage = None
        self._setupSubscriptions()
        self.timer = SimpleTimer()

    def _setupSubscriptions(self):
        sub = lcmUtils.addSubscriber('ATLAS_STATUS', lcmdrc.atlas_status_t, self.onAtlasStatus)
        sub.setSpeedLimit(60)

    def onAtlasStatus(self, message):
        self.lastAtlasStatusMessage = message

    def getBehaviorMap(self):
        '''
        Return a dict that maps behavior ids (int) to behavior names (string).
        '''
        msg = lcmdrc.atlas_status_t
        behaviors = {
                    msg.BEHAVIOR_NONE        : 'none',
                    msg.BEHAVIOR_FREEZE      : 'freeze',
                    msg.BEHAVIOR_STAND_PREP  : 'prep',
                    msg.BEHAVIOR_STAND       : 'stand',
                    msg.BEHAVIOR_WALK        : 'walk',
                    msg.BEHAVIOR_STEP        : 'step',
                    msg.BEHAVIOR_MANIPULATE  : 'manip',
                    msg.BEHAVIOR_USER        : 'user',
                    8                        : 'calibrate',
                    9                        : 'stop',
                    10                       : 'mit_stand',
                    }
        return behaviors

    def getCurrentBehaviorName(self):
        '''
        Returns the current behavior name as a string.  Returns None if the
        current behavior is unknown.  The current behavior is unknown if no
        atlas status messages have arrived since this class was initialized.
        The returned string is one of the behavior names in the values of
        the behavior map returned by getBehaviorMap().
        '''
        if not self.lastAtlasStatusMessage:
            return None

        behaviors = self.getBehaviorMap()
        behaviorId = self.lastAtlasStatusMessage.behavior
        assert behaviorId in behaviors
        return behaviors[behaviorId]

    def getCurrentInletPressure(self):
        if self.lastAtlasStatusMessage:
            return self.lastAtlasStatusMessage.pump_inlet_pressure
        return 0.0

    def getCurrentSupplyPressure(self):
        if self.lastAtlasStatusMessage:
            return self.lastAtlasStatusMessage.pump_supply_pressure
        return 0.0

    def getCurrentReturnPressure(self):
        if self.lastAtlasStatusMessage:
            return self.lastAtlasStatusMessage.pump_return_pressure
        return 0.0

    def getCurrentAirSumpPressure(self):
        if self.lastAtlasStatusMessage:
            return self.lastAtlasStatusMessage.air_sump_pressure
        return 0.0

    def getCurrentPumpRpm(self):
        if self.lastAtlasStatusMessage:
            return self.lastAtlasStatusMessage.current_pump_rpm
        return 0.0

    def sendBehaviorCommand(self, behaviorName):

        assert behaviorName in self.getBehaviorMap().values()

        msg = lcmdrc.atlas_behavior_command_t()
        msg.utime = getUtime()
        msg.command = behaviorName
        lcmUtils.publish('ATLAS_BEHAVIOR_COMMAND', msg)

    def sendStopCommand(self):
        self.sendBehaviorCommand('stop')

    def sendFreezeCommand(self):
        self.sendBehaviorCommand('freeze')

    def sendPrepCommand(self):
        self.sendBehaviorCommand('prep')

    def sendStandCommand(self):
        self.sendBehaviorCommand('stand')

    def sendMITStandCommand(self):
        self.sendBehaviorCommand('mit_stand')

    def sendManipCommand(self):
        self.sendBehaviorCommand('manip')

    def sendUserCommand(self):
        self.sendBehaviorCommand('user')

    def sendCalibrateCommand(self):
        self.sendBehaviorCommand('calibrate')

    def sendCalibrateEncodersCommand(self):
        msg = lcmdrc.utime_t()
        msg.utime = getUtime()
        lcmUtils.publish('CALIBRATE_ARM_ENCODERS', msg)

    def sendPlanUsingBdiHeight(self, enabled):
        msg = lcmdrc.plan_adjust_mode_t()
        msg.utime = getUtime()
        msg.mode = 1 if enabled else 0
        lcmUtils.publish('PLAN_USING_BDI_HEIGHT', msg)


    def getPelvisHeightLimits(self):
        '''
        returns pelvis height limits in meters: min, max
        '''
        return (0.66, 0.92)


    def sendPelvisHeightCommand(self, height):

        heightLimit = self.getPelvisHeightLimits()
        assert heightLimit[0] <= height <= heightLimit[1]

        pelvisParams = lcmdrc.atlas_behavior_pelvis_servo_params_t()
        pelvisParams.com_v0 = 0.0
        pelvisParams.com_v1 = 0.0
        pelvisParams.pelvis_height = height
        pelvisParams.pelvis_yaw = 0.0
        pelvisParams.pelvis_pitch = 0.0
        pelvisParams.pelvis_roll = 0.0

        msg = lcmdrc.atlas_behavior_manipulate_params_t()
        msg.use_demo_mode = 0
        msg.use_desired = 1
        msg.desired = pelvisParams

        lcmUtils.publish('ATLAS_MANIPULATE_PARAMS', msg)



def init(outputConsole):

    global driver
    driver = AtlasDriver()

    global systemStatus
    systemStatus = SystemStatusListener(outputConsole)

    return driver

