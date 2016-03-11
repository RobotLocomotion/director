import os
import vtkAll as vtk
import math
import numpy as np
from collections import deque

from director import transformUtils
from director import lcmUtils
from director.timercallback import TimerCallback
from director import objectmodel as om
from director import visualization as vis
from director import applogic as app
from director.debugVis import DebugData
from director import ioUtils
from director.utime import getUtime
import time

import drc as lcmdrc
import bot_core


class ValkyrieDriver(object):

    #def __init__(self):
    #    #self._setupSubscriptions()

    #def _setupSubscriptions(self):

    def sendWholeBodyCommand(self, wholeBodyMode):
        msg = lcmdrc.int64_stamped_t()
        msg.utime = getUtime()
        msg.data = wholeBodyMode
        lcmUtils.publish('IHMC_CONTROL_MODE_COMMAND', msg)

def init():

    global driver
    driver = ValkyrieDriver()

    return driver
