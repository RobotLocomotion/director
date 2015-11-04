import ddapp
import math
import textwrap
import drc as lcmdrc
import time
import drake as lcmdrake
import bot_core as lcmbotcore
import vtkAll as vtk
from collections import namedtuple
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp import objectmodel as om
from ddapp import lcmUtils
from ddapp import ik
from ddapp import cameraview
from ddapp import affordanceupdater
from ddapp import affordancemanager
from ddapp import segmentation
from ddapp import robotstate
from ddapp.debugVis import DebugData
from ddapp.utime import getUtime
from ddapp.ikplanner import ConstraintSet
import ddapp.tasks.robottasks as rt
from ddapp.ikparameters import IkParameters
from ddapp.timercallback import TimerCallback


import os
import functools
import numpy as np
import scipy.io
from ddapp.tasks.taskuserpanel import TaskUserPanel
from ddapp import drcargs

Wrench_Time = namedtuple('wrenchTime', ['wrench','time'])
class ExternalForce(object):

    def __init__(self, robotSystem):
        self.robotSystem = robotSystem

        # keys = linkNames, wrench = 6 x 1 Torque-Force vector, all in body frame
        self.externalForces = dict()
        self.timeout = 1e10 # this is for testing
        self.publishChannel = 'EXTERNAL_FORCE_TORQUE'

        # setup timercallback to publish, lets say at 5 hz
        self.timer = TimerCallback(targetFps=5)
        self.timer.callback = self.publish


    # linkName is a string, wrench is an np.array
    def addForceToDict(self, linkName, wrench=None, forceDirection=None, forceMagnitude=None, forceLocation=None):

        d = dict()
        d['linkName'] = linkName

        # need at least one of wrench, or forceDirection and forceMagnitude
        assert (wrench is not None) or ((forceDirection is not None) and (forceMagnitude is not None) and (forceLocation is not None))


        if wrench is not None:
            d['wrench'] = wrench
            d['forceLocation'] = np.array([0,0,0])
            d['forceDirection'] = wrench[3:end]
            d['forceMagnitude'] = np.norm(wrench[3:end])
            d['isWrench'] = True
        else:
            d['wrench'] = self.computeWrench(linkName, forceDirection, forceMagnitude, forceLocation)
            d['forceDirection'] = forceDirection
            d['forceMagnitude'] = forceMagnitude
            d['forceLocation'] = forceLocation

        d['time'] = time.time()
        self.externalForces[linkName] = d

    def computeWrench(linkName, forceDirection, forceMagnitude, forceLocation):
        outputFrame = vtk.vtkTransform()
        wrenchFrame = vtk.vtkTransform()
        wrenchFrame.Translate(forceLocation)

        forceMomentTransform = transformUtils.forceMomentTransformation(wrenchFrame, outputFrame)

        wrench = np.zeros(6,1)
        wrench[3:] = forceMagnitude*forceDirection
        wrenchTransformed = np.dot(forceMomentTransform, wrench)

        return wrenchTransformed


    def removeForceFromDict(self, linkName):
        del self.externalForces[linkName]

    # remove forces from dict that haven't been refreshed in at least self.timeout seconds
    def cleanupExternalForces(self):
        keysToRemove = []
        for linkName, value in self.externalForces.iteritems():
            elapsed = time.time() - value['time']
    
            if elapsed > self.timeout:
                keysToRemove.append(linkName)


        for key in keysToRemove:
            self.removeForceFromDict(key)

    def setupTest(self):
        w = np.array([1,2,3,4,5,6])
        self.addForceToDict('pelvis', wrench=w)
        self.startPublishing()

    def constructTestFrames(self):
        T = vtk.vtkTransform();
        S = vtk.vtkTransform()
        S.Translate([1,2,0])
        FM = transformUtils.forceMomentTransformation(S,T)
        print FM
        return T,S, FM

    def publish(self):

        if len(self.externalForces) == 0:
            return

        # self.cleanupExternalForces()
        msg = lcmdrake.lcmt_external_force_torque()
        msg.num_external_forces = len(self.externalForces);
        # msg.body_names = []
        # msg.tx = []
        # msg.ty = []
        # msg.tz = []
        # msg.fx = []
        # msg


        for linkName, val in self.externalForces.iteritems():
            msg.body_names.append(linkName)
            msg.tx.append(val['wrench'][0])
            msg.ty.append(val['wrench'][1])
            msg.tz.append(val['wrench'][2])
            msg.fx.append(val['wrench'][3])
            msg.fy.append(val['wrench'][4])
            msg.fz.append(val['wrench'][5])

        lcmUtils.publish(self.publishChannel, msg)


    def startPublishing(self):
        self.timer.start()

    def stopPublishing(self):
        self.timer.stop()



