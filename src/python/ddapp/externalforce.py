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
        self.robotStateModel = robotSystem.robotStateModel
        self.robotStateModel.connectModelChanged(self.onModelChanged)

        # keys = linkNames, wrench = 6 x 1 Torque-Force vector, all in body frame
        self.externalForces = dict()
        self.timeout = 1e10 # this is for testing
        self.publishChannel = 'EXTERNAL_FORCE_TORQUE'

        # setup timercallback to publish, lets say at 5 hz
        self.timer = TimerCallback(targetFps=5)
        self.timer.callback = self.publish
        self.startPublishing()


    # linkName is a string, wrench is an np.array
    def addForce(self, linkName, wrench=None, forceDirection=None, forceMagnitude=None, forceLocation=None, inWorldFrame=False):


        d = dict()
        # need at least one of wrench, or forceDirection and forceMagnitude
        assert (wrench is not None) or ((forceDirection is not None) and (forceMagnitude is not None) and (forceLocation is not None))

        visName = linkName + ' external force'
        om.removeFromObjectModel(om.findObjectByName(visName))


        if wrench is not None:
            if inWorldFrame:
                raise ValueError('do not support specifying wrench in world frame')
            d['wrench'] = wrench
            d['forceLocation'] = np.array([0,0,0])
            d['forceDirection'] = wrench[3:]/np.linalg.norm(wrench[3:])
            d['forceMagnitude'] = np.linalg.norm(wrench[3:])
            d['isWrench'] = True
        else:
            if inWorldFrame:
                linkToWorld = self.robotStateModel.getLinkFrame(linkName)
                worldToLink = linkToWorld.GetLinearInverse()
                forceLocation = np.array(worldToLink.TransformPoint(forceLocation))
                forceDirection = np.array(worldToLink.TransformDoubleVector(forceDirection))

            forceDirection = forceDirection/np.linalg.norm(forceDirection)
            d['forceDirection'] = forceDirection
            d['forceMagnitude'] = forceMagnitude
            d['forceLocation'] = forceLocation
            d['isWrench'] = False


        d['time'] = time.time()
        self.externalForces[linkName] = d
        self.updateContactWrench(linkName)
        self.drawForces()

    def computeWrench(self, linkName, forceDirection, forceMagnitude, forceLocation):
        outputFrame = vtk.vtkTransform()
        wrenchFrame = vtk.vtkTransform()
        wrenchFrame.Translate(forceLocation)

        forceMomentTransform = transformUtils.forceMomentTransformation(wrenchFrame, outputFrame)

        wrench = np.zeros(6)
        wrench[3:] = forceMagnitude*forceDirection
        wrenchTransformed = np.dot(forceMomentTransform, wrench)

        return wrenchTransformed


    def removeForce(self, linkName, callFromFrameObj=False):
        self.externalForces.pop(linkName, None)
        name = linkName + ' external force'

        if not callFromFrameObj:
            om.removeFromObjectModel(om.findObjectByName(linkName))

    # remove forces from dict that haven't been refreshed in at least self.timeout seconds
    def removeStaleExternalForces(self):
        keysToRemove = []
        for linkName, value in self.externalForces.iteritems():
            elapsed = time.time() - value['time']
    
            if elapsed > self.timeout:
                keysToRemove.append(linkName)


        for key in keysToRemove:
            self.removeForce(key)

    

    def publish(self):

        if len(self.externalForces) == 0:
            return

        # self.removeStaleExternalForces()
        msg = lcmdrake.lcmt_external_force_torque()
        msg.num_external_forces = len(self.externalForces);


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

    # connect this with an on model changed
    def drawForces(self):
        if len(self.externalForces) == 0:
            return



        for linkName, val in self.externalForces.iteritems():
            name = linkName + ' external force'
            linkToWorld = self.robotStateModel.getLinkFrame(linkName)

            forceLocationInWorld = np.array(linkToWorld.TransformPoint(val['forceLocation']))
            forceDirectionInWorld = np.array(linkToWorld.TransformDoubleVector(val['forceDirection']))

            point = forceLocationInWorld - 0.1*forceDirectionInWorld

            d = DebugData()
            d.addSphere(forceLocationInWorld, radius=0.01)
            d.addLine(point, forceLocationInWorld, radius=0.005)

            #Green is for a force, red is for a wrench
            color = [0,1,0]
            if val['isWrench']:
                color = [1,0,0]

            obj = vis.updatePolyData(d.getPolyData(), name, color=color)
            obj.actor.SetPickable(False)
            obj.connectRemovedFromObjectModel(self.removeForceFromFrameObject)
            obj.addProperty('magnitude', 0.0)
            obj.addProperty('linkName', linkName)
            obj.properties.connectPropertyChanged(functools.partial(self.onPropertyChanged, obj))

    def onModelChanged(self, model):
        self.drawForces()

    def onPropertyChanged(self, frameObj, propertySet, propertyName):
        if propertyName != 'magnitude':
            return
        linkName = frameObj.getProperty('linkName')
        magnitude = frameObj.getProperty('magnitude')
        if magnitude < 0:
            print "you must specify a positive magnitude"
            print "external forces can only PUSH, NOT PULL"
            return

        self.externalForces[linkName]['forceMagnitude'] = magnitude
        self.updateContactWrench(linkName)

    def updateContactWrench(self, linkName):
        if not self.externalForces.has_key(linkName):
            return

        val = self.externalForces[linkName]

        # if it was specified as a wrench, then don't overwrite it
        if val['isWrench']:
            return

        val['wrench'] = self.computeWrench(linkName, val['forceDirection'],  val['forceMagnitude'], val['forceLocation'])


    def removeForceFromFrameObject(self, tree_, frameObj):
        linkName = frameObj.getProperty('linkName')
        self.removeForce(linkName, callFromFrameObj=True)

    def printForces(self):
        for key in self.externalForces.keys():
            print key

    # these are test methods
    def setupTest(self):
        w = np.array([1,2,3,4,5,6])
        self.addForce('pelvis', wrench=w)
        self.startPublishing()

    def test1(self):
        forceDirection = np.array([0,0,1])
        forceMagnitude = 100
        forceLocation = np.array([0,0,0])
        linkName = 'pelvis'

        self.addForce(linkName, forceDirection=forceDirection, forceMagnitude=forceMagnitude, forceLocation=forceLocation)
        self.drawForces()

    def test2(self):
        wrench = np.array([0,0,0,0,0,100])
        linkName = 'pelvis'
        self.addForce(linkName, wrench=wrench)


    def constructTestFrames(self):
        T = vtk.vtkTransform();
        S = vtk.vtkTransform()
        S.Translate([1,2,0])
        FM = transformUtils.forceMomentTransformation(S,T)
        print FM
        return T,S, FM





