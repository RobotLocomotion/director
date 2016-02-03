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
import os.path
import csv
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
        self.publishChannel = 'EXTERNAL_FORCE_TORQUE'
        self.captureMode = False
        self.captureModeCounter = 0
        self.showContactRay = True
        self.showContactFilterEstimate = True
        self.showActiveLinkEstimate = True
        self.addSubscribers()
        self.createPlunger()
        self.createMeshDataAndLocators()

        self.visObjectDrawTime = dict()
        self.timeout = 1.5

        # setup timercallback to publish, lets say at 5 hz
        self.timer = TimerCallback(targetFps=5)
        self.timer.callback = self.publish
        # self.startPublishing() # this now gets activated when you start linkSelection

        self.visObjectCleanupTimer = TimerCallback(targetFps = 1)
        self.visObjectCleanupTimer.callback = self.removeStaleVisObjects
        self.visObjectCleanupTimer.start()

    def addSubscribers(self):
        lcmUtils.addSubscriber('CONTACT_FILTER_POINT_ESTIMATE', lcmdrc.contact_filter_estimate_t, self.onContactEstimate)
        lcmUtils.addSubscriber("CONTACT_FILTER_BODY_WRENCH_ESTIMATE", lcmdrc.contact_filter_body_wrench_estimate_t, self.onActiveLinkContactEstimate)

        # lcmUtils.addSubscriber("EXTERNAL_FORCE_TORQUE", lcmdrake.lcmt_external_force_torque(), self.onActiveLinkContactEstimate)

    def createMeshDataAndLocators(self):
        self.linkMeshData = dict()

        drakeModelLinkNames = self.robotStateModel.model.getLinkNames()

        for linkName in drakeModelLinkNames:
            linkName = str(linkName)
            data = dict()

            polyData = vtk.vtkPolyData()
            self.robotStateModel.model.getLinkModelMesh(linkName, polyData)
            transform = self.robotStateModel.getLinkFrame(linkName)

            data['linkName'] = linkName
            data['polyData'] = polyData
            data['transform'] = transformUtils.copyFrame(self.robotStateModel.getLinkFrame(linkName))
            data['locator'] = self.buildCellLocator(polyData)
            self.linkMeshData[linkName] = data



    # linkName is a string, wrench is an np.array
    def addForce(self, linkName, wrench=None, forceDirection=None, forceMagnitude=None, forceLocation=None, inWorldFrame=False):

        linkName = str(linkName) # getting a weird u in front otherwise
        d = dict()
        # need at least one of wrench, or forceDirection and forceMagnitude
        assert (wrench is not None) or ((forceDirection is not None) and (forceMagnitude is not None) and (forceLocation is not None))

        if self.captureMode:
            self.captureModeCounter += 1
            key = linkName + "_" + str(self.captureModeCounter)
        else:
            key = linkName


        # check to see if a force on this body already exists, if so then use that as the forceMagnitude
        if self.externalForces.has_key(key):
            forceMagnitude = self.externalForces[key]['forceMagnitude']


        visName = key + ' external force'
        om.removeFromObjectModel(om.findObjectByName(visName))


        if wrench is not None:
            if inWorldFrame:
                raise ValueError('do not support specifying wrench in world frame')
            d['wrench'] = wrench
            d['forceLocation'] = np.array([0,0,0])
            d['forceDirection'] = wrench[3:]/np.linalg.norm(wrench[3:])
            d['forceMagnitude'] = np.linalg.norm(wrench[3:])
            d['isWrench'] = True
            d['linkName'] = linkName
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
            d['linkName'] = linkName


        


        d['time'] = time.time()
        self.externalForces[key] = d
        self.updateContactWrench(key)
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


    def removeForce(self, key, callFromFrameObj=False):
        if not self.externalForces.has_key(key):
            return

        visObjectName = key + ' external force'
        self.externalForces.pop(key, None)        

        if not callFromFrameObj:
            om.removeFromObjectModel(om.findObjectByName(visObjectName))

    def removeAllForces(self):
        keyList = list(self.externalForces.keys())

        for key in keyList:
            self.removeForce(key)



    # remove forces from dict that haven't been refreshed in at least self.timeout seconds
    def removeStaleExternalForces(self):
        keysToRemove = []
        for key, value in self.externalForces.iteritems():
            elapsed = time.time() - value['time']
    
            if elapsed > self.timeout:
                keysToRemove.append(key)


        for key in keysToRemove:
            self.removeForce(key)


    def removeStaleVisObjects(self):
        keysToRemove = []
        for key, val in self.visObjectDrawTime.iteritems():
            elapsed = time.time() - val
            if elapsed > self.timeout:
                keysToRemove.append(key)


        for key in keysToRemove:
            om.removeFromObjectModel(om.findObjectByName(key))
            del self.visObjectDrawTime[key]


    

    def publish(self):

        # if len(self.externalForces) == 0:
        #     return

        tol = 1e-3
        numExternalForces = 0
        msg = lcmdrake.lcmt_external_force_torque()
        

        for key, val in self.externalForces.iteritems():
            # don't publish it if the force is very small
            if np.linalg.norm(val['wrench']) < tol:
                continue

            numExternalForces += 1

            msg.body_names.append(val['linkName'])
            msg.tx.append(val['wrench'][0])
            msg.ty.append(val['wrench'][1])
            msg.tz.append(val['wrench'][2])
            msg.fx.append(val['wrench'][3])
            msg.fy.append(val['wrench'][4])
            msg.fz.append(val['wrench'][5])

        msg.num_external_forces = numExternalForces;
        lcmUtils.publish(self.publishChannel, msg)


    def startPublishing(self):
        self.captureMode = False
        self.captureModeCounter = 0
        self.removeAllForces()
        self.timer.start()

    def stopPublishing(self): 
        print "stopping publishing"
        self.timer.stop()

    def startCaptureMode(self):
        self.stopPublishing()
        print "starting capture mode"
        self.removeAllForces()
        self.captureMode = True
        self.captureModeCounter = 0

    def onContactEstimate(self, msg):

        if not self.showContactFilterEstimate:
            return

        name = 'estimated external force'

        for i in xrange(0,msg.num_contact_points):
            subMsg = msg.single_contact_estimate[i]
            forceLocation = np.array(subMsg.contact_position)
            force = np.array(subMsg.contact_force)

            eps = 0.5
            name = 'estimated external force ' + str(i)
            if np.linalg.norm(force) < eps:
                # om.removeFromObjectModel(om.findObjectByName(name))
                return

            self.drawForce(name, subMsg.body_name, forceLocation, force, color=[0,0,1])
            self.visObjectDrawTime[name] = time.time()


    def drawForce(self, name, linkName, forceLocation, force, color, key=None):
        
        forceDirection = force/np.linalg.norm(force)
        # om.removeFromObjectModel(om.findObjectByName(name))

        linkToWorld = self.robotStateModel.getLinkFrame(linkName)
        forceLocationInWorld = np.array(linkToWorld.TransformPoint(forceLocation))
        forceDirectionInWorld = np.array(linkToWorld.TransformDoubleVector(forceDirection))

        # point = forceLocationInWorld - 0.1*forceDirectionInWorld

        # d = DebugData()
        # # d.addArrow(point, forceLocationInWorld, headRadius=0.025, tubeRadius=0.005, color=color)
        # d.addSphere(forceLocationInWorld, radius=0.01)
        # d.addLine(point, forceLocationInWorld, radius=0.005)

        transformForVis = transformUtils.getTransformFromOriginAndNormal(forceLocationInWorld, forceDirectionInWorld)

        obj = vis.updatePolyData(self.plungerPolyData, name, color=color)
        obj.actor.SetUserTransform(transformForVis)
        

        if key is not None and om.findObjectByName(name) is not None:
            obj.addProperty('magnitude', self.externalForces[key]['forceMagnitude'])
            obj.addProperty('linkName', linkName)
            obj.addProperty('key', key)
            obj.connectRemovedFromObjectModel(self.removeForceFromFrameObject)        

        obj.properties.connectPropertyChanged(functools.partial(self.onPropertyChanged, obj))
        return obj


    # connect this with an on model changed
    def drawForces(self):
        if len(self.externalForces) == 0:
            return



        for key, val in self.externalForces.iteritems():
            linkName = val['linkName']
            name = key + ' external force'
            #Green is for a force, red is for a wrench
            color = [0,1,0]
            if val['isWrench']:
                color = [1,0,0]

            # linkToWorld = self.robotStateModel.getLinkFrame(linkName)

            # forceLocationInWorld = np.array(linkToWorld.TransformPoint(val['forceLocation']))
            # forceDirectionInWorld = np.array(linkToWorld.TransformDoubleVector(val['forceDirection']))

            # point = forceLocationInWorld - 0.1*forceDirectionInWorld

            

            # d = DebugData()
            # # d.addArrow(point, forceLocationInWorld, headRadius=0.025, tubeRadius=0.005, color=color)
            # d.addSphere(forceLocationInWorld, radius=0.01)
            # d.addLine(point, forceLocationInWorld, radius=0.005)


            obj = self.drawForce(name, linkName, val['forceLocation'], val['forceDirection'], color, key=key)
            

    def onModelChanged(self, model):
        self.drawForces()

    def onPropertyChanged(self, frameObj, propertySet, propertyName):
        if propertyName != 'magnitude':
            return
        key = frameObj.getProperty('key')
        linkName = frameObj.getProperty('linkName')
        magnitude = frameObj.getProperty('magnitude')
        if magnitude < 0:
            print "you must specify a positive magnitude"
            print "external forces can only PUSH, NOT PULL"
            return

        self.externalForces[key]['forceMagnitude'] = magnitude
        self.updateContactWrench(key)

    def updateContactWrench(self, key):
        if not self.externalForces.has_key(key):
            return

        val = self.externalForces[key]

        # if it was specified as a wrench, then don't overwrite it
        if val['isWrench']:
            return

        val['wrench'] = self.computeWrench(val['linkName'], val['forceDirection'],  val['forceMagnitude'], val['forceLocation'])


    def removeForceFromFrameObject(self, tree_, frameObj):
        key = frameObj.getProperty('key')
        self.removeForce(key, callFromFrameObj=True)




    #############
    # these methods deal with computing contact force and location
    # uses the linkMeshData to do the intersection
    def computeContactLocation(self, linkName, force, torque):
        # want to find contactPoint such that force applied at contactPoint
        # leads to given torque, i.e. we want to solve for contactPoint such that
        # torque = contactPoint x force, where x denotes the cross product. This is
        # the same as solving torque = -force x contactPoint = -forceCross * contactPoint

        # everything here is in link frame
        forceCross = transformUtils.crossProductMatrix(force)
        forceCrossPseudoInverse = np.linalg.pinv(forceCross)
        contactPoint_d = -np.dot(forceCrossPseudoInverse, torque)

        forceNorm = np.linalg.norm(force)
        if forceNorm < 0.5:
            return None

        forceNormalized = force/forceNorm       
        

        # now intersect line with linkMesh, choose the start and end of the ray
        # so that we find a contact point where the force is pointing "into" the link
        # mesh
        rayOrigin = contactPoint_d - 0.5*forceNormalized
        rayEnd = contactPoint_d + 0.5*forceNormalized

        ############# DEBUGGING
        # print ""
        # print "force", force
        # print "torque", torque
        # print "r_d", contactPoint_d
        # impliedTorque = np.cross(contactPoint_d, force)
        # print "implied torque", impliedTorque
        if self.showContactRay:
            linkToWorld = self.robotStateModel.getLinkFrame(linkName)
            rayOriginInWorld = np.array(linkToWorld.TransformPoint(rayOrigin))
            rayEndInWorld = np.array(linkToWorld.TransformPoint(rayEnd))
            d = DebugData()
            d.addLine(rayOriginInWorld, rayEndInWorld, radius=0.005)
            color=[1,1,0]
            name = linkName + " contact ray world frame"
            obj = vis.updatePolyData(d.getPolyData(), name, color=color)
            self.visObjectDrawTime[name] = time.time()
        ################## DEBUGGING

        
        pt = self.raycastAgainstLinkMesh(linkName, rayOrigin, rayEnd)


        # if pt is None:
        #     print ""
        #     print "no intersection found on " + linkName
        #     print ""
        
        return pt


        # if we found a contact point, then draw the force


        # rayOrigin and rayEnd should be in link frame
        # the method transforms them to the correct world frame for the mesh
    def raycastAgainstLinkMesh(self, linkName, rayOrigin, rayEnd):
        meshToWorld = self.linkMeshData[linkName]['transform']
        rayOriginInWorld = np.array(meshToWorld.TransformPoint(rayOrigin))
        rayEndInWorld = np.array(meshToWorld.TransformPoint(rayEnd))

        # ### DEBUGGING
        # if self.showContactRay:
        #     d = DebugData()
        #     d.addLine(rayOriginInWorld, rayEndInWorld, radius=0.005)
        #     color=[1,0,0]
        #     obj = vis.updatePolyData(d.getPolyData(), "raycast ray in mesh frame", color=color)

        tolerance = 0.0 # intersection tolerance
        pt = [0.0, 0.0, 0.0] # data coordinate where intersection occurs
        lineT = vtk.mutable(0.0) # parametric distance along line segment where intersection occurs
        pcoords = [0.0, 0.0, 0.0] # parametric location within cell (triangle) where intersection occurs
        subId = vtk.mutable(0) # sub id of cell intersection

        result = self.linkMeshData[linkName]['locator'].IntersectWithLine(rayOriginInWorld, rayEndInWorld, tolerance, lineT, pt, pcoords, subId)

        # this means we didn't find an intersection
        if not result:
            return None

        # otherwise we need to transform it back to linkFrame
        worldToMesh = meshToWorld.GetLinearInverse()
        ptInLinkFrame = worldToMesh.TransformPoint(pt)
        return ptInLinkFrame


    def onActiveLinkContactEstimate(self, msg):


        if not self.showActiveLinkEstimate:
            return

        numBodies = msg.num_bodies

        for i in xrange(0,numBodies):     
            linkName = msg.body_name[i]
            fx = msg.fx[i]
            fy = msg.fy[i]
            fz = msg.fz[i]
            tx = msg.tx[i]
            ty = msg.ty[i]
            tz = msg.tz[i]

            name = linkName + " active link estimated external force"
            force = np.array([fx, fy, fz])
            torque = np.array([tx, ty, tz])


            eps = 0.5
            if np.linalg.norm(force) < eps:
                om.removeFromObjectModel(om.findObjectByName(name))
                return


            forceLocation = self.computeContactLocation(linkName, force, torque)
            # print "forceLocation", forceLocation


            if forceLocation is None:
                continue        

            self.drawForce(name, linkName, forceLocation, force, color=[1,0,0])
            self.visObjectDrawTime[name] = time.time()

    
    def printForces(self):
        for key in self.externalForces.keys():
            print key

    def saveForceLocationsToFile(self, filename=None, verbose=False, overwrite=False):
        if filename is None:
            filename = "testDirector.csv"


        drcBase = os.getenv('DRC_BASE')
        fullFilePath = drcBase + "/software/control/residual_detector/src/particle_grids//" + filename

        if os.path.isfile(fullFilePath) and not overwrite:
            print "FILE ALREADY EXISTS, set the overwrite flag to true to overwrite"
            return

        fileObject = open(fullFilePath, 'w')
        for key, val in self.externalForces.iteritems():
            line = str(val['linkName']) + ","

            for i in range(0,3):
                line += str(val['forceLocation'][i]) + ","

            for i in range(0,3):
                line += str(val['forceDirection'][i]) + ","

            line += "\n"

            if verbose:
                print line

            fileObject.write(line)

        fileObject.close()

    def addForcesFromFile(self, filename=None):
        self.startCaptureMode()
        if filename is None:
            filename = "testDirector.csv"

        drcBase = os.getenv('DRC_BASE')
        fullFilePath = drcBase + "/software/control/residual_detector/src/particle_grids/" + filename
        fileObject = open(fullFilePath, 'r')

        reader = csv.reader(fileObject)
        for row in reader:
            line = []
            for col in row:
                line.append(col)

            linkName = line[0]
            forceLocation = np.array([float(line[1]), float(line[2]), float(line[3])])
            forceDirection = np.array([float(line[4]), float(line[5]), float(line[6])])
            self.addForce(linkName, wrench=None, forceDirection=forceDirection, forceMagnitude=0.0, forceLocation=forceLocation, inWorldFrame=False)

    def createPlunger(self):
        forceLocationInWorld = np.array([0,0,0])
        forceDirectionInWorld = np.array([0,0,1])

        point = forceLocationInWorld - 0.1*forceDirectionInWorld
        color = [1,0,0]
        d = DebugData()
        # d.addArrow(point, forceLocationInWorld, headRadius=0.025, tubeRadius=0.005, color=color)
        d.addSphere(forceLocationInWorld, radius=0.01)
        d.addLine(point, forceLocationInWorld, radius=0.005)
        self.plungerPolyData = d.getPolyData()


    @staticmethod
    def buildCellLocator(polyData):
        print "buidling cell locator"

        loc = vtk.vtkCellLocator()
        loc.SetDataSet(polyData)
        loc.BuildLocator()
        return loc

    @staticmethod
    def raycast(locator, rayOrigin, rayEnd):
        tolerance = 0.0 # intersection tolerance
        pt = [0.0, 0.0, 0.0] # data coordinate where intersection occurs
        lineT = vtk.mutable(0.0) # parametric distance along line segment where intersection occurs
        pcoords = [0.0, 0.0, 0.0] # parametric location within cell (triangle) where intersection occurs
        subId = vtk.mutable(0) # sub id of cell intersection

        result = locator.IntersectWithLine(rayOrigin, rayEnd, tolerance, lineT, pt, pcoords, subId)

        return pt if result else None


    def visualizeMesh(self, linkName):
        if linkName not in self.linkMeshData:
            print "I can't find a mesh corresponding to " + linkName
            return

        vis.showPolyData(self.linkMeshData[linkName]['polyData'], linkName + ' mesh')

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





