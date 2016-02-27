__author__ = 'manuelli'
import ddapp
from ddapp import roboturdf
import numpy as np
import vtkAll as vtk
import PythonQt
import matplotlib.pyplot as plt
import Queue
import collections


import os
import os.path
import csv
import copy
import time
import itertools
import scipy.stats
import contactfilterutils as cfUtils


from PythonQt import QtCore, QtGui
from ddapp import transformUtils
from ddapp import lcmUtils
from ddapp import contactfiltergurobi
from ddapp.debugVis import DebugData
from ddapp import visualization as vis
from ddapp import gurobiutils as grbUtils
from ddapp.timercallback import TimerCallback
from ddapp import objectmodel as om
from ddapp import contactpointlocator


import drc as lcmdrc
import drake as lcmdrake


FRICTION_CONE_APPROX_SIZE = 4
MU = 0.4

class ContactFilter(object):

    def __init__(self, robotStateModel, robotStateJointController):

        self.robotStateJointController = robotStateJointController
        self.robotStateModel = robotStateModel
        self.initializeConstants()
        self.loadDrakeModelFromFilename()
        self.contactFilterPointDict = dict()
        self.contactFilterPointListAll = []
        self.loadContactFilterPointsFromFile()
        self.running = False
        self.publishResidual = False
        self.doMultiContactEstimate = True
        self.addSubscribers()
        self.initializePublishChannels()
        self.initializeGurobiModel()
        self.initializeColorsForParticleSets()
        self.initializeTestParticleSet()
        self.initializeOptions()
        self.initializeContactPointLocator()

        # self.initializeCellLocator()
        self.setupMotionModelData()
        self.setCurrentUtime(0)

        self.residual = None
        self.particleSetList = []
        self.eventTimes = {'lastContactAdded': 0, 'lastContactRemoved': 0} # should be in simulator time
        self.removedParticleSet = False
        self.mostLikelySolnData = None

        self.initializeTestTimers()


        # self.addTestParticleSetToParticleSetList()


    def start(self):
        self.running = True

    def stop(self):
        self.running = False


    def addSubscribers(self):
        lcmUtils.addSubscriber('RESIDUAL_OBSERVER_STATE_W_FOOT_FT', lcmdrc.residual_observer_state_t,
                               self.onResidualObserverState)
        lcmUtils.addSubscriber('EXTERNAL_FORCE_TORQUE', lcmdrake.lcmt_external_force_torque,
                               self.onExternalForceTorque)

        lcmUtils.addSubscriber("EXTERNAL_CONTACT_LOCATION", lcmdrc.multiple_contact_location_t, self.onExternalContactLocation)

    def initializePublishChannels(self):

        # maybe call it CONTACT_FILTER_POINT_ESTIMATE_PYTHON so that we can compare the results . . .
        self.contactEstimatePublishChannel = "CONTACT_FILTER_POINT_ESTIMATE"

    #
    # def initializeThresholdVars(self):
    #     self.vars['threshold']['lastTime']


    def initializeConstants(self):
        mu = MU
        self.frictionCone = np.array([[mu,-mu,0,0],
                                      [0,0,mu,-mu],
                                      [1,1,1,1]])

    def initializeOptions(self):
        self.options = {}

        self.options['thresholds'] = {}
        self.options['thresholds']['addContactPointTimeout'] = 1.0
        self.options['thresholds']['removeContactPointTimeout'] = 1.0
        self.options['thresholds']['addContactPointSquaredError'] = 10.0 # threshold on squared error to add contact point
        self.options['thresholds']['removeContactPointSquaredError'] = 10.0
        self.options['thresholds']['removeContactPointForce'] = 5.0 # threshold on force magnitude to eliminate a force that gets too small
        self.options['thresholds']['squaredErrorBoundForMostLikelyParticleAveraging'] = 10.0


        self.options['motionModel'] = {}
        self.options['motionModel']['var'] = 0.05

        self.options['measurementModel'] = {}
        self.options['measurementModel']['var'] = 0.05 # this is totally made up at the moment
        self.weightMatrix = np.eye(self.drakeModel.numJoints)

        self.options['numParticles'] = 100


        self.options['debug'] = {}
        self.options['debug']['maxNumParticleSets'] = 4
        self.options['debug']['forceThreshold'] = 1.0
        self.options['debug']['numQPSolves'] = 0
        self.options['debug']['totalQPSolveTime'] = 0.0
        self.options['debug']['jacobianTime'] = 0.0
        self.options['debug']['measurementUpdateTime'] = 0.0
        self.options['debug']['avgQPSolveTime'] = 0.0
        self.options['debug']['haveShownLikelihoodPlot'] = False

        self.options['noise'] = {}
        self.options['noise']['addNoise'] = False
        self.options['noise']['stddev'] = 0.1

        self.options['vis'] = {}
        self.options['vis']['draw'] = False
        self.options['vis']['publish'] = True


    def initializeTestTimers(self):
        self.justAppliedMotionModel = False
        self.particleFilterTestTimer = TimerCallback(targetFps=1)
        self.particleFilterTestTimer.callback = self.testFullParticleFilterCallback

    def initializeCellLocator(self):

        # for now use the zero pose to build the cell locator
        # later we can use something better like the safe nominal pose
        self.locatorData = {}
        polyData = vtk.vtkPolyData()
        self.robotStateModel.model.getModelMeshWithLinkInfoAndNormals(polyData)
        self.locatorData['polyData'] = polyData
        self.locatorData['linkIdArray'] = polyData.GetCellData().GetArray("linkId")
        self.locatorData['normals'] = polyData.GetCellData().GetNormals()
        self.locatorData['linkFrames'] = {}

        linkNames = self.robotStateModel.model.getLinkNames()

        for linkName in linkNames:
            linkName = str(linkName)
            linkFrame = vtk.vtkTransform()
            linkFrame.DeepCopy(self.robotStateModel.getLinkFrame(linkName))
            self.locatorData['linkFrames'][linkName] = linkFrame

        loc = vtk.vtkCellLocator()
        loc.SetDataSet(polyData)
        loc.BuildLocator()
        self.locatorData['locator'] = loc

    def initializeContactPointLocator(self):
        self.contactPointLocator = contactpointlocator.ContactPointLocator(self.robotStateModel)
        self.contactPointLocator.loadCellsFromFile(filename="test2")

    def initializeColorsForParticleSets(self):
        colorList = []

        colorList.append([0.5, 0, 0.5]) # purple
        colorList.append([1,0.64,0]) # orange
        colorList.append([1,1,0]) # yellow
        colorList.append([0.13,0.7,0.66]) # blue-green

        self.colorForParticleSets = itertools.cycle(colorList)


    def addTestParticleSetToParticleSetList(self):
        self.particleSetList.append(self.testParticleSet)


    def loadDrakeModelFromFilename(self, filename=None):
        self.drakeModel = PythonDrakeModel()
        self.drakeModel.loadRobotModelFromURDFFilename(filename)


    def squaredErrorNoContacts(self, verbose=True, residual=None):
        if self.residual is None:
            "don't have a residual, returning"
            return

        if residual is None:
            residual = self.residual

        squaredError =np.dot(np.dot((residual).transpose(), self.weightMatrix),
                                    (residual))

        if verbose:
            print "squared error no contacts", squaredError

        return squaredError


    def loadContactFilterPointsFromFile(self, filename=None):
        if filename is None:
            filename = "directorDense.csv"

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
            bodyId = self.drakeModel.model.findLinkID(linkName)


            outputFrame = vtk.vtkTransform()
            wrenchFrame = vtk.vtkTransform()
            wrenchFrame.Translate(forceLocation)
            forceMomentTransform = transformUtils.forceMomentTransformation(wrenchFrame, outputFrame)

            t = transformUtils.getTransformFromOriginAndNormal([0.0,0.0,0.0], forceDirection)
            rotatedFrictionCone = np.zeros((3,4))
            for i in xrange(0,4):
                rotatedFrictionCone[:,i] = t.TransformVector(self.frictionCone[:,i])


            # need to be careful, the force moment transform is for a wrench, we just have a force
            # J_alpha = 6 x 4, since there are 4 things in the friction cone
            J_alpha = np.dot(forceMomentTransform[:,3:], rotatedFrictionCone)

            contactFilterPoint = ContactFilterPoint(linkName=linkName, contactLocation=forceLocation,
                                  contactNormal=forceDirection, bodyId=bodyId,
                                  forceMomentTransform=forceMomentTransform,
                                  rotatedFrictionCone=rotatedFrictionCone,
                                  J_alpha = J_alpha)

            if self.contactFilterPointDict.has_key(linkName):
                self.contactFilterPointDict[linkName].append(contactFilterPoint)
            else:
                self.contactFilterPointDict[linkName] = [contactFilterPoint]

            self.contactFilterPointListAll.append(contactFilterPoint)

    def setupMotionModelData(self, withinLinkOnly=False):
        # need to make sure you call loadContactFilterPointsFromFile before you get here

        self.motionModelData = {}
        var = self.options['motionModel']['var']

        if withinLinkOnly:
            # only allows motion modle to put positive probability on other particles in the
            # same link
            for linkName, cfpList in self.contactFilterPointDict.iteritems():
                for cfp in cfpList:
                    numCFP = len(cfpList)
                    xk = np.arange(0,numCFP)
                    pk = np.zeros(numCFP)

                    for idx, cfpNext in enumerate(cfpList):
                        distance = np.linalg.norm(cfp.contactLocation - cfpNext.contactLocation)
                        prob = np.exp(-1.0/(2*var)*distance**2) # note that this is not properly normalized
                        pk[idx] = prob

                    pk = pk/np.sum(pk) #normalize the distribution so it is really a probability
                    rv = scipy.stats.rv_discrete(values=(xk,pk))
                    d = {'cfpList': cfpList, 'randomVar': rv}
                    self.motionModelData[cfp] = d


        else: # in this case we allow motion model to move any particle to any other with a given
            # probability. The probability depends only on the cartesian distance between particles
            # in the world frame evaluated at the zero pose of the robot q = zeros.

            # default pose of zeros where we can run doKinematics to figure out
            # the distances between the different cfp's for use in the motion model
            q = np.zeros(self.drakeModel.numJoints)
            self.drakeModel.model.doKinematics(q, 0*q, False, False)

            # compute location, in world frame of all
            worldPosition = {}
            for linkName, cfpList in self.contactFilterPointDict.iteritems():
                linkToWorld = vtk.vtkTransform()
                self.drakeModel.model.getLinkToWorld(linkName, linkToWorld)

                for cfp in cfpList:
                    contactPointInWorld = linkToWorld.TransformPoint(cfp.contactLocation)
                    worldPosition[cfp] = np.array(contactPointInWorld)


            numCFP = len(self.contactFilterPointListAll)
            for cfp in self.contactFilterPointListAll:

                cfpList = self.contactFilterPointListAll
                xk = np.arange(0,numCFP)
                pk = np.zeros(numCFP)

                # compute distance to all other cfp's in the list. This is the distance between
                # them in world frame evaluated at the zero pose. This is just a rough approximation for
                # now
                for idx, cfpNext in enumerate(cfpList):
                    distance = np.linalg.norm(worldPosition[cfp] - worldPosition[cfpNext])
                    prob = np.exp(-1.0/(2*var)*distance**2) # note that this is not properly normalized
                    pk[idx] = prob

                pk = pk/np.sum(pk)
                rv = scipy.stats.rv_discrete(values=(xk,pk))
                d = {'cfpList': cfpList, 'randomVar': rv}
                self.motionModelData[cfp] = d


    def initializeGurobiModel(self):
        # careful here, Mosek models leak memory apparently. I am using gurobi instead
        numContactsList = [1,2,3,4]
        self.gurobi = contactfiltergurobi.ContactFilterGurobi(numContactsList=numContactsList)

    def initializeTestParticleSet(self):
        # creates a particle set with all particles
        self.testParticleSet = SingleContactParticleSet(color=self.colorForParticleSets.next())

        for linkName, cfpList in self.contactFilterPointDict.iteritems():
            for cfp in cfpList:
                particle = ContactFilterParticle(cfp)
                self.testParticleSet.addParticle(particle)

    def createParticleSet(self, onlyUseLinks=[], dontUseLinks=[]):
        linkNames = set(self.contactFilterPointDict.keys())

        if onlyUseLinks and dontUseLinks:
            raise ValueError("can only specify one of the options onlyUseLinks or dontUseLinks, not both")

        if onlyUseLinks:
            linkNames = onlyUseLinks

        if dontUseLinks:
            linkNames = linkNames.difference(dontUseLinks)

        particleSet = SingleContactParticleSet(color=self.colorForParticleSets.next())

        for link in linkNames:
            cfpList = self.contactFilterPointDict[link]
            for cfp in cfpList:
                particle = ContactFilterParticle(cfp)
                particleSet.addParticle(particle)

        return particleSet

    # make sure you call doKinematics before getting here
    def computeJacobianToFrictionCone(self, contactPoint):
        linkJacobian = self.drakeModel.geometricJacobian(0, contactPoint.bodyId, contactPoint.bodyId,
                                                         0, False)

        H = np.dot(linkJacobian.transpose(), contactPoint.J_alpha)
        return H


    # inside this need to setup and solve the QP . . .
    # should have already called doKinematics before you get here

    def computeSingleLikelihood(self, residual, cfpList):


        # this section could be slow
        H_list = []
        for cfp in cfpList:
            H_list.append(self.computeJacobianToFrictionCone(cfp))

        # self.options['debug']['jacobianTime'] += time.time() - startTime

        # this is where the solve is really happening
        numContacts = len(cfpList)
        startTime = time.time()
        grbSolnData = self.gurobi.solve(numContacts, residual, H_list, self.weightMatrix)
        self.options['debug']['totalQPSolveTime'] += time.time() - startTime
        self.options['debug']['numQPSolves'] += 1.0


        alphaVals = np.zeros((numContacts, FRICTION_CONE_APPROX_SIZE))

        for i in xrange(0,numContacts):
            for j in xrange(0, FRICTION_CONE_APPROX_SIZE):
                alphaVals[i,j] = grbSolnData['alphaVals'][i,j]

        cfpData = []
        impliedResidual = 0*residual
        for idx, cfp in enumerate(cfpList):
            d = {'ContactFilterPoint': cfp}
            d['force'] = np.dot(cfp.rotatedFrictionCone, alphaVals[idx,:])
            d['alpha'] = alphaVals[idx,:]
            cfpData.append(d)
            impliedResidual = impliedResidual + np.dot(H_list[idx], alphaVals[idx,:])


        squaredError = np.dot(np.dot((residual - impliedResidual).transpose(), self.weightMatrix),
                                    (residual - impliedResidual))

        likelihood = np.exp(-1/(2.0*self.options['measurementModel']['var'])*squaredError)

        # record the data somehow . . .
        solnData = {'cfpData': cfpData, 'impliedResidual': impliedResidual, 'squaredError': squaredError,
                    "numContactPoints": len(cfpList), 'gurobiObjValue': grbSolnData['objectiveValue'],
                    'likelihood': likelihood, 'time':self.currentTime}
        return solnData

    def computeLikelihoodFull(self, residual, publish=True, verbose=False):


        q = self.getCurrentPose()
        self.drakeModel.model.doKinematics(q, 0*q, False, False)

        startTime = time.time()
        # this stores the current measurement update information
        self.measurementUpdateSolnDataList = []

        if not self.doMultiContactEstimate:
            for linkName, cfpList in self.contactFilterPointDict.iteritems():
                for cfp in cfpList:
                    self.measurementUpdateSolnDataList.append(self.computeSingleLikelihood(residual, [cfp]))


        if self.doMultiContactEstimate:
            activeLinkContactPointList = []
            if len(self.linksWithExternalForce) == 0:
                return

            for linkName in self.linksWithExternalForce:
                activeLinkContactPointList.append(self.contactFilterPointDict[linkName])

            for cfpList in itertools.product(*activeLinkContactPointList):
                solnData = self.computeSingleLikelihood(residual, cfpList)
                self.measurementUpdateSolnDataList.append(solnData)

        elapsedTime = time.time() - startTime
        if verbose:
            print "computing full likelihood took " + str(elapsedTime) + " seconds"


        if publish:
            self.publishMostLikelyEstimate()


    def measurementUpdateSingleParticleSet(self, residual, particleSet, externalParticles = []):
        q = self.getCurrentPose()

        # be careful here, this doKinematics call could be the slow thing? But hopefully not because
        # this call is ultimately getting pushed through to c++
        self.drakeModel.model.doKinematics(q, 0*q, False, False)
        # be smart about it, see if we have already computed the QP for a particle with the same cfp!!!

        alreadySolved = {} # should be a dict with ContactFilterPoint as key, solnData as key
        externalCFPList = []

        for particle in externalParticles:
            externalCFPList.append(particle.cfp)

        for particle in particleSet.particleList:

            # check if we have already solved the problem for this particular contact filter point
            if particle.cfp in alreadySolved:

                # this deepcopy is what's killing us
                # solnDataCopy = copy.deepcopy(alreadySolved[particle.cfp].solnData)
                particle.solnData = alreadySolved[particle.cfp]
            else:
                cfpList = [particle.cfp]
                cfpList.extend(externalCFPList)

                particleList = [particle]
                particleList.extend(externalParticles)

                solnData = self.computeSingleLikelihood(residual, cfpList)
                solnData['force'] = solnData['cfpData'][0]['force']

                # this just makes sure we record the particle in addition to the cfp in the soln data
                for idx, d in enumerate(solnData['cfpData']):
                    d['particle'] = particleList[idx]

                particle.solnData = solnData
                alreadySolved[particle.cfp] = solnData

        # note this doesn't update the most likely particle
        # only do that after doing importance resampling

    def computeMeasurementUpdate(self, residual, publish=True):

        self.options['debug']['numQPSolves'] = 0.0
        self.options['debug']['totalQPSolveTime'] = 0.0
        self.options['debug']['jacobianTime'] = 0.0

        startTime = time.time()

        for particleSet in self.particleSetList:
            otherParticleSets = copy.copy(self.particleSetList)
            otherParticleSets.remove(particleSet)
            externalParticles = []

            for ps in otherParticleSets:
                otherHistoricalMostLikely = ps.historicalMostLikely
                if otherHistoricalMostLikely['particle'] is not None:
                    externalParticles.append(otherHistoricalMostLikely['particle'])

            self.measurementUpdateSingleParticleSet(residual, particleSet, externalParticles=externalParticles)


        self.options['debug']['measurementUpdateTime'] = time.time() - startTime

        if (self.options['debug']['numQPSolves'] > 0):
            self.options['debug']['avgQPSolveTime'] = self.options['debug']['totalQPSolveTime']/self.options['debug']['numQPSolves']
        else:
            self.options['debug']['avgQPSolveTime'] = None

        if publish:
            self.publishMostLikelyEstimate()

        # don't think we should embed this here, just leave it as a separate step
        # self.manageParticleSets()

    def singleMeasurementUpdateForParticleSetRemoval(self, residual, particleSet):
        squaredErrorWithoutParticle = {}
        mostLikelySolnData = particleSet.mostLikelyParticle.solnData

        cfpData = mostLikelySolnData['cfpData']


        cfpListOrig = []
        for d in cfpData:
            cfpListOrig.append(d['ContactFilterPoint'])

        for d in cfpData:
            cfpList = copy.copy(cfpListOrig)
            cfpList.remove(d['ContactFilterPoint'])
            if len(cfpList) == 0:
                squaredErrorWithoutParticle[d['particle']] = self.squaredErrorNoContacts(verbose=False, residual=residual)
            else:
                solnData = self.computeSingleLikelihood(residual, cfpList)
                squaredErrorWithoutParticle[d['particle']] = solnData['squaredError']


        particleSet.squaredErrorWithoutParticle = squaredErrorWithoutParticle


    def measurementUpdateForParticleSetRemoval(self, residual):
        for particleSet in self.particleSetList:
            self.singleMeasurementUpdateForParticleSetRemoval(residual, particleSet)


    def checkTimeoutForSetAdditionRemoval(self):

        val = True

        if (self.currentTime - self.eventTimes['lastContactRemoved']) < self.options['thresholds']['removeContactPointTimeout']:
            val = False

        if (self.currentTime - self.eventTimes['lastContactAdded']) < self.options['thresholds']['addContactPointTimeout']:
            val = False

        return val

    def manageParticleSets(self, verbose=True):

        # solve the QP's that are necessary for particle set removal
        self.measurementUpdateForParticleSetRemoval(self.residual)


        solnData = self.mostLikelySolnData
        newParticleSet = None

        timeoutSatisfied = self.checkTimeoutForSetAdditionRemoval()


        if solnData is None:
            if (self.squaredErrorNoContacts(verbose=False) > self.options['thresholds']['addContactPointSquaredError']):
                newParticleSet = self.createParticleSet()
            else:
                return

        elif solnData['squaredError'] > self.options['thresholds']['addContactPointSquaredError']:
            linksWithContactPoints = set()
            for d in solnData['cfpData']:
                cfp = d['ContactFilterPoint']
                linksWithContactPoints.add(cfp.linkName)


            newParticleSet = self.createParticleSet(dontUseLinks=linksWithContactPoints)

            # this is a hack for now
            # if len(self.particleSetList) >= 2:
            #     print "particle set list is already of size 2, not adding any more"
            #     return

        # this means we have encountered a situation where we should add a new particle set
        # for now will only add one if a sufficient time has passed since we last added a contact
        if newParticleSet is not None:
            if timeoutSatisfied:
                if len(self.particleSetList) >= self.options['debug']['maxNumParticleSets']:
                    if verbose:
                        print "reached max num particle sets"
                        return
                if verbose:
                    print "adding a particle set"
                self.particleSetList.append(newParticleSet)
                self.eventTimes['lastContactAdded'] = self.currentTime

            else:
                if verbose:
                    print "below timeout threshold when trying to ADD a new particle set, returning"
            return

        # if we reach this point it means we are not going to add a ParticleSet
        # however, we may still remove a ParticleSet, this is what we are going
        # to check below

        for particleSet in self.particleSetList:
            squaredErrorWithoutParticle = particleSet.squaredErrorWithoutParticle
            for particle, squaredError in squaredErrorWithoutParticle.iteritems():
                if squaredError < self.options['thresholds']['removeContactPointSquaredError']:
                    if timeoutSatisfied:
                        if verbose:
                            print "removing particle didn't have adverse affect on estimation, REMOVING particle set"
                        particleSetToRemove = particle.containingParticleSet


                        # make sure we don't try to remove a particle set that isn't in the the current
                        # particleSetList
                        if particleSetToRemove in self.particleSetList:
                            self.particleSetList.remove(particleSetToRemove)
                            self.eventTimes['lastContactRemoved'] = self.currentTime
                            # this return statement only allows you to remove a single particle at a time
                        else:
                            if verbose:
                                print "didn't find particle set I am trying to remove in current particle set list"
                    else:
                        if verbose:
                            print "below timeout threshold when trying to REMOVE a new particle set, returning"
                        particleSetToRemove = particle.containingParticleSet


                    # only allow one particle set to be removed in a single pass
                    return

    def applyMotionModelSingleParticleSet(self, particleSet, useNewMotionModel=True):
        for particle in particleSet.particleList:
            cfp = particle.cfp
            if useNewMotionModel:
                cfpNext = self.motionModelSingleCFP(cfp, visualize=False)
            else:
                motionData = self.motionModelData[cfp]
                cfpNextIdx = motionData['randomVar'].rvs()
                cfpNext = motionData['cfpList'][cfpNextIdx]

            particle.cfp = cfpNext

    def applyMotionModel(self):
        for particleSet in self.particleSetList:
            self.applyMotionModelSingleParticleSet(particleSet)


    def importanceResamplingSingleParticleSet(self, particleSet, numParticles=None):
        if numParticles is None:
            numParticles = len(particleSet.particleList)

        newParticleList = []
        numExistingParticles = len(particleSet.particleList)
        xk = np.arange(0,numExistingParticles)
        pk = np.zeros(numExistingParticles)

        for idx, particle in enumerate(particleSet.particleList):
            pk[idx] = particle.solnData['likelihood']

        # normalize the probabilities
        # having some numerical issues here, I think it is because we essentially dividing by zero or something
        # put in a hack that if sumProb < tol, then we just draw from all the particles equally . . .
        sumProb = np.sum(pk)

        tol = 1e-6
        if sumProb < tol:
            print "sum of probabilities really small, falling back to drawing randomly"
            pk = 1.0/numExistingParticles * np.ones(numExistingParticles)
        else:
            pk = pk/np.sum(pk)
        rv = scipy.stats.rv_discrete(values=(xk,pk)) # the random variable with importance weights

        for i in xrange(0,numParticles):
            # draw new particle
            randomIdx = rv.rvs()
            newParticle = particleSet.particleList[randomIdx].deepCopy(keepSolnData=True)
            newParticleList.append(newParticle)

        particleSet.particleList = newParticleList

    def applyImportanceResampling(self):
        for particleSet in self.particleSetList:
            self.importanceResamplingSingleParticleSet(particleSet)


    # takes avg of particles below some threshold
    def updateSingleParticleSetMostLikelyData(self, particleSet, verbose=False):
        smallestSquaredErrorParticle = None
        particlesBelowThreshold = []


        for particle in particleSet.particleList:
            if (smallestSquaredErrorParticle is None or
                    (particle.solnData['squaredError'] < smallestSquaredErrorParticle.solnData['squaredError'])):
                smallestSquaredErrorParticle = particle

            if particle.solnData['squaredError'] < self.options['thresholds']['squaredErrorBoundForMostLikelyParticleAveraging']:
                particlesBelowThreshold.append(particle)


        if len(particlesBelowThreshold) > 0:
            # find particle that is at the average
            numParticles = len(particlesBelowThreshold)
            particleLocationsInWorld = np.zeros((3,numParticles))
            for idx, particle in enumerate(particlesBelowThreshold):
                linkFrame = self.robotStateModel.getLinkFrame(particle.cfp.linkName)
                particleLocationsInWorld[:,idx] = np.array(linkFrame.TransformPoint(particle.cfp.contactLocation))

            particleLocationAvg = np.mean(particleLocationsInWorld, axis=1)

            closestPointData = self.contactPointLocator.findClosestPoint(particleLocationAvg)
            mostLikelyParticle = self.createContactFilterParticleFromClosestPointData(closestPointData,
                                                                                      containingParticleSet = particleSet)
            externalParticleList = self.getExternalMostLikelyParticles(particleSet)
            self.computeSingleLikelihoodForParticle(self.residual, mostLikelyParticle, externalParticleList)
            particleSet.setMostLikelyParticle(self.currentTime, mostLikelyParticle)

            if verbose:
                print "doing average"
        else:
            particleSet.setMostLikelyParticle(self.currentTime, smallestSquaredErrorParticle)
            if verbose:
                print "doing smallest squared error"


    def computeSingleLikelihoodForParticle(self, residual, particle, externalParticleList):

        particleList = [particle]
        particleList.extend(externalParticleList)
        cfpList = []
        for p in particleList:
            cfpList.append(p.cfp)

        solnData = self.computeSingleLikelihood(residual, cfpList)
        solnData['force'] = solnData['cfpData'][0]['force']

        # this just makes sure we record the particle in addition to the cfp in the soln data
        for idx, d in enumerate(solnData['cfpData']):
            d['particle'] = particleList[idx]

        # this isn't working correctly
        particle.solnData = solnData

    def getExternalMostLikelyCFP(self, particleSet):
        otherParticleSets = copy.copy(self.particleSetList)
        otherParticleSets.remove(particleSet)
        externalCFP = []

        for ps in otherParticleSets:
            otherHistoricalMostLikely = ps.historicalMostLikely
            if otherHistoricalMostLikely['particle'] is not None:
                externalCFP.append(otherHistoricalMostLikely['particle'].cfp)

        return externalCFP

    def getExternalMostLikelyParticles(self, particleSet):
        otherParticleSets = copy.copy(self.particleSetList)

        if particleSet in otherParticleSets:
            otherParticleSets.remove(particleSet)
        externalParticles = []

        for ps in otherParticleSets:
            otherHistoricalMostLikely = ps.historicalMostLikely
            if otherHistoricalMostLikely['particle'] is not None:
                externalParticles.append(otherHistoricalMostLikely['particle'])

        return externalParticles


    def updateAllParticleSetsMostLikelyParticle(self, useAvg=True):

        for particleSet in self.particleSetList:
            if useAvg:
                self.updateSingleParticleSetMostLikelyData(particleSet)
            else:
                particleSet.updateMostLikelyParticleUsingMode(self.currentTime)

    # this definitely needs some work
    # overall there are a ton of hacks in here, should get rid of some of them . . . .
    def updateMostLikelySolnData(self):
        if not self.particleSetList:
            self.mostLikelySolnData = None
            # this means that we currently have no particles
            return


        mostLikelySolnData = None
        cfpData = []

        for particleSet in self.particleSetList:
            particle = particleSet.mostLikelyParticle
            if particle is None:
                continue
            if mostLikelySolnData is None:
                mostLikelySolnData = particle.solnData

            cfpData.append(particle.solnData['cfpData'][0])

        mostLikelySolnData['cfpData'] = cfpData

        self.mostLikelySolnData = mostLikelySolnData # store this for debugging and publishing

    # this is a test method
    def computeAndPublishResidual(self, msg):
        if not self.publishResidual:
            return

        residual = np.zeros((self.drakeModel.numJoints,))

        # need to call doKinematics before we can use geometricJacobian
        q = self.getCurrentPose()
        self.drakeModel.model.doKinematics(q, 0*q, False, False)

        for idx, linkName in enumerate(msg.body_names):
            linkName = str(linkName)
            wrench = np.array([msg.tx[idx], msg.ty[idx], msg.tz[idx], msg.fx[idx],
                               msg.fy[idx],msg.fz[idx]])

            bodyId = self.drakeModel.model.findLinkID(linkName)
            linkJacobian = self.drakeModel.geometricJacobian(0, bodyId, bodyId,
                                                         0, False)

            residual = residual + np.dot(linkJacobian.transpose(), wrench)

        self.trueResidual = residual

        msg = lcmdrc.residual_observer_state_t()
        msg.utime = self.currentUtime
        msg.num_joints = self.drakeModel.numJoints
        msg.joint_name = self.drakeModel.jointNames
        msg.residual = residual
        msg.gravity = 0*residual
        msg.internal_torque = 0*residual
        msg.foot_contact_torque = 0*residual

        lcmUtils.publish("TRUE_RESIDUAL", msg)

    def setCurrentUtime(self, utime):
        self.currentUtime = utime
        self.currentTime = 1.0*utime/1e6


    def publishMostLikelyEstimate(self):
        # if self.mostLikelySolnData is None:
        #     return
        self.publishEstimate(self.mostLikelySolnData)

    def getCFPLocationInWorld(self, cfp):
        linkFrame = self.robotStateModel.getLinkFrame(cfp.linkName)
        contactLocationInWorld = linkFrame.TransformPoint(cfp.contactLocation)

        return contactLocationInWorld


    # currently only support single contact
    def publishEstimate(self, solnData):


        if solnData is None:
            msg = lcmdrc.contact_filter_estimate_t()
            msg.utime = self.currentUtime
            msg.num_contact_points = 0
            msg.logLikelihood = self.squaredErrorNoContacts(verbose=False)
            lcmUtils.publish(self.contactEstimatePublishChannel, msg)
            return

        msg = lcmdrc.contact_filter_estimate_t()
        msg.utime = self.currentUtime
        msg.num_contact_points = solnData['numContactPoints']

        msg.num_velocities = self.drakeModel.numJoints
        msg.logLikelihood = solnData['squaredError']
        msg.velocity_names = self.drakeModel.jointNames
        msg.implied_residual = solnData['impliedResidual']

        msg.single_contact_estimate = [None]*msg.num_contact_points

        msgEstimatedContactLocations = lcmdrc.multiple_contact_location_t()
        msgEstimatedContactLocations.num_contacts = msg.num_contact_points

        for i in xrange(0, msg.num_contact_points):
            msg.single_contact_estimate[i] = self.msgFromSolnCFPData(solnData['cfpData'][i])

            cfp = solnData['cfpData'][i]['ContactFilterPoint']
            contactLocationInWorld = self.getCFPLocationInWorld(cfp)
            msgContactLocation = lcmdrc.contact_location_t()
            msgContactLocation.link_name = cfp.linkName
            msgContactLocation.contact_location_in_world = contactLocationInWorld
            msgEstimatedContactLocations.contacts.append(msgContactLocation)

        lcmUtils.publish(self.contactEstimatePublishChannel, msg)

        msgAllContactLocations = lcmdrc.actual_and_estimated_contact_locations_t()
        msgAllContactLocations.utime = self.currentUtime
        msgAllContactLocations.actual_contact_location = self.externalContactLocationMsg
        msgAllContactLocations.estimated_contact_location = msgEstimatedContactLocations
        lcmUtils.publish("ACTUAL_AND_ESTIMATED_CONTACT_LOCATIONS", msgAllContactLocations)

    def msgFromSolnCFPData(self, d):
        msg = lcmdrc.single_contact_filter_estimate_t()
        msg.body_name = d['ContactFilterPoint'].linkName
        msg.contact_force = d['force']
        msg.contact_normal = d['ContactFilterPoint'].contactNormal
        msg.contact_position = d['ContactFilterPoint'].contactLocation

        return msg


    def getCurrentPose(self):
        return self.robotStateJointController.q

    def onResidualObserverState(self, msg):
        self.setCurrentUtime(msg.utime)
        msgJointNames = msg.joint_name
        msgData = msg.residual

        residual = self.drakeModel.extractDataFromMessage(msgJointNames, msgData)
        self.residual = residual

        if self.options['noise']['addNoise']:
            residualSize = np.size(self.residual)
            self.residual = self.residual + np.random.normal(scale=self.options['noise']['stddev'], size=residualSize)

        if self.running:
            self.contactParticleFilterStep(self.residual, drawParticleSets=self.options['vis']['draw'],
                                           applyMotionModel=True)


    def contactParticleFilterStep(self, residual, drawParticleSets=True, applyMotionModel=True):

        if applyMotionModel:
            self.applyMotionModel()

        # if len(self.particleSetList) == 0:
        #     self.manageParticleSets(verbose=True)

        self.computeMeasurementUpdate(self.residual, publish=False)
        self.applyImportanceResampling()
        self.updateAllParticleSetsMostLikelyParticle()
        self.updateMostLikelySolnData()
        self.publishMostLikelyEstimate()
        if self.options['vis']['publish']:
            self.publishVisualizationData()
        self.manageParticleSets(verbose=True) # there are timeouts inside of this

        if drawParticleSets:
            self.testParticleSetDrawAll(drawMostLikely=True, drawHistoricalMostLikely=True)



    def onExternalForceTorque(self, msg):
        self.linksWithExternalForce = [str(linkName) for linkName in msg.body_names]
        self.computeAndPublishResidual(msg)

    def onExternalContactLocation(self, msg):
        self.externalContactLocationMsg = msg

    def resetParticleFilter(self):
        self.stop()
        self.particleSetList = []

    def drawParticleSet(self, particleSet, name="particle set", color=None, drawMostLikely=True,
                        drawHistoricalMostLikely=True):

        # set the color if it was passed in
        defaultColor = [0.5,0,0.5]
        mostLikelyColor = [1,0.4,0.7] # hot pink
        historicalMostLikelyColor = [1,0,0]

        if color is not None:
            defaultColor = color

        numParticlesAtCFP = {}
        numTotalParticles = len(particleSet.particleList)

        for particle in particleSet.particleList:
            cfp = particle.cfp
            if numParticlesAtCFP.has_key(cfp):
                numParticlesAtCFP[cfp] += 1
            else:
                numParticlesAtCFP[cfp] = 1

        # now we need to draw this
        plungerMaxLength = 0.4
        plungerMinLength = 0.02

        d = DebugData()
        q = self.getCurrentPose()
        for cfp, numParticles in numParticlesAtCFP.iteritems():
            color = defaultColor

            # if particleSet.mostLikelyParticle is not None:
            #     if cfp == particleSet.mostLikelyParticle.cfp:
            #         color = mostLikelyColor

            rayLength = plungerMinLength + 1.0*numParticles/numTotalParticles*plungerMaxLength
            self.addPlungerToDebugData(d, cfp.linkName, cfp.contactLocation, cfp.contactNormal, rayLength, color)
            # forceDirectionWorldFrame, forceLocationWorldFrame =\
            #     cfUtils.getForceDirectionInWorld(q, self.robotStateModel,
            #                                                             cfp.linkName,
            #                                                             cfp.contactLocation,
            #                                                             cfp.contactNormal)
            #
            # rayEnd = forceLocationWorldFrame - forceDirectionWorldFrame*rayLength
            # d.addSphere(forceLocationWorldFrame, radius=0.01, color=color)
            # d.addLine(rayEnd, forceLocationWorldFrame, radius = 0.005, color=color)

        if drawHistoricalMostLikely and (particleSet.historicalMostLikely['particle'] is not None):
            particle = particleSet.historicalMostLikely['particle']
            cfp = particle.cfp
            color = historicalMostLikelyColor
            rayLength = 0.3
            forceDirection = cfp.contactNormal
            if particle.solnData is not None:
                forceDirection = particle.solnData['force']
                forceDirection = forceDirection/np.linalg.norm(forceDirection)
            self.addPlungerToDebugData(d, cfp.linkName, cfp.contactLocation, forceDirection, rayLength, color)

        if drawMostLikely and (particleSet.mostLikelyParticle is not None):
            particle = particleSet.mostLikelyParticle
            cfp = particle.cfp
            color = mostLikelyColor
            rayLength = 0.4

            forceDirection = cfp.contactNormal
            if particle.solnData is not None:
                forceDirection = particle.solnData['force']
                forceDirection = forceDirection/np.linalg.norm(forceDirection)

            self.addPlungerToDebugData(d, cfp.linkName, cfp.contactLocation, forceDirection, rayLength, color)

        vis.showPolyData(d.getPolyData(), name, colorByName='RGB255')

    def addPlungerToDebugData(self, d, linkName, contactLocation, contactDirection, rayLength, color):
        q = self.getCurrentPose()
        forceDirectionWorldFrame, forceLocationWorldFrame =\
                cfUtils.getForceDirectionInWorld(q, self.robotStateModel,
                                                                        linkName,
                                                                        contactLocation,
                                                                        contactDirection)

        rayEnd = forceLocationWorldFrame - forceDirectionWorldFrame*rayLength
        d.addSphere(forceLocationWorldFrame, radius=0.01, color=color)
        d.addLine(rayEnd, forceLocationWorldFrame, radius = 0.005, color=color)


    def drawContactFilterPoint(self, cfp, name="test cfp"):
        d = DebugData()
        rayLength = 0.1
        color=[0,1,0]
        self.addPlungerToDebugData(d,cfp.linkName, cfp.contactLocation, cfp.contactNormal, rayLength, color)
        vis.updatePolyData(d.getPolyData(), name, colorByName="RGB255")

    def testParticleSetDraw(self):
        self.drawParticleSet(self.testParticleSet)

    def testParticleSetDrawAll(self, drawMostLikely=False, drawHistoricalMostLikely=True):
        # colorList = []
        #
        # colorList.append([0.5, 0, 0.5]) # purple
        # colorList.append([1,0.64,0]) # orange
        # colorList.append([1,1,0]) # yellow
        # colorList.append([0.13,0.7,0.66]) # blue-green

        numParticleSets = len(self.particleSetList)
        maxNumParticleSets = 4
        for i in xrange(0,maxNumParticleSets):
            name = "particle set " + str(i+1)
            om.removeFromObjectModel(om.findObjectByName(name))

            if i < numParticleSets:
                self.drawParticleSet(self.particleSetList[i], name=name, color=self.particleSetList[i].color,
                                     drawMostLikely=drawMostLikely, drawHistoricalMostLikely=drawHistoricalMostLikely)


    def testFullParticleFilterCallback(self, verbose=False):
        if self.residual is None:
            return

        # make sure we can try to add a particle set if we need to
        if len(self.particleSetList) == 0:
            self.manageParticleSets(verbose=True)
            self.justAppliedMotionModel=True

        if not self.justAppliedMotionModel:
            if verbose:
                print "applying motion model"
            self.applyMotionModel()
            if self.options['vis']['draw']:
                self.testParticleSetDrawAll(drawMostLikely=False, drawHistoricalMostLikely=True)
            self.justAppliedMotionModel = True
        else:
            if verbose:
                print "measurement update and importance resampling"
            # self.measurementUpdateSingleParticleSet(self.residual, self.testParticleSet)
            # self.importanceResamplingSingleParticleSet(self.testParticleSet)
            self.computeMeasurementUpdate(self.residual, publish=False)
            self.applyImportanceResampling()
            self.updateAllParticleSetsMostLikelyParticle()
            self.updateMostLikelySolnData()
            self.publishMostLikelyEstimate()
            self.manageParticleSets(verbose=True) # there are timeouts inside of this
            if self.options['vis']['draw']:
                self.testParticleSetDrawAll(drawMostLikely=True, drawHistoricalMostLikely=True)

            if self.options['vis']['publish']:
                self.publishVisualizationData()

            self.justAppliedMotionModel = False

    def printDebugData(self):
        print "total measurement update time", self.options['debug']['measurementUpdateTime']
        print "total QP solve time", self.options['debug']['totalQPSolveTime']
        print "avg QP solve time", self.options['debug']['avgQPSolveTime']
        print "numQPSolves", self.options['debug']['numQPSolves']
        # print "jacobianTime", self.options['debug']['jacobianTime']

    def testLikelihood(self, numContacts = 2):
        cfpList = [self.contactFilterPointDict['pelvis'][0]]

        if numContacts > 1:
            cfpList = self.contactFilterPointDict['pelvis'][0:numContacts]

        residual = np.zeros(self.drakeModel.numJoints)
        # since we aren't calling it via computeLikelihoodFull we need to manually call doKinematics
        q = self.getCurrentPose()
        self.drakeModel.model.doKinematics(q, 0*q, False, False)
        solnData = self.computeSingleLikelihood(residual, cfpList)

        return solnData

    def testLikelihoodFull(self):
        residual = np.zeros(self.drakeModel.numJoints)
        self.computeLikelihoodFull(residual, verbose=True)

    def testMeasurementUpdate(self, verbose=False):

        if self.residual is None:
            print "didn't find residual, using all zeros"
            residual = np.zeros(self.drakeModel.numJoints)
        else:
            residual = self.residual
        startTime = time.time()
        self.measurementUpdateSingleParticleSet(residual, self.testParticleSet)
        elapsed = time.time() - startTime

        self.testParticleSet.updateMostLikelyParticle(self.currentTime)
        particle = self.testParticleSet.mostLikelyParticle
        self.testParticleSetDraw()

        if verbose:
            print "single measurement update took " + str(elapsed) + " seconds"
            particle.printObject()


    # these are all test methods
    def startFilterTest(self):
        self.particleFilterTestTimer.start()

    def stopFilterTest(self):
        self.particleFilterTestTimer.stop()

    def removeStaleParticleDraw(self):
        om.removeFromObjectModel(om.findObjectByName('particle set'))


    def testLASSOSolve(self, lam=1, linkNames=None):
        if linkNames is None:
            linkNames = self.contactFilterPointDict.keys()


        cfpList = []
        for bodyName in linkNames:
            cfpListTemp = self.contactFilterPointDict[bodyName]
            cfpList = cfpList + cfpListTemp


        numContacts = len(cfpList)
        d = self.gurobi.createLassoModel(numContacts)
        residual = self.residual

        H_list = []
        for cfp in cfpList:
            H_list.append(self.computeJacobianToFrictionCone(cfp))

        # this is where the solve is really happening
        self.gurobi.solveLasso(d, residual, H_list, self.weightMatrix, lam)


        alphaVals = np.zeros((numContacts, FRICTION_CONE_APPROX_SIZE))

        for i in xrange(0,numContacts):
            for j in xrange(0, FRICTION_CONE_APPROX_SIZE):
                alphaVals[i,j] = d['alphaVars'][i,j].getAttr('X')


        impliedResidual = 0
        cfpData = []
        for idx, cfp in enumerate(cfpList):
            data = {'ContactFilterPoint': cfp}
            data['force'] = np.dot(cfp.rotatedFrictionCone, alphaVals[idx,:])
            data['alpha'] = alphaVals[idx,:]
            cfpData.append(data)
            impliedResidual = impliedResidual + np.dot(H_list[idx], alphaVals[idx,:])


        squaredError = np.dot(np.dot((residual - impliedResidual).transpose(), self.weightMatrix),
                                    (residual - impliedResidual))

        # record the data somehow . . .
        solnData = {'cfpData': cfpData, 'impliedResidual': impliedResidual, 'squaredError': squaredError,
                    "numContactPoints": len(cfpList), 'time': self.currentTime}

        self.testPlotCFPData(cfpData)
        return d, solnData


    def testImportanceResampling(self):
        self.importanceResamplingSingleParticleSet(self.testParticleSet, numParticles=None)
        self.testParticleSetDraw()

    def testMotionModel(self):
        self.applyMotionModelSingleParticleSet(self.testParticleSet)
        self.testParticleSetDraw()


    def testPlotCFPData(self, cfpData, name="cfp data", verbose=True):
        d = DebugData()

        color = [0,0,1]
        rayLength = 0.2
        q = self.getCurrentPose()


        for data in cfpData:
            cfp = data['ContactFilterPoint']
            force = data['force']
            if np.linalg.norm(force) < self.options['debug']['forceThreshold']:
                continue
            forceDirection = force/np.linalg.norm(force)

            if verbose:
                print ""
                print "contact on ", cfp.linkName
                print "force magnitude is ", np.linalg.norm(force)
                print ""
            self.addPlungerToDebugData(d, cfp.linkName, cfp.contactLocation, forceDirection, rayLength, color)


        vis.updatePolyData(d.getPolyData(), name, colorByName='RGB255')


    def barPlot(self, data, title=None):

        if title is not None:
            plt.title(title)

        barWidth = 0.5
        numBars = np.size(data)
        index = np.arange(0,numBars)
        plt.bar(index, data, barWidth/2.0)


    def plotLikelihoodData(self, particleSet=None):

        if not self.options['debug']['haveShownLikelihoodPlot']:
            plt.figure()

        barWidth = 0.5
        barCounter = 0


        # draw the test particle set by default
        if particleSet is None:
            particleSet = self.testParticleSet

        cfpPlotted = set()
        likelihood = []
        squaredError = []
        # importanceWeights = []

        for particle in particleSet.particleList:
            cfp = particle.cfp

            # skip if we have already logged the data for this particular cfp
            if cfp in cfpPlotted:
                continue

            cfpPlotted.add(cfp)

            likelihood.append(particle.solnData['likelihood'])
            squaredError.append(particle.solnData['squaredError'])

        # bookkeeping
        likelihood = np.array(likelihood)
        squaredError = np.array(squaredError)
        importanceWeights = likelihood/np.sum(likelihood)

        plt.clf()

        plt.subplot(3,1,1)
        self.barPlot(squaredError, title="Squared Error")

        plt.subplot(3,1,2)
        self.barPlot(likelihood, title="Likelihood")

        plt.subplot(3,1,3)
        self.barPlot(importanceWeights, title="Importance Weights")


        if not self.options['debug']['haveShownLikelihoodPlot']:
            self.options['debug']['haveShownLikelihoodPlot'] = True
            plt.show()
        else:
            plt.draw()



    def testLocator(self, point=[0.0, 0.0, 0.0], verbose=True):
        cell = vtk.vtkGenericCell()
        cellId = vtk.mutable(0)
        subId = vtk.mutable(0)
        dist2 = vtk.mutable(0)
        closestPoint = [0.0, 0.0, 0.0]

        self.locatorData['locator'].FindClosestPoint(point, closestPoint, cellId, subId, dist2)
        linkId = int(self.locatorData['linkIdArray'].GetTuple(cellId)[0])
        linkName = self.robotStateModel.model.getBodyOrFrameName(linkId)
        normal = np.array(self.locatorData['normals'].GetTuple(cellId))
        if verbose:
            # also want to transform it to local frame
            linkToWorld = self.locatorData['linkFrames'][linkName]
            worldToLink = linkToWorld.GetLinearInverse()
            closestPointInLinkFrame = worldToLink.TransformPoint(closestPoint)

            normalLinkFrame = worldToLink.TransformVector(normal)

            d = DebugData()
            d.addSphere(point, radius=0.03, color=[1,0,0])
            d.addSphere(closestPoint, radius=0.03, color=[0,1,0])
            om.removeFromObjectModel(om.findObjectByName("locator data"))
            vis.showPolyData(d.getPolyData(),name="locator data",colorByName="RGB255")

            print "-------- Closest Point Data -------------"
            print "linkId = ", linkId
            print "link name = " + linkName
            print "closest point = ", closestPoint
            print "closest point in link frame = ", closestPointInLinkFrame
            print "normal = ", normal
            print "normal link frame = ", normalLinkFrame
            print " ------------------------------- "
            print ""


    def createTestCFP(self):
        self.testCFP = self.contactFilterPointDict['l_uarm'][0]


    def findClosestPoint(self, point):
        cell = vtk.vtkGenericCell()
        cellId = vtk.mutable(0)
        subId = vtk.mutable(0)
        dist2 = vtk.mutable(0)
        closestPoint = [0.0, 0.0, 0.0]
        self.locatorData['locator'].FindClosestPoint(point, closestPoint, cellId, subId, dist2)
        linkId = int(self.locatorData['linkIdArray'].GetTuple(cellId)[0])
        linkName = self.robotStateModel.model.getBodyOrFrameName(linkId)


        # NOTE: I AM PUTTING A NEGATIVE SIGN HERE AS A TEMPORARY HACK
        normal = -np.array(self.locatorData['normals'].GetTuple(cellId))

        closestPointData = {'closestPoint': closestPoint, 'linkName': linkName, 'normal': normal, 'dist': dist2}
        return closestPointData

    def motionModelSingleCFP(self, cfp, visualize=False, tangentSampling=False):

        linkToWorld = self.robotStateModel.getLinkFrame(cfp.linkName)
        contactLocationWorldFrame = linkToWorld.TransformPoint(cfp.contactLocation)

        contactNormalWorldFrame = linkToWorld.TransformVector(cfp.contactNormal)
        randomVector = np.random.random(3)
        tangentVector = np.cross(cfp.contactNormal, contactNormalWorldFrame)
        tangentVector = tangentVector/np.linalg.norm(tangentVector)

        if tangentSampling:
            deltaToNewContactLocation = tangentVector*np.random.normal(scale=self.options['motionModel']['var'], size=1)
        else:
            deltaToNewContactLocation = np.random.normal(scale=self.options['motionModel']['var'], size=3)

        closestPointLookupLocation = contactLocationWorldFrame + deltaToNewContactLocation

        closestPointData = self.contactPointLocator.findClosestPoint(closestPointLookupLocation)


        newLinkName = closestPointData['linkName']
        # worldToLink = self.robotStateModel.getLinkFrame(newLinkName).GetLinearInverse()
        # newContactLocation = worldToLink.TransformPoint(closestPointData['closestPoint'])
        # newContactNormal = worldToLink.TransformVector(closestPointData['normal'])
        newContactLocation = closestPointData['closestPoint']
        newContactNormal = closestPointData['normal']
        bodyId = self.drakeModel.model.findLinkID(newLinkName)

        newCFP = self.createContactFilterPoint(linkName=newLinkName, contactLocation=newContactLocation,
                                    contactNormal=newContactNormal, bodyId=bodyId)

        if visualize:
            d = DebugData()
            d.addSphere(contactLocationWorldFrame, radius=0.01, color=[0,0,1])
            d.addSphere(closestPointLookupLocation, radius=0.01, color=[1,0,0])
            vis.updatePolyData(d.getPolyData(), "locator data", colorByName="RGB255")
            self.drawContactFilterPoint(newCFP)

        return newCFP

    def createContactFilterPointFromClosestPointData(self, closestPointData):
        newLinkName = closestPointData['linkName']
        # worldToLink = self.robotStateModel.getLinkFrame(newLinkName).GetLinearInverse()
        # newContactLocation = worldToLink.TransformPoint(closestPointData['closestPoint'])
        # newContactNormal = worldToLink.TransformVector(closestPointData['normal'])
        newContactLocation = closestPointData['closestPoint']
        newContactNormal = closestPointData['normal']
        bodyId = self.drakeModel.model.findLinkID(newLinkName)

        newCFP = self.createContactFilterPoint(linkName=newLinkName, contactLocation=newContactLocation,
                                    contactNormal=newContactNormal, bodyId=bodyId)

        return newCFP

    def createContactFilterParticleFromClosestPointData(self, closestPointData, containingParticleSet=None):
        if containingParticleSet is None:
            raise ValueError('must specify a containing particle set')
        cfp = self.createContactFilterPointFromClosestPointData(closestPointData)
        particle = ContactFilterParticle(cfp=cfp)
        particle.containingParticleSet = containingParticleSet
        return particle


    def createContactFilterPoint(self, linkName=None, contactLocation=None,
                                    contactNormal=None, bodyId=None):
        outputFrame = vtk.vtkTransform()
        wrenchFrame = vtk.vtkTransform()
        wrenchFrame.Translate(contactLocation)
        forceMomentTransform = transformUtils.forceMomentTransformation(wrenchFrame, outputFrame)

        t = transformUtils.getTransformFromOriginAndNormal([0.0,0.0,0.0], contactNormal)
        rotatedFrictionCone = np.zeros((3,4))
        for i in xrange(0,4):
            rotatedFrictionCone[:,i] = t.TransformVector(self.frictionCone[:,i])


        # need to be careful, the force moment transform is for a wrench, we just have a force
        # J_alpha = 6 x 4, since there are 4 things in the friction cone
        J_alpha = np.dot(forceMomentTransform[:,3:], rotatedFrictionCone)

        contactFilterPoint = ContactFilterPoint(linkName=linkName, contactLocation=contactLocation,
                              contactNormal=contactNormal, bodyId=bodyId,
                              forceMomentTransform=forceMomentTransform,
                              rotatedFrictionCone=rotatedFrictionCone,
                              J_alpha = J_alpha)

        return contactFilterPoint


    def testNewMotionModel(self):
        self.testCFP = self.motionModelSingleCFP(self.testCFP, visualize=True)

    def testNewMotionModelFull(self):
        self.applyMotionModelSingleParticleSet(self.testParticleSet)
        self.testParticleSetDraw()

    @staticmethod
    def encodeParticle(utime, particle):
        if type(particle) is not ContactFilterParticle:
            print "particle is of type", type(particle)
        assert type(particle) is ContactFilterParticle

        msg = lcmdrc.CPF_particle_t()
        msg.utime = utime
        msg.link_name = particle.cfp.linkName
        msg.contact_location = particle.cfp.contactLocation.tolist()
        msg.contact_normal = particle.cfp.contactNormal.tolist()

        if particle.solnData is not None:
            msg.contact_force = particle.solnData['force'].tolist()
        else:
            msg.contact_force = particle.cfp.contactNormal.tolist()

        return msg

    @staticmethod
    def encodeParticleSet(utime, particleSet):
        assert type(particleSet) is SingleContactParticleSet

        msg = lcmdrc.CPF_particle_set_t()
        msg.utime = utime

        msg.num_particles = particleSet.getNumberOfParticles()
        msg.particle_list = msg.num_particles *[None]

        for idx, particle in enumerate(particleSet.particleList):
            msg.particle_list[idx] = ContactFilter.encodeParticle(utime, particle)


        msg.most_likely_particle = ContactFilter.encodeParticle(utime, particleSet.mostLikelyParticle)
        msg.historical_most_likely_particle = ContactFilter.encodeParticle(utime, particleSet.historicalMostLikely['particle'])
        msg.color = particleSet.color
        return msg


    @staticmethod
    def encodeCPFData(utime, particleSetList):

        msg = lcmdrc.CPF_data_t()
        msg.utime = utime

        msg.num_particle_sets = len(particleSetList)
        msg.particle_sets = msg.num_particle_sets * [None]

        for idx, particleSet in enumerate(particleSetList):
            msg.particle_sets[idx] = ContactFilter.encodeParticleSet(utime, particleSet)


        return msg


    @staticmethod
    def decodeCPFData(msg):
        particleSetList = []
        for particleSetMsg in msg.particle_sets:
            particleSetList.append(ContactFilter.decodeParticleSet(particleSetMsg))

        return particleSetList

    @staticmethod
    def decodeParticleSet(msg):
        particleSet = SingleContactParticleSet()
        particleSet.color = msg.color
        particleSet.mostLikelyParticle = ContactFilter.decodeParticle(msg.most_likely_particle)
        particleSet.historicalMostLikely = {'particle': ContactFilter.decodeParticle(msg.historical_most_likely_particle)}

        for particleMsg in msg.particle_list:
            particleSet.addParticle(ContactFilter.decodeParticle(particleMsg))

        return particleSet


    @staticmethod
    def decodeParticle(msg):
        cfp = ContactFilterPoint(linkName=msg.link_name, contactLocation=msg.contact_location, contactNormal=msg.contact_normal, bodyId=1,
                                 forceMomentTransform=1, rotatedFrictionCone=1, J_alpha=1)
        particle = ContactFilterParticle(cfp=cfp)
        particle.solnData = {'force':np.array(msg.contact_force)}
        return particle

    def publishVisualizationData(self):
        msg = ContactFilter.encodeCPFData(self.currentUtime, self.particleSetList)
        lcmUtils.publish("CONTACT_PARTICLE_FILTER_DATA", msg)

    def testDecodeCFPData(self):
        msg = ContactFilter.encodeCPFData(self.currentUtime, self.particleSetList)
        return ContactFilter.decodeCPFData(msg)



class PythonDrakeModel(object):

    def __init__(self):
        self.loadRobotModelFromURDFFilename()
        self.jointMap = self.getJointMap()
        self.jointNames = self.model.getJointNames()


    def loadRobotModelFromURDFFilename(self, filename=None):
        if filename is None:
            drcBase = os.getenv("DRC_BASE")
            # modelName = "model_LR_RR.urdf"
            modelName = "model_chull.urdf"
            filename = drcBase + "/software/models/atlas_v5/" + modelName


        self.model = PythonQt.dd.ddDrakeModel()
        if not self.model.loadFromFile(filename):
            print "failed to load model"

        self.nv = self.model.numberOfJoints()
        self.numJoints = self.model.numberOfJoints()

    def getJointMap(self):

        jointNames = self.model.getJointNames()

        jointMap = dict()

        for idx, jointName in enumerate(jointNames):
            jointName = str(jointName)
            jointMap[jointName] = idx

        return jointMap

    def extractDataFromMessage(self, msgJointNames, msgData):

        msgJointMap = {}
        for msgName, msgData in zip(msgJointNames, msgData):
            msgJointMap[msgName] = msgData


        data = np.zeros(self.numJoints)
        for jointName, idx in self.jointMap.iteritems():
            data[idx] = msgJointMap[jointName]


        return data


    def geometricJacobian(self, base_body_or_frame_ind, end_effector_body_or_frame_id,
                          expressed_in_body_or_frame_ind, gradient_order, in_terms_of_qdot=False):

        linkJacobianVec = np.array(self.model.geometricJacobian(base_body_or_frame_ind, end_effector_body_or_frame_id,
                          expressed_in_body_or_frame_ind, gradient_order, in_terms_of_qdot));

        linkJacobian = linkJacobianVec.reshape(6,self.nv)

        return linkJacobian

    def testGeometricJacobian(self):
        q = np.zeros(self.nv)
        self.model.doKinematics(q,0*q,False, False)
        return self.geometricJacobian(0,1,1,0,False)

class ContactFilterPoint(object):

    def __init__(self, linkName=None, contactLocation=None, contactNormal=None, bodyId=None, forceMomentTransform=None,
                 rotatedFrictionCone=None, J_alpha = None):

        optionalArgsList = [linkName, contactLocation, contactNormal, bodyId, forceMomentTransform, rotatedFrictionCone, J_alpha]
        if None in optionalArgsList:
            raise ValueError("must specify all the optional input arguments")

        self.linkName = linkName
        self.contactLocation = np.array(contactLocation)
        self.contactNormal = np.array(contactNormal)
        self.bodyId = bodyId
        self.forceMomentTransform = forceMomentTransform
        self.rotatedFrictionCone = rotatedFrictionCone
        self.J_alpha = J_alpha


    def printObject(self):
        print "linkName", self.linkName
        print "bodyId", self.bodyId
        print "contactLocation", self.contactLocation
        print "contactNormal", self.contactNormal
        print "forceMomentTransform", self.forceMomentTransform
        print "rotatedFrictionCone", self.rotatedFrictionCone
        print "J_alpha", self.J_alpha

    def printObjectShort(self):
        print "linkName", self.linkName
        print "bodyId", self.bodyId
        print "contactLocation", self.contactLocation


class ContactFilterParticle(object):

    def __init__(self, cfp=None):
        self.solnData = None #this records soln data from QP solves
        if cfp is not None:
            self.setContactFilterPoint(cfp)

        self.containingParticleSet = None

    def setContactFilterPoint(self, cfp):
        assert type(cfp) is ContactFilterPoint, "cfp is not of type ContactFilterPoint"
        self.cfp = cfp

    def setContainingParticleSet(self, containingParticleSet):
        self.containingParticleSet = containingParticleSet

    def printObject(self):
        self.cfp.printObjectShort()

        if self.solnData is not None:
            print "squared error = ", self.solnData['squaredError']
            print "force in body frame = ", self.solnData['force']

    def deepCopy(self, keepSolnData=False):
        newParticle = ContactFilterParticle(cfp=self.cfp)
        newParticle.setContainingParticleSet(self.containingParticleSet)

        if keepSolnData:
            # this is only temporary, used for updateMostLikelySolnData
            # should be overwritten by the next measurementUpdate . . .
            newParticle.solnData = self.solnData
        return newParticle


class SingleContactParticleSet(object):

    def __init__(self, solnDataQueueTimeout=0.5, color=[0,0,1]):
        self.particleList = []
        self.mostLikelyParticle = None
        self.historicalMostLikely = {'solnData': None, 'particle': None}
        self.solnDataTimeout = solnDataQueueTimeout
        self.solnDataSet = []
        self.squaredErrorWithoutParticle = {}
        self.color = color


    def addParticle(self, particle):
        self.particleList.append(particle)
        particle.setContainingParticleSet(self)

    # will need to update this when we go to the continuous version. For right now let it be the
    # mode of the distribution
    def updateMostLikelyParticle(self, currentTime):
        bestSquaredError = None

        for particle in self.particleList:

            squaredError = particle.solnData['squaredError']
            if (bestSquaredError is None) or (squaredError < bestSquaredError):
                bestSquaredError = squaredError
                self.mostLikelyParticle = particle

        self.updateSolnDataSet(currentTime, solnData=self.mostLikelyParticle.solnData)


    # choose the most likely particle to be the mode of the particle set
    # make sure that you perform importance resampling before you get here
    def updateMostLikelyParticleUsingMode(self, currentTime):
        bestCFP = None
        bestParticle = None

        cfpCounterDict = {}

        for particle in self.particleList:
            cfp = particle.cfp
            if cfpCounterDict.has_key(cfp):
                cfpCounterDict[cfp]['numParticles'] += 1
            else:
                cfpCounterDict[cfp] = {'numParticles': 1, 'particle': particle}

        maxNumParticles = 0
        mostLikelyParticle = None
        for cfp, d in cfpCounterDict.iteritems():
            if d['numParticles'] > maxNumParticles:
                maxNumParticles = d['numParticles']
                mostLikelyParticle = d['particle']


        # bookkeeping
        self.cfpCounterDict  = cfpCounterDict # this is for debugging purposes
        self.mostLikelyParticle = mostLikelyParticle
        self.updateSolnDataSet(currentTime, solnData=self.mostLikelyParticle.solnData)

    def setMostLikelyParticle(self, currentTime, mostLikelyParticle):
        self.mostLikelyParticle = mostLikelyParticle
        self.updateSolnDataSet(currentTime, solnData = self.mostLikelyParticle.solnData)

    def updateSolnDataSet(self, currentTime, solnData=None):
        self.solnData = solnData
        if solnData is not None:
            self.solnDataSet.append(solnData)
        self.cleanupSet(currentTime)
        self.updateHistoricalMostLikely()

    def cleanupSet(self, currentTime):
        toRemove = []
        for solnData in self.solnDataSet:
            if (currentTime - solnData['time']) > self.solnDataTimeout:
                toRemove.append(solnData)

        cfUtils.removeElementsFromList(self.solnDataSet, toRemove)

    def updateHistoricalMostLikely(self):
        bestSquaredError = None
        for solnData in self.solnDataSet:
            squaredError = solnData['squaredError']

            if bestSquaredError is None:
                # this is redundnat, should just store the particle, it has the solution data inside of it . . .
                self.historicalMostLikely['solnData'] = solnData
                self.historicalMostLikely['particle'] = solnData['cfpData'][0]['particle']
                bestSquaredError = squaredError

            if solnData['squaredError'] < bestSquaredError:
                self.historicalMostLikely['solnData'] = solnData
                self.historicalMostLikely['particle'] = solnData['cfpData'][0]['particle']
                bestSquaredError = squaredError

    def getNumberOfParticles(self):
        return len(self.particleList)


