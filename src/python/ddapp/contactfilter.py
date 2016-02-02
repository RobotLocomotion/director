__author__ = 'manuelli'
import ddapp
from ddapp import roboturdf
import numpy as np
import vtkAll as vtk
import PythonQt

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


import drc as lcmdrc
import drake as lcmdrake


FRICTION_CONE_APPROX_SIZE = 4
MU = 0.4

class ContactFilter(object):

    def __init__(self, robotSystem):

        self.robotSystem = robotSystem
        self.robotStateModel = robotSystem.robotStateModel
        self.initializeConstants()
        self.loadDrakeModelFromFilename()
        self.contactFilterPointDict = dict()
        self.loadContactFilterPointsFromFile()
        self.running = False
        self.publishResidual = False
        self.doMultiContactEstimate = True
        self.addSubscribers()
        self.initializePublishChannels()
        self.initializeGurobiModel()
        self.initializeTestParticleSet()
        self.initializeOptions()
        self.setupMotionModelData()
        self.currentUtime = 0.0

        self.residual = None
        self.particleSetList = []
        self.lastTimeContactAdded = time.time()
        self.removedParticleSet = False

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
        self.options['thresholds']['addContactPoint'] = 10.0 # threshold on squared error to add contact point
        self.options['thresholds']['removeContactPoint'] = 3.0 # threshold on force magnitude to eliminate a force that gets too small
        self.options['thresholds']['timeout'] = 2.0

        self.options['motionModel'] = {}
        self.options['motionModel']['var'] = 0.1

        self.options['measurementModel'] = {}
        self.options['measurementModel']['var'] = 1.0 # this is totally made up at the moment
        self.weightMatrix = 1/self.options['measurementModel']['var']*np.eye(self.drakeModel.numJoints)


    def initializeTestTimers(self):
        self.justAppliedMotionModel = False
        self.particleFilterTestTimer = TimerCallback(targetFps=1)
        self.particleFilterTestTimer.callback = self.testFullParticleFilterCallback


    def addTestParticleSetToParticleSetList(self):
        self.particleSetList.append(self.testParticleSet)


    def loadDrakeModelFromFilename(self, filename=None):
        self.drakeModel = PythonDrakeModel()
        self.drakeModel.loadRobotModelFromURDFFilename(filename)


    def squaredErrorNoContacts(self, verbose=True):
        if self.residual is None:
            "don't have a residual, returning"
            return

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

    def setupMotionModelData(self):
        # need to make sure you call loadContactFilterPointsFromFile before you get here

        self.motionModelData = {}
        var = self.options['motionModel']['var']

        # currently only allow motion model to move you within the same link
        for linkName, cfpList in self.contactFilterPointDict.iteritems():
            for cfp in cfpList:
                numCFP = len(cfpList)
                xk = np.arange(0,numCFP)
                pk = np.zeros(numCFP)

                for idx, cfpNext in enumerate(cfpList):
                    distance = np.linalg.norm(cfp.contactLocation - cfpNext.contactLocation)
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
        self.testParticleSet = SingleContactParticleSet()

        for linkName, cfpList in self.contactFilterPointDict.iteritems():
            for cfp in cfpList:
                particle = ContactFilterParticle(cfp)
                self.testParticleSet.addParticle(particle)

    def createParticleSet(self, onlyUseLinks=[], dontUseLinks=[]):
        linkNames = self.contactFilterPointDict.keys()

        if onlyUseLinks and dontUseLinks:
            raise ValueError("can only specify one of the options onlyUseLinks or dontUseLinks, not both")

        if onlyUseLinks:
            linkNames = onlyUseLinks

        if dontUseLinks:
            for linkToRemove in dontUseLinks:
                linkNames.remove(linkToRemove)

        particleSet = SingleContactParticleSet()

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
    def computeSingleLikelihood(self, residual, cfpList):

        H_list = []
        for cfp in cfpList:
            H_list.append(self.computeJacobianToFrictionCone(cfp))



        # this is where the solve is really happening
        numContacts = len(cfpList)
        grbSolnData = self.gurobi.solve(numContacts, residual, H_list, self.weightMatrix)


        alphaVals = np.zeros((numContacts, FRICTION_CONE_APPROX_SIZE))

        for i in xrange(0,numContacts):
            for j in xrange(0, FRICTION_CONE_APPROX_SIZE):
                alphaVals[i,j] = grbSolnData['alphaVals'][i,j]

        cfpData = []
        impliedResidual = 0*residual
        for idx, cfp in enumerate(cfpList):
            d = {'ContactFilterPoint': cfp}
            d['force'] = np.dot(cfp.rotatedFrictionCone, alphaVals[i,:])
            d['alpha'] = alphaVals[i,:]
            cfpData.append(d)
            impliedResidual = impliedResidual + np.dot(H_list[idx], alphaVals[i,:])


        squaredError = np.dot(np.dot((residual - impliedResidual).transpose(), self.weightMatrix),
                                    (residual - impliedResidual))

        likelihood = np.exp(-1/2.0*squaredError)

        # record the data somehow . . .
        solnData = {'cfpData': cfpData, 'impliedResidual': impliedResidual, 'squaredError': squaredError,
                    "numContactPoints": len(cfpList), 'gurobiObjValue': grbSolnData['objectiveValue'],
                    'likelihood': likelihood}
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
        self.drakeModel.model.doKinematics(q, 0*q, False, False)
        # be smart about it, see if we have already computed the QP for a particle with the same cfp!!!

        alreadySolved = {} # should be a dict with ContactFilterPoint as key, ContactFilterParticle as value\
        externalCFPList = []

        for particle in externalParticles:
            externalCFPList.append(particle.cfp)

        for particle in particleSet.particleList:

            # check if we have already solved the problem for this particular contact filter point
            if particle.cfp in alreadySolved:
                solnDataCopy = copy.deepcopy(alreadySolved[particle.cfp].solnData)
                particle.solnData = solnDataCopy
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

        particleSet.updateMostLikelyParticle()


        # should probably update "most likely particle again"

    def computeMeasurementUpdate(self, residual, publish=True):
        for particleSet in self.particleSetList:
            otherParticleSets = copy.copy(self.particleSetList)
            otherParticleSets.remove(particleSet)
            externalParticles = []

            for ps in otherParticleSets:
                if ps.mostLikelyParticle is not None:
                    externalParticles.append(ps.mostLikelyParticle)

            self.measurementUpdateSingleParticleSet(residual, particleSet, externalParticles=externalParticles)


        if publish:
            self.publishMostLikelyEstimate()

        self.manageParticleSets()

    def manageParticleSets(self, verbose=True):
        solnData = self.mostLikelySolnData
        newParticleSet = None

        if solnData is None:
            if (self.squaredErrorNoContacts(verbose=False) > self.options['thresholds']['addContactPoint']):
                newParticleSet = self.createParticleSet()

        elif solnData['squaredError'] > self.options['thresholds']['addContactPoint']:
            linksWithContactPoints = []
            for d in solnData['cfpData']:
                cfp = d['ContactFilterPoint']
                linksWithContactPoints.append(cfp.linkName)


            newParticleSet = self.createParticleSet(dontUseLinks=linksWithContactPoints)

            # this is a hack for now
            # if len(self.particleSetList) >= 2:
            #     print "particle set list is already of size 2, not adding any more"
            #     return

        # this means we have encountered a situation where we should add a new particle set
        # for now will only add one if a sufficient time has passed since we last added a contact
        if newParticleSet is not None:
            if (time.time() - self.lastTimeContactAdded) > self.options['thresholds']['timeout'] or self.removedParticleSet:
                if verbose:
                    print "squared error is large, adding a contact point"
                self.particleSetList.append(newParticleSet)
                self.lastTimeContactAdded = time.time()
            else:
                if verbose:
                    print "below timeout threshold when trying to add a new particle set, returning"

            self.removedParticleSet = False
            return

        if solnData is None:
            self.removedParticleSet = False
            return

        # now remove any particleSets that have sufficiently small "best" forces
        for d in solnData['cfpData']:
            if (np.linalg.norm(d['force']) < self.options['thresholds']['removeContactPoint'] and
                (self.squaredErrorNoContacts(verbose=False) < self.options['thresholds']['addContactPoint'])):
                if verbose:
                    print "force is below threshold, eliminating containing particle set"
                self.removedParticleSet = True
                particleSetToRemove = d['particle'].containingParticleSet
                self.particleSetList.remove(particleSetToRemove)

                # this return statement only allows you to remove a single particle at a time
                return

    def applyMotionModelSingleParticleSet(self, particleSet):
        for particle in particleSet.particleList:
            cfp = particle.cfp
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
        pk = pk/np.sum(pk)
        rv = scipy.stats.rv_discrete(values=(xk,pk)) # the random variable with importance weights

        for i in xrange(0,numParticles):
            # draw new particle
            randomIdx = rv.rvs()
            newParticle = particleSet.particleList[randomIdx].deepCopy()
            newParticleList.append(newParticle)

        particleSet.particleList = newParticleList

    def applyImportanceResampling(self):
        pass

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


    def publishMostLikelyEstimate(self):

        if not self.particleSetList:
            self.mostLikelySolnData = None
            # this means that we currently have no particles
            return

        # currently this will only be correct for the single contact case

        mostLikelySolnData = None
        cfpData = []

        for particleSet in self.particleSetList:
            particle = particleSet.mostLikelyParticle
            if mostLikelySolnData is None:
                mostLikelySolnData = particle.solnData

            cfpData.append(particle.solnData['cfpData'][0])

        mostLikelySolnData['cfpData'] = cfpData

        self.mostLikelySolnData = mostLikelySolnData # store this for debugging
        self.publishEstimate(mostLikelySolnData)


    # currently only support single contact
    def publishEstimate(self, solnData):
        msg = lcmdrc.contact_filter_estimate_t()
        msg.num_contact_points = solnData['numContactPoints']

        msg.num_velocities = self.drakeModel.numJoints
        msg.logLikelihood = solnData['gurobiObjValue']
        msg.velocity_names = self.drakeModel.jointNames
        msg.implied_residual = solnData['impliedResidual']

        msg.single_contact_estimate = [None]*msg.num_contact_points
        for i in xrange(0, msg.num_contact_points):
            msg.single_contact_estimate[i] = self.msgFromSolnCFPData(solnData['cfpData'][i])

        lcmUtils.publish(self.contactEstimatePublishChannel, msg)

    def msgFromSolnCFPData(self, d):
        msg = lcmdrc.single_contact_filter_estimate_t()
        msg.body_name = d['ContactFilterPoint'].linkName
        msg.contact_force = d['force']
        msg.contact_normal = d['ContactFilterPoint'].contactNormal
        msg.contact_position = d['ContactFilterPoint'].contactLocation

        return msg


    def getCurrentPose(self):
        return self.robotSystem.robotStateJointController.q


    def onResidualObserverState(self, msg):
        self.currentUtime = msg.utime
        msgJointNames = msg.joint_name
        msgData = msg.residual

        residual = self.drakeModel.extractDataFromMessage(msgJointNames, msgData)
        self.residual = residual

        if self.running:
            self.computeMeasurementUpdate(residual)


    def onExternalForceTorque(self, msg):
        self.linksWithExternalForce = [str(linkName) for linkName in msg.body_names]
        self.computeAndPublishResidual(msg)


    def drawParticleSet(self, particleSet, name="particle set", drawMostLikely=True):
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
        defaultColor = [0.5, 0, 0.5]
        mostLikelyColor = [0,0,1]

        for cfp, numParticles in numParticlesAtCFP.iteritems():
            color = defaultColor

            if particleSet.mostLikelyParticle is not None:
                if cfp == particleSet.mostLikelyParticle.cfp:
                    color = mostLikelyColor

            rayLength = plungerMinLength + 1.0*numParticles/numTotalParticles*plungerMaxLength
            forceDirectionWorldFrame, forceLocationWorldFrame =\
                cfUtils.getForceDirectionInWorld(q, self.robotStateModel,
                                                                        cfp.linkName,
                                                                        cfp.contactLocation,
                                                                        cfp.contactNormal)

            rayEnd = forceLocationWorldFrame - forceDirectionWorldFrame*rayLength
            d.addSphere(forceLocationWorldFrame, radius=0.01, color=color)
            d.addLine(rayEnd, forceLocationWorldFrame, radius = 0.005, color=color)


        vis.updatePolyData(d.getPolyData(), 'particle set', colorByName='RGB255')

    def testParticleSetDraw(self):
        self.drawParticleSet(self.testParticleSet)


    def testFullParticleFilterCallback(self):
        if self.residual is None:
            return

        if not self.justAppliedMotionModel:
            self.applyMotionModelSingleParticleSet(self.testParticleSet)
            self.testParticleSetDraw()
            self.justAppliedMotionModel = True
        else:
            self.measurementUpdateSingleParticleSet(self.residual, self.testParticleSet)
            self.importanceResamplingSingleParticleSet(self.testParticleSet)
            self.testParticleSetDraw()
            self.justAppliedMotionModel = False

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

    def testMeasurementUpdate(self):

        if self.residual is None:
            print "didn't find residual, using all zeros"
            residual = np.zeros(self.drakeModel.numJoints)
        else:
            residual = self.residual
        startTime = time.time()
        self.measurementUpdateSingleParticleSet(residual, self.testParticleSet)
        elapsed = time.time() - startTime

        self.testParticleSet.updateMostLikelyParticle()
        particle = self.testParticleSet.mostLikelyParticle

        print "single measurement update took " + str(elapsed) + " seconds"
        particle.printObject()



    # these are all test methods
    def startFilterTest(self):
        self.particleFilterTestTimer.start()

    def stopFilterTest(self):
        self.particleFilterTestTimer.stop()

    def removeStaleParticleDraw(self):
        om.removeFromObjectModel(om.findObjectByName('particle set'))

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
        self.contactLocation = contactLocation
        self.contactNormal = contactNormal
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
        self.cfp = cfp

    def setContainingParticleSet(self, containingParticleSet):
        self.containingParticleSet = containingParticleSet

    def printObject(self):
        self.cfp.printObjectShort()

        if self.solnData is not None:
            print "squared error = ", self.solnData['squaredError']
            print "force in body frame = ", self.solnData['force']

    def deepCopy(self):
        newParticle = ContactFilterParticle(cfp=self.cfp)
        newParticle.setContainingParticleSet(self.containingParticleSet)
        return newParticle


class SingleContactParticleSet(object):

    def __init__(self):
        self.particleList = []
        self.mostLikelyParticle = None

    def addParticle(self, particle):
        self.particleList.append(particle)
        particle.setContainingParticleSet(self)

    def updateMostLikelyParticle(self):
        bestSquaredError = None

        for particle in self.particleList:

            squaredError = particle.solnData['squaredError']
            if bestSquaredError is None or (squaredError < bestSquaredError):
                bestSquaredError = squaredError
                self.mostLikelyParticle = particle



    def getNumberofParticles(self):
        return len(self.particleList)


