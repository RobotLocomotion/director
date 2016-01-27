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
from PythonQt import QtCore, QtGui
from ddapp import transformUtils
from ddapp import lcmUtils
from ddapp import contactfiltergurobi
from ddapp import gurobiutils as grbUtils
import drc as lcmdrc
import drake as lcmdrake


FRICTION_CONE_APPROX_SIZE = 4
MU = 0.4

class ContactFilter(object):

    def __init__(self, robotSystem):

        self.robotSystem = robotSystem
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
        self.currentUtime = 0.0

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


    def initializeConstants(self):
        mu = MU
        self.frictionCone = np.array([[mu,-mu,0,0],
                                      [0,0,mu,-mu],
                                      [1,1,1,1]])


    def loadDrakeModelFromFilename(self, filename=None):
        self.drakeModel = PythonDrakeModel()
        self.drakeModel.loadRobotModelFromURDFFilename(filename)
        self.weightMatrix = 1.0*np.eye(self.drakeModel.numJoints)


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

            contactFilterPoint = {'linkName': linkName, 'contactLocation': forceLocation,
                                  'contactNormal': forceDirection, 'bodyId': bodyId,
                                  'forceMomentTransform': forceMomentTransform,
                                  'rotatedFrictionCone': rotatedFrictionCone,
                                  'J_alpha': J_alpha}

            if self.contactFilterPointDict.has_key(linkName):
                self.contactFilterPointDict[linkName].append(contactFilterPoint)
            else:
                self.contactFilterPointDict[linkName] = [contactFilterPoint]


    def initializeGurobiModel(self):
        # careful here, Mosek models leak memory apparently
        numContactsList = [1,2,3,4]
        self.gurobi = contactfiltergurobi.ContactFilterGurobi(numContactsList=numContactsList)


    # make sure you call doKinematics before getting here
    def computeJacobianToFrictionCone(self, contactPoint):
        linkJacobian = self.drakeModel.geometricJacobian(0, contactPoint['bodyId'], contactPoint['bodyId'],
                                                         0, False)

        H = np.dot(linkJacobian.transpose(), contactPoint['J_alpha'])
        return H


    # inside this need to setup and solve the QP . . .
    def computeSingleLikelihood(self, residual, cfpList):
        # do kinematics at the correct pose
        # q = self.getCurrentPose()
        # self.drakeModel.model.doKinematics(q, 0*q, False, False)

        H_list = []
        for cfp in cfpList:
            H_list.append(self.computeJacobianToFrictionCone(cfp))


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
            d['force'] = np.dot(cfp['rotatedFrictionCone'], alphaVals[i,:])
            d['alpha'] = alphaVals[i,:]
            cfpData.append(d)
            impliedResidual = impliedResidual + np.dot(H_list[idx], alphaVals[i,:])


        squaredError = np.dot(np.dot((residual - impliedResidual).transpose(), self.weightMatrix),
                                    (residual - impliedResidual))

        # record the data somehow . . .
        solnData = {'cfpData': cfpData, 'impliedResidual': impliedResidual, 'squaredError': squaredError,
                    "numContactPoints": len(cfpList), 'gurobiObjValue': grbSolnData['objectiveValue']}
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

        mostLikelySolnData = self.measurementUpdateSolnDataList[0]

        for solnData in self.measurementUpdateSolnDataList:
            if solnData['gurobiObjValue'] < mostLikelySolnData['gurobiObjValue']:
                mostLikelySolnData = solnData

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
        msg.body_name = d['ContactFilterPoint']['linkName']
        msg.contact_force = d['force']
        msg.contact_normal = d['ContactFilterPoint']['contactNormal']
        msg.contact_position = d['ContactFilterPoint']['contactLocation']

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
            self.computeLikelihoodFull(residual)

    def onExternalForceTorque(self, msg):
        self.linksWithExternalForce = [str(linkName) for linkName in msg.body_names]
        self.computeAndPublishResidual(msg)


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

