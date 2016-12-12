import os
import json
from collections import OrderedDict
import numpy as np
import scipy.interpolate

from director import ikconstraints
from director import ikconstraintencoder
from director import ikparameters
from director import plannerPublisher
from director import transformUtils
from director import roboturdf
from director.fieldcontainer import FieldContainer
from director.simpletimer import FPSCounter
from director import vtkAll as vtk
from director import transformUtils

from director import lcmUtils

import pydrake
import pydrake.solvers.ik as pydrakeik

try:
    from robotlocomotion import robot_plan_t
except ImportError:
    from drc import robot_plan_t


class PyDrakePlannerPublisher(plannerPublisher.PlannerPublisher):

    def _setup(self):

        self.counter = FPSCounter()
        self.counter.printToConsole = True

        # disabled for now
        #self._setupLocalServer()

    def _setupLocalServer(self):

        initArgs = FieldContainer(
            urdfFile=self.ikPlanner.robotModel.getProperty('Filename'),
            packagePaths=roboturdf.getPackagePaths()
            )

        self.ikServer = PyDrakeIkServer()
        self.ikServer.initInstance(initArgs)

    def testEncodeDecode(self, fields):

        encoded = json.dumps(fields, cls=ikconstraintencoder.ConstraintEncoder)
        decoded = json.loads(encoded, object_hook=ikconstraintencoder.ConstraintDecoder)

        del decoded['class']
        fields = FieldContainer(**decoded)

        del fields.options['class']
        fields.options = ikparameters.IkParameters(**fields.options)

        constraints = []

        for c in fields.constraints:
            objClass = getattr(ikconstraints, c['class'])
            del c['class']
            obj = objClass()
            constraints.append(obj)

            for attr, value in c.iteritems():
                if isinstance(value, dict) and 'position' in value and 'quaternion' in value:
                    value = transformUtils.transformFromPose(value['position'], value['quaternion'])
                setattr(obj, attr, value)

        fields.constraints = constraints

        return fields

    def makePlanMessage(self, poses, poseTimes, info, fields):

        from director import robotstate

        # rescale poseTimes
        poseTimes = np.array(poseTimes)*2.0

        states = [robotstate.drakePoseToRobotState(pose) for pose in poses]
        for i, state in enumerate(states):
            state.utime = poseTimes[i]*1e6

        msg = robot_plan_t()
        msg.utime = fields.utime
        msg.robot_name = 'robot'
        msg.num_states = len(states)
        msg.plan = states
        msg.plan_info = [info]*len(states)
        msg.num_bytes = 0
        msg.matlab_data = []
        msg.num_grasp_transitions = 0
        msg.left_arm_control_type = msg.NONE
        msg.right_arm_control_type = msg.NONE
        msg.left_leg_control_type = msg.NONE
        msg.right_leg_control_type = msg.NONE

        return msg

    def processIK(self, constraints, ikParameters, positionCosts, nominalPoseName="", seedPoseName=""):

        fields = self.setupFields(constraints, ikParameters, positionCosts, nominalPoseName, seedPoseName)
        fields = self.testEncodeDecode(fields)
        endPose, info = self.ikServer.runIk(fields)
        self.counter.tick()
        return endPose, info

    def processTraj(self, constraints, ikParameters, positionCosts, nominalPoseName="", seedPoseName="", endPoseName=""):

        fields = self.setupFields(constraints, ikParameters, positionCosts, nominalPoseName, seedPoseName, endPoseName)
        fields = self.testEncodeDecode(fields)
        poses, poseTimes, info = self.ikServer.runIkTraj(fields)
        plan = self.makePlanMessage(poses, poseTimes, info, fields)
        lcmUtils.publish('CANDIDATE_MANIP_PLAN', plan)
        return plan, info


class RigidBodyTreeCompatNew(object):

    @staticmethod
    def get_body(rbt, i):
        return rbt.get_body(i)

    @staticmethod
    def get_num_bodies(rbt):
        return rbt.get_num_bodies()

    @staticmethod
    def get_body_name(rbt, i):
        return rbt.get_body(i).get_name()

    @staticmethod
    def get_position_name(rbt, i):
        return rbt.get_position_name(i)

    @staticmethod
    def get_num_positions(rbt):
        return rbt.get_num_positions()

    @staticmethod
    def get_num_velocities(rbt):
        return rbt.get_num_velocities()


class RigidBodyTreeCompatOld(object):

    @staticmethod
    def get_body(rbt, i):
        return rbt.bodies[i]

    @staticmethod
    def get_num_bodies(rbt):
        return len(rbt.bodies)

    @staticmethod
    def get_body_name(rbt, i):
        return rbt.bodies[i].linkname

    @staticmethod
    def get_position_name(rbt, i):
        return rbt.getPositionName(i)

    @staticmethod
    def get_num_positions(rbt):
        return rbt.num_positions

    @staticmethod
    def get_num_velocities(rbt):
        return rbt.num_velocities


if hasattr(pydrake.rbtree.RigidBodyTree, 'get_num_bodies'):
    rbt = RigidBodyTreeCompatNew
else:
    rbt = RigidBodyTreeCompatOld


class PyDrakeIkServer(object):

    def __init__(self):
        self.rigidBodyTree = None
        self.trajInterpolationMode = 'cubic'

    def initInstance(self, fields):

        self.rigidBodyTree = self.loadRigidBodyTree(fields.urdfFile, fields.packagePaths)

        self.bodyNames = [rbt.get_body_name(self.rigidBodyTree, i) for i in xrange(rbt.get_num_bodies(self.rigidBodyTree))]
        self.bodyNameToId = {}
        for i, name in enumerate(self.bodyNames):
            self.bodyNameToId[name] = i

        self.positionNames = [rbt.get_position_name(self.rigidBodyTree, i) for i in xrange(rbt.get_num_positions(self.rigidBodyTree))]
        self.positionNameToId = {}
        for i, name in enumerate(self.positionNames):
            self.positionNameToId[name] = i

    def loadRigidBodyTree(self, urdfFile, packagePaths):

        assert os.path.isfile(urdfFile)

        packageMap = pydrake.rbtree.PackageMap()
        for path in packagePaths:
            packageMap.Add(os.path.basename(path), path)

        urdfString = open(urdfFile, 'r').read()
        baseDir = str(os.path.dirname(urdfFile))
        floatingBaseType = pydrake.rbtree.kRollPitchYaw
        weldFrame = None

        rigidBodyTree = pydrake.rbtree.RigidBodyTree()

        pydrake.rbtree.AddModelInstanceFromUrdfStringSearchingInRosPackages(
          urdfString,
          packageMap,
          baseDir,
          floatingBaseType,
          weldFrame,
          rigidBodyTree)

        return rigidBodyTree

    def makeIkOptions(self, fields):

        options = pydrakeik.IKoptions(self.rigidBodyTree)
        options.setQ(np.diag(fields.positionCosts))
        #options.setQv()
        #options.setQa()
        #options.setDebug(True)

        options.setMajorOptimalityTolerance(fields.options.majorOptimalityTolerance)
        options.setMajorFeasibilityTolerance(fields.options.majorFeasibilityTolerance)
        options.setMajorIterationsLimit(fields.options.majorIterationsLimit)
        #options.setIterationsLimit(limit)
        #options.setSuperbasicsLimit(limit)


        # for ik pointwise, whether to use q_seed at each point (False),
        # or to use the solution from the previous point (True)
        #options.setSequentialSeedFlag(True)

        # for ik traj, if initial state is not fixed
        # then here you can set the lb/ub of initial q
        options.setFixInitialState(fields.options.fixInitialState)
        #options.setq0(lb, ub)

        # for ik traj, set lower and upper bound of
        # initial and final velocity
        #options.setqd0(lb, ub)
        #options.setqdf(lb, ub)

        # for ik traj, additional time samples in addition to knot points
        # to check constraints
        #options.setAdditionaltSamples([])

        return options

    def handleFixedLinkFromRobotPoseConstraint(self, c, fields):

        bodyId = self.bodyNameToId[c.linkName]
        pose = np.asarray(fields.poses[c.poseName], dtype=float)
        pointInBodyFrame = np.zeros(3)
        tolerance = np.radians(c.angleToleranceInDegrees)
        tspan = np.asarray(c.tspan, dtype=float)

        bodyToWorld = self.computeBodyToWorld(pose, c.linkName, pointInBodyFrame)
        bodyPos = transformUtils.transformations.translation_from_matrix(bodyToWorld)
        bodyQuat = transformUtils.transformations.quaternion_from_matrix(bodyToWorld)

        lowerBound = bodyPos + c.lowerBound
        upperBound = bodyPos + c.upperBound

        pc = pydrakeik.WorldPositionConstraint(self.rigidBodyTree, bodyId, pointInBodyFrame, lowerBound, upperBound, tspan)
        qc = pydrakeik.WorldQuatConstraint(self.rigidBodyTree, bodyId, bodyQuat, tolerance, tspan)

        return [pc, qc]

    def handlePositionConstraint(self, c, fields):

        bodyId = self.bodyNameToId[c.linkName]
        pointInBodyFrame = np.asarray(c.pointInLink, dtype=float)
        referenceFrame = transformUtils.getNumpyFromTransform(c.referenceFrame)
        lowerBound = np.asarray(c.positionTarget, dtype=float) + c.lowerBound
        upperBound = np.asarray(c.positionTarget, dtype=float) + c.upperBound
        tspan = np.asarray(c.tspan, dtype=float)

        pc = pydrakeik.WorldPositionInFrameConstraint(self.rigidBodyTree, bodyId, pointInBodyFrame, referenceFrame, lowerBound, upperBound, tspan)

        return pc

    def handleQuatConstraint(self, c, fields):

        bodyId = self.bodyNameToId[c.linkName]
        tolerance = np.radians(c.angleToleranceInDegrees)
        tspan = np.asarray(c.tspan, dtype=float)

        if isinstance(c.quaternion, vtk.vtkTransform):
            quat = transformUtils.getNumpyFromTransform(c.quaternion)
        else:
            quat = np.asarray(c.quaternion, dtype=float)
        if quat.shape == (4,4):
            quat = transformUtils.transformations.quaternion_from_matrix(quat)

        qc = pydrakeik.WorldQuatConstraint(self.rigidBodyTree, bodyId, quat, tolerance, tspan)
        return qc

    def handleWorldGazeDirConstraint(self, c, fields):

        bodyId = self.bodyNameToId[c.linkName]
        tspan = np.asarray(c.tspan, dtype=float)
        worldAxis = np.asarray(c.targetAxis, dtype=float)
        bodyAxis = np.asarray(c.bodyAxis, dtype=float)
        coneThreshold = c.coneThreshold

        c.targetFrame.TransformVector(worldAxis, worldAxis)

        wc = pydrakeik.WorldGazeDirConstraint(self.rigidBodyTree, bodyId, bodyAxis, worldAxis, coneThreshold, tspan)
        return wc

    def handlePostureConstraint(self, c, fields):

        tspan = np.asarray(c.tspan, dtype=float)

        pose = np.asarray(fields.poses[c.postureName], dtype=float)
        positionInds = np.array([self.positionNameToId[name] for name in c.joints], dtype=np.int32)
        lowerLimit = pose[positionInds] + c.jointsLowerBound
        upperLimit = pose[positionInds] + c.jointsUpperBound

        pc = pydrakeik.PostureConstraint(self.rigidBodyTree, tspan)
        pc.setJointLimits(positionInds, lowerLimit, upperLimit)

        return pc

    def handleQuasiStaticConstraint(self, c, fields):

        # todo
        # shrinkFactor should not be a class member but an instance member
        c.shrinkFactor = fields.options.quasiStaticShrinkFactor

        tspan = np.asarray(c.tspan, dtype=float)
        shrinkFactor = c.shrinkFactor
        active = c.leftFootEnabled or c.rightFootEnabled
        leftFootBodyId = self.bodyNameToId[c.leftFootLinkName]
        rightFootBodyId = self.bodyNameToId[c.rightFootLinkName]
        groups = ['heel', 'toe']

        qsc = pydrakeik.QuasiStaticConstraint(self.rigidBodyTree, tspan)
        qsc.setActive(active)
        qsc.setShrinkFactor(shrinkFactor)

        if c.leftFootEnabled:
            body = rbt.get_body(self.rigidBodyTree, leftFootBodyId)
            pts = np.hstack([self.rigidBodyTree.getTerrainContactPoints(body, groupName) for groupName in groups])
            qsc.addContact([leftFootBodyId], pts)

        if c.rightFootEnabled:
            body = rbt.get_body(self.rigidBodyTree, rightFootBodyId)
            pts = np.hstack([self.rigidBodyTree.getTerrainContactPoints(body, groupName) for groupName in groups])
            qsc.addContact([rightFootBodyId], pts)

        return qsc

    def makeConstraints(self, fields):

        dispatchMap = {
            ikconstraints.PositionConstraint : self.handlePositionConstraint,
            ikconstraints.FixedLinkFromRobotPoseConstraint : self.handleFixedLinkFromRobotPoseConstraint,
            ikconstraints.QuatConstraint : self.handleQuatConstraint,
            ikconstraints.WorldGazeDirConstraint : self.handleWorldGazeDirConstraint,
            ikconstraints.PostureConstraint : self.handlePostureConstraint,
            ikconstraints.QuasiStaticConstraint : self.handleQuasiStaticConstraint,
            }

        constraints = [dispatchMap[type(c)](c, fields) for c in fields.constraints]

        constraintList = []
        for c in constraints:
            if c is None:
                continue
            if isinstance(c, list):
                constraintList.extend(c)
            else:
                constraintList.append(c)

        return constraintList

    def computeBodyToWorld(self, pose, bodyName, pointInBody=None):
        if pointInBody is None:
            pointInBody = np.zeros(3)

        assert len(pose) == self.rigidBodyTree.get_num_positions()
        v = np.zeros(self.rigidBodyTree.get_num_velocities())
        kinsol = self.rigidBodyTree.doKinematics(pose, v)

        t = self.rigidBodyTree.relativeTransform(kinsol, self.bodyNameToId['world'], self.bodyNameToId[bodyName])
        assert t.shape == (4,4)

        #tt = transformUtils.getTransformFromNumpy(t)
        #pos = transformUtils.transformations.translation_from_matrix(t)
        #quat = transformUtils.transformations.quaternion_from_matrix(t)
        return t

    def checkJointNameOrder(self, fields):

        assert len(fields.jointNames) == rbt.get_num_positions(self.rigidBodyTree)

        for i in xrange(len(self.positionNames)):
            if self.positionNames[i] != fields.jointNames[i]:
                raise Exception('joint name order mismatch. input=%r, rigidBodyTree=%r' % (fields.jointNames, self.positionNames))

    def makeTimeSamplesFromConstraints(self, fields):

        timeSamples = np.hstack([constraint.tspan for constraint in fields.constraints])
        timeSamples = [x for x in timeSamples if x not in [-np.inf, np.inf]]
        timeSamples.append(0.0)
        timeSamples = np.unique(timeSamples).tolist()
        timeSamples += np.linspace(timeSamples[0], timeSamples[-1], fields.options.numberOfAddedKnots + 2).tolist()
        timeSamples = np.unique(timeSamples)
        return timeSamples

    def runIk(self, fields):

        self.checkJointNameOrder(fields)

        ikoptions = self.makeIkOptions(fields)

        constraints = self.makeConstraints(fields)

        # todo
        # set joint limits on rigidBodyTree from fields.jointLimits

        q_nom = np.asarray(fields.poses[fields.nominalPose], dtype=float)
        q_seed = np.asarray(fields.poses[fields.seedPose], dtype=float)


        if rbt is RigidBodyTreeCompatOld:
            results = pydrakeik.inverseKinSimple(self.rigidBodyTree, q_seed, q_nom, constraints, ikoptions)
            q_end = results.q_sol
            info = results.INFO
        else:
            results = pydrakeik.InverseKin(self.rigidBodyTree, q_seed, q_nom, constraints, ikoptions)
            q_end = results.q_sol[0]
            info = results.info[0]

        #for i in xrange(len(results.infeasible_constraints)):
        #    print 'infeasible constraint:', results.infeasible_constraints[i]

        # convert shape from Nx1 to N
        q_end.shape = q_end.shape[0]

        return q_end, info

    def getInterpolationFunction(self, x, y, kind):
        '''
        Given arrays x and y and an interpolation kind 'linear', 'cubic' or 'pchip',
        this function returns the result of scipy.interpolate.interp1d or
        scipy.interpolate.pchip.

        If the interpolation is cubic or pchip, this function will duplicate the
        values at the end points of x and y (and add/subtract a very small x delta)
        to enforce zero slope at the end points.

        See scipy documentation for more details.
        '''
        assert kind in ('linear', 'cubic', 'pchip')

        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        assert x.ndim == 1

        if kind == 'linear':
            kind == 'slinear'

        xdelta = 1e-9
        if kind in ('cubic', 'pchip'):
            x = np.hstack(([x[0]-xdelta], x, [x[-1]+xdelta]))
            y = np.vstack(([y[0]], y, [y[-1]]))

        if kind == 'pchip':
            return scipy.interpolate.pchip(x, y, axis=0)
        else:
            return scipy.interpolate.interp1d(x, y, axis=0, kind=kind)

    def runIkTraj(self, fields):

        timeSamples = self.makeTimeSamplesFromConstraints(fields)
        ikoptions = self.makeIkOptions(fields)
        constraints = self.makeConstraints(fields)

        q_nom = np.asarray(fields.poses[fields.nominalPose])
        q_seed = np.asarray(fields.poses[fields.seedPose])
        q_seed_end = np.asarray(fields.poses[fields.endPose])

        q_nom_array = np.tile(q_nom, (len(timeSamples), 1)).transpose()

        values = np.vstack((q_seed, q_seed_end))
        timeRange = [timeSamples[0], timeSamples[-1]]

        q_seed_array = self.getInterpolationFunction(timeRange, values,
                                kind=self.trajInterpolationMode)(timeSamples).transpose()

        assert q_seed_array.shape == (len(q_nom), len(timeSamples))
        #print 'time range:', timeRange
        #print 'time samples:', timeSamples
        #print 'q_seed:', q_seed
        #print 'q_seed_end:', q_seed_end
        #print 'q_seed_array:', q_seed_array
        #print 'q_nom_array:', q_nom_array

        results = pydrakeik.InverseKinTraj(self.rigidBodyTree, timeSamples, q_seed_array, q_nom_array, constraints, ikoptions)

        assert len(results.q_sol) == len(timeSamples)

        poses = []
        for i in xrange(len(results.q_sol)):
            q = results.q_sol[i]
            q.shape = q.shape[0]
            poses.append(q)

        info = results.info[0]


        if fields.options.usePointwise:
            numPointwiseSamples = 20
            pointwiseTimeSamples = np.linspace(timeRange[0], timeRange[1], numPointwiseSamples)

            q_seed_array = self.getInterpolationFunction(timeSamples, np.array(poses),
                                    kind=self.trajInterpolationMode)(pointwiseTimeSamples).transpose()
            assert q_seed_array.shape == (len(q_nom), numPointwiseSamples)

            results = pydrakeik.InverseKinPointwise(self.rigidBodyTree, pointwiseTimeSamples, q_seed_array, q_seed_array, constraints, ikoptions)


            assert len(results.q_sol) == len(pointwiseTimeSamples)

            print 'pointwise info len:', len(results.info)

            poses = []
            for i in xrange(len(results.q_sol)):
                q = results.q_sol[i]
                q.shape = q.shape[0]
                poses.append(q)

            info = results.info[0]
            timeSamples = pointwiseTimeSamples


        return poses, timeSamples, info
