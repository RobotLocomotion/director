import os
import math
import types
import functools
import numpy as np
import director
from director import matlab
from director import callbacks
from director.asynctaskqueue import AsyncTaskQueue
from director.ikconstraints import *

from director import drcargs

class AsyncIKCommunicator():

    STARTUP_COMPLETED = 'STARTUP_COMPLETED'

    def __init__(self, robotURDF, fixedPointFile, leftFootLink, rightFootLink, pelvisLink):

        self.comm = None
        self.outputConsole = None
        self.ready = False
        self.restarted = False
        self.robotURDF = robotURDF
        self.fixedPointFile = fixedPointFile
        self.leftFootLink = leftFootLink
        self.rightFootLink = rightFootLink
        self.pelvisLink = pelvisLink
        self.infoFunc = None

        self.callbacks = callbacks.CallbackRegistry([self.STARTUP_COMPLETED])


    def _sendStartupCommands(self):

        if self.restarted:
            return

        commands = []
        commands.append('\n%-------- startup --------\n')
        commands.append('format long e')
        commands.append('addpath_control')
        commands.append("addpath([getenv('DRC_BASE'), '/software/director/src/matlab'])")
        commands.append("robotURDF = [getenv('DRC_BASE'), '/%s'];" % os.path.relpath(self.robotURDF, director.getDRCBaseDir()))
        commands.append("fixed_point_file = [getenv('DRC_BASE'), '/%s'];" % os.path.relpath(self.fixedPointFile, director.getDRCBaseDir()))
        commands.append("left_foot_link = '%s';" % self.leftFootLink)
        commands.append("right_foot_link = '%s';" % self.rightFootLink)
        commands.append("pelvis_link = '%s';" % self.pelvisLink)
        commands.append('runIKServer')
        commands.append('\n%------ startup end ------\n')
        return self.comm.sendCommandsAsync(commands)

    def _createMatlabClient(self):

        hostname = drcargs.args().matlab_host
        if hostname is not None:
            return matlab.MatlabSocketClient(host=hostname)
        else:
            return matlab.MatlabPipeClient()

    def startServerAsync(self):

        self.comm = matlab.MatlabCommunicator(self._createMatlabClient())
        self.comm.echoToStdOut = False
        self.comm.outputConsole = self.outputConsole

        taskQueue = AsyncTaskQueue()
        taskQueue.addTask(functools.partial(self.comm.sendCommandsAsync, ['\n']))
        taskQueue.addTask(self._checkServerRestarted)
        taskQueue.addTask(self._sendStartupCommands)
        taskQueue.addTask(self._checkServerStartup)
        taskQueue.addTask(self._notifyStartupCompleted)
        taskQueue.addTask(functools.partial(setattr, self.comm, 'echoToStdOut', True))

        self.taskQueue = taskQueue
        self.taskQueue.start()

    def connectStartupCompleted(self, func):
        return self.callbacks.connect(self.STARTUP_COMPLETED, func)

    def disconnectStartupCompleted(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def _checkServerStartup(self):
        started = self.comm.getFloatArray("exist('ikServerStarted')")
        self.ready = len(started) and started[0] == 1

    def _checkServerRestarted(self):
        self._checkServerStartup()
        self.restarted = self.ready

    def _notifyStartupCompleted(self):
        self.callbacks.process(self.STARTUP_COMPLETED, self, self.ready)

    def interact(self):
        self.comm.interact()


    def fetchPoseFromServer(self, poseName):
        return self.comm.getFloatArray(poseName)


    def sendPoseToServer(self, pose, poseName):
        self.comm.assignFloatArray(pose, poseName)


    def constructVisualizer(self):
        commands = []
        commands.append("v = r.constructVisualizer(struct('use_contact_shapes', true));")
        self.comm.sendCommands(commands)

    def getFrozenGroupString(self):
        frozenGroups = []
        if getattr(self,"leftArmLocked",False):
            frozenGroups.append("l_arm")
        if getattr(self,"rightArmLocked",False):
            frozenGroups.append("r_arm")
        if getattr(self,"baseLocked",False):
            frozenGroups.append("pelvis")
        if getattr(self,"backLocked",False):
            frozenGroups.append("back")
        if frozenGroups:
            return "{'" + "','".join(frozenGroups) + "'}"
        else:
            return "{}"


    def draw(self):
        commands = []
        commands.append('v.draw(0, q_end);');
        self.comm.sendCommands(commands)



    def getConstraintCommands(self, constraintNames):

        commands = []

        for constraintName in constraintNames:
            if constraintName.endswith('position_constraint'):
                commands.extend(self.updatePositionConstraint(constraintName.replace('_position_constraint', ''), execute=False))
            elif constraintName.endswith('orient_constraint'):
                commands.extend(self.updateOrientationConstraint(constraintName.replace('_orient_constraint', ''), execute=False))
            elif constraintName.endswith('gaze_constraint'):
                commands.extend(self.updateGazeConstraint(constraintName.replace('_gaze_constraint', ''), execute=False))

        commands.append('active_constraints = {%s};' % ', '.join(constraintNames))
        return commands


    def updateJointLimits(self, limitData):
        commands = []
        commands.append('joint_limit_min_new = r.joint_limit_min;')
        commands.append('joint_limit_max_new = r.joint_limit_max;')

        for jointName, epsilon, jointPosition in limitData:
            arrayName = 'joint_limit_min_new' if epsilon < 0 else 'joint_limit_max_new'
            commands.append('%s(joints.%s) = %s(joints.%s) + %f;' % (arrayName, jointName, arrayName, jointName, epsilon))

        commands.append('s = s.setJointLimits(joint_limit_min_new, joint_limit_max_new);')
        commands.append('r = s.robot_and_environment;')
        self.taskQueue.addTask(functools.partial(self.comm.sendCommandsAsync, commands))
        self.taskQueue.start()


    def setEnvironment(self, urdf_string):
        commands = []
        urdf_lines = urdf_string.splitlines()
        urdf_lines = ["'%s'" % x for x in urdf_lines]
        urdf_lines = '...\n'.join(urdf_lines)
        self.comm.send('environment_urdf_string = [%s];' % urdf_lines )
        self.comm.waitForResult()
        commands.append('s = s.setEnvironment(environment_urdf_string);')
        commands.append('r = s.robot_and_environment;')
        self.comm.sendCommands(commands)

    def clearEnvironment(self):
        self.setEnvironment('')

    def runIk(self, constraints, ikParameters, nominalPostureName, seedPostureName):

        commands = []
        commands.append('\n%-------- runIk --------\n')
        constraintNames = []
        commands.append('excluded_collision_groups = struct(\'name\',{},\'tspan\',{});\n')
        commands.append('default_shrink_factor = %s;' % ikParameters.quasiStaticShrinkFactor)
        for constraintId, constraint in enumerate(constraints):
            if not constraint.enabled:
                continue
            constraint.getCommands(commands, constraintNames, suffix='_%d' % constraintId)
            commands.append('\n')

        commands.append('{0} = [{0}; zeros(r.getNumPositions()-numel({0}),1)];'.format(nominalPostureName))
        commands.append('{0} = [{0}; zeros(r.getNumPositions()-numel({0}),1)];'.format(seedPostureName))
        commands.append('active_constraints = {%s};' % ', '.join(constraintNames))
        commands.append('ik_seed_pose = %s;' % seedPostureName)
        commands.append('ik_nominal_pose = %s;' % nominalPostureName)
        commands.append('ik_seed_pose = [ik_seed_pose; zeros(r.getNumPositions()-numel(ik_seed_pose),1)];')
        commands.append('ik_nominal_pose = [ik_nominal_pose; zeros(r.getNumPositions()-numel(ik_nominal_pose),1)];')
        commands.append('options = struct();')
        commands.append('options.MajorIterationsLimit = %s;' % ikParameters.majorIterationsLimit)
        commands.append('options.MajorFeasibilityTolerance = %s;' % ikParameters.majorFeasibilityTolerance)
        commands.append('options.MajorOptimalityTolerance = %s;' % ikParameters.majorOptimalityTolerance)
        commands.append('options.MinDistance = %f;' % ikParameters.collisionMinDistance)
        commands.append('s = s.setupOptions(options);')
        commands.append('clear q_end;')
        commands.append('clear info;')
        commands.append('clear infeasible_constraint;')
        commands.append('\n')
        commands.append('use_collision = %s;' % ('false' if (ikParameters.useCollision == 'none') else 'true'))
        commands.append('[q_end, info, infeasible_constraint] = s.runIk(ik_seed_pose, ik_nominal_pose, active_constraints, use_collision);')
        commands.append('\n')

        commands.append('q_end(s.robot.getNumPositions()+1:end) = [];')
        commands.append('\n%-------- runIk end --------\n')

        self.comm.sendCommands(commands)
        endPose = self.comm.getFloatArray('q_end')
        info = self.comm.getFloatArray('info')[0]

        return endPose, info


    def sampleTraj(self, t):

        commands = []
        commands.append('tdelta = qtraj.tspan(end) - qtraj.tspan(1);')
        commands.append('q_trajPose = eval(qtraj, qtraj.tspan(1) + %f*tdelta);' % t)

        self.comm.sendCommands(commands)
        self.fetchPoseFromServer('q_trajPose')


    def runIkTraj(self, constraints, poseStart, poseEnd, nominalPose, ikParameters, timeSamples=None, additionalTimeSamples=0, graspToHandLinkFrame=None):

        if timeSamples is None:
            timeSamples = np.hstack([constraint.tspan for constraint in constraints])
            timeSamples = [x for x in timeSamples if x not in [-np.inf, np.inf]]
            timeSamples.append(0.0)
            timeSamples = np.unique(timeSamples).tolist()
            timeSamples += np.linspace(timeSamples[0], timeSamples[-1], ikParameters.numberOfAddedKnots + 2).tolist()
            timeSamples = np.unique(timeSamples).tolist()

        assert ikParameters.rrtHand in ('left', 'right')
        collisionEndEffectorName = ( self.handModels[0].handLinkName if ikParameters.rrtHand == 'left' else self.handModels[1].handLinkName )

        commands = []
        commands.append('\n%-------- runIkTraj --------\n')
        commands.append('{0} = [{0}; zeros(r.getNumPositions()-numel({0}),1)];'.format(poseStart))
        commands.append('{0} = [{0}; zeros(r.getNumPositions()-numel({0}),1)];'.format(poseEnd))
        commands.append('{0} = [{0}; zeros(r.getNumPositions()-numel({0}),1)];'.format(nominalPose))
        commands.append('excluded_collision_groups = struct(\'name\',{},\'tspan\',{});\n')
        commands.append("end_effector_name = '%s';" % collisionEndEffectorName)
        commands.append("end_effector_name_left = '%s';" % self.handModels[0].handLinkName)
        if (len(self.handModels) > 1):
            commands.append("end_effector_name_right = '%s';" % self.handModels[1].handLinkName)
        commands.append("end_effector_pt = [];")
        commands.append('default_shrink_factor = %s;' % ikParameters.quasiStaticShrinkFactor)

        constraintNames = []
        for constraintId, constraint in enumerate(constraints):
            if not constraint.enabled:
                continue
            constraint.getCommands(commands, constraintNames, suffix='_%d' % constraintId)
            commands.append('\n')

        commands.append('active_constraints = {%s};' % ', '.join(constraintNames))
        commands.append('t = [%s];' % ', '.join([repr(x) for x in timeSamples]))
        commands.append('nt = size(t, 2);')
        commands.append('clear xtraj;')
        commands.append('clear info;')
        commands.append('clear infeasible_constraint;')
        if additionalTimeSamples:
            commands.append('additionalTimeSamples = linspace(t(1), t(end), %d);' % additionalTimeSamples)
        else:
            commands.append('additionalTimeSamples = [];')
        commands.append('options = struct();')
        commands.append('options.MajorIterationsLimit = %s;' % ikParameters.majorIterationsLimit)
        commands.append('options.MajorFeasibilityTolerance = %s;' % ikParameters.majorFeasibilityTolerance)
        commands.append('options.MajorOptimalityTolerance = %s;' % ikParameters.majorOptimalityTolerance)
        commands.append('options.FixInitialState = %s;' % ('true' if ikParameters.fixInitialState else 'false'))
        commands.append('s = s.setupOptions(options);')
        commands.append('ikoptions = s.ikoptions.setAdditionaltSamples(additionalTimeSamples);')
        #commands.append('ikoptions = ikoptions.setSequentialSeedFlag(true);')
        commands.append('\n')

        if ikParameters.useCollision == 'RRT Connect':
            commands.append('q_seed_traj = PPTrajectory(foh([t(1), t(end)], [%s, %s]));' % (poseStart, poseEnd))
            commands.append('q_nom_traj = ConstantTrajectory(q_nom);')
            commands.append('options.n_interp_points = %s;' % ikParameters.numberOfInterpolatedCollisionChecks)
            commands.append('options.min_distance = %s;' % ikParameters.collisionMinDistance)
            commands.append('options.t_max = %s;' % ikParameters.maxPlanDuration)
            commands.append('options.excluded_collision_groups = excluded_collision_groups;')
            commands.append('options.end_effector_name = end_effector_name;')
            commands.append('options.end_effector_name_left = end_effector_name_left;')
            commands.append('options.end_effector_name_right = end_effector_name_right;')
            commands.append('options.end_effector_pt = end_effector_pt;')
            commands.append('options.left_foot_link = left_foot_link;')
            commands.append('options.right_foot_link = right_foot_link;')
            commands.append("options.frozen_groups = %s;" % self.getFrozenGroupString())
            commands.append('options.RRTMaxEdgeLength = %s;' % ikParameters.rrtMaxEdgeLength)
            commands.append('options.RRTGoalBias = %s;' % ikParameters.rrtGoalBias)
            commands.append('options.N = %s;' % ikParameters.rrtMaxNumVertices)
            commands.append('options.n_smoothing_passes = %s;' % ikParameters.rrtNSmoothingPasses)
            commands.append('[xtraj,info] = collisionFreePlanner(r,t,q_seed_traj,q_nom_traj,options,active_constraints{:},s.ikoptions);')
            commands.append('if (info > 10), fprintf(\'The solver returned with info %d:\\n\',info); snoptInfo(info); end')
        elif ikParameters.useCollision == 'RRT*':
#            reachingElbowLink = ( elbowLinks[0] if ikParameters.rrtHand == 'left' else elbowLinks[1] )
#            commands.append('options.reachingElbowLink = \'%s\';' % reachingElbowLink)
            commands.append('options.end_effector_name = end_effector_name;')
            commands.append("options.graspingHand = '%s';" % ikParameters.rrtHand)
            commands.append('options.point_in_link_frame = reshape(%s, 3, []);' % ConstraintBase.toColumnVectorString(graspToHandLinkFrame.GetPosition()) )
            commands.append('options.left_foot_link = left_foot_link;')
            commands.append('options.right_foot_link = right_foot_link;')
            commands.append("options.pelvisLink = pelvis_link;")
            commands.append('options.fixed_point_file = fixed_point_file;')
            commands.append("options.frozen_groups = %s;" % self.getFrozenGroupString())

            commands.append('planner = optimalCollisionFreePlanner(r, %s, %s, options, active_constraints);\n' % (poseStart, poseEnd))
            commands.append('[xtraj, info] = planner.findCollisionFreeTraj({:s});'.format(poseEnd))
            commands.append('if (info > 10), fprintf(\'The solver returned with info %d:\\n\',info); snoptInfo(info); end')
        else:
            commands.append('q_nom_traj = PPTrajectory(foh(t, repmat(%s, 1, nt)));' % nominalPose)
            commands.append('q_seed_traj = PPTrajectory(spline([t(1), t(end)], [zeros(r.getNumPositions(),1), %s, %s, zeros(r.getNumPositions(),1)]));' % (poseStart, poseEnd))
            commands.append('\n')
            commands.append('[xtraj, info, infeasible_constraint] = inverseKinTraj(r, t, q_seed_traj, q_nom_traj, active_constraints{:}, ikoptions);')
            commands.append('\n')
            commands.append('if (info > 10) display(infeasibleConstraintMsg(infeasible_constraint)); end;')

        commands.append('if ~isempty(xtraj), qtraj = xtraj(1:r.getNumPositions()); else, qtraj = []; end;')
        commands.append('if ~isempty(qtraj), qtraj_orig = qtraj; end;')
        commands.append('if ~isempty(qtraj), joint_v_max = repmat(%s*pi/180, r.getNumVelocities()-6, 1); end;' % ikParameters.maxDegreesPerSecond)
        commands.append('if ~isempty(qtraj), xyz_v_max = repmat(%s, 3, 1); end;' % ikParameters.maxBaseMetersPerSecond)
        commands.append('if ~isempty(qtraj), rpy_v_max = repmat(%s*pi/180, 3, 1); end;' % ikParameters.maxBaseRPYDegreesPerSecond)
        commands.append('if ~isempty(qtraj), v_max = [xyz_v_max; rpy_v_max; joint_v_max]; end;')
        commands.append("if ~isempty(qtraj), v_max(r.findPositionIndices('back')) = %s*pi/180; end;" % ikParameters.maxBackDegreesPerSecond)

        commands.append("max_body_translation_speed = %r;" % ikParameters.maxBodyTranslationSpeed)
        commands.append("max_body_rotation_speed = %r;" % ikParameters.maxBodyRotationSpeed)
        commands.append('rescale_body_ids = [%s];' % (','.join(['links.%s' % linkName for linkName in ikParameters.rescaleBodyNames])))
        commands.append('rescale_body_pts = reshape(%s, 3, []);' % ConstraintBase.toColumnVectorString(ikParameters.rescaleBodyPts))
        commands.append("body_rescale_options = struct('body_id',rescale_body_ids,'pts',rescale_body_pts,'max_v',max_body_translation_speed,'max_theta',max_body_rotation_speed,'robot',r);")

        if ikParameters.usePointwise:
            assert ikParameters.useCollision == 'none'
            commands.append('\n%--- pointwise ik --------\n')
            commands.append('if ~isempty(qtraj), num_pointwise_time_points = 20; end;')
            commands.append('if ~isempty(qtraj), pointwise_time_points = linspace(qtraj.tspan(1), qtraj.tspan(2), num_pointwise_time_points); end;')
            #commands.append('spline_traj = PPTrajectory(spline(t, [ zeros(size(xtraj, 1),1), xtraj.eval(t), zeros(size(xtraj, 1),1)]));')
            #commands.append('q_seed_pointwise = spline_traj.eval(pointwise_time_points);')
            commands.append('if ~isempty(qtraj), q_seed_pointwise = qtraj.eval(pointwise_time_points); end;')
            commands.append('if ~isempty(qtraj), q_seed_pointwise = q_seed_pointwise(1:r.getNumPositions(),:); end;')
            commands.append('if ~isempty(qtraj), [qtraj_pw, info_pw] = inverseKinPointwise(r, pointwise_time_points, q_seed_pointwise, q_seed_pointwise, active_constraints{:}, ikoptions); else, qtraj_pw = []; end;')
            commands.append('if ~isempty(qtraj_pw), qtraj_pw = PPTrajectory(foh(pointwise_time_points, qtraj_pw)); end;')
            commands.append('if ~isempty(qtraj_pw), info = info_pw(end); end;')
            commands.append('if ~isempty(qtraj_pw), if (any(info_pw > 10)) disp(\'pointwise info:\'); disp(info_pw); end; end;')
            commands.append('if ~isempty(qtraj_pw), qtraj_orig = qtraj_pw; end;')
            commands.append('\n%--- pointwise ik end --------\n')

        commands.append('if ~isempty(qtraj_orig), qtraj = rescalePlanTiming(qtraj_orig, v_max, %s, %s, body_rescale_options); end;' % (ikParameters.accelerationParam, ikParameters.accelerationFraction))


        publish = True
        if publish:
            commands.append('if ~isempty(qtraj_orig), s.publishTraj(qtraj, info); end;')

        commands.append('\n%--- runIKTraj end --------\n')
        #self.taskQueue.addTask(functools.partial(self.comm.sendCommandsAsync, commands))
        #self.taskQueue.start()
        self.comm.sendCommands(commands)

        info = self.comm.getFloatArray('info')[0]
        if self.infoFunc:
            self.infoFunc(info)

        return info


    def tick(self):

        if self.handleAsyncTasks() > 0:
            return
        
    def addAffordanceToLink(self, linkName, affordance, q, affordanceName):

        affStr = ''
        desc = affordance.getDescription()
        if desc['classname'] == 'BoxAffordanceItem':
            affStr += 'RigidBodyBox(%s, %s, quat2rpy(%s));' % (desc['Dimensions'], list(desc['pose'][0]), list(desc['pose'][1]))
        elif desc['classname'] == 'CylinderAffordanceItem':
            affStr += 'RigidBodyCylinder(%s, %s, %s, quat2rpy(%s));' % (desc['Radius'], desc['Length'], list(desc['pose'][0]), list(desc['pose'][1]))
        else:
            raise Exception('Unsupported affordance type: ' + desc['classname'])


        commands = []
        commands.append('aff = %s;\n' % affStr )          
        commands.append('s = s.addAffordanceToLink(\'%s\', aff, %s, \'%s\');' % (linkName, ConstraintBase.toColumnVectorString(q), affordanceName))
        self.comm.sendCommands(commands)
    
    def removeAffordanceFromLink(self, linkName, affordanceName, nominalPoseName):
        commands = []        
        commands.append('s = s.removeAffordanceFromLink(\'%s\', \'%s\');' % (linkName, affordanceName))
        self.comm.sendCommands(commands)
    
    def searchFinalPose(self, constraints, side, eeName, eePose, nominalPoseName, capabilityMapFile, ikParameters):
        commands = []
        commands.append('default_shrink_factor = %s;' % ikParameters.quasiStaticShrinkFactor)
        constraintNames = []
        for constraintId, constraint in enumerate(constraints):
            if not constraint.enabled:
                continue
            constraint.getCommands(commands, constraintNames, suffix='_%d' % constraintId)
            commands.append('\n')
        commands.append('eeId = r.findLinkId(\'{:s}\');'.format(eeName))
        commands.append('additional_constraints = {};')
        commands.append('goal_constraints = {};')
        commands.append('capability_map = CapabilityMap([\'{:s}\', \'/{:s}\']);'.format(os.path.dirname(drcargs.args().directorConfigFile), drcargs.getDirectorConfig()['capabilityMapFile']))
        for constraint in constraintNames:
            commands.append('if isa({0:s}, \'Point2PointDistanceConstraint\') && {0:s}.body_a.idx == eeId '
                            '|| isa({0:s}, \'EulerConstraint\') && {0:s}.body == eeId '
                            'goal_constraints = {{goal_constraints{{:}}, {0:s}}}; else '
                            'additional_constraints = {{additional_constraints{{:}}, {0:s}}};end'.format(constraint))
        commands.append('cost = Point(r.getPositionFrame(),10);')
        commands.append('for i = r.getNumBodies():-1:1 '
                        'if all(r.getBody(i).parent > 0) && all(r.getBody(r.getBody(i).parent).position_num > 0) '
                        'cost(r.getBody(r.getBody(i).parent).position_num) = '
                        'cost(r.getBody(r.getBody(i).parent).position_num) + cost(r.getBody(i).position_num);end;end')
        commands.append('cost(1:6) = max(cost(7:end))/2;')
        commands.append('cost = cost/min(cost);')
        commands.append('Q = diag(cost);')
        commands.append('ikoptions = IKoptions(r);')
        commands.append('ikoptions = ikoptions.setMajorIterationsLimit({:d});'.format(ikParameters.majorIterationsLimit))
        commands.append('ikoptions = ikoptions.setQ(Q);')
        commands.append('ikoptions = ikoptions.setMajorOptimalityTolerance({:f});' .format(ikParameters.majorOptimalityTolerance))
#        commands.append('{:s}'.format(ConstraintBase.toColumnVectorString(xGoal)))
        commands.append('fpp = FinalPoseProblem(r, eeId, reach_start, {:s}, additional_constraints,'
                        '{:s}, \'capabilitymap\', capability_map, \'ikoptions\', ikoptions, \'graspinghand\', \'{:s}\');'.format(ConstraintBase.toColumnVectorString(eePose), nominalPoseName, side))
        commands.append('[x_goal, info] = fpp.findFinalPose();')
        self.comm.sendCommands(commands)
        
        info = self.comm.getFloatArray('info')[0]
        if info == 1:
            endPose = self.comm.getFloatArray('x_goal(8:end)')
        else:
            endPose = []

        return endPose, info
#        commands.append(
