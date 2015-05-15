classdef IKServer

  properties
    robot
    robot_and_environment
    environment_urdf_string = []
    ikoptions
    q_nom
    plan_publisher
    min_distance = 1;
  end

  methods

    function obj = IKServer()

      options.floating = true;
      options.ignore_terrain_collisions = true;
      options.terrain = [];

      obj.robot = RigidBodyManipulator([], options);

      obj = obj.compile();
    end

    function obj = loadNominalData(obj,filename)
      if nargin < 1
        filename = [getenv('DRC_BASE'), ...
          '/software/control/matlab/data/atlas_bdi_fp.mat'];
      end
      %nom_data = load([getDrakePath() '/examples/Atlas/data/atlas_bdi_fp.mat']);
      nom_data = load(filename);
      nq = obj.robot.getNumPositions();
      obj.q_nom = nom_data.xstar(1:nq);
    end

    function obj = setupOptions(obj, options)
      if isempty(obj.ikoptions)
        obj.ikoptions = IKoptions(obj.robot_and_environment);
      end
      if nargin < 2
        ikoptions_old = obj.ikoptions;
        obj.ikoptions = obj.ikoptions.updateRobot(obj.robot_and_environment);
        obj.ikoptions = obj.ikoptions.setMex(ikoptions_old.use_mex);
        obj.ikoptions = obj.ikoptions.setMajorIterationsLimit(ikoptions_old.SNOPT_MajorIterationsLimit);
        obj.ikoptions = obj.ikoptions.setIterationsLimit(ikoptions_old.SNOPT_IterationsLimit);
        obj.ikoptions = obj.ikoptions.setMajorFeasibilityTolerance(ikoptions_old.SNOPT_MajorFeasibilityTolerance);
        obj.ikoptions = obj.ikoptions.setMajorOptimalityTolerance(ikoptions_old.SNOPT_MajorOptimalityTolerance);
      else
        if isfield(options, 'MajorIterationsLimit')
          obj.ikoptions = obj.ikoptions.setMajorIterationsLimit(options.MajorIterationsLimit);
        end
        if isfield(options, 'IterationsLimit')
          obj.ikoptions = obj.ikoptions.setIterationsLimit(options.IterationsLimit);
        end
        if isfield(options, 'MajorFeasibilityTolerance')
          obj.ikoptions = obj.ikoptions.setMajorFeasibilityTolerance(options.MajorFeasibilityTolerance);
        end
        if isfield(options, 'MajorOptimalityTolerance')
          obj.ikoptions = obj.ikoptions.setMajorOptimalityTolerance(options.MajorOptimalityTolerance);
        end
        if isfield(options, 'FixInitialState')
          obj.ikoptions = obj.ikoptions.setFixInitialState(options.FixInitialState);
        end
        if isfield(options, 'MinDistance')
          obj.min_distance = options.MinDistance;
        end
      end
    end

    function obj = setupCosts(obj)
      leftLegCost = 1e3;
      rightLegCost = 1e3;
      leftArmCost = 1;
      rightArmCost = 1;
      backCost = 1e4;
      neckCost = 1e6;

      nq = obj.ikoptions.robot.getNumPositions();

      if nq > 0
        cost = ones(nq,1);

        base_x_index = findPositionIndices(obj.ikoptions.robot, 'base_x');
        cost(base_x_index) = 0;

        base_y_index = findPositionIndices(obj.ikoptions.robot, 'base_y');
        cost(base_y_index) = 0;

        base_z_index = findPositionIndices(obj.ikoptions.robot, 'base_z');
        cost(base_z_index) = 0;

        base_roll_index = findPositionIndices(obj.ikoptions.robot, 'base_roll');
        cost(base_roll_index) = 1e3;

        base_pitch_index = findPositionIndices(obj.ikoptions.robot, 'base_pitch');
        cost(base_pitch_index) = 1e3;

        base_yaw_index = findPositionIndices(obj.ikoptions.robot, 'base_yaw');
        cost(base_yaw_index) = 0;

        l_arm_indices = findPositionIndices(obj.ikoptions.robot, 'l_arm');
        cost(l_arm_indices) = leftArmCost;

        r_arm_indices = findPositionIndices(obj.ikoptions.robot, 'r_arm');
        cost(r_arm_indices) = rightArmCost;

        l_leg_indices = findPositionIndices(obj.ikoptions.robot, 'l_leg');
        cost(l_leg_indices) = leftLegCost;

        r_leg_indices = findPositionIndices(obj.ikoptions.robot, 'r_leg');
        cost(r_leg_indices) = rightLegCost;

        back_indices = findPositionIndices(obj.ikoptions.robot, 'back');
        cost(back_indices) = backCost;

        neck_indices = findPositionIndices(obj.ikoptions.robot, 'neck');
        cost(neck_indices) = neckCost;

        vel_cost = cost*0.05;
        accel_cost = cost*0.05;

        obj.ikoptions = obj.ikoptions.setQ(diag(cost(1:nq)));
        obj.ikoptions = obj.ikoptions.setQa(diag(vel_cost(1:nq)));
        obj.ikoptions = obj.ikoptions.setQv(diag(accel_cost(1:nq)));
        obj.ikoptions = obj.ikoptions.setqd0(zeros(nq,1), zeros(nq,1)); % upper and lower bnd on initial velocity.
        obj.ikoptions = obj.ikoptions.setqdf(zeros(nq,1), zeros(nq,1)); % upper and lower bnd on final velocity.
      end

    end


    function obj = addRobot(obj, filename)

      options.floating = true;
      options.ignore_terrain_collisions = true;
      options.terrain = [];

      xyz = zeros(3,1);
      rpy = zeros(3,1);
      obj.robot = obj.robot.addRobotFromURDF(filename , xyz, rpy, options);
      obj.plan_publisher = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN', true, obj.robot.getPositionFrame.coordinates);
      obj = obj.compile();
    end

    function obj = setJointLimits(obj, joint_limit_min_new, joint_limit_max_new)
      obj.robot = obj.robot.setJointLimits(joint_limit_min_new, ...
                                           joint_limit_max_new);
      obj = obj.compile();
    end

    function obj = compile(obj)
      obj.robot = obj.robot.compile();
      if isempty(obj.environment_urdf_string)
        obj.robot_and_environment = obj.robot;
      else
        obj.robot_and_environment = ...
          obj.robot.addRobotFromURDFString(obj.environment_urdf_string);
      end
      obj = obj.setupOptions();
      obj = obj.setupCosts();
    end

    function obj = setEnvironment(obj, urdf_string)
      if nargin < 2 || isempty(urdf_string)
        obj.environment_urdf_string = [];
      else
        typecheck(urdf_string, 'char'); 
        if ~strcmp(obj.environment_urdf_string, urdf_string)
          obj.environment_urdf_string = urdf_string;
        end
      end
      obj = obj.compile();
    end

    function obj = addAffordance(obj, affordanceName)
      filename = [getenv('DRC_PATH'), '/drake/systems/plants/test/', affordanceName, '.urdf'];
      options = struct('floating', false);
      xyz = zeros(3,1);
      rpy = zeros(3,1);
      obj.robot = obj.robot.addRobotFromURDF(filename , xyz, rpy, options);
      obj.robot = compile(obj.robot);
    end

    function linkNames = getLinkNames(obj)
      linkNames = {obj.robot.body.linkname}';
    end

    function pts = getLinkPoints(obj, linkName)
      bodyIndex = obj.robot.findLinkId(linkName);
      pts = obj.robot.body(bodyIndex).getTerrainContactPoints({'toe', 'heel'});
    end

    function pts = getPelvisPoints(obj)
      bodyIndex = obj.robot.findLinkId('pelvis');
      pts = obj.robot.body(bodyIndex).getTerrainContactPoints();
    end

    function [q, info, infeasible_constraint] = runIk(obj, ik_seed_pose, ik_nominal_pose, constraints, use_collision)
      if nargin < 5, use_collision = false; end
      [q, info, infeasible_constraint] = inverseKin(obj.robot_and_environment, ik_seed_pose, ik_nominal_pose, constraints{:}, obj.ikoptions);
      if (info > 10) 
        display(infeasibleConstraintMsg(infeasible_constraint)); 
        return;
      end;
      if use_collision
        [phi, ~, ~, ~, idxA, idxB] = obj.robot_and_environment.collisionDetect(q);
        if any(phi < obj.min_distance);
          active_collision_options.body_idx = union(idxA(phi < 2*obj.min_distance), idxB(phi < 2*obj.min_distance)); 
          collision_constraint = MinDistanceConstraint(obj.robot_and_environment, 1.2*obj.min_distance, active_collision_options); 
          [q, info, infeasible_constraint] = obj.runIk(q, ik_nominal_pose, [constraints, {collision_constraint}],false);
          phi = obj.robot_and_environment.collisionDetect(q);
          if info < 10 && any(phi < obj.min_distance);
            info = 13;
            display('Could not satisfiy collision avoidance constraints.');
          end
        end
      end
    end

    function plan_time = getPlanTimeForJointVelocity(obj, xtraj, maxDegreesPerSecond)

      qdot_desired = maxDegreesPerSecond*pi/180;

      nq = obj.robot.getNumPositions();
      ts = xtraj.pp.breaks;

      sfine = linspace(ts(1), ts(end), 50);
      coords = obj.robot.getStateFrame.coordinates(1:nq);
      neck_idx = strcmp(coords, 'neck_ay');
      back_joints = cellfun(@(s) ~isempty(strfind(s,'back_bk')), coords);

      qtraj = xtraj(1:nq);
      dqtraj = fnder(qtraj, 1);
      qdot_breaks = dqtraj.eval(sfine);
      weighted_qdot_breaks = qdot_breaks(~neck_idx,:);
      weighted_qdot_breaks(back_joints,:) = 2*weighted_qdot_breaks(back_joints,:);
      plan_time = max(max(abs(weighted_qdot_breaks),[],2))/qdot_desired;

    end

    function publishTraj(obj, xtraj, snopt_info)

      utime = now() * 24 * 60 * 60;
      nq_atlas = obj.robot.getNumPositions;
      ts = xtraj.pp.breaks;
      q = xtraj.eval(ts);
      xtraj_atlas = zeros(2+2*nq_atlas,length(ts));
      xtraj_atlas(2+(1:nq_atlas),:) = q(1:nq_atlas,:);
      snopt_info_vector = snopt_info*ones(1, size(xtraj_atlas,2));

      ts = ts - ts(1);
      obj.plan_publisher.publish(xtraj_atlas, ts, utime, snopt_info_vector);

    end

  end

end
