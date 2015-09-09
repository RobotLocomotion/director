classdef optimalCollisionFreePlanner
  
  properties
    robot
    endEffectorId
    point_in_link_frame
    qStart
    xStart
    xGoal
    additionalConstraints
    activeCollisionOptions
    optionsPlanner
    ikoptions
    qNom
    goalConstraints
    graspingHand
  end
  
  methods
    
    function obj = optimalCollisionFreePlanner(robot, qStart, xGoal, options, objectGrasped)
      if nargin < 6
        objectGrasped = false;
      end
      
      obj.robot = robot;
      obj.endEffectorId = robot.findLinkId(options.end_effector_name);
      obj.qStart = qStart;
      obj.xGoal = xGoal;
      obj.graspingHand = options.graspingHand;
      obj.point_in_link_frame = options.point_in_link_frame;

      reachingElbowLink = robot.findLinkId(options.reachingElbowLink);
      pelvisLink = robot.findLinkId(options.pelvisLink);
      %LeftHipYawLink = robot.findLinkId('LeftHipYawLink');
      %RightHipYawLink = robot.findLinkId('RightHipYawLink');
      %LowerNeckPitchLink = robot.findLinkId('LowerNeckPitchLink');
      %TorsoPitchLink = robot.findLinkId('TorsoPitchLink');
      %TorsoYawLink = robot.findLinkId('TorsoYawLink');

      l_foot = robot.findLinkId(options.left_foot_link);
      r_foot = robot.findLinkId(options.right_foot_link);
      
      kinsol = robot.doKinematics(obj.qStart);
      obj.xStart = [robot.forwardKin(kinsol, obj.endEffectorId, obj.point_in_link_frame, 2); obj.qStart'];
      
      leftFootPose = robot.forwardKin(kinsol,l_foot, [0; 0; 0], 2);
      leftFootPosConstraint = WorldPositionConstraint(robot, l_foot, [0; 0; 0], leftFootPose(1:3), leftFootPose(1:3));
      leftFootQuatConstraint = WorldQuatConstraint(robot, l_foot, leftFootPose(4:7), 0.0, [0.0, 1.0]);
      rightFootPose = robot.forwardKin(kinsol,r_foot, [0; 0; 0], 2);
      rightFootPosConstraint = WorldPositionConstraint(robot, r_foot, [0; 0; 0], rightFootPose(1:3), rightFootPose(1:3));
      rightFootQuatConstraint = WorldQuatConstraint(robot, r_foot, rightFootPose(4:7), 0.0, [0.0, 1.0]);
      
      l_foot_pts = robot.getBody(l_foot).getTerrainContactPoints();
      r_foot_pts = robot.getBody(r_foot).getTerrainContactPoints();
      quasiStaticConstraint = QuasiStaticConstraint(robot, [-inf, inf], 1);
      quasiStaticConstraint = quasiStaticConstraint.setShrinkFactor(0.5);
      quasiStaticConstraint = quasiStaticConstraint.setActive(true);
      quasiStaticConstraint = quasiStaticConstraint.addContact(l_foot, l_foot_pts);
      quasiStaticConstraint = quasiStaticConstraint.addContact(r_foot, r_foot_pts);
      
      nonGraspingHandConstraint = Point2PointDistanceConstraint(robot, reachingElbowLink, pelvisLink, [0; 0; 0], [0; 0; 0], 0.2, Inf);
      
      obj.additionalConstraints = {leftFootPosConstraint, leftFootQuatConstraint,...
        rightFootPosConstraint, rightFootQuatConstraint, quasiStaticConstraint, nonGraspingHandConstraint};
      
      obj.xGoal = obj.xGoal(:);
      goalFrame = [eye(3) obj.xGoal(1:3); 0 0 0 1];
      goalEulerConstraint = WorldEulerConstraint(obj.robot, obj.endEffectorId, [0;0; -pi], [0; 0; pi]);
      
      goalDistConstraint = Point2PointDistanceConstraint(obj.robot, obj.endEffectorId, obj.robot.findLinkId('world'), obj.point_in_link_frame, goalFrame(1:3, 4), -0.001, 0.001);
      if ~objectGrasped
        obj.goalConstraints = {goalDistConstraint, goalEulerConstraint};
      else
        obj.goalConstraints = {goalDistConstraint};
      end      
      
      %Set IK options
      cost = Point(robot.getPositionFrame(),10);
      for i = robot.getNumBodies():-1:1
        if all(robot.getBody(i).parent > 0) && all(robot.getBody(robot.getBody(i).parent).position_num > 0)
          cost(robot.getBody(robot.getBody(i).parent).position_num) = ...
            cost(robot.getBody(robot.getBody(i).parent).position_num) + cost(robot.getBody(i).position_num);
        end
      end
      cost(1:6) = max(cost(7:end))/2;
      cost = cost/min(cost);
      Q = diag(cost);
      ikoptions = IKoptions(robot);
      ikoptions = ikoptions.setMajorIterationsLimit(100);
      ikoptions = ikoptions.setQ(Q);
      ikoptions = ikoptions.setMajorOptimalityTolerance(1e-3);
      obj.ikoptions = ikoptions;
      
      optionsPlanner = struct();
      if ~isfield(optionsPlanner,'goal_bias'), optionsPlanner.goal_bias = 0.5; end;
      if ~isfield(optionsPlanner,'n_smoothing_passes'), optionsPlanner.n_smoothing_passes = 10; end;
      if ~isfield(optionsPlanner,'planning_mode'), optionsPlanner.planning_mode = 'multiRRT'; end;
      if ~isfield(optionsPlanner,'visualize'), optionsPlanner.visualize = true; end;
      if ~isfield(optionsPlanner,'scene'), optionsPlanner.scene = 6; end;
      if ~isfield(optionsPlanner,'model'), optionsPlanner.model = 'val2'; end;
      if ~isfield(optionsPlanner,'convex_hull'), optionsPlanner.convex_hull = false; end;
      if ~isfield(optionsPlanner,'graspingHand'), optionsPlanner.graspingHand = 'right'; end;
      if ~isfield(optionsPlanner,'costType'), optionsPlanner.costType = 'length'; end;
      if ~isfield(optionsPlanner,'firstFeasibleTraj'), optionsPlanner.firstFeasibleTraj = false; end;
      if ~isfield(optionsPlanner,'robot'), optionsPlanner.robot = []; end;
      if ~isfield(optionsPlanner,'nTrees'), optionsPlanner.nTrees = 4; end;
      if ~isfield(optionsPlanner,'goalObject'), optionsPlanner.goalObject = 1; end;
      optionsPlanner.floating = true;
      optionsPlanner.terrain = RigidBodyFlatTerrain(); %Changed to a smaller terrain to avoid visualization problem when zooming
      optionsPlanner.joint_v_max = 15*pi/180;
      obj.optionsPlanner = optionsPlanner;
      
      fp = load(options.fixed_point_file);
      obj.qNom = fp.xstar(1:robot.getNumPositions());
      
      %inactive_collision_bodies = [l_foot, r_foot, LowerNeckPitchLink, RightHipYawLink,...
      %  LeftHipYawLink, TorsoPitchLink, TorsoYawLink];
      %obj.activeCollisionOptions = struct('body_idx', setdiff(1:robot.getNumBodies(), inactive_collision_bodies));
      obj.activeCollisionOptions = struct('body_idx', 1:robot.getNumBodies() );
    end
    
    function [xGoalFull, info] = findFinalPose(obj)
      capability_map_file = [getDrakePath(), '/../../control/matlab/data/val_description/capabilityMap.mat'];
      cm = CapabilityMap(capability_map_file);
      finalPose = FinalPoseProblem(obj.robot, obj.endEffectorId, obj.xStart, obj.xGoal, ...
        obj.additionalConstraints, obj.goalConstraints, obj.qNom, ...
        'capabilityMap', cm, 'graspinghand', obj.graspingHand, ...
        'activecollisionoptions', obj.activeCollisionOptions, ...
        'ikoptions', obj.ikoptions, ...
        'endeffectorpoint', obj.point_in_link_frame);
      [xGoalFull, info] = finalPose.findFinalPose(obj.optionsPlanner);
    end
    
    function [xtraj, info] = findCollisionFreeTraj(obj,xGoalFull)
      
      multiTree = MultipleTreeProblem(obj.robot, obj.endEffectorId, obj.xStart, obj.xGoal, [],...
        obj.additionalConstraints, obj.qNom,...
        'ikoptions', obj.ikoptions, 'endeffectorpoint', obj.point_in_link_frame);
      [~, info, ~, q_path] = multiTree.rrtStar(obj.optionsPlanner, xGoalFull);
      
      if info == 1
        path_length = size(q_path,2);
        xtraj = PPTrajectory(pchip(linspace(0, 1, path_length), [q_path(8:end,:); zeros(obj.robot.getNumVelocities(), size(q_path,2))] ));
      else
        xtraj = [];
        info = 13;
      end
    end
    
  end
end
