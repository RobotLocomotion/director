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
    
    function obj = optimalCollisionFreePlanner(robot, qStart, xGoal, options, additionalConstraints, objectGrasped)
      if nargin < 5
        additionalConstraints = {};
      end
      if nargin < 6
        objectGrasped = false;
      end
      
      obj.robot = robot;
      obj.endEffectorId = robot.findLinkId(options.end_effector_name);
      obj.qStart = qStart;
      obj.xGoal = xGoal;
      obj.graspingHand = options.graspingHand;
      obj.point_in_link_frame = options.point_in_link_frame;
      
      kinsol = robot.doKinematics(obj.qStart);
      obj.xStart = [robot.forwardKin(kinsol, obj.endEffectorId, obj.point_in_link_frame, 2); obj.qStart];
      
      obj.additionalConstraints = additionalConstraints;
      
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
      obj.ikoptions = IKoptions(robot);
      obj.ikoptions = obj.ikoptions.setMajorIterationsLimit(100);
      obj.ikoptions = obj.ikoptions.setQ(Q);
      obj.ikoptions = obj.ikoptions.setMajorOptimalityTolerance(1e-3);
      
      obj.optionsPlanner = struct();
      if ~isfield(obj.optionsPlanner,'goal_bias'), obj.optionsPlanner.goal_bias = 0.5; end;
      if ~isfield(obj.optionsPlanner,'n_smoothing_passes'), obj.optionsPlanner.n_smoothing_passes = 10; end;
      if ~isfield(obj.optionsPlanner,'visualize'), obj.optionsPlanner.visualize = true; end;
      if ~isfield(obj.optionsPlanner,'model'), obj.optionsPlanner.model = 'val2'; end;
      if ~isfield(obj.optionsPlanner,'convex_hull'), obj.optionsPlanner.convex_hull = false; end;
      if ~isfield(obj.optionsPlanner,'costType'), obj.optionsPlanner.costType = 'length'; end;
      if ~isfield(obj.optionsPlanner,'firstFeasibleTraj'), obj.optionsPlanner.firstFeasibleTraj = false; end;
      if ~isfield(obj.optionsPlanner,'nTrees'), obj.optionsPlanner.nTrees = 4; end;
      obj.optionsPlanner.floating = true;
      obj.optionsPlanner.terrain = RigidBodyFlatTerrain();
      obj.optionsPlanner.joint_v_max = 15*pi/180;
      
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
