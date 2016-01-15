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
    point_cloud
  end
  
  methods
    
    function obj = optimalCollisionFreePlanner(robot, qStart, xGoal, options, ...
        point_cloud, additionalConstraints, objectGrasped)
      if nargin < 6
        additionalConstraints = {};
      end
      if nargin < 7
        objectGrasped = false;
      end

      % Parse frozen groups
      if ~isempty(options.frozen_groups)
        back_frozen = any(strcmp('back',options.frozen_groups));
        pelvis_frozen = any(strcmp('pelvis',options.frozen_groups));
        l_arm_frozen = any(strcmp('l_leg',options.frozen_groups));
        r_arm_frozen = any(strcmp('r_leg',options.frozen_groups));
        if pelvis_frozen
          robot = robot.addToIgnoredListOfCollisionFilterGroup({'r_leg','r_uleg','l_leg','l_uleg'},'core');
          robot = robot.addToIgnoredListOfCollisionFilterGroup({'r_leg','r_uleg','l_leg','l_uleg'},'ignore_core');
          robot = robot.addToIgnoredListOfCollisionFilterGroup({'r_leg','core','ignore_core'},'l_leg');
          robot = robot.addToIgnoredListOfCollisionFilterGroup({'l_leg','core','ignore_core'},'r_leg');
        end
        if l_arm_frozen
          robot = robot.addToIgnoredListOfCollisionFilterGroup({'l_arm'},'core');
          robot = robot.addToIgnoredListOfCollisionFilterGroup({'l_arm'},'ignore_core');
          robot = robot.addToIgnoredListOfCollisionFilterGroup({'core','ignore_core'},'l_arm');
        end
        if r_arm_frozen
          robot = robot.addToIgnoredListOfCollisionFilterGroup({'r_arm'},'core');
          robot = robot.addToIgnoredListOfCollisionFilterGroup({'r_arm'},'ignore_core');
          robot = robot.addToIgnoredListOfCollisionFilterGroup({'core','ignore_core'},'r_arm');
        end
        if pelvis_frozen && back_frozen
          if l_arm_frozen
            robot = robot.addToIgnoredListOfCollisionFilterGroup({'l_arm'},'l_leg');
            robot = robot.addToIgnoredListOfCollisionFilterGroup({'l_arm'},'l_uleg');
            robot = robot.addToIgnoredListOfCollisionFilterGroup({'l_arm'},'r_leg');
            robot = robot.addToIgnoredListOfCollisionFilterGroup({'l_arm'},'r_uleg');
            robot = robot.addToIgnoredListOfCollisionFilterGroup({'l_leg','l_uleg','r_leg','r_uleg'},'l_arm');
            if r_arm_frozen
              robot = robot.addToIgnoredListOfCollisionFilterGroup({'l_arm'},'r_arm');
              robot = robot.addToIgnoredListOfCollisionFilterGroup({'r_arm'},'l_arm');
            end
          end
          if r_arm_frozen
            robot = robot.addToIgnoredListOfCollisionFilterGroup({'r_arm'},'l_leg');
            robot = robot.addToIgnoredListOfCollisionFilterGroup({'r_arm'},'l_uleg');
            robot = robot.addToIgnoredListOfCollisionFilterGroup({'r_arm'},'r_leg');
            robot = robot.addToIgnoredListOfCollisionFilterGroup({'r_arm'},'r_uleg');
            robot = robot.addToIgnoredListOfCollisionFilterGroup({'l_leg','l_uleg','r_leg','r_uleg'},'r_arm');
          end
        end
        robot = compile(robot);
        for gr = 1:numel(robot.collision_filter_groups.keys)
          fprintf('Gruop %s ignores groups ', robot.collision_filter_groups.keys{gr});
          disp(robot.collision_filter_groups(robot.collision_filter_groups.keys{gr}).ignored_collision_fgs)
          fprintf('\n')
        end
      end
      
      obj.robot = robot;
      obj.endEffectorId = obj.robot.findLinkId(options.end_effector_name);
      obj.qStart = qStart;
      obj.xGoal = xGoal;
      obj.graspingHand = options.graspingHand;
      obj.point_in_link_frame = options.point_in_link_frame;
      obj.point_cloud = point_cloud;
      
      kinsol = obj.robot.doKinematics(obj.qStart);
      obj.xStart = [obj.robot.forwardKin(kinsol, obj.endEffectorId, obj.point_in_link_frame, 2); obj.qStart];
      
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
      cost = Point(obj.robot.getPositionFrame(),10);
      for i = obj.robot.getNumBodies():-1:1
        if all(obj.robot.getBody(i).parent > 0) && all(obj.robot.getBody(obj.robot.getBody(i).parent).position_num > 0)
          cost(obj.robot.getBody(obj.robot.getBody(i).parent).position_num) = ...
            cost(obj.robot.getBody(obj.robot.getBody(i).parent).position_num) + cost(obj.robot.getBody(i).position_num);
        end
      end
%       cost(1:6) = max(cost(7:end))/2;
      cost = cost/min(cost);
      Q = diag(cost);
      obj.ikoptions = IKoptions(obj.robot);
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
      obj.qNom = fp.xstar(1:obj.robot.getNumPositions());
      
      %inactive_collision_bodies = [l_foot, r_foot, LowerNeckPitchLink, RightHipYawLink,...
      %  LeftHipYawLink, TorsoPitchLink, TorsoYawLink];
      %obj.activeCollisionOptions = struct('body_idx', setdiff(1:obj.robot.getNumBodies(), inactive_collision_bodies));
      obj.activeCollisionOptions = struct('body_idx', 1:obj.robot.getNumBodies() );
    end
    
    function [xGoalFull, info] = findFinalPose(obj)
      capability_map_file = [getDrakePath(), '/../../control/matlab/data/val_description/capabilityMap.mat'];
      cm = CapabilityMap(capability_map_file);
      finalPose = FinalPosePlanner(obj.robot, obj.endEffectorId, obj.xStart, obj.xGoal, ...
        obj.additionalConstraints, obj.goalConstraints, obj.qNom, ...
        'capabilityMap', cm, 'graspinghand', obj.graspingHand, ...
        'activecollisionoptions', obj.activeCollisionOptions, ...
        'ikoptions', obj.ikoptions, ...
        'endeffectorpoint', obj.point_in_link_frame);
      [xGoalFull, info] = finalPose.findFinalPose(obj.optionsPlanner);
    end
    
    function [xtraj, info] = findCollisionFreeTraj(obj,xGoal)
      multiTree = MultipleTreePlanner(obj.robot, obj.endEffectorId, obj.xStart, obj.xGoal, [],...
        obj.additionalConstraints, obj.qNom, obj.point_cloud, ...
        'ikoptions', obj.ikoptions, 'endeffectorpoint', obj.point_in_link_frame);
      kinsol = obj.robot.doKinematics(xGoal);
      eePose = obj.robot.forwardKin(kinsol, obj.endEffectorId, obj.point_in_link_frame, 2);
      xGoalFull = [eePose; xGoal];
      [~, info, ~, q_path] = multiTree.rrtStar(obj.optionsPlanner, xGoalFull);
      
      if info == 1
        path_length = size(q_path,2);
        xtraj = PPTrajectory(pchip(linspace(0, 1, path_length), [q_path(8:end,:); zeros(obj.robot.getNumVelocities(), size(q_path,2))] ));
      else
        xtraj = [];
      end
    end
    
  end
end
