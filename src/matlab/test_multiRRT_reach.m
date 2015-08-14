function info = test_multiRRT_reach(r, s, grHand, qStart, xGoal, objectGrasped)
  if nargin < 6
    objectGrasped = false;
  end  
  hand.left = r.findLinkId('LeftPalm');
  hand.right = r.findLinkId('RightPalm');
  point_in_link_frame.right = [0.08; -0.07; 0];
  point_in_link_frame.left = [0.08; 0.07; 0];
  point_in_link_frame = point_in_link_frame.(grHand);
  
%   r.body(hand.(grHand)).collision_geometry  
%   r.body(hand.(grHand)).visual_geometry
  
  addpath(fullfile(getDrakePath(), '..', 'ddapp/src/matlab') )
  fixed_point_file = [getDrakePath(), '/../control/matlab/data/val_description/valkyrie_fp_june2015.mat'];
  left_foot_link = 'LeftFoot';
  right_foot_link = 'RightFoot';
%   runRRTIKServer
  kinsol = r.doKinematics(qStart);
  xStart = [r.forwardKin(kinsol, hand.(grHand), point_in_link_frame, 2); qStart'];
  
  l_foot = r.findLinkId('LeftFoot');
  r_foot = r.findLinkId('RightFoot');
  leftFootPose = r.forwardKin(kinsol,l_foot, [0; 0; 0], 2);
  leftFootPosConstraint = WorldPositionConstraint(r, l_foot, [0; 0; 0], leftFootPose(1:3), leftFootPose(1:3));
  leftFootQuatConstraint = WorldQuatConstraint(r, l_foot, leftFootPose(4:7), 0.0, [0.0, 1.0]);
  rightFootPose = r.forwardKin(kinsol,r_foot, [0; 0; 0], 2);
  rightFootPosConstraint = WorldPositionConstraint(r, r_foot, [0; 0; 0], rightFootPose(1:3), rightFootPose(1:3));
  rightFootQuatConstraint = WorldQuatConstraint(r, r_foot, rightFootPose(4:7), 0.0, [0.0, 1.0]);
  
%   xGoal = [(leftFootPose(1:2) + rightFootPose(1:2))/2; 0]' + [0.1; -0.3; 0.9]'; 
  
  xGoal = xGoal(:);
  goalFrame = [eye(3) xGoal(1:3); 0 0 0 1];
  goalEulerConstraint = WorldEulerConstraint(r, hand.(grHand), [0;0; -pi], [0; 0; pi]);
  
  goalDistConstraint = Point2PointDistanceConstraint(r, hand.(grHand), r.findLinkId('world'), point_in_link_frame, goalFrame(1:3, 4), -0.001, 0.001);
  if ~objectGrasped
    goalConstraints = {goalDistConstraint, goalEulerConstraint};
  else
    goalConstraints = {goalDistConstraint};
  end
  
  l_foot_pts = r.getBody(l_foot).getTerrainContactPoints();
  r_foot_pts = r.getBody(r_foot).getTerrainContactPoints();
  quasiStaticConstraint = QuasiStaticConstraint(r, [-inf, inf], 1);
  quasiStaticConstraint = quasiStaticConstraint.setShrinkFactor(0.5);
  quasiStaticConstraint = quasiStaticConstraint.setActive(true);
  quasiStaticConstraint = quasiStaticConstraint.addContact(l_foot, l_foot_pts);
  quasiStaticConstraint = quasiStaticConstraint.addContact(r_foot, r_foot_pts);  

  elbow.right = r.findLinkId('RightElbowPitchLink');
  elbow.left = r.findLinkId('LeftElbowPitchLink');
  trunk = r.findLinkId('Torso');
  nonGraspingHandConstraint = Point2PointDistanceConstraint(r, elbow.(grHand), trunk, [0; 0; 0], [0; 0; 0], 0.2, Inf);
      
  additionalConstraints = {leftFootPosConstraint, leftFootQuatConstraint,...
    rightFootPosConstraint, rightFootQuatConstraint, quasiStaticConstraint};%, nonGraspingHandConstraint};
  
  cm = CapabilityMap([fileparts(which('exploringRRT')) '/CapabilityMap/capabilityMap.mat']);
  
  LeftHipYawLink = r.findLinkId('LeftHipYawLink');
  RightHipYawLink = r.findLinkId('RightHipYawLink');
  LowerNeckPitchLink = r.findLinkId('LowerNeckPitchLink');
  TorsoPitchLink = r.findLinkId('TorsoPitchLink');
  TorsoYawLink = r.findLinkId('TorsoYawLink');

  inactive_collision_bodies = [l_foot, r_foot, LowerNeckPitchLink, RightHipYawLink,...
    LeftHipYawLink, TorsoPitchLink, TorsoYawLink];
  
  %Set IK options
  cost = Point(r.getPositionFrame(),10);
  for i = r.getNumBodies():-1:1
    if all(r.getBody(i).parent > 0) && all(r.getBody(r.getBody(i).parent).position_num > 0)
      cost(r.getBody(r.getBody(i).parent).position_num) = ...
        cost(r.getBody(r.getBody(i).parent).position_num) + cost(r.getBody(i).position_num);
    end
  end
  cost(1:6) = max(cost(7:end))/2;
  cost = cost/min(cost);
  Q = diag(cost);
  ikoptions = IKoptions(r);
  ikoptions = ikoptions.setMajorIterationsLimit(100);
  ikoptions = ikoptions.setQ(Q);
  ikoptions = ikoptions.setMajorOptimalityTolerance(1e-3);
  
  options = struct();
  
  if ~isfield(options,'goal_bias'), options.goal_bias = 0.5; end;
  if ~isfield(options,'n_smoothing_passes'), options.n_smoothing_passes = 10; end;
  if ~isfield(options,'planning_mode'), options.planning_mode = 'multiRRT'; end;
  if ~isfield(options,'visualize'), options.visualize = true; end;
  if ~isfield(options,'scene'), options.scene = 6; end;
  if ~isfield(options,'model'), options.model = 'val2'; end;
  if ~isfield(options,'convex_hull'), options.convex_hull = false; end;
  if ~isfield(options,'graspingHand'), options.graspingHand = 'right'; end;
  if ~isfield(options,'costType'), options.costType = 'length'; end;
  if ~isfield(options,'firstFeasibleTraj'), options.firstFeasibleTraj = false; end;
  if ~isfield(options,'robot'), options.robot = []; end;
  if ~isfield(options,'nTrees'), options.nTrees = 4; end;
  if ~isfield(options,'goalObject'), options.goalObject = 1; end;
  
  
  options.floating = true;
  options.terrain = RigidBodyFlatTerrain(); %Changed to a smaller terrain to avoid visualization problem when zooming
  options.joint_v_max = 15*pi/180;
  
  fp = load([getDrakePath(), '/../control/matlab/data/val_description/valkyrie_fp_june2015.mat']);
  qNom = fp.xstar(1:r.getNumPositions());
  xGoal
  multiTree = MultipleTreeProblem(r, hand.(grHand), xStart, xGoal, [],...
    goalConstraints, additionalConstraints, qNom,...
    'capabilityMap', cm, 'graspingHand', grHand, 'activecollisionoptions',...
    struct('body_idx', setdiff(1:r.getNumBodies(), inactive_collision_bodies)),...
    'ikoptions', ikoptions, 'endeffectorpoint', point_in_link_frame);
  [~, info, ~, q_path] = multiTree.rrt(options);
  disp(info)
  info = info.status;
  disp(info)
  
  path_length = size(q_path,2);  
  q_traj = PPTrajectory(pchip(linspace(0, 1, path_length), q_path(8:end,:)));
  s.publishTraj(q_traj, 1);  
  
end