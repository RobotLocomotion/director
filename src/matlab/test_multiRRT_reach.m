function [r, grHand, qStart, xGoal, obstacles] = test_multiRRT_reach(r, grHand, qStart, xGoal, obstacles)
  world = r.findLinkId('world');
  hand.left = r.findLinkId('LeftPalm');
  hand.right = r.findLinkId('RightPalm');
  for ob = 1:numel(obstacles)
    r.addGeometryToBody(world, ob)
  end
  r = r.setTerrain(MyRigidBodyFlatTerrain());
  r = r.compile();
  
  kinsol = r.doKinematics(qStart);
  xStart = [r.forwardKin(kinsol, hand.(grHand), [0;0;0], 2); qStart'];
  
  goalFrame = [eye(3) xGoal(1:3)'; 0 0 0 1];
  point_in_link_frame = [0.08; -0.07; 0];
  goalEulerConstraint = WorldEulerConstraint(r, hand.(grHand), [0;0; -pi], [0; 0; pi]);
  
  goalDistConstraint = Point2PointDistanceConstraint(r, hand.(grHand), r.findLinkId('world'), point_in_link_frame, goalFrame(1:3, 4), -0.001, 0.001);
  goalConstraints = {goalDistConstraint, goalEulerConstraint};
  
  l_foot = r.findLinkId('LeftFoot');
  r_foot = r.findLinkId('RightFoot');
  footPose = r.forwardKin(kinsol,l_foot, [0; 0; 0], 2);
  leftFootPosConstraint = WorldPositionConstraint(r, l_foot, [0; 0; 0], footPose(1:3), footPose(1:3));
  leftFootQuatConstraint = WorldQuatConstraint(r, l_foot, footPose(4:7), 0.0, [0.0, 1.0]);
  footPose = r.forwardKin(kinsol,r_foot, [0; 0; 0], 2);
  rightFootPosConstraint = WorldPositionConstraint(r, r_foot, [0; 0; 0], footPose(1:3), footPose(1:3));
  rightFootQuatConstraint = WorldQuatConstraint(r, r_foot, footPose(4:7), 0.0, [0.0, 1.0]);  
  
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
  
  disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
  disp(1:r.getNumBodies())
  
  MultipleTreeProblem(r, hand.(grHand), xStart, xGoal(1:3)', [],...
    goalConstraints, additionalConstraints, qStart',...
    'capabilityMap', cm, 'graspingHand', grHand, 'activecollisionoptions',...
    struct('body_idx', []),...setdiff(1:r.getNumBodies(), inactive_collision_bodies)),...
    'ikoptions', ikoptions, 'endeffectorpoint', point_in_link_frame);
end