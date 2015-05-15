s = IKServer();

warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');

s = s.addRobot(robotURDF);
s = s.setupCosts();
s = s.loadNominalData(fixed_point_file);

r = s.robot_and_environment;

nq = r.getNumPositions();
q_nom = s.q_nom;
q_zero = zeros(nq, 1);
q_end = q_nom;

try
  l_foot_pts = s.getLinkPoints(left_foot_link);
  r_foot_pts = s.getLinkPoints(right_foot_link);
  pelvis_pts = s.getPelvisPoints();
catch
  % no-op
end

links = struct();
linknames = s.getLinkNames();
for i = 1:size(linknames, 1)
  nameComponents = regexp(linknames{i}, '\+', 'split');
  for name = nameComponents
    links.(name{1}) = i;
  end
end

joints = Point(r.getStateFrame, (1:r.getStateFrame.dim)');
ikServerStarted = true;
