s = IKServer();

warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');

s = s.addRobot(robotURDF);
s = s.setupCosts();
s = s.loadNominalData(fixed_point_file);

r = s.robot;

nq = r.getNumPositions();
q_nom = s.q_nom;
q_zero = zeros(nq, 1);
q_start = q_nom;
q_end = q_nom;

world = r.findLinkId('world');
l_foot = r.findLinkId('l_foot');
r_foot = r.findLinkId('r_foot');
l_hand = r.findLinkId('l_hand');
r_hand = r.findLinkId('r_hand');
utorso = r.findLinkId('utorso');
pelvis = r.findLinkId('pelvis');
head = r.findLinkId('head');
try
  l_foot_pts = s.getLeftFootPoints();
  r_foot_pts = s.getRightFootPoints();
catch
  % no-op
end

joints = Point(r.getStateFrame, (1:r.getStateFrame.dim)');
plan_publisher = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN', true, r.getStateFrame.coordinates(1:nq));
