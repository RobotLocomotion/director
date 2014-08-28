s = IKServer();

warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');

drakeExamplePath = [getenv('DRC_BASE'), '/software/drake/examples/'];

robotURDF = [getenv('DRC_PATH'), '/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'];
%robotURDF = [drakeExamplePath, 'Atlas/urdf/atlas_convex_hull.urdf'];

s = s.addRobot(robotURDF);
s = s.setupCosts();
s = s.loadNominalData();

r = s.robot;

nq = r.getNumPositions();
q_nom = s.q_nom;
q_zero = zeros(nq, 1);
q_start = q_nom;
q_end = q_nom;

world = r.findLinkInd('world');
l_foot = r.findLinkInd('l_foot');
r_foot = r.findLinkInd('r_foot');
l_hand = r.findLinkInd('l_hand');
r_hand = r.findLinkInd('r_hand');
utorso = r.findLinkInd('utorso');
pelvis = r.findLinkInd('pelvis');
head = r.findLinkInd('head');
l_foot_pts = s.getLeftFootPoints();
r_foot_pts = s.getRightFootPoints();

joints = Point(r.getStateFrame, (1:r.getStateFrame.dim)');
plan_publisher = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN', true, r.getStateFrame.coordinates(1:nq));

%r = r.replaceContactShapesWithCHull([l_hand, r_hand, head]);
%r = compile(r);
