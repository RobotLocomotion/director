s = IKServer();

robotURDF = [getenv('DRC_PATH'), '/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'];

s = s.addRobot(robotURDF);
s = s.setupCosts();
s = s.loadNominalData();

r = s.robot;

rbmoptions.floating = true;
atlas = Atlas(strcat(getenv('DRC_BASE'),'/software/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),rbmoptions);

joint_names = atlas.getStateFrame.coordinates(1:getNumDOF(atlas));
joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase');
plan_pub = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN', true, joint_names);

nq = r.getNumDOF();
q_nom = s.q_nom;
q_zero = zeros(nq, 1);
q_start = q_nom;
q_end = q_nom;

l_foot = r.findLinkInd('l_foot');
r_foot = r.findLinkInd('r_foot');
l_hand = r.findLinkInd('l_hand');
r_hand = r.findLinkInd('r_hand');
utorso = r.findLinkInd('utorso');
pelvis = r.findLinkInd('pelvis');
l_foot_pts = s.getLeftFootPoints();
r_foot_pts = s.getRightFootPoints();

joints = Point(r.getStateFrame, (1:r.getStateFrame.dim)');
