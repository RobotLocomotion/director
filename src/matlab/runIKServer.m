
s = IKServer();

robotURDF = [getenv('DRC_PATH'), '/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_fixedjoint_hands.urdf'];
%robotURDF = [getenv('DRC_PATH'), '/models/mit_gazebo_models/mit_robot/model_LN_RI.urdf'];

s = s.addRobot(robotURDF);
%s = s.addAffordance('table');
s = s.setupCosts();
s = s.loadNominalData();

r = s.robot;

nq = r.getNumDOF();
q_nom = s.q_nom;
q_zero = zeros(size(q_nom, 1), 1);
q_start = q_nom;
q_end = q_nom;

l_foot = r.findLinkInd('l_foot');
r_foot = r.findLinkInd('r_foot');
l_hand = r.findLinkInd('l_hand');
r_hand = r.findLinkInd('r_hand');
utorso = r.findLinkInd('utorso');
pelvis = r.findLinkInd('pelvis');

tspan = [0,1];
shrink_factor = 0.5;


l_foot_pts = s.getLeftFootPoints();
r_foot_pts = s.getRightFootPoints();
both_feet_qsc = QuasiStaticConstraint(r, tspan, 1);
both_feet_qsc = both_feet_qsc.setShrinkFactor(shrink_factor);
both_feet_qsc = both_feet_qsc.addContact(r_foot, r_foot_pts);
both_feet_qsc = both_feet_qsc.addContact(l_foot, l_foot_pts);
both_feet_qsc = both_feet_qsc.setActive(true);
