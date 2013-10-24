
s = IKServer();
s = s.addRobot('model_minimal_contact_fixedjoint_hands');
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
gaze_theta = 0.02;
shrink_factor = 0.5;
closest_distance = 0.05;


l_foot_pts = s.getLeftFootPoints();
r_foot_pts = s.getRightFootPoints();
both_feet_qsc = QuasiStaticConstraint(r, 1, tspan);
both_feet_qsc = both_feet_qsc.setShrinkFactor(shrink_factor);
both_feet_qsc = both_feet_qsc.addContact(r_foot, r_foot_pts);
both_feet_qsc = both_feet_qsc.addContact(l_foot, l_foot_pts);
both_feet_qsc = both_feet_qsc.setActive(true);

utorso_gaze_constraint = WorldGazeDirConstraint(r, utorso, [0;0;1], [0;0;1], gaze_theta, tspan);
