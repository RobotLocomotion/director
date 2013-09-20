
s = IKServer();
s = s.addRobot('model');
%s = s.addAffordance('table');
s = s.setupCosts();
s = s.loadNominalData();

r = s.robot;

q_nom = s.q_nom;

l_foot = r.findLinkInd('l_foot');
r_foot = r.findLinkInd('r_foot');
l_hand = r.findLinkInd('l_hand');
r_hand = r.findLinkInd('r_hand');
utorso = r.findLinkInd('utorso');

l_hand_pts = [0;0;0];
r_hand_pts = [0;0;0];
tspan = [0,1];

l_foot_pts = s.getLeftFootPoints();
r_foot_pts = s.getRightFootPoints();

kinsol = doKinematics(r, q_nom);
l_foot_target_start = r.forwardKin(kinsol, l_foot, l_foot_pts);
r_foot_target_start = r.forwardKin(kinsol, r_foot, r_foot_pts);

r_hand_target_start = [0.5; -0.2; 1.4];

l_foot_target = l_foot_target_start;
r_foot_target = r_foot_target_start;
r_hand_target = r_hand_target_start;

l_foot_position_constraint = WorldPositionConstraint(r, l_foot, l_foot_pts, l_foot_target, l_foot_target, tspan);
r_foot_position_constraint = WorldPositionConstraint(r, r_foot, r_foot_pts, r_foot_target, r_foot_target, tspan);
r_hand_position_constraint = WorldPositionConstraint(r, r_hand, r_hand_pts, r_hand_target, r_hand_target, tspan);
utorso_gaze_constraint = WorldGazeDirConstraint(r, utorso, [0;0;1], [0;0;1], 0.02, tspan);

scc = AllBodiesClosestDistanceConstraint(r, 0.05, 1e3, tspan);

both_feet_qsc = QuasiStaticConstraint(r);
both_feet_qsc = both_feet_qsc.setShrinkFactor(0.5);
both_feet_qsc = both_feet_qsc.addContact(r_foot, r_foot_pts);
both_feet_qsc = both_feet_qsc.addContact(l_foot, l_foot_pts);
both_feet_qsc = both_feet_qsc.setActive(true);

l_feet_qsc = QuasiStaticConstraint(r);
l_feet_qsc = both_feet_qsc.setShrinkFactor(0.5);
l_feet_qsc = both_feet_qsc.addContact(l_foot, l_foot_pts);
l_feet_qsc = both_feet_qsc.setActive(true);

r_feet_qsc = QuasiStaticConstraint(r);
r_feet_qsc = both_feet_qsc.setShrinkFactor(0.5);
r_feet_qsc = both_feet_qsc.addContact(l_foot, l_foot_pts);
r_feet_qsc = both_feet_qsc.setActive(true);

active_constraints = {both_feet_qsc, l_foot_position_constraint, r_foot_position_constraint, utorso_gaze_constraint};

[q_start, info] = inverseKin(r, q_nom, q_nom, active_constraints{:}, s.ikoptions);
q_end = q_start;
