
s = IKServer();
s = s.addRobot('model');
%s = s.addAffordance('table');
s = s.setupCosts();
s = s.loadNominalData();

r = s.robot

q_nom = s.q_nom;

l_foot = r.findLinkInd('l_foot');
r_foot = r.findLinkInd('r_foot');
l_hand = r.findLinkInd('l_hand');
r_hand = r.findLinkInd('r_hand');

l_toes_x = 0.0;
l_hand_pts = [0;0;0];
r_hand_pts = [0;-0.1;0];
hand_goal = [0.5;-0.2;1.4];
tspan = [0,1];

l_foot_pts = s.getLeftFootPoints();
r_foot_pts = s.getRightFootPoints();


kinsol = doKinematics(r, q_nom);
l_foot_target_start = r.forwardKin(kinsol,l_foot,l_foot_pts);
r_foot_target = r.forwardKin(kinsol,r_foot,r_foot_pts);


l_foot_target = vertcat(l_foot_target_start(1:2,:), l_foot_target_start(3,:)+0.1);


kc2l = WorldPositionConstraint(r, l_foot, l_foot_pts, l_foot_target, l_foot_target, tspan);
kc2r = WorldPositionConstraint(r, r_foot, r_foot_pts, r_foot_target, r_foot_target, tspan);

kc3 = WorldPositionConstraint(r,r_hand,r_hand_pts,hand_goal,hand_goal,[1 1]);
kc4 = WorldGazeDirConstraint(r,r_hand,[0;-0.5;1],[0;0;1],0.1,[1,1]);

qsc = QuasiStaticConstraint(r);
qsc = qsc.setShrinkFactor(0.5);
qsc = qsc.addContact(r_foot, r_foot_pts);
qsc = qsc.addContact(l_foot, l_foot_pts);
qsc = qsc.setActive(true);

q_seed = q_nom;

[q_start, info] = inverseKin(r, q_seed, q_nom, qsc, kc2l, kc2r, kc3, kc4, s.ikoptions)

