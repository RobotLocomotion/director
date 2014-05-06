
q_start = [0.0;0.0;0.9182347451272753;0.0;-0.04000360317860477;0.0;0.0;0.10108478362268643;0.0;0.448768638863074;-1.3;1.9090893714526962;1.2;0.0;0.0;0.025;-0.15024286361765363;0.40046989398720956;-0.21021393971080285;-0.025;0.0;0.4487686388630743;1.3;1.9090893714527;-1.2;0.0;0.0;-0.025;-0.1502429446261945;0.40047012281409394;-0.21021419704244187;0.025;0.0;0.015075092984253066];
grasp_end_pose = [0.0;0.0;0.599265694554918;0.0;-0.04000360317860477;0.0;0.3396036439776378;0.3;0.1643826115309001;0.448768638863074;-1.3;1.909089371452696;1.2;0.0;-0.0006326106657452376;0.04035055974731892;-0.9645586795483393;2.005213296870122;-0.9999961928669501;-0.040351131070352;0.0;-1.291487547498241;0.5859024420072335;2.551640113602723;0.0;0.9211493027547477;0.000626109485991348;-0.0403510022197885;-0.964558942370358;2.0052133769072;-0.9999978276955481;0.04032502262753835;-1.160438174495118;0.01507509298425307];
pre_grasp_end_pose = [0.0;0.0;0.599265694554918;0.0;-0.04000360317860477;0.0;0.3396036439776378;0.3;0.1643826115309001;0.448768638863074;-1.3;1.909089371452696;1.2;0.0;-0.0006326106657452376;0.04035055974731892;-0.9645586795483393;2.005213296870122;-0.9999961928669501;-0.040351131070352;0.0;-1.229163707421572;-0.03506759618515682;2.883387648667306;-0.6679088555173767;0.4859861140275853;0.000626109485991348;-0.0403510022197885;-0.964558942370358;2.0052133769072;-0.9999978276955481;0.04032502262753835;-1.1781;0.01507509298425307];

%-------- runIkTraj --------

qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(0.5);
qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(l_foot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(r_foot, r_foot_pts);


point_in_link_frame = [0.0; 0.0; 0.0];
ref_frame = [1.0, 0.0, 0.0, 0.0; 0.0, 1.0, 0.0, 0.0; 0.0, 0.0, 1.0, 0.0; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [-0.01448723334717765; 0.10990102365868805; 0.081118870223845674] + [0.0; 0.0; 0.0];
upper_bounds = [-0.01448723334717765; 0.10990102365868805; 0.081118870223845674] + [0.0; 0.0; 0.0];
position_constraint_1 = WorldPositionInFrameConstraint(r, l_foot, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [-inf, inf]);


quat_constraint_2 = WorldQuatConstraint(r, l_foot, [0.99999987495976839; -1.0001797447809155e-05; -1.5063122306352283e-06; 0.00049997814218506287], 0.0, [-inf, inf]);


point_in_link_frame = [0.0; 0.0; 0.0];
ref_frame = [1.0, 0.0, 0.0, 0.0; 0.0, 1.0, 0.0, 0.0; 0.0, 0.0, 1.0, 0.0; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [-0.014487263836492709; -0.10990102325931325; 0.081118884987964379] + [0.0; 0.0; 0.0];
upper_bounds = [-0.014487263836492709; -0.10990102325931325; 0.081118884987964379] + [0.0; 0.0; 0.0];
position_constraint_3 = WorldPositionInFrameConstraint(r, r_foot, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [-inf, inf]);


quat_constraint_4 = WorldQuatConstraint(r, r_foot, [0.99999987496036868; 1.0001770081385264e-05; -1.5610517753164795e-06; -0.00049997677416967591], 0.0, [-inf, inf]);


posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.back_bkx; joints.back_bky; joints.back_bkz];
joints_lower_limit = q_zero(joint_inds) + [-0.2; -0.1; -0.6];
joints_upper_limit = q_zero(joint_inds) + [0.2; 0.3; 0.6];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_6 = PostureConstraint(r, [0.0, 0.7]);
joint_inds = [joints.base_x; joints.base_y; joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = q_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = q_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_6 = posture_constraint_6.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_7 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.l_arm_usy; joints.l_arm_shx; joints.l_arm_ely; joints.l_arm_elx; joints.l_arm_uwy; joints.l_arm_mwx];
joints_lower_limit = q_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = q_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_7 = posture_constraint_7.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [4.8881567872971488e-05; -0.2444999900441342; -0.011199999999999988];
ref_frame = [0.0; 0.0; 0.0; 1.0; 0.0; 0.0; 0.0];
lower_bounds = [0.20000000000000001; -0.69999999999999996; 0.0] + [0.0; 0.0; 0.0];
upper_bounds = [0.20000000000000001; -0.69999999999999996; 0.0] + [0.0; 0.0; 0.0];
relative_position_constraint_8 = RelativePositionConstraint(r, point_in_link_frame, lower_bounds, upper_bounds, r_hand, utorso, ref_frame, [0.35, 0.35]);


posture_constraint_9 = PostureConstraint(r, [0.7, 1.0]);
joint_inds = [joints.base_x; joints.base_y; joints.base_z; joints.base_roll; joints.base_pitch; joints.base_yaw; joints.back_bkz; joints.back_bky; joints.back_bkx; joints.r_leg_hpz; joints.r_leg_hpx; joints.r_leg_hpy; joints.r_leg_kny; joints.r_leg_aky; joints.r_leg_akx; joints.l_leg_hpz; joints.l_leg_hpx; joints.l_leg_hpy; joints.l_leg_kny; joints.l_leg_aky; joints.l_leg_akx];
joints_lower_limit = grasp_end_pose(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = grasp_end_pose(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_9 = posture_constraint_9.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [4.8881567872971488e-05; -0.2444999900441342; -0.011199999999999988];
ref_frame = [-0.003614334564691173, -0.13876008969197298, -0.9903194303326226, 0.80231846659663497; 0.96997860997412289, 0.24032609747395822, -0.037213748341250581, -0.15428377967616039; 0.2431633870020791, -0.96072316740131392, 0.13372569999840705, 0.71673797229622094; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.02; -0.02; -0.02];
upper_bounds = [0.0; 0.0; 0.0] + [0.02; 0.02; 0.02];
position_constraint_10 = WorldPositionInFrameConstraint(r, r_hand, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [0.7, 0.7]);


quat_constraint_11 = WorldQuatConstraint(r, r_hand, [0.47343487303202131; 0.52691396491091436; -0.39433339141221113; -0.58542486366429425], 0.12217304763960307, [0.7, 0.7]);


point_in_link_frame = [4.8881567872971488e-05; -0.2444999900441342; -0.011199999999999988];
ref_frame = [-0.003614334564691173, -0.13876008969197298, -0.9903194303326226, 0.77456644865824043; 0.96997860997412289, 0.24032609747395822, -0.037213748341250581, -0.10621856018136874; 0.2431633870020791, -0.96072316740131392, 0.13372569999840705, 0.52459333881595815; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0050000000000000001; -0.0050000000000000001; -0.0050000000000000001];
upper_bounds = [0.0; 0.0; 0.0] + [0.0050000000000000001; 0.0050000000000000001; 0.0050000000000000001];
position_constraint_12 = WorldPositionInFrameConstraint(r, r_hand, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


quat_constraint_13 = WorldQuatConstraint(r, r_hand, [0.47343487303202131; 0.52691396491091436; -0.39433339141221113; -0.58542486366429425], 0.05235987755982989, [1.0, 1.0]);


active_constraints = {qsc_constraint_0, position_constraint_1, quat_constraint_2, position_constraint_3, quat_constraint_4, posture_constraint_5, posture_constraint_6, posture_constraint_7, relative_position_constraint_8, posture_constraint_9, position_constraint_10, quat_constraint_11, position_constraint_12, quat_constraint_13};
t = [0.0, 0.35, 0.7];
nt = size(t, 2);
q_nom_traj = PPTrajectory(foh(t, repmat(q_nom, 1, nt)));
q_seed_traj = PPTrajectory(spline([t(1), t(end)], [zeros(nq,1), q_start, pre_grasp_end_pose, zeros(nq,1)]));
clear xtraj;
clear info;
clear infeasible_constraint;
additionalTimeSamples = [];
ikoptions = s.ikoptions.setAdditionaltSamples(additionalTimeSamples);


[xtraj, info, infeasible_constraint] = inverseKinTraj(r, t, q_seed_traj, q_nom_traj, active_constraints{:}, ikoptions);


if (info > 10) display(infeasibleConstraintMsg(infeasible_constraint)); end;
qtraj = xtraj(1:nq);
max_degrees_per_second = 30.000000;
plan_time = s.getPlanTimeForJointVelocity(xtraj, max_degrees_per_second);

%--- pointwise ik --------

num_pointwise_time_points = 20;
pointwise_time_points = linspace(t(1), t(end), num_pointwise_time_points);
spline_traj = PPTrajectory(spline(t, [ zeros(size(xtraj, 1),1), xtraj.eval(t), zeros(size(xtraj, 1),1)]));
q_seed_pointwise = spline_traj.eval(pointwise_time_points);
q_seed_pointwise = q_seed_pointwise(1:nq,:);
[xtraj_pw, info] = inverseKinPointwise(r, pointwise_time_points, q_seed_pointwise, q_seed_pointwise, active_constraints{:}, ikoptions);
xtraj_pw = PPTrajectory(foh(pointwise_time_points, xtraj_pw));
info = info(end);

%--- pointwise ik end --------

s.publishTraj(plan_pub, atlas, xtraj_pw, info, plan_time);

%--- runIKTraj end --------

disp(info)

