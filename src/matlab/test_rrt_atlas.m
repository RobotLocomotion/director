% test rrt script for atlas v3 to reach around a box (unseen)
% using the ddapp integration script - ikServer
% (its actually a hard planning problem)


%-------- startup --------

format long e
addpath_control
addpath([getenv('DRC_BASE'), '/software/ddapp/src/matlab'])
robotURDF = '/home/mfallon/drc/software/models/atlas_v3/./model_convex_hull_robotiq_hands.urdf';
fixed_point_file = '/home/mfallon/drc/software/models/atlas_v3/../../control/matlab/data/atlas_bdi_fp.mat';
left_foot_link = 'l_foot';
right_foot_link = 'r_foot';
runIKServer

%------ startup end ------

disp(ikServerStarted)
posture_goal_0 = [0.0;0.0;0.8525;0.0;0.0;0.0;0.0;0.0;0.0;0.27;-1.33;2.1;0.5;0.0;0.0;0.055;-0.49;1.0;-0.51;-0.06;0.0;0.27;1.33;2.1;-0.5;0.0;0.0;-0.055;-0.49;1.0;-0.51;0.06;0.0;0.0];
posture_goal_1 = [0.0;0.0;0.8525;0.0;0.0;0.0;0.0;0.0;0.0;0.27;-0.9456216;1.70588337;1.54094826;1.1246892199999998;0.0;0.055;-0.49;1.0;-0.51;-0.06;0.0;0.27;1.33;2.1;-0.5;0.0;0.0;-0.055;-0.49;1.0;-0.51;0.06;0.0;0.0];

%-------- runIkTraj --------

posture_goal_0 = [posture_goal_0; zeros(r.getNumPositions()-numel(posture_goal_0),1)];
posture_goal_1 = [posture_goal_1; zeros(r.getNumPositions()-numel(posture_goal_1),1)];
posture_goal_0 = [posture_goal_0; zeros(r.getNumPositions()-numel(posture_goal_0),1)];
excluded_collision_groups = struct('name',{},'tspan',{});

end_effector_name = 'l_hand';
end_effector_name_left = 'l_hand';
end_effector_name_right = 'r_hand';
end_effector_pt = [];
posture_constraint_0 = PostureConstraint(r, [1.0, 1.0]);
joint_inds = [joints.base_x; joints.base_y; joints.base_z; joints.base_roll; joints.base_pitch; joints.base_yaw; joints.back_bkz; joints.back_bky; joints.back_bkx; joints.l_arm_usy; joints.l_arm_shx; joints.l_arm_ely; joints.l_arm_elx; joints.l_arm_uwy; joints.l_leg_hpz; joints.l_leg_hpx; joints.l_leg_hpy; joints.l_leg_kny; joints.l_leg_aky; joints.l_leg_akx; joints.l_arm_mwx; joints.r_arm_usy; joints.r_arm_shx; joints.r_arm_ely; joints.r_arm_elx; joints.r_arm_uwy; joints.r_leg_hpz; joints.r_leg_hpx; joints.r_leg_hpy; joints.r_leg_kny; joints.r_leg_aky; joints.r_leg_akx; joints.r_arm_mwx; joints.neck_ay];
joints_lower_limit = posture_goal_1(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = posture_goal_1(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_0 = posture_constraint_0.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


active_constraints = {posture_constraint_0};
t = [0.0, 1.0];
nt = size(t, 2);
clear xtraj;
clear info;
clear infeasible_constraint;
additionalTimeSamples = [];
options = struct();
options.MajorIterationsLimit = 500;
options.MajorFeasibilityTolerance = 1e-06;
options.MajorOptimalityTolerance = 0.0001;
s = s.setupOptions(options);
ikoptions = s.ikoptions.setAdditionaltSamples(additionalTimeSamples);


q_nom_traj = PPTrajectory(foh(t, repmat(posture_goal_0, 1, nt)));
q_seed_traj = PPTrajectory(spline([t(1), t(end)], [zeros(r.getNumPositions(),1), posture_goal_0, posture_goal_1, zeros(r.getNumPositions(),1)]));


[xtraj, info, infeasible_constraint] = inverseKinTraj(r, t, q_seed_traj, q_nom_traj, active_constraints{:}, ikoptions);


if (info > 10) display(infeasibleConstraintMsg(infeasible_constraint)); end;
if ~isempty(xtraj), qtraj = xtraj(1:r.getNumPositions()); else, qtraj = []; end;
if ~isempty(qtraj), joint_v_max = repmat(30.0*pi/180, r.getNumVelocities()-6, 1); end;
if ~isempty(qtraj), xyz_v_max = repmat(0.05, 3, 1); end;
if ~isempty(qtraj), rpy_v_max = repmat(2*pi/180, 3, 1); end;
if ~isempty(qtraj), v_max = [xyz_v_max; rpy_v_max; joint_v_max]; end;
if ~isempty(qtraj), qtraj = rescalePlanTiming(qtraj, v_max, 2, 0.3); end;

%--- pointwise ik --------

if ~isempty(qtraj), num_pointwise_time_points = 20; end;
if ~isempty(qtraj), pointwise_time_points = linspace(qtraj.tspan(1), qtraj.tspan(2), num_pointwise_time_points); end;
if ~isempty(qtraj), q_seed_pointwise = qtraj.eval(pointwise_time_points); end;
if ~isempty(qtraj), q_seed_pointwise = q_seed_pointwise(1:r.getNumPositions(),:); end;
if ~isempty(qtraj), [qtraj_pw, info_pw] = inverseKinPointwise(r, pointwise_time_points, q_seed_pointwise, q_seed_pointwise, active_constraints{:}, ikoptions); else, qtraj_pw = []; end;
if ~isempty(qtraj_pw), qtraj_pw = PPTrajectory(foh(pointwise_time_points, qtraj_pw)); end;
if ~isempty(qtraj_pw), info = info_pw(end); end;
if ~isempty(qtraj_pw), if (any(info_pw > 10)) disp('pointwise info:'); disp(info_pw); end; end;

%--- pointwise ik end --------

if ~isempty(qtraj_pw), s.publishTraj(qtraj_pw, info); end;

%--- runIKTraj end --------

disp(info)
reach_start = [0.0;0.0;0.8525;0.0;0.0;0.0;0.0;0.0;0.0;0.27000001072883606;-0.9456216096878052;1.7058833837509155;1.5409482717514038;1.1246892213821411;0.0;0.054999999701976776;-0.49000000953674316;1.0;-0.5099999904632568;-0.05999999865889549;0.0;0.27000001072883606;1.3300000429153442;2.0999999046325684;-0.5;0.0;0.0;-0.054999999701976776;-0.49000000953674316;1.0;-0.5099999904632568;0.05999999865889549;0.0;0.0];
reach_start = [0.0;0.0;0.8525;0.0;0.0;0.0;0.0;0.0;0.0;0.27000001072883606;-0.9456216096878052;1.7058833837509155;1.5409482717514038;1.1246892213821411;0.0;0.054999999701976776;-0.49000000953674316;1.0;-0.5099999904632568;-0.05999999865889549;0.0;0.27000001072883606;1.3300000429153442;2.0999999046325684;-0.5;0.0;0.0;-0.054999999701976776;-0.49000000953674316;1.0;-0.5099999904632568;0.05999999865889549;0.0;0.0];

%-------- runIk --------

excluded_collision_groups = struct('name',{},'tspan',{});

qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(0.2);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(links.l_foot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.r_foot, r_foot_pts);


posture_constraint_1 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.neck_ay];
joints_lower_limit = reach_start(joint_inds) + [0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0];
posture_constraint_1 = posture_constraint_1.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.l_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_2 = WorldPositionConstraint(r, links.l_foot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_2 = WorldQuatConstraint(r, links.l_foot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.r_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_3 = WorldPositionConstraint(r, links.r_foot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_3 = WorldQuatConstraint(r, links.r_foot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


posture_constraint_4 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.back_bkx; joints.back_bky; joints.back_bkz];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_4 = posture_constraint_4.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_x; joints.base_y; joints.base_z; joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [5.5511151231257827e-17; 0.24449999999999994; 0.011199999999999988];
ref_frame = [-0.25481744682195506, 0.96201089775750814, 0.097995415152613646, 0.50508354258314259; 0.14405734743697041, 0.13797591993200675, -0.97990312080753172, 0.13685424225359161; -0.95619848851823841, -0.23557945180917073, -0.17374340977638972, 0.99726838439722876; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_6 = WorldPositionInFrameConstraint(r, links.l_hand, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


gaze_dir_constraint_7 = WorldGazeDirConstraint(r, links.l_hand, [0.0; 1.0; 0.0], [0.9620108977575081; 0.13797591993200675; -0.23557945180917073], 0.0, [1.0, 1.0]);


posture_constraint_8 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.r_arm_usy; joints.r_arm_shx; joints.r_arm_ely; joints.r_arm_elx; joints.r_arm_uwy; joints.r_arm_mwx];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_8 = posture_constraint_8.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


q_nom = [q_nom; zeros(r.getNumPositions()-numel(q_nom),1)];
reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
active_constraints = {qsc_constraint_0, posture_constraint_1, position_constraint_2, quaternion_constraint_2, position_constraint_3, quaternion_constraint_3, posture_constraint_4, posture_constraint_5, position_constraint_6, gaze_dir_constraint_7, posture_constraint_8};
ik_seed_pose = reach_start;
ik_nominal_pose = q_nom;
ik_seed_pose = [ik_seed_pose; zeros(r.getNumPositions()-numel(ik_seed_pose),1)];
ik_nominal_pose = [ik_nominal_pose; zeros(r.getNumPositions()-numel(ik_nominal_pose),1)];
options = struct();
options.MajorIterationsLimit = 500;
options.MajorFeasibilityTolerance = 1e-06;
options.MajorOptimalityTolerance = 0.0001;
options.MinDistance = 0.030000;
s = s.setupOptions(options);
clear q_end;
clear info;
clear infeasible_constraint;


use_collision = false;
[q_end, info, infeasible_constraint] = s.runIk(ik_seed_pose, ik_nominal_pose, active_constraints, use_collision);


q_end(s.robot.getNumPositions()+1:end) = [];

%-------- runIk end --------

disp(q_end)
disp(info)

%-------- runIk --------

excluded_collision_groups = struct('name',{},'tspan',{});

qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(0.2);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(links.l_foot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.r_foot, r_foot_pts);


posture_constraint_1 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.neck_ay];
joints_lower_limit = reach_start(joint_inds) + [0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0];
posture_constraint_1 = posture_constraint_1.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.l_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_2 = WorldPositionConstraint(r, links.l_foot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_2 = WorldQuatConstraint(r, links.l_foot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.r_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_3 = WorldPositionConstraint(r, links.r_foot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_3 = WorldQuatConstraint(r, links.r_foot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


posture_constraint_4 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.back_bkx; joints.back_bky; joints.back_bkz];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_4 = posture_constraint_4.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_x; joints.base_y; joints.base_z; joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [5.5511151231257827e-17; 0.24449999999999994; 0.011199999999999988];
ref_frame = [-0.25481744682195506, 0.96201089775750814, 0.097995415152613646, 0.52851735854737236; 0.14405734743697041, 0.13797591993200675, -0.97990312080753172, 0.13995509325247424; -0.95619848851823841, -0.23557945180917073, -0.17374340977638972, 0.99149066685223941; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_6 = WorldPositionInFrameConstraint(r, links.l_hand, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


gaze_dir_constraint_7 = WorldGazeDirConstraint(r, links.l_hand, [0.0; 1.0; 0.0], [0.9620108977575081; 0.13797591993200675; -0.23557945180917073], 0.0, [1.0, 1.0]);


posture_constraint_8 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.r_arm_usy; joints.r_arm_shx; joints.r_arm_ely; joints.r_arm_elx; joints.r_arm_uwy; joints.r_arm_mwx];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_8 = posture_constraint_8.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


q_nom = [q_nom; zeros(r.getNumPositions()-numel(q_nom),1)];
reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
active_constraints = {qsc_constraint_0, posture_constraint_1, position_constraint_2, quaternion_constraint_2, position_constraint_3, quaternion_constraint_3, posture_constraint_4, posture_constraint_5, position_constraint_6, gaze_dir_constraint_7, posture_constraint_8};
ik_seed_pose = reach_start;
ik_nominal_pose = q_nom;
ik_seed_pose = [ik_seed_pose; zeros(r.getNumPositions()-numel(ik_seed_pose),1)];
ik_nominal_pose = [ik_nominal_pose; zeros(r.getNumPositions()-numel(ik_nominal_pose),1)];
options = struct();
options.MajorIterationsLimit = 500;
options.MajorFeasibilityTolerance = 1e-06;
options.MajorOptimalityTolerance = 0.0001;
options.MinDistance = 0.030000;
s = s.setupOptions(options);
clear q_end;
clear info;
clear infeasible_constraint;


use_collision = false;
[q_end, info, infeasible_constraint] = s.runIk(ik_seed_pose, ik_nominal_pose, active_constraints, use_collision);


q_end(s.robot.getNumPositions()+1:end) = [];

%-------- runIk end --------

disp(q_end)
disp(info)

%-------- runIk --------

excluded_collision_groups = struct('name',{},'tspan',{});

qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(0.2);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(links.l_foot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.r_foot, r_foot_pts);


posture_constraint_1 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.neck_ay];
joints_lower_limit = reach_start(joint_inds) + [0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0];
posture_constraint_1 = posture_constraint_1.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.l_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_2 = WorldPositionConstraint(r, links.l_foot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_2 = WorldQuatConstraint(r, links.l_foot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.r_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_3 = WorldPositionConstraint(r, links.r_foot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_3 = WorldQuatConstraint(r, links.r_foot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


posture_constraint_4 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.back_bkx; joints.back_bky; joints.back_bkz];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_4 = posture_constraint_4.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_x; joints.base_y; joints.base_z; joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [5.5511151231257827e-17; 0.24449999999999994; 0.011199999999999988];
ref_frame = [-0.25481744682195506, 0.96201089775750814, 0.097995415152613646, 0.54222517624163746; 0.14405734743697041, 0.13797591993200675, -0.97990312080753172, 0.16107334686947244; -0.95619848851823841, -0.23557945180917073, -0.17374340977638972, 0.99101926733299761; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_6 = WorldPositionInFrameConstraint(r, links.l_hand, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


gaze_dir_constraint_7 = WorldGazeDirConstraint(r, links.l_hand, [0.0; 1.0; 0.0], [0.9620108977575081; 0.13797591993200675; -0.23557945180917073], 0.0, [1.0, 1.0]);


posture_constraint_8 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.r_arm_usy; joints.r_arm_shx; joints.r_arm_ely; joints.r_arm_elx; joints.r_arm_uwy; joints.r_arm_mwx];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_8 = posture_constraint_8.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


q_nom = [q_nom; zeros(r.getNumPositions()-numel(q_nom),1)];
reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
active_constraints = {qsc_constraint_0, posture_constraint_1, position_constraint_2, quaternion_constraint_2, position_constraint_3, quaternion_constraint_3, posture_constraint_4, posture_constraint_5, position_constraint_6, gaze_dir_constraint_7, posture_constraint_8};
ik_seed_pose = reach_start;
ik_nominal_pose = q_nom;
ik_seed_pose = [ik_seed_pose; zeros(r.getNumPositions()-numel(ik_seed_pose),1)];
ik_nominal_pose = [ik_nominal_pose; zeros(r.getNumPositions()-numel(ik_nominal_pose),1)];
options = struct();
options.MajorIterationsLimit = 500;
options.MajorFeasibilityTolerance = 1e-06;
options.MajorOptimalityTolerance = 0.0001;
options.MinDistance = 0.030000;
s = s.setupOptions(options);
clear q_end;
clear info;
clear infeasible_constraint;


use_collision = false;
[q_end, info, infeasible_constraint] = s.runIk(ik_seed_pose, ik_nominal_pose, active_constraints, use_collision);


q_end(s.robot.getNumPositions()+1:end) = [];

%-------- runIk end --------

disp(q_end)
disp(info)
environment_urdf_string = ['<?xml version="1.0"?>'...
'<robot name="affordance_environment">'...
'  <link name="link_245409c2-f701-11e4-a705-6067205ecde8">'...
'    <visual>'...
'      <origin xyz="0.638627405976 0.34540543214 0.977333747376" rpy="0.0 0.0 0.0"/>'...
'      <geometry>'...
'        <box size="0.25 0.04 0.25"/>'...
'      </geometry>'...
'      <material name="material_245409c2-f701-11e4-a705-6067205ecde8">'...
'        <color rgba="1.0 1.0 1.0 1"/>'...
'      </material>'...
'    </visual>'...
'    <collision>'...
'      <origin xyz="0.638627405976 0.34540543214 0.977333747376" rpy="0.0 0.0 0.0"/>'...
'      <geometry>'...
'        <box size="0.25 0.04 0.25"/>'...
'      </geometry>'...
'    </collision>'...
'  </link>'...
'</robot>'];
s = s.setEnvironment(environment_urdf_string);
r = s.robot_and_environment;
reach_start = [0.0;0.0;0.8525;0.0;0.0;0.0;0.0;0.0;0.0;0.27000001072883606;-0.9456216096878052;1.7058833837509155;1.5409482717514038;1.1246892213821411;0.0;0.054999999701976776;-0.49000000953674316;1.0;-0.5099999904632568;-0.05999999865889549;0.0;0.27000001072883606;1.3300000429153442;2.0999999046325684;-0.5;0.0;0.0;-0.054999999701976776;-0.49000000953674316;1.0;-0.5099999904632568;0.05999999865889549;0.0;0.0];

%-------- runIk --------

excluded_collision_groups = struct('name',{},'tspan',{});

qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(0.5);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(links.l_foot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.r_foot, r_foot_pts);


posture_constraint_1 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.neck_ay];
joints_lower_limit = reach_start(joint_inds) + [0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0];
posture_constraint_1 = posture_constraint_1.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.l_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_2 = WorldPositionConstraint(r, links.l_foot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_2 = WorldQuatConstraint(r, links.l_foot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.r_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_3 = WorldPositionConstraint(r, links.r_foot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_3 = WorldQuatConstraint(r, links.r_foot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


posture_constraint_4 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.back_bkx; joints.back_bky; joints.back_bkz];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_4 = posture_constraint_4.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_x; joints.base_y; joints.base_z; joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [5.5511151231257827e-17; 0.24449999999999994; 0.011199999999999988];
ref_frame = [-0.25481744682195506, 0.96201089775750814, 0.097995415152613646, 0.54222517624163746; 0.14405734743697041, 0.13797591993200675, -0.97990312080753172, 0.16107334686947244; -0.95619848851823841, -0.23557945180917073, -0.17374340977638972, 0.99101926733299761; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_6 = WorldPositionInFrameConstraint(r, links.l_hand, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


gaze_dir_constraint_7 = WorldGazeDirConstraint(r, links.l_hand, [0.0; 1.0; 0.0], [0.9620108977575081; 0.13797591993200675; -0.23557945180917073], 0.0, [1.0, 1.0]);


posture_constraint_8 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.r_arm_usy; joints.r_arm_shx; joints.r_arm_ely; joints.r_arm_elx; joints.r_arm_uwy; joints.r_arm_mwx];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_8 = posture_constraint_8.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


q_nom = [q_nom; zeros(r.getNumPositions()-numel(q_nom),1)];
reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
active_constraints = {qsc_constraint_0, posture_constraint_1, position_constraint_2, quaternion_constraint_2, position_constraint_3, quaternion_constraint_3, posture_constraint_4, posture_constraint_5, position_constraint_6, gaze_dir_constraint_7, posture_constraint_8};
ik_seed_pose = reach_start;
ik_nominal_pose = q_nom;
ik_seed_pose = [ik_seed_pose; zeros(r.getNumPositions()-numel(ik_seed_pose),1)];
ik_nominal_pose = [ik_nominal_pose; zeros(r.getNumPositions()-numel(ik_nominal_pose),1)];
options = struct();
options.MajorIterationsLimit = 500;
options.MajorFeasibilityTolerance = 5e-05;
options.MajorOptimalityTolerance = 0.001;
options.MinDistance = 0.030000;
s = s.setupOptions(options);
clear q_end;
clear info;
clear infeasible_constraint;


use_collision = true;
[q_end, info, infeasible_constraint] = s.runIk(ik_seed_pose, ik_nominal_pose, active_constraints, use_collision);


q_end(s.robot.getNumPositions()+1:end) = [];

%-------- runIk end --------

disp(q_end)
disp(info)
reach_end = [0.0;0.0;0.8525;0.0;0.0;0.0;0.0;0.0;0.0;-0.2691483683273967;-1.213799792183031;2.308442772552465;1.603449763358924;0.9949600210743484;-0.0005907366767957936;0.05485481589206059;-0.4899187706442589;1.000180184515811;-0.5097772818789881;-0.06016903746096289;-1.062116651206765;0.2700000107288361;1.330000042915344;2.099999904632568;-0.5;0.0;-0.0001767492765520921;-0.05498520185235379;-0.4900734337108894;0.9998520321577852;-0.5100646445387296;0.06024621312881388;0.0;0.0];

%-------- runIkTraj --------

reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
reach_end = [reach_end; zeros(r.getNumPositions()-numel(reach_end),1)];
q_nom = [q_nom; zeros(r.getNumPositions()-numel(q_nom),1)];
excluded_collision_groups = struct('name',{},'tspan',{});

end_effector_name = 'l_hand';
end_effector_name_left = 'l_hand';
end_effector_name_right = 'r_hand';
end_effector_pt = [];
qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(0.5);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(links.l_foot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.r_foot, r_foot_pts);


posture_constraint_1 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.neck_ay];
joints_lower_limit = reach_start(joint_inds) + [0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0];
posture_constraint_1 = posture_constraint_1.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.l_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_2 = WorldPositionConstraint(r, links.l_foot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_2 = WorldQuatConstraint(r, links.l_foot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.r_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_3 = WorldPositionConstraint(r, links.r_foot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_3 = WorldQuatConstraint(r, links.r_foot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


posture_constraint_4 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.back_bkx; joints.back_bky; joints.back_bkz];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_4 = posture_constraint_4.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_x; joints.base_y; joints.base_z; joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [5.5511151231257827e-17; 0.24449999999999994; 0.011199999999999988];
ref_frame = [-0.25481744682195506, 0.96201089775750814, 0.097995415152613646, 0.54222517624163746; 0.14405734743697041, 0.13797591993200675, -0.97990312080753172, 0.16107334686947244; -0.95619848851823841, -0.23557945180917073, -0.17374340977638972, 0.99101926733299761; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_6 = WorldPositionInFrameConstraint(r, links.l_hand, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


gaze_dir_constraint_7 = WorldGazeDirConstraint(r, links.l_hand, [0.0; 1.0; 0.0], [0.9620108977575081; 0.13797591993200675; -0.23557945180917073], 0.0, [1.0, 1.0]);


posture_constraint_8 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.r_arm_usy; joints.r_arm_shx; joints.r_arm_ely; joints.r_arm_elx; joints.r_arm_uwy; joints.r_arm_mwx];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_8 = posture_constraint_8.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


active_constraints = {qsc_constraint_0, posture_constraint_1, position_constraint_2, quaternion_constraint_2, position_constraint_3, quaternion_constraint_3, posture_constraint_4, posture_constraint_5, position_constraint_6, gaze_dir_constraint_7, posture_constraint_8};
t = [0.0, 0.3333333333333333, 0.6666666666666666, 1.0];
nt = size(t, 2);
clear xtraj;
clear info;
clear infeasible_constraint;
additionalTimeSamples = [];
options = struct();
options.MajorIterationsLimit = 500;
options.MajorFeasibilityTolerance = 5e-05;
options.MajorOptimalityTolerance = 0.001;
s = s.setupOptions(options);
ikoptions = s.ikoptions.setAdditionaltSamples(additionalTimeSamples);


q_seed_traj = PPTrajectory(foh([t(1), t(end)], [reach_start, reach_end]));
q_nom_traj = ConstantTrajectory(q_nom);
options.n_interp_points = 2;
options.min_distance = 0.03;
options.t_max = 30.0;
options.excluded_collision_groups = excluded_collision_groups;
options.end_effector_name = end_effector_name;
options.end_effector_name_left = end_effector_name_left;
options.end_effector_name_right = end_effector_name_right;
options.end_effector_pt = end_effector_pt;
options.left_foot_link = left_foot_link;
options.right_foot_link = right_foot_link;
options.frozen_groups = {'r_arm','pelvis','back'};
options.RRTMaxEdgeLength = 0.05;
options.RRTGoalBias = 1.0;
options.N = 5000;
options.n_smoothing_passes = 10;
[xtraj,info] = collisionFreePlanner(r,t,q_seed_traj,q_nom_traj,options,active_constraints{:},s.ikoptions);
if (info > 10), fprintf('The solver returned with info %d:\n',info); snoptInfo(info); end
if ~isempty(xtraj), qtraj = xtraj(1:r.getNumPositions()); else, qtraj = []; end;
if ~isempty(qtraj), joint_v_max = repmat(30.0*pi/180, r.getNumVelocities()-6, 1); end;
if ~isempty(qtraj), xyz_v_max = repmat(0.05, 3, 1); end;
if ~isempty(qtraj), rpy_v_max = repmat(2*pi/180, 3, 1); end;
if ~isempty(qtraj), v_max = [xyz_v_max; rpy_v_max; joint_v_max]; end;
if ~isempty(qtraj), qtraj = rescalePlanTiming(qtraj, v_max, 2, 0.3); end;
if ~isempty(qtraj), s.publishTraj(qtraj, info); end;

%--- runIKTraj end --------

disp(info)