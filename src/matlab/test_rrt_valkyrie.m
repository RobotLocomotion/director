% test rrt script for valkyrie to reach around a table to a goal
% this is generated with the ddapp using and can be used to test end-to-end collision free planning


disp(exist('ikServerStarted'))

%-------- startup --------

format long e
addpath_control
addpath([getenv('DRC_BASE'), '/software/ddapp/src/matlab'])
robotURDF = [getenv('DRC_BASE'), '/software/models/valkyrie/V1_sim_shells_reduced_polygon_count_mit.urdf'];
fixed_point_file = [getenv('DRC_BASE'), '/software/control/matlab/data/valkyrie_fp.mat'];
left_foot_link = 'LeftUpperFoot';
right_foot_link = 'RightUpperFoot';
runIKServer

%------ startup end ------

disp(exist('ikServerStarted'))
dp.options.quat_tol = 0.0;
dp.options.tol = 0.0;
dp.options.seed_with_current = 0;

% ------ driving planner startup ------

addpath([getenv('DRC_BASE'), '/software/control/matlab/planners/driving_planner']);
clear driving_planner_options;
driving_planner_options.listen_to_lcm_flag = 0;
driving_planner_options.qstar = q_nom;
dp = drivingPlanner(s.robot, driving_planner_options);

% ------ driving planner startup end ------

reach_start = [0.6049843430519104;0.011641984805464745;0.9870909452438354;0.00014356752519618894;-0.000862828340404233;-0.0012383394197264452;0.00016771702212281525;-0.002413212787359953;-0.0003066938661504537;-0.019860072061419487;0.0027591893449425697;-0.10177750140428543;0.29846006631851196;-0.2507118582725525;-0.0031037062872201204;-1.0180786848068237;0.007225702982395887;-0.00852723978459835;0.027473105117678642;0.2736058235168457;-0.20332899689674377;-0.007728921249508858;-0.8228764533996582;-0.011880670674145222;-0.014143239706754684;-0.004426778759807348;-0.02175675705075264;-0.0012304894626140594;-0.5803829431533813;-1.394121766090393;-0.8128416538238525;0.0009738004882819951;0.013520746491849422;0.03186595439910889;-0.5863568186759949;-1.398168683052063;-0.8108968734741211;-0.032056864351034164];

%-------- runIk --------

excluded_collision_groups = struct('name',{},'tspan',{});

default_shrink_factor = 0.2;
qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(default_shrink_factor);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(links.LeftUpperFoot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.RightUpperFoot, r_foot_pts);


posture_constraint_1 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.LowerNeckExtensor; joints.NeckRotator; joints.UpperNeckExtensor];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_1 = posture_constraint_1.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.LeftUpperFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_2 = WorldPositionConstraint(r, links.LeftUpperFoot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_2 = WorldQuatConstraint(r, links.LeftUpperFoot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.RightUpperFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_3 = WorldPositionConstraint(r, links.RightUpperFoot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_3 = WorldQuatConstraint(r, links.RightUpperFoot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


posture_constraint_4 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.WaistLateralExtensor; joints.WaistExtensor; joints.WaistRotator];
joints_lower_limit = q_zero(joint_inds) + [-0.08726646259971647; -0.08726646259971647; -inf];
joints_upper_limit = q_zero(joint_inds) + [0.08726646259971647; 0.08726646259971647; inf];
posture_constraint_4 = posture_constraint_4.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_6 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.LeftKneeExtensor; joints.RightKneeExtensor];
joints_lower_limit = q_zero(joint_inds) + [-1.8999999999999999; -1.8999999999999999];
joints_upper_limit = q_zero(joint_inds) + [-0.59999999999999998; -0.59999999999999998];
posture_constraint_6 = posture_constraint_6.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0.080000000000000071; 1.1102230246251565e-16; -0.039999999999999925];
ref_frame = [-0.84791242924277743, 0.52815466696978675, -0.045794760547959487, 0.72230483461479911; 0.13263626715034052, 0.12771289504749045, -0.98290235378445001, 0.33988027987026054; -0.51327588387925926, -0.83948916859994194, -0.17834181459024925, 0.65997760423934249; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_7 = WorldPositionInFrameConstraint(r, links.LeftPalm, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


quat_constraint_8 = WorldQuatConstraint(r, links.LeftPalm, [0.67807314293993803; -0.55162863078920821; 0.32639691542416871; 0.35970504600125613], 0.0, [1.0, 1.0]);


posture_constraint_9 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.RightShoulderExtensor; joints.RightShoulderAdductor; joints.RightShoulderSupinator; joints.RightElbowExtensor; joints.RightForearmSupinator; joints.RightWristExtensor; joints.RightWrist];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_9 = posture_constraint_9.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


end_effector_name = 'LeftPalm';
end_effector_pt = [0.080000000000000071; 1.1102230246251565e-16; -0.039999999999999925];


q_nom = [q_nom; zeros(r.getNumPositions()-numel(q_nom),1)];
reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
active_constraints = {qsc_constraint_0, posture_constraint_1, position_constraint_2, quaternion_constraint_2, position_constraint_3, quaternion_constraint_3, posture_constraint_4, posture_constraint_5, posture_constraint_6, position_constraint_7, quat_constraint_8, posture_constraint_9};
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

default_shrink_factor = 0.2;
qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(default_shrink_factor);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(links.LeftUpperFoot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.RightUpperFoot, r_foot_pts);


posture_constraint_1 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.LowerNeckExtensor; joints.NeckRotator; joints.UpperNeckExtensor];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_1 = posture_constraint_1.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.LeftUpperFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_2 = WorldPositionConstraint(r, links.LeftUpperFoot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_2 = WorldQuatConstraint(r, links.LeftUpperFoot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.RightUpperFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_3 = WorldPositionConstraint(r, links.RightUpperFoot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_3 = WorldQuatConstraint(r, links.RightUpperFoot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


posture_constraint_4 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.WaistLateralExtensor; joints.WaistExtensor; joints.WaistRotator];
joints_lower_limit = q_zero(joint_inds) + [-0.08726646259971647; -0.08726646259971647; -inf];
joints_upper_limit = q_zero(joint_inds) + [0.08726646259971647; 0.08726646259971647; inf];
posture_constraint_4 = posture_constraint_4.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_6 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.LeftKneeExtensor; joints.RightKneeExtensor];
joints_lower_limit = q_zero(joint_inds) + [-1.8999999999999999; -1.8999999999999999];
joints_upper_limit = q_zero(joint_inds) + [-0.59999999999999998; -0.59999999999999998];
posture_constraint_6 = posture_constraint_6.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0.080000000000000071; 1.1102230246251565e-16; -0.039999999999999925];
ref_frame = [-5.5511151231257827e-17, 1.0, 2.7755575615628914e-16, 1.05; -2.7755575615628914e-16, 2.7755575615628914e-16, -1.0, 0.40000000000000002; -1.0, -5.5511151231257827e-17, 2.7755575615628914e-16, 1.0; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_7 = WorldPositionInFrameConstraint(r, links.LeftPalm, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


quat_constraint_8 = WorldQuatConstraint(r, links.LeftPalm, [0.70710678118654746; -0.70710678118654757; 1.6580176363700663e-12; 1.8044680104537952e-12], 0.0, [1.0, 1.0]);


posture_constraint_9 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.RightShoulderExtensor; joints.RightShoulderAdductor; joints.RightShoulderSupinator; joints.RightElbowExtensor; joints.RightForearmSupinator; joints.RightWristExtensor; joints.RightWrist];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_9 = posture_constraint_9.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


end_effector_name = 'LeftPalm';
end_effector_pt = [0.080000000000000071; 1.1102230246251565e-16; -0.039999999999999925];


q_nom = [q_nom; zeros(r.getNumPositions()-numel(q_nom),1)];
reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
active_constraints = {qsc_constraint_0, posture_constraint_1, position_constraint_2, quaternion_constraint_2, position_constraint_3, quaternion_constraint_3, posture_constraint_4, posture_constraint_5, posture_constraint_6, position_constraint_7, quat_constraint_8, posture_constraint_9};
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
reach_start = [0.6049843430519104;0.011641984805464745;0.9870909452438354;0.00014356752519618894;-0.000862828340404233;-0.0012383394197264452;0.00016771702212281525;-0.002413212787359953;-0.0003066938661504537;-0.019860072061419487;0.0027591893449425697;-0.10177750140428543;0.29846006631851196;-0.2507118582725525;-0.0031037062872201204;-1.0180786848068237;0.007225702982395887;-0.00852723978459835;0.027473105117678642;0.2736058235168457;-0.20332899689674377;-0.007728921249508858;-0.8228764533996582;-0.011880670674145222;-0.014143239706754684;-0.004426778759807348;-0.02175675705075264;-0.0012304894626140594;-0.5803829431533813;-1.394121766090393;-0.8128416538238525;0.0009738004882819951;0.013520746491849422;0.03186595439910889;-0.5863568186759949;-1.398168683052063;-0.8108968734741211;-0.032056864351034164];

%-------- runIk --------

excluded_collision_groups = struct('name',{},'tspan',{});

default_shrink_factor = 0.2;
qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(default_shrink_factor);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(links.LeftUpperFoot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.RightUpperFoot, r_foot_pts);


posture_constraint_1 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.LowerNeckExtensor; joints.NeckRotator; joints.UpperNeckExtensor];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_1 = posture_constraint_1.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.LeftUpperFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_2 = WorldPositionConstraint(r, links.LeftUpperFoot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_2 = WorldQuatConstraint(r, links.LeftUpperFoot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.RightUpperFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_3 = WorldPositionConstraint(r, links.RightUpperFoot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_3 = WorldQuatConstraint(r, links.RightUpperFoot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


posture_constraint_4 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.WaistLateralExtensor; joints.WaistExtensor; joints.WaistRotator];
joints_lower_limit = q_zero(joint_inds) + [-0.08726646259971647; -0.08726646259971647; -inf];
joints_upper_limit = q_zero(joint_inds) + [0.08726646259971647; 0.08726646259971647; inf];
posture_constraint_4 = posture_constraint_4.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_x; joints.base_y; joints.base_z; joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0.080000000000000071; 1.1102230246251565e-16; -0.039999999999999925];
ref_frame = [-5.5511151231257827e-17, 1.0, 2.7755575615628914e-16, 1.05; -2.7755575615628914e-16, 2.7755575615628914e-16, -1.0, 0.40000000000000002; -1.0, -5.5511151231257827e-17, 2.7755575615628914e-16, 1.0; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_6 = WorldPositionInFrameConstraint(r, links.LeftPalm, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


quat_constraint_7 = WorldQuatConstraint(r, links.LeftPalm, [0.70710678118654746; -0.70710678118654757; 1.6580176363700663e-12; 1.8044680104537952e-12], 0.0, [1.0, 1.0]);


posture_constraint_8 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.RightShoulderExtensor; joints.RightShoulderAdductor; joints.RightShoulderSupinator; joints.RightElbowExtensor; joints.RightForearmSupinator; joints.RightWristExtensor; joints.RightWrist];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_8 = posture_constraint_8.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


end_effector_name = 'LeftPalm';
end_effector_pt = [0.080000000000000071; 1.1102230246251565e-16; -0.039999999999999925];


q_nom = [q_nom; zeros(r.getNumPositions()-numel(q_nom),1)];
reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
active_constraints = {qsc_constraint_0, posture_constraint_1, position_constraint_2, quaternion_constraint_2, position_constraint_3, quaternion_constraint_3, posture_constraint_4, posture_constraint_5, position_constraint_6, quat_constraint_7, posture_constraint_8};
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
reach_start = [0.6049843430519104;0.011641984805464745;0.9870909452438354;0.00014356752519618894;-0.000862828340404233;-0.0012383394197264452;0.00016771702212281525;-0.002413212787359953;-0.0003066938661504537;-0.019860072061419487;0.0027591893449425697;-0.10177750140428543;0.29846006631851196;-0.2507118582725525;-0.0031037062872201204;-1.0180786848068237;0.007225702982395887;-0.00852723978459835;0.027473105117678642;0.2736058235168457;-0.20332899689674377;-0.007728921249508858;-0.8228764533996582;-0.011880670674145222;-0.014143239706754684;-0.004426778759807348;-0.02175675705075264;-0.0012304894626140594;-0.5803829431533813;-1.394121766090393;-0.8128416538238525;0.0009738004882819951;0.013520746491849422;0.03186595439910889;-0.5863568186759949;-1.398168683052063;-0.8108968734741211;-0.032056864351034164];

%-------- runIk --------

excluded_collision_groups = struct('name',{},'tspan',{});

default_shrink_factor = 0.2;
qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(default_shrink_factor);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(links.LeftUpperFoot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.RightUpperFoot, r_foot_pts);


posture_constraint_1 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.LowerNeckExtensor; joints.NeckRotator; joints.UpperNeckExtensor];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_1 = posture_constraint_1.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.LeftUpperFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_2 = WorldPositionConstraint(r, links.LeftUpperFoot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_2 = WorldQuatConstraint(r, links.LeftUpperFoot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.RightUpperFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_3 = WorldPositionConstraint(r, links.RightUpperFoot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_3 = WorldQuatConstraint(r, links.RightUpperFoot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


posture_constraint_4 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.WaistLateralExtensor; joints.WaistExtensor; joints.WaistRotator];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_4 = posture_constraint_4.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_x; joints.base_y; joints.base_z; joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0.080000000000000071; 1.1102230246251565e-16; -0.039999999999999925];
ref_frame = [-5.5511151231257827e-17, 1.0, 2.7755575615628914e-16, 1.05; -2.7755575615628914e-16, 2.7755575615628914e-16, -1.0, 0.40000000000000002; -1.0, -5.5511151231257827e-17, 2.7755575615628914e-16, 1.0; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_6 = WorldPositionInFrameConstraint(r, links.LeftPalm, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


quat_constraint_7 = WorldQuatConstraint(r, links.LeftPalm, [0.70710678118654746; -0.70710678118654757; 1.6580176363700663e-12; 1.8044680104537952e-12], 0.0, [1.0, 1.0]);


posture_constraint_8 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.RightShoulderExtensor; joints.RightShoulderAdductor; joints.RightShoulderSupinator; joints.RightElbowExtensor; joints.RightForearmSupinator; joints.RightWristExtensor; joints.RightWrist];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_8 = posture_constraint_8.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


end_effector_name = 'LeftPalm';
end_effector_pt = [0.080000000000000071; 1.1102230246251565e-16; -0.039999999999999925];


q_nom = [q_nom; zeros(r.getNumPositions()-numel(q_nom),1)];
reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
active_constraints = {qsc_constraint_0, posture_constraint_1, position_constraint_2, quaternion_constraint_2, position_constraint_3, quaternion_constraint_3, posture_constraint_4, posture_constraint_5, position_constraint_6, quat_constraint_7, posture_constraint_8};
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
'  <link name="link_11438a0a-2f8d-11e5-a81a-f8b156a6acac">'...
'    <visual>'...
'      <origin xyz="0.62 -1.33 0.8" rpy="0.0 0.0 -0.530773497203"/>'...
'      <geometry>'...
'        <box size="0.02 0.02 0.02"/>'...
'      </geometry>'...
'      <material name="material_11438a0a-2f8d-11e5-a81a-f8b156a6acac">'...
'        <color rgba="1 0 0 1"/>'...
'      </material>'...
'    </visual>'...
'    <collision>'...
'      <origin xyz="0.62 -1.33 0.8" rpy="0.0 0.0 -0.530773497203"/>'...
'      <geometry>'...
'        <box size="0.02 0.02 0.02"/>'...
'      </geometry>'...
'    </collision>'...
'  </link>'...
'  <link name="link_1198be94-2f8d-11e5-a81a-f8b156a6acac">'...
'    <visual>'...
'      <origin xyz="1.23144034784 -0.00461424030011 0.852527508572" rpy="-0.0131706633997 -0.0382269827827 -0.0407010075347"/>'...
'      <geometry>'...
'        <box size="0.524889469147 1.01455557346 0.0236955936998"/>'...
'      </geometry>'...
'      <material name="material_1198be94-2f8d-11e5-a81a-f8b156a6acac">'...
'        <color rgba="0.0 1.0 0.0 1"/>'...
'      </material>'...
'    </visual>'...
'    <collision>'...
'      <origin xyz="1.23144034784 -0.00461424030011 0.852527508572" rpy="-0.0131706633997 -0.0382269827827 -0.0407010075347"/>'...
'      <geometry>'...
'        <box size="0.524889469147 1.01455557346 0.0236955936998"/>'...
'      </geometry>'...
'    </collision>'...
'  </link>'...
'</robot>'];
s = s.setEnvironment(environment_urdf_string);
r = s.robot_and_environment;
reach_start = [0.6049843430519104;0.011641984805464745;0.9870909452438354;0.00014356752519618894;-0.000862828340404233;-0.0012383394197264452;0.00016771702212281525;-0.002413212787359953;-0.0003066938661504537;-0.019860072061419487;0.0027591893449425697;-0.10177750140428543;0.29846006631851196;-0.2507118582725525;-0.0031037062872201204;-1.0180786848068237;0.007225702982395887;-0.00852723978459835;0.027473105117678642;0.2736058235168457;-0.20332899689674377;-0.007728921249508858;-0.8228764533996582;-0.011880670674145222;-0.014143239706754684;-0.004426778759807348;-0.02175675705075264;-0.0012304894626140594;-0.5803829431533813;-1.394121766090393;-0.8128416538238525;0.0009738004882819951;0.013520746491849422;0.03186595439910889;-0.5863568186759949;-1.398168683052063;-0.8108968734741211;-0.032056864351034164];

%-------- runIk --------

excluded_collision_groups = struct('name',{},'tspan',{});

default_shrink_factor = 0.5;
qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(default_shrink_factor);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(links.LeftUpperFoot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.RightUpperFoot, r_foot_pts);


posture_constraint_1 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.LowerNeckExtensor; joints.NeckRotator; joints.UpperNeckExtensor];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_1 = posture_constraint_1.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.LeftUpperFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_2 = WorldPositionConstraint(r, links.LeftUpperFoot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_2 = WorldQuatConstraint(r, links.LeftUpperFoot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.RightUpperFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_3 = WorldPositionConstraint(r, links.RightUpperFoot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_3 = WorldQuatConstraint(r, links.RightUpperFoot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


posture_constraint_4 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.WaistLateralExtensor; joints.WaistExtensor; joints.WaistRotator];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_4 = posture_constraint_4.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_x; joints.base_y; joints.base_z; joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0.080000000000000071; 1.1102230246251565e-16; -0.039999999999999925];
ref_frame = [-5.5511151231257827e-17, 1.0, 2.7755575615628914e-16, 1.05; -2.7755575615628914e-16, 2.7755575615628914e-16, -1.0, 0.40000000000000002; -1.0, -5.5511151231257827e-17, 2.7755575615628914e-16, 1.0; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_6 = WorldPositionInFrameConstraint(r, links.LeftPalm, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


quat_constraint_7 = WorldQuatConstraint(r, links.LeftPalm, [0.70710678118654746; -0.70710678118654757; 1.6580176363700663e-12; 1.8044680104537952e-12], 0.0, [1.0, 1.0]);


posture_constraint_8 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.RightShoulderExtensor; joints.RightShoulderAdductor; joints.RightShoulderSupinator; joints.RightElbowExtensor; joints.RightForearmSupinator; joints.RightWristExtensor; joints.RightWrist];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_8 = posture_constraint_8.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


end_effector_name = 'LeftPalm';
end_effector_pt = [0.080000000000000071; 1.1102230246251565e-16; -0.039999999999999925];


q_nom = [q_nom; zeros(r.getNumPositions()-numel(q_nom),1)];
reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
active_constraints = {qsc_constraint_0, posture_constraint_1, position_constraint_2, quaternion_constraint_2, position_constraint_3, quaternion_constraint_3, posture_constraint_4, posture_constraint_5, position_constraint_6, quat_constraint_7, posture_constraint_8};
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
reach_end = [0.6049843430519104;0.01164198480546474;0.9870909452438354;0.0001435675251961889;-0.000862828340404233;-0.001238339419726445;0.0001677170221228153;-0.002413212787359953;-0.0003066938661504537;-0.01986007206141949;0.00275918934494257;-0.1017775014042854;0.298460066318512;-0.2507118582725525;-0.00310370628722012;-1.018078684806824;0.007225702982395887;-0.00852723978459835;0.02747310511767864;-0.158511371309515;-0.3513519686156629;-0.1588335324612793;-1.418748484033932;-0.3250942230682028;-0.1964206879292225;0.06065386601444667;-0.01973353484513398;-0.001206718268875966;-0.5800716034146463;-1.393723894647471;-0.8058825974152747;0.001001420414330363;0.01215022244900293;0.03161507884777454;-0.5861432921056874;-1.39777562143399;-0.8063849922913632;-0.03007422161572503];

%-------- runIkTraj --------

reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
reach_end = [reach_end; zeros(r.getNumPositions()-numel(reach_end),1)];
q_nom = [q_nom; zeros(r.getNumPositions()-numel(q_nom),1)];
excluded_collision_groups = struct('name',{},'tspan',{});

end_effector_name = 'LeftPalm';
end_effector_name_left = 'LeftPalm';
end_effector_name_right = 'RightPalm';
end_effector_pt = [];
default_shrink_factor = 0.5;
qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(default_shrink_factor);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(links.LeftUpperFoot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.RightUpperFoot, r_foot_pts);


posture_constraint_1 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.LowerNeckExtensor; joints.NeckRotator; joints.UpperNeckExtensor];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_1 = posture_constraint_1.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.LeftUpperFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_2 = WorldPositionConstraint(r, links.LeftUpperFoot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_2 = WorldQuatConstraint(r, links.LeftUpperFoot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.RightUpperFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_3 = WorldPositionConstraint(r, links.RightUpperFoot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_3 = WorldQuatConstraint(r, links.RightUpperFoot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


posture_constraint_4 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.WaistLateralExtensor; joints.WaistExtensor; joints.WaistRotator];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_4 = posture_constraint_4.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_x; joints.base_y; joints.base_z; joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0.080000000000000071; 1.1102230246251565e-16; -0.039999999999999925];
ref_frame = [-5.5511151231257827e-17, 1.0, 2.7755575615628914e-16, 1.05; -2.7755575615628914e-16, 2.7755575615628914e-16, -1.0, 0.40000000000000002; -1.0, -5.5511151231257827e-17, 2.7755575615628914e-16, 1.0; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_6 = WorldPositionInFrameConstraint(r, links.LeftPalm, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


quat_constraint_7 = WorldQuatConstraint(r, links.LeftPalm, [0.70710678118654746; -0.70710678118654757; 1.6580176363700663e-12; 1.8044680104537952e-12], 0.0, [1.0, 1.0]);


posture_constraint_8 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.RightShoulderExtensor; joints.RightShoulderAdductor; joints.RightShoulderSupinator; joints.RightElbowExtensor; joints.RightForearmSupinator; joints.RightWristExtensor; joints.RightWrist];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_8 = posture_constraint_8.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


end_effector_name = 'LeftPalm';
end_effector_pt = [0.080000000000000071; 1.1102230246251565e-16; -0.039999999999999925];


active_constraints = {qsc_constraint_0, posture_constraint_1, position_constraint_2, quaternion_constraint_2, position_constraint_3, quaternion_constraint_3, posture_constraint_4, posture_constraint_5, position_constraint_6, quat_constraint_7, posture_constraint_8};
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
options.FixInitialState = true;
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
if ~isempty(qtraj), qtraj_orig = qtraj; end;
if ~isempty(qtraj), joint_v_max = repmat(30.0*pi/180, r.getNumVelocities()-6, 1); end;
if ~isempty(qtraj), xyz_v_max = repmat(0.05, 3, 1); end;
if ~isempty(qtraj), rpy_v_max = repmat(2*pi/180, 3, 1); end;
if ~isempty(qtraj), v_max = [xyz_v_max; rpy_v_max; joint_v_max]; end;
if ~isempty(qtraj), v_max(r.findPositionIndices('back')) = 10.0*pi/180; end;
max_body_translation_speed = 0.5;
max_body_rotation_speed = 10;
rescale_body_ids = [];
rescale_body_pts = reshape([], 3, []);
body_rescale_options = struct('body_id',rescale_body_ids,'pts',rescale_body_pts,'max_v',max_body_translation_speed,'max_theta',max_body_rotation_speed,'robot',r);
if ~isempty(qtraj_orig), qtraj = rescalePlanTiming(qtraj_orig, v_max, 2, 0.3, body_rescale_options); end;
if ~isempty(qtraj_orig), s.publishTraj(qtraj, info); end;

%--- runIKTraj end --------

disp(info)
