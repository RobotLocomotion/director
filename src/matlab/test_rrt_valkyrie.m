% test rrt script for valkyrie (version 2) to reach around a table to a
% goal on the table
% this script was generated with the director directly and can be used to test end-to-end collision free planning


disp(exist('ikServerStarted'))

%-------- startup --------

format long e
addpath_control
addpath([getenv('DRC_BASE'), '/software/director/src/matlab'])
robotURDF = [getenv('DRC_BASE'), '/software/models/val_description/urdf/valkyrie_A_sim_drake.urdf'];
fixed_point_file = [getenv('DRC_BASE'), '/software/control/matlab/data/val_description/valkyrie_fp_june2015.mat'];
left_foot_link = 'leftFoot';
right_foot_link = 'rightFoot';
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

reach_start = [0.0;0.0;1.025;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.30019663134302466;1.3700834628155487;0.0;0.7853981633974483;1.571;0.0;0.0;0.30019663134302466;-1.3700834628155487;0.0;-0.7853981633974483;1.571;0.0;0.0;0.0;0.0;-0.49;1.205;-0.71;0.0;0.0;0.0;-0.49;1.205;-0.71;0.0];

%-------- runIk --------

excluded_collision_groups = struct('name',{},'tspan',{});

default_shrink_factor = 0.2;
qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(default_shrink_factor);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(links.leftFoot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.rightFoot, r_foot_pts);


posture_constraint_1 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.lowerNeckPitch; joints.neckYaw; joints.upperNeckPitch];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_1 = posture_constraint_1.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.leftFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_2 = WorldPositionConstraint(r, links.leftFoot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_2 = WorldQuatConstraint(r, links.leftFoot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.rightFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_3 = WorldPositionConstraint(r, links.rightFoot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_3 = WorldQuatConstraint(r, links.rightFoot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


posture_constraint_4 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.torsoYaw; joints.torsoPitch; joints.torsoRoll];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_4 = posture_constraint_4.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_6 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.leftKneePitch; joints.rightKneePitch];
joints_lower_limit = q_zero(joint_inds) + [0.59999999999999998; 0.59999999999999998];
joints_upper_limit = q_zero(joint_inds) + [1.8999999999999999; 1.8999999999999999];
posture_constraint_6 = posture_constraint_6.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0.040000000000391667; 0.079999999999804117; -1.9584334154387761e-13];
ref_frame = [-0.022890304929222172, 0.99798757633456925, -0.059134012396437979, 0.32420285370477708; 0.19215897337516036, -0.053654578779544465, -0.9798959715844312, 0.3478790675945822; -0.9810968162682282, -0.03379324870213779, -0.19054409844050235, 1.0025125650269087; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_7 = WorldPositionInFrameConstraint(r, links.leftPalm, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


quat_constraint_8 = WorldQuatConstraint(r, links.leftPalm, [0.6834303594924942; 0.057930454239059438; 0.078074671546915983; -0.72351320088748738], 0.0, [1.0, 1.0]);


posture_constraint_9 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.rightShoulderPitch; joints.rightShoulderRoll; joints.rightShoulderYaw; joints.rightElbowPitch; joints.rightForearmYaw; joints.rightWristRoll; joints.rightWristPitch];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_9 = posture_constraint_9.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


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
environment_urdf_string = ['<?xml version="1.0"?>'...
'<robot name="affordance_environment">'...
'  <link name="link_60108c86-4f1e-11e5-9b3b-5cc5d4de4cf2">'...
'    <visual>'...
'      <origin xyz="0.405144102437 0.366667483845 0.754991047842" rpy="0.0 0.0 0.0"/>'...
'      <geometry>'...
'        <box size="0.25 0.25 0.25"/>'...
'      </geometry>'...
'      <material name="material_60108c86-4f1e-11e5-9b3b-5cc5d4de4cf2">'...
'        <color rgba="1.0 1.0 1.0 1"/>'...
'      </material>'...
'    </visual>'...
'    <collision>'...
'      <origin xyz="0.405144102437 0.366667483845 0.754991047842" rpy="0.0 0.0 0.0"/>'...
'      <geometry>'...
'        <box size="0.25 0.25 0.25"/>'...
'      </geometry>'...
'    </collision>'...
'  </link>'...
'</robot>'];
s = s.setEnvironment(environment_urdf_string);
r = s.robot_and_environment;
reach_start = [0.0;0.0;1.025;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.30019663134302466;1.3700834628155487;0.0;0.7853981633974483;1.571;0.0;0.0;0.30019663134302466;-1.3700834628155487;0.0;-0.7853981633974483;1.571;0.0;0.0;0.0;0.0;-0.49;1.205;-0.71;0.0;0.0;0.0;-0.49;1.205;-0.71;0.0];

%-------- runIk --------

excluded_collision_groups = struct('name',{},'tspan',{});

default_shrink_factor = 0.5;
qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(default_shrink_factor);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(links.leftFoot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.rightFoot, r_foot_pts);


posture_constraint_1 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.lowerNeckPitch; joints.neckYaw; joints.upperNeckPitch];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_1 = posture_constraint_1.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.leftFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_2 = WorldPositionConstraint(r, links.leftFoot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_2 = WorldQuatConstraint(r, links.leftFoot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.rightFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_3 = WorldPositionConstraint(r, links.rightFoot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_3 = WorldQuatConstraint(r, links.rightFoot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


posture_constraint_4 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.torsoYaw; joints.torsoPitch; joints.torsoRoll];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_4 = posture_constraint_4.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_6 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.leftKneePitch; joints.rightKneePitch];
joints_lower_limit = q_zero(joint_inds) + [0.59999999999999998; 0.59999999999999998];
joints_upper_limit = q_zero(joint_inds) + [1.8999999999999999; 1.8999999999999999];
posture_constraint_6 = posture_constraint_6.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0.040000000000391667; 0.079999999999804117; -1.9584334154387761e-13];
ref_frame = [-0.022890304929222172, 0.99798757633456925, -0.059134012396437979, 0.32420285370477708; 0.19215897337516036, -0.053654578779544465, -0.9798959715844312, 0.3478790675945822; -0.9810968162682282, -0.03379324870213779, -0.19054409844050235, 1.0025125650269087; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_7 = WorldPositionInFrameConstraint(r, links.leftPalm, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


quat_constraint_8 = WorldQuatConstraint(r, links.leftPalm, [0.6834303594924942; 0.057930454239059438; 0.078074671546915983; -0.72351320088748738], 0.0, [1.0, 1.0]);


posture_constraint_9 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.rightShoulderPitch; joints.rightShoulderRoll; joints.rightShoulderYaw; joints.rightElbowPitch; joints.rightForearmYaw; joints.rightWristRoll; joints.rightWristPitch];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_9 = posture_constraint_9.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


q_nom = [q_nom; zeros(r.getNumPositions()-numel(q_nom),1)];
reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
active_constraints = {qsc_constraint_0, posture_constraint_1, position_constraint_2, quaternion_constraint_2, position_constraint_3, quaternion_constraint_3, posture_constraint_4, posture_constraint_5, posture_constraint_6, position_constraint_7, quat_constraint_8, posture_constraint_9};
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
reach_end = [-0.09597705732094487;-0.006315191820675806;1.058745045027815;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.3001966313430247;1.370083462815549;0.0;0.7853981633974483;1.571;0.0;0.0;-0.08567482304086929;-1.252340499138301;-0.05576118791712967;-1.348604245789871;1.457558417156194;0.1093637326531114;-0.1437676674474725;0.001249214967062414;0.008099133997244807;-0.5661457607552003;1.074353659224561;-0.5084366133710585;-0.007901398295658073;0.0008814474171516169;0.007865123894731969;-0.5649532046137299;1.071710481026259;-0.5070043941067548;-0.007931044712341311];

%-------- runIkTraj --------

reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
reach_end = [reach_end; zeros(r.getNumPositions()-numel(reach_end),1)];
q_nom = [q_nom; zeros(r.getNumPositions()-numel(q_nom),1)];
excluded_collision_groups = struct('name',{},'tspan',{});

end_effector_name = 'leftPalm';
end_effector_name_left = 'leftPalm';
end_effector_name_right = 'rightPalm';
end_effector_pt = [];
default_shrink_factor = 0.5;
qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(default_shrink_factor);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(links.leftFoot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.rightFoot, r_foot_pts);


posture_constraint_1 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.lowerNeckPitch; joints.neckYaw; joints.upperNeckPitch];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_1 = posture_constraint_1.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.leftFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_2 = WorldPositionConstraint(r, links.leftFoot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_2 = WorldQuatConstraint(r, links.leftFoot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.rightFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_3 = WorldPositionConstraint(r, links.rightFoot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_3 = WorldQuatConstraint(r, links.rightFoot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


posture_constraint_4 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.torsoYaw; joints.torsoPitch; joints.torsoRoll];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_4 = posture_constraint_4.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_6 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.leftKneePitch; joints.rightKneePitch];
joints_lower_limit = q_zero(joint_inds) + [0.59999999999999998; 0.59999999999999998];
joints_upper_limit = q_zero(joint_inds) + [1.8999999999999999; 1.8999999999999999];
posture_constraint_6 = posture_constraint_6.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0.040000000000391667; 0.079999999999804117; -1.9584334154387761e-13];
ref_frame = [-0.022890304929222172, 0.99798757633456925, -0.059134012396437979, 0.32420285370477708; 0.19215897337516036, -0.053654578779544465, -0.9798959715844312, 0.3478790675945822; -0.9810968162682282, -0.03379324870213779, -0.19054409844050235, 1.0025125650269087; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_7 = WorldPositionInFrameConstraint(r, links.leftPalm, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


quat_constraint_8 = WorldQuatConstraint(r, links.leftPalm, [0.6834303594924942; 0.057930454239059438; 0.078074671546915983; -0.72351320088748738], 0.0, [1.0, 1.0]);


posture_constraint_9 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.rightShoulderPitch; joints.rightShoulderRoll; joints.rightShoulderYaw; joints.rightElbowPitch; joints.rightForearmYaw; joints.rightWristRoll; joints.rightWristPitch];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_9 = posture_constraint_9.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


active_constraints = {qsc_constraint_0, posture_constraint_1, position_constraint_2, quaternion_constraint_2, position_constraint_3, quaternion_constraint_3, posture_constraint_4, posture_constraint_5, posture_constraint_6, position_constraint_7, quat_constraint_8, posture_constraint_9};
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
options.frozen_groups = {'r_arm','back'};
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
