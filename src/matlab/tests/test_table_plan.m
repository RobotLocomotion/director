%----- Example of an faulty reaching plan from RRT connect ---
% Problem: reach over a table to a goal using only the arm
% base and back are locked
% Result creates an odd plan - at the last sample

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

reach_start = [0.6;0.0;1.025;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.30000001;-0.2;0.0;-0.80000001;0.0;0.0;0.0;0.30000001;-0.2;0.0;-0.80000001;0.0;0.0;0.0;0.0;0.0;-0.49000001;-1.22000003;-0.72000003;0.0;0.0;0.0;-0.49000001;-1.22000003;-0.72000003;0.0];

%-------- runIk --------

excluded_collision_groups = struct('name',{},'tspan',{});

default_shrink_factor = 0.2;
qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(default_shrink_factor);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(links.LeftUpperFoot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.RightUpperFoot, r_foot_pts);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.LeftUpperFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [0.0; 0.0; 0.0];
upper_bounds = xyz_quat(1:3) + [0.0; 0.0; 0.0];
position_constraint_1 = WorldPositionConstraint(r, links.LeftUpperFoot, point_in_link_frame, lower_bounds, upper_bounds, [-inf, inf]);quaternion_constraint_1 = WorldQuatConstraint(r, links.LeftUpperFoot, xyz_quat(4:7), 0.0, [-inf, inf]);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.RightUpperFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [0.0; 0.0; 0.0];
upper_bounds = xyz_quat(1:3) + [0.0; 0.0; 0.0];
position_constraint_2 = WorldPositionConstraint(r, links.RightUpperFoot, point_in_link_frame, lower_bounds, upper_bounds, [-inf, inf]);quaternion_constraint_2 = WorldQuatConstraint(r, links.RightUpperFoot, xyz_quat(4:7), 0.0, [-inf, inf]);


posture_constraint_3 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.WaistLateralExtensor; joints.WaistExtensor; joints.WaistRotator];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_3 = posture_constraint_3.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_4 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_x; joints.base_y; joints.base_z; joints.base_roll; joints.base_pitch; joints.base_yaw; joints.RightHipRotator; joints.RightHipAdductor; joints.RightHipExtensor; joints.RightKneeExtensor; joints.RightAnkleExtensor; joints.RightAnkle; joints.LeftHipRotator; joints.LeftHipAdductor; joints.LeftHipExtensor; joints.LeftKneeExtensor; joints.LeftAnkleExtensor; joints.LeftAnkle];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_4 = posture_constraint_4.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.RightShoulderExtensor; joints.RightShoulderAdductor; joints.RightShoulderSupinator; joints.RightElbowExtensor; joints.RightForearmSupinator; joints.RightWristExtensor; joints.RightWrist];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0.080000000000000071; 1.1102230246251565e-16; -0.039999999999999925];
ref_frame = [-5.5511151231257827e-17, 1.0, 2.7755575615628914e-16, 1.05; -2.7755575615628914e-16, 2.7755575615628914e-16, -1.0, 0.40000000000000002; -1.0, -5.5511151231257827e-17, 2.7755575615628914e-16, 1.0; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_6 = WorldPositionInFrameConstraint(r, links.LeftPalm, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


quat_constraint_7 = WorldQuatConstraint(r, links.LeftPalm, [0.70710678118654746; -0.70710678118654757; 1.6580176363700663e-12; 1.8044680104537952e-12], 0.0, [1.0, 1.0]);


q_nom = [q_nom; zeros(r.getNumPositions()-numel(q_nom),1)];
reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
active_constraints = {qsc_constraint_0, position_constraint_1, quaternion_constraint_1, position_constraint_2, quaternion_constraint_2, posture_constraint_3, posture_constraint_4, posture_constraint_5, position_constraint_6, quat_constraint_7};
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
'  <link name="link_d2dfccdc-2970-11e5-a5c8-54ee75451821">'...
'    <visual>'...
'      <origin xyz="1.23499370906 -0.00356437651325 0.852272611728" rpy="-0.0131938816181 -0.0303441377388 -0.0297479247715"/>'...
'      <geometry>'...
'        <box size="0.52113699913 1.01214981079 0.0221933610737"/>'...
'      </geometry>'...
'      <material name="material_d2dfccdc-2970-11e5-a5c8-54ee75451821">'...
'        <color rgba="0.0 1.0 0.0 1"/>'...
'      </material>'...
'    </visual>'...
'    <collision>'...
'      <origin xyz="1.23499370906 -0.00356437651325 0.852272611728" rpy="-0.0131938816181 -0.0303441377388 -0.0297479247715"/>'...
'      <geometry>'...
'        <box size="0.52113699913 1.01214981079 0.0221933610737"/>'...
'      </geometry>'...
'    </collision>'...
'  </link>'...
'</robot>'];
s = s.setEnvironment(environment_urdf_string);
r = s.robot_and_environment;
reach_end = [0.6;0.0;1.025;0.0;0.0;0.0;0.0;0.0;0.0;-0.00576615882873254;0.5308786080096353;-0.1457981852988174;0.30000001;-0.2;0.0;-0.80000001;0.0;0.0;0.0;2.303918452532793;-2.168457427384427;-1.5707963259;-1.280119960932588;-1.501651550729199;0.3157481470732724;-0.5235987753;0.0;0.0;-0.49000001;-1.22000003;-0.72000003;0.0;0.0;0.0;-0.49000001;-1.22000003;-0.72000003;0.0];

%-------- runIkTraj --------

reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
reach_end = [reach_end; zeros(r.getNumPositions()-numel(reach_end),1)];
q_nom = [q_nom; zeros(r.getNumPositions()-numel(q_nom),1)];
excluded_collision_groups = struct('name',{},'tspan',{});

end_effector_name = 'LeftPalm';
end_effector_name_left = 'LeftPalm';
end_effector_name_right = 'RightPalm';
end_effector_pt = [];
default_shrink_factor = 0.2;
qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(default_shrink_factor);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(links.LeftUpperFoot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.RightUpperFoot, r_foot_pts);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.LeftUpperFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [0.0; 0.0; 0.0];
upper_bounds = xyz_quat(1:3) + [0.0; 0.0; 0.0];
position_constraint_1 = WorldPositionConstraint(r, links.LeftUpperFoot, point_in_link_frame, lower_bounds, upper_bounds, [-inf, inf]);quaternion_constraint_1 = WorldQuatConstraint(r, links.LeftUpperFoot, xyz_quat(4:7), 0.0, [-inf, inf]);


point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.RightUpperFoot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [0.0; 0.0; 0.0];
upper_bounds = xyz_quat(1:3) + [0.0; 0.0; 0.0];
position_constraint_2 = WorldPositionConstraint(r, links.RightUpperFoot, point_in_link_frame, lower_bounds, upper_bounds, [-inf, inf]);quaternion_constraint_2 = WorldQuatConstraint(r, links.RightUpperFoot, xyz_quat(4:7), 0.0, [-inf, inf]);


posture_constraint_3 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.WaistLateralExtensor; joints.WaistExtensor; joints.WaistRotator];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_3 = posture_constraint_3.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_4 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_x; joints.base_y; joints.base_z; joints.base_roll; joints.base_pitch; joints.base_yaw; joints.RightHipRotator; joints.RightHipAdductor; joints.RightHipExtensor; joints.RightKneeExtensor; joints.RightAnkleExtensor; joints.RightAnkle; joints.LeftHipRotator; joints.LeftHipAdductor; joints.LeftHipExtensor; joints.LeftKneeExtensor; joints.LeftAnkleExtensor; joints.LeftAnkle];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_4 = posture_constraint_4.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.RightShoulderExtensor; joints.RightShoulderAdductor; joints.RightShoulderSupinator; joints.RightElbowExtensor; joints.RightForearmSupinator; joints.RightWristExtensor; joints.RightWrist];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0.080000000000000071; 1.1102230246251565e-16; -0.039999999999999925];
ref_frame = [-5.5511151231257827e-17, 1.0, 2.7755575615628914e-16, 1.05; -2.7755575615628914e-16, 2.7755575615628914e-16, -1.0, 0.40000000000000002; -1.0, -5.5511151231257827e-17, 2.7755575615628914e-16, 1.0; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_6 = WorldPositionInFrameConstraint(r, links.LeftPalm, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


quat_constraint_7 = WorldQuatConstraint(r, links.LeftPalm, [0.70710678118654746; -0.70710678118654757; 1.6580176363700663e-12; 1.8044680104537952e-12], 0.0, [1.0, 1.0]);


active_constraints = {qsc_constraint_0, position_constraint_1, quaternion_constraint_1, position_constraint_2, quaternion_constraint_2, posture_constraint_3, posture_constraint_4, posture_constraint_5, position_constraint_6, quat_constraint_7};
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
options.frozen_groups = {};
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