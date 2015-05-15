% test rrt script for atlas v3 to reach around a box (unseen)
% using the ddapp integration script - ikServer


%-------- startup --------

format long e
addpath_control
addpath([getenv('DRC_BASE'), '/software/ddapp/src/matlab'])
robotURDF = '/home/mfallon/drc/software/models/valkyrie/./V1_sim_shells_reduced_polygon_count_mit.urdf';
fixed_point_file = '/home/mfallon/drc/software/models/valkyrie/../../control/matlab/data/valkyrie_fp.mat';
left_foot_link = 'LeftUpperFoot';
right_foot_link = 'RightUpperFoot';
runIKServer

%------ startup end ------

disp(ikServerStarted)
posture_goal_0 = [0.0;0.0;1.025;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.30000001;-0.2;0.0;-0.80000001;0.0;0.0;0.0;0.30000001;-0.2;0.0;-0.80000001;0.0;0.0;0.0;0.0;0.0;-0.49000001;-1.22000003;-0.72000003;0.0;0.0;0.0;-0.49000001;-1.22000003;-0.72000003;0.0];
posture_goal_1 = [0.0;0.0;1.025;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.30000001;-0.2;0.0;-0.80000001;0.0;0.0;0.0;-0.2513274121439997;-0.2;0.0;-1.2931144420659;-0.1460840583087002;0.0;0.0;0.0;0.0;-0.49000001;-1.22000003;-0.72000003;0.0;0.0;0.0;-0.49000001;-1.22000003;-0.72000003;0.0];

%-------- runIkTraj --------

posture_goal_0 = [posture_goal_0; zeros(r.getNumPositions()-numel(posture_goal_0),1)];
posture_goal_1 = [posture_goal_1; zeros(r.getNumPositions()-numel(posture_goal_1),1)];
posture_goal_0 = [posture_goal_0; zeros(r.getNumPositions()-numel(posture_goal_0),1)];
excluded_collision_groups = struct('name',{},'tspan',{});

end_effector_name = 'LeftPalm';
end_effector_name_left = 'LeftPalm';
end_effector_name_right = 'RightPalm';
end_effector_pt = [];
posture_constraint_0 = PostureConstraint(r, [1.0, 1.0]);
joint_inds = [joints.base_x; joints.base_y; joints.base_z; joints.base_roll; joints.base_pitch; joints.base_yaw; joints.WaistRotator; joints.WaistExtensor; joints.WaistLateralExtensor; joints.LowerNeckExtensor; joints.NeckRotator; joints.UpperNeckExtensor; joints.RightShoulderExtensor; joints.RightShoulderAdductor; joints.RightShoulderSupinator; joints.RightElbowExtensor; joints.RightForearmSupinator; joints.RightWristExtensor; joints.RightWrist; joints.LeftShoulderExtensor; joints.LeftShoulderAdductor; joints.LeftShoulderSupinator; joints.LeftElbowExtensor; joints.LeftForearmSupinator; joints.LeftWristExtensor; joints.LeftWrist; joints.LeftHipRotator; joints.LeftHipAdductor; joints.LeftHipExtensor; joints.LeftKneeExtensor; joints.LeftAnkleExtensor; joints.LeftAnkle; joints.RightHipRotator; joints.RightHipAdductor; joints.RightHipExtensor; joints.RightKneeExtensor; joints.RightAnkleExtensor; joints.RightAnkle];
joints_lower_limit = posture_goal_1(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = posture_goal_1(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
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
% mfallon0: initial posture plan: - raise up the arm
%if ~isempty(qtraj_pw), s.publishTraj(qtraj_pw, info); end;

%--- runIKTraj end --------

disp(info)
reach_start = [0.0;0.0;1.025;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.30000001192092896;-0.20000000298023224;0.0;-0.800000011920929;0.0;0.0;0.0;-0.25132742524147034;-0.20000000298023224;0.0;-1.293114423751831;-0.14608405530452728;0.0;0.0;0.0;0.0;-0.49000000953674316;-1.2200000286102295;-0.7200000286102295;0.0;0.0;0.0;-0.49000000953674316;-1.2200000286102295;-0.7200000286102295;0.0];

%-------- runIk --------

excluded_collision_groups = struct('name',{},'tspan',{});

qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(0.2);
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


posture_constraint_6 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.LeftShoulderExtensor; joints.LeftShoulderAdductor; joints.LeftShoulderSupinator; joints.LeftElbowExtensor; joints.LeftForearmSupinator; joints.LeftWristExtensor; joints.LeftWrist];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_6 = posture_constraint_6.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


posture_constraint_7 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.RightShoulderExtensor; joints.RightShoulderAdductor; joints.RightShoulderSupinator; joints.RightElbowExtensor; joints.RightForearmSupinator; joints.RightWristExtensor; joints.RightWrist];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_7 = posture_constraint_7.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


q_nom = [q_nom; zeros(r.getNumPositions()-numel(q_nom),1)];
reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
active_constraints = {qsc_constraint_0, posture_constraint_1, position_constraint_2, quaternion_constraint_2, position_constraint_3, quaternion_constraint_3, posture_constraint_4, posture_constraint_5, posture_constraint_6, posture_constraint_7};
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

% ik of shooter position
options.visualize = true;
v = r.constructVisualizer();
%v.draw(0,q_end); % visualize robot with hand in pile
%keyboard


q_end(s.robot.getNumPositions()+1:end) = [];

%-------- runIk end --------

disp(q_end)
disp(info)
reach_start = [0.0;0.0;1.025;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.30000001192092896;-0.20000000298023224;0.0;-0.800000011920929;0.0;0.0;0.0;-0.25132742524147034;-0.20000000298023224;0.0;-1.293114423751831;-0.14608405530452728;0.0;0.0;0.0;0.0;-0.49000000953674316;-1.2200000286102295;-0.7200000286102295;0.0;0.0;0.0;-0.49000000953674316;-1.2200000286102295;-0.7200000286102295;0.0];

%-------- runIk --------

excluded_collision_groups = struct('name',{},'tspan',{});

qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(0.2);
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
ref_frame = [-0.023595378178727758, 0.99829382683475598, 0.053410611606875956, 0.48193442325433178; 0.046360558013270319, 0.054460645987365594, -0.9974390892176509, 0.11894480915774672; -0.99864606182035309, -0.021058806762411543, -0.047566478411191986, 0.99406136507740461; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_6 = WorldPositionInFrameConstraint(r, links.LeftPalm, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


quat_constraint_7 = WorldQuatConstraint(r, links.LeftPalm, [0.72322556357937695; -0.68999398374377696; -0.011183165553384122; 0.026981908585027501], 0.0, [1.0, 1.0]);


posture_constraint_8 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.RightShoulderExtensor; joints.RightShoulderAdductor; joints.RightShoulderSupinator; joints.RightElbowExtensor; joints.RightForearmSupinator; joints.RightWristExtensor; joints.RightWrist];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_8 = posture_constraint_8.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


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

% mfallon2: ik at goal position
%v.draw(0,q_end); 
%keyboard

q_end(s.robot.getNumPositions()+1:end) = [];

%-------- runIk end --------

disp(q_end)
disp(info)

%-------- runIk --------

excluded_collision_groups = struct('name',{},'tspan',{});

qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(0.2);
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
ref_frame = [-0.023595378178727758, 0.99829382683475598, 0.053410611606875956, 0.48431344711855917; 0.046360558013270319, 0.054460645987365594, -0.9974390892176509, 0.087371098082090204; -0.99864606182035309, -0.021058806762411543, -0.047566478411191986, 0.99253939559302085; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_6 = WorldPositionInFrameConstraint(r, links.LeftPalm, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


quat_constraint_7 = WorldQuatConstraint(r, links.LeftPalm, [0.72322556357937695; -0.68999398374377696; -0.011183165553384122; 0.026981908585027501], 0.0, [1.0, 1.0]);


posture_constraint_8 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.RightShoulderExtensor; joints.RightShoulderAdductor; joints.RightShoulderSupinator; joints.RightElbowExtensor; joints.RightForearmSupinator; joints.RightWristExtensor; joints.RightWrist];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_8 = posture_constraint_8.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


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

% mfallon3: ik at goal position (different to 2
%v.draw(0,q_end); 
%keyboard


q_end(s.robot.getNumPositions()+1:end) = [];

%-------- runIk end --------

disp(q_end)
disp(info)
environment_urdf_string = ['<?xml version="1.0"?>'...
'<robot name="affordance_environment">'...
'  <link name="link_7f039124-f6fe-11e4-86d1-6067205ecde8">'...
'    <visual>'...
'      <origin xyz="0.589115043716 0.229667668448 0.97741464478" rpy="0.0 0.0 0.0"/>'...
'      <geometry>'...
'        <box size="0.25 0.05 0.25"/>'...
'      </geometry>'...
'      <material name="material_7f039124-f6fe-11e4-86d1-6067205ecde8">'...
'        <color rgba="1.0 1.0 1.0 1"/>'...
'      </material>'...
'    </visual>'...
'    <collision>'...
'      <origin xyz="0.589115043716 0.229667668448 0.97741464478" rpy="0.0 0.0 0.0"/>'...
'      <geometry>'...
'        <box size="0.25 0.05 0.25"/>'...
'      </geometry>'...
'    </collision>'...
'  </link>'...
'</robot>'];
s = s.setEnvironment(environment_urdf_string);
r = s.robot_and_environment;
reach_start = [0.0;0.0;1.025;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.30000001192092896;-0.20000000298023224;0.0;-0.800000011920929;0.0;0.0;0.0;-0.25132742524147034;-0.20000000298023224;0.0;-1.293114423751831;-0.14608405530452728;0.0;0.0;0.0;0.0;-0.49000000953674316;-1.2200000286102295;-0.7200000286102295;0.0;0.0;0.0;-0.49000000953674316;-1.2200000286102295;-0.7200000286102295;0.0];

%-------- runIk --------

excluded_collision_groups = struct('name',{},'tspan',{});

qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(0.5);
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
%drawFrame(xyz_quat,'reach_start - r foot constraint', 0.1, [1,1,0]);


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
% was:
ref_frame = [-0.023595378178727758, 0.99829382683475598, 0.053410611606875956, 0.48431344711855917; 0.046360558013270319, 0.054460645987365594, -0.9974390892176509, 0.087371098082090204; -0.99864606182035309, -0.021058806762411543, -0.047566478411191986, 0.99253939559302085; 0.0, 0.0, 0.0, 1.0];
%ref_frame = [-0.023595378178727758, 0.99829382683475598, 0.053410611606875956, 0.48431344711855917; 0.046360558013270319, 0.054460645987365594, -0.9974390892176509, 0.257371098082090204; -0.99864606182035309, -0.021058806762411543, -0.047566478411191986, 0.99253939559302085; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_6 = WorldPositionInFrameConstraint(r, links.LeftPalm, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


quat_constraint_7 = WorldQuatConstraint(r, links.LeftPalm, [0.72322556357937695; -0.68999398374377696; -0.011183165553384122; 0.026981908585027501], 0.0, [1.0, 1.0]);


posture_constraint_8 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.RightShoulderExtensor; joints.RightShoulderAdductor; joints.RightShoulderSupinator; joints.RightElbowExtensor; joints.RightForearmSupinator; joints.RightWristExtensor; joints.RightWrist];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_8 = posture_constraint_8.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


q_nom = [q_nom; zeros(r.getNumPositions()-numel(q_nom),1)];
reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
active_constraints = {qsc_constraint_0, posture_constraint_1, position_constraint_2, quaternion_constraint_2, position_constraint_3, quaternion_constraint_3, posture_constraint_4, posture_constraint_5, posture_constraint_8, position_constraint_6 quat_constraint_7, };
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

% mfallon4: ik at goal position (different to 2 and 3). with collision?
v.draw(0,q_end); 


kinsol = r.doKinematics(q_end);
fk_opts.rotation_type = 2;
r_foot_xyz_quat = r.forwardKin(kinsol,links.RightUpperFoot,[0,0,0]',fk_opts); 
%drawFrame(r_foot_xyz_quat,'reach_start - l foot fk', 0.1, [1,1,0]);

q_end(s.robot.getNumPositions()+1:end) = [];

%%%%

%-------- runIk end --------

disp(q_end)
disp(info)
reach_end = [0.0;0.0;1.025;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.300000011920929;-0.2000000029802322;0.0;-0.800000011920929;0.0;0.0;0.0;-0.4973341410271395;-0.2126777012740941;0.8760247351101337;-1.211771266134896;-0.5527537169565565;0.6804137727851287;-0.08369767470040165;0.001746523048541414;-0.1955677616429999;0.188654902503105;0.2181661563574649;0.002574368586076398;0.07440153350346551;-0.002063491159391878;-0.1942915526521191;0.1875986500345759;0.2181661449304783;0.00913893441736692;0.06978728780773374];

reach_end =  [0.        ,  0.        ,  1.025     ,  0.        ,  0.        ,        0.        ,  0.        ,  0.        ,  0.        ,  0.        ,        0.        ,  0.        ,  0.30000001, -0.2       ,  0.        ,       -0.80000001,  0.        ,  0.        ,  0.        ,  0.30000001,       -0.2       ,  0.        , -0.80000001,  0.        ,  0.        ,        0.        ,  0.        ,  0.        , -0.49000001, -1.22000003,       -0.72000003,  0.        ,  0.        ,  0.        , -0.49000001,       -1.22000003, -0.72000003,  0.    ]';

%-------- runIkTraj --------

reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
reach_end = [reach_end; zeros(r.getNumPositions()-numel(reach_end),1)];
q_nom = [q_nom; zeros(r.getNumPositions()-numel(q_nom),1)];
excluded_collision_groups = struct('name',{},'tspan',{});

end_effector_name = 'LeftPalm';
end_effector_name_left = 'LeftPalm';
end_effector_name_right = 'RightPalm';
end_effector_pt = [];
qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(0.5);
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
ref_frame = [-0.023595378178727758, 0.99829382683475598, 0.053410611606875956, 0.48431344711855917; 0.046360558013270319, 0.054460645987365594, -0.9974390892176509, 0.087371098082090204; -0.99864606182035309, -0.021058806762411543, -0.047566478411191986, 0.99253939559302085; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_6 = WorldPositionInFrameConstraint(r, links.LeftPalm, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


quat_constraint_7 = WorldQuatConstraint(r, links.LeftPalm, [0.72322556357937695; -0.68999398374377696; -0.011183165553384122; 0.026981908585027501], 0.0, [1.0, 1.0]);


posture_constraint_8 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.RightShoulderExtensor; joints.RightShoulderAdductor; joints.RightShoulderSupinator; joints.RightElbowExtensor; joints.RightForearmSupinator; joints.RightWristExtensor; joints.RightWrist];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_8 = posture_constraint_8.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


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
