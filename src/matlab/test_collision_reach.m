function test_collision_reach(scenario,iktraj_collision_constraint_type,visualize)
  if nargin < 1 || isempty(scenario)
    scenario = 1;
  end
  if nargin < 2 || isempty(iktraj_collision_constraint_type)
    iktraj_collision_constraint_type = 'integrated_mex';
  end
  if nargin < 3
    visualize = false;
  end
% setup
do_setup = true;

if (do_setup)

  %addpath_control
  addpath([getenv('DRC_BASE'), '/software/ddapp/pod-build/lib/python2.7/dist-packages/ddapp/../../../../../src/matlab'])

  checkDependency('lcmgl');
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),  'test_collision_reach');

  s = IKServer();

  warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');

  drakeExamplePath = [getenv('DRC_BASE'), '/software/drake/examples/'];

  robotURDF = [drakeExamplePath, 'Atlas/urdf/atlas_chull.urdf'];

  s = s.addRobot(robotURDF);
  s = s.setupCosts();
  s = s.loadNominalData();
end;

r = s.robot;
nq = r.getNumPositions();
q_nom = s.q_nom;
q_zero = zeros(nq, 1);
q_start = q_nom;
q_end = q_nom;

world = r.findLinkId('world');
l_foot = r.findLinkId('l_foot');
r_foot = r.findLinkId('r_foot');
l_hand = r.findLinkId('l_hand');
l_farm = r.findLinkId('l_farm');
l_larm = r.findLinkId('l_larm');
r_hand = r.findLinkId('r_hand');
utorso = r.findLinkId('utorso');
pelvis = r.findLinkId('pelvis');
head = r.findLinkId('head');
l_foot_pts = s.getLeftFootPoints();
r_foot_pts = s.getRightFootPoints();

joints = Point(r.getStateFrame, (1:r.getStateFrame.dim)');
plan_publisher = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN', true, r.getStateFrame.coordinates(1:nq));


switch scenario
  case 0 % No objects in the world
  case 1 
    % main table
    table = RigidBodyBox([0.8,2.0,0.1],[-0.6;0.8;0.975],[0;0;140*pi/180.0]); r = r.addShapeToBody(findLinkId(r,'world'), table);
    r = r.addContactShapeToBody(findLinkId(r,'world'), table);
    table.toLCMGL(lcmgl);
  case 2
    % main table
    table = RigidBodyBox([0.8,2.0,0.1],[-0.6;0.8;0.975],[0;0;140*pi/180.0]); r = r.addShapeToBody(findLinkId(r,'world'), table);
    %table_chull = RigidBodyMeshPoints(table.getPoints());
    %r = r.addContactShapeToBody(findLinkId(r,'world'), table_chull);
    %r = r.addVisualShapeToBody(findLinkId(r,'world'), table);
    r = r.addShapeToBody(findLinkId(r,'world'), table);
    % obstacle on table
    height = 0.5;
    obstacle = RigidBodyBox([0.5, 0.3, height],[-0.9;0.2;1.025+height/2],[0;0;140]*pi/180);
    %obstacle_chull = RigidBodyMeshPoints(obstacle.getPoints());
    %r = r.addContactShapeToBody(findLinkId(r,'world'), obstacle_chull);
    %r = r.addVisualShapeToBody(findLinkId(r,'world'), obstacle);
    r = r.addShapeToBody(findLinkId(r,'world'), obstacle);
    table.toLCMGL(lcmgl);
    obstacle.toLCMGL(lcmgl);
  case 3
    % main table
    table = RigidBodyBox([0.8,2.0,0.1],[-0.6;0.8;0.975],[0;0;140*pi/180.0]); r = r.addShapeToBody(findLinkId(r,'world'), table);
    r = r.addContactShapeToBody(findLinkId(r,'world'), table);
    % obstacle on table
    obstacle1 = RigidBodyBox([0.5, 0.3, 2.5],[-0.9;0.2;1.07],[0*pi/180;0;140*pi/180.0]);
    r = r.addShapeToBody(findLinkId(r,'world'), obstacle1);
    obstacle2 = RigidBodyBox([0.65, 0.4, 4.5],[-0.9;0.2;2.1],[130;0;140]*pi/180.0);
    r = r.addShapeToBody(findLinkId(r,'world'), obstacle2);
    table.toLCMGL(lcmgl);
    obstacle1.toLCMGL(lcmgl);
    obstacle2.toLCMGL(lcmgl);
  case 4
    % main table
    table = RigidBodyBox([0.8,2.0,0.1],[-0.6;0.8;0.975],[0;0;140*pi/180.0]); r = r.addShapeToBody(findLinkId(r,'world'), table);
    r = r.addContactShapeToBody(findLinkId(r,'world'), table);
    % obstacle on table
    obstacle = RigidBodyBox([0.5, 0.3, 2.5],[-0.9;0.2;1.07],[0*pi/180;0;140*pi/180.0]);
    r = r.addShapeToBody(findLinkId(r,'world'), obstacle);
    table.toLCMGL(lcmgl);
    obstacle.toLCMGL(lcmgl);
  case 5
    % main table
    table = RigidBodyBox([0.8,2.0,0.1],[-0.6;0.8;0.975],[0;0;140*pi/180.0]); r = r.addShapeToBody(findLinkId(r,'world'), table);
    r = r.addContactShapeToBody(findLinkId(r,'world'), table);
    % obstacle on table
    table = RigidBodyBox([2.75, 0.4, 2.5],[-0.9;0.1;1.07],[0*pi/180;0;140*pi/180.0]);
    r = r.addShapeToBody(findLinkId(r,'world'), table);
end

r = r.replaceContactShapesWithCHull([l_hand, r_hand, head]);


r = compile(r);
if visualize
  v = r.constructVisualizer(struct('use_contact_shapes',false));
end




%------------------

reach_start = [0.0;0.3;0.8525;0.0;0.0;2.443460952792061;0.0;0.0;0.0;0.27;-1.33;2.1;0.5;0.0;0.0;0.055;-0.49;1.0;-0.51;-0.06;0.0;0.27;1.33;2.1;-0.5;0.0;0.0;-0.055;-0.49;1.0;-0.51;0.06;0.0;0.0];

qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(0.5);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(l_foot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(r_foot, r_foot_pts);

% left foot fixed position/quat
point_in_link_frame = [0.0; 0.0; 0.0];
ref_frame = [1.0, 0.0, 0.0, 0.0; 0.0, 1.0, 0.0, 0.0; 0.0, 0.0, 1.0, 0.0; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [-0.066008905600879214; 0.18381992255977597; 0.081845120720401843] + [0.0; 0.0; 0.0];
upper_bounds = [-0.066008905600879214; 0.18381992255977597; 0.081845120720401843] + [0.0; 0.0; 0.0];
position_constraint_1 = WorldPositionInFrameConstraint(r, l_foot, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [-inf, inf]);

quat_constraint_2 = WorldQuatConstraint(r, l_foot, [0.34201907451327768; -0.00085504946763698261; -0.0023492291048493271; 0.93968968424799793], 0.0, [-inf, inf]);

% right foot fixed position/quat
point_in_link_frame = [0.0; 0.0; 0.0];
ref_frame = [1.0, 0.0, 0.0, 0.0; 0.0, 1.0, 0.0, 0.0; 0.0, 0.0, 1.0, 0.0; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.10295271484131015; 0.38518054073229446; 0.081845120720401843] + [0.0; 0.0; 0.0];
upper_bounds = [0.10295271484131015; 0.38518054073229446; 0.081845120720401843] + [0.0; 0.0; 0.0];
position_constraint_3 = WorldPositionInFrameConstraint(r, r_foot, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [-inf, inf]);

quat_constraint_4 = WorldQuatConstraint(r, r_foot, [0.34201907451327768; 0.00085504946763705623; 0.0023492291048493575; 0.93968968424799793], 0.0, [-inf, inf]);

% fixed back
posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.back_bkz; joints.back_bky; joints.back_bkx];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);

% fixed lower body (legs, base cannot move)
posture_constraint_6 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_x; joints.base_y; joints.base_z; joints.base_roll; joints.base_pitch; joints.base_yaw; joints.r_leg_hpz; joints.r_leg_hpx; joints.r_leg_hpy; joints.r_leg_kny; joints.r_leg_aky; joints.r_leg_akx; joints.l_leg_hpz; joints.l_leg_hpx; joints.l_leg_hpy; joints.l_leg_kny; joints.l_leg_aky; joints.l_leg_akx;joints.neck_ay];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_6 = posture_constraint_6.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);

% fixed right arm
posture_constraint_7 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.r_arm_usy; joints.r_arm_shx; joints.r_arm_ely; joints.r_arm_elx; joints.r_arm_uwy; joints.r_arm_mwx];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_7 = posture_constraint_7.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);

% left hand position/quat constraint
point_in_link_frame = [-5.5511151231257827e-17; 0.24449999999999994; 0.011199999999999988];
ref_frame = [-0.011647833880822878, -0.31436714535882543, 0.94923001737451806, -0.7173478957948709; 0.035146459192054294, 0.948579122841679, 0.31458285731293772, 0.6828552326612789; -0.99931429208840306, 0.037026282933262197, 2.6020852139652114e-18, 1.2; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0; -0; -0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_8 = WorldPositionInFrameConstraint(r, l_hand, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);
grasp_point = ref_frame(1:3,4);
lcmgl.glColor4f(0,1,0,0.5);
lcmgl.sphere(grasp_point,0.05,20,20);
lcmgl.glColor4f(0.7,0.7,0.7,1)

quat_constraint_9 = WorldQuatConstraint(r, l_hand, [0.69586839433919834; -0.099715900534340721; 0.700040526813002; 0.12556742316296038], 0.0, [1.0, 1.0]);

% collision constraint
min_distance = 0.05;
collision_constraint = AllBodiesClosestDistanceConstraint(r, min_distance, 1e3);

%active_constraints = { qsc_constraint_0, position_constraint_1, quat_constraint_2, position_constraint_3, quat_constraint_4, posture_constraint_5, posture_constraint_6, posture_constraint_7, position_constraint_8, quat_constraint_9};
active_constraints = { qsc_constraint_0, position_constraint_1, quat_constraint_2, position_constraint_3, quat_constraint_4, posture_constraint_6, posture_constraint_7, position_constraint_8, quat_constraint_9};


% run ik to find reach end pose
q_seed = reach_start;
[q_end, info, infeasible_constraint] = inverseKin(r, q_seed, q_nom, active_constraints{:}, s.ikoptions);
disp(info);
if (info > 10), display(infeasibleConstraintMsg(infeasible_constraint)); end;
if visualize
  v.draw(0, q_end);
end
lcmgl.switchBuffers();
%drawConstraint(collision_constraint, q_end, lcmgl);
active_constraints = { qsc_constraint_0, collision_constraint, position_constraint_1, quat_constraint_2, position_constraint_3, quat_constraint_4, posture_constraint_6, posture_constraint_7, position_constraint_8, quat_constraint_9};
q_seed = q_end;
[q_end, info, infeasible_constraint] = inverseKin(r, q_seed, q_nom, active_constraints{:}, s.ikoptions);
disp(info);
if (info > 10) 
  display(infeasibleConstraintMsg(infeasible_constraint)); 
  return
end;
if visualize
  v.draw(0, q_end);
end

reach_end = q_end;

if visualize
  v.draw(0, q_end);
end



% compute seed trajector for ik traj
tspan = [0.0, 1.0];
q_nom_traj_foh = PPTrajectory(foh([tspan(1), tspan(end)], [reach_start, reach_end]));
q_seed_traj_foh = PPTrajectory(foh([tspan(end), tspan(end)], [reach_start, reach_start]));
q_seed_traj_spline = PPTrajectory(spline([tspan(1), tspan(end)], [zeros(nq,1), reach_start, reach_start, zeros(nq,1)]));

q_seed_traj = q_nom_traj_foh;
q_nom_traj = q_nom_traj_foh;
%options.frozen_groups = {'pelvis','r_arm'};
options.frozen_groups = {'pelvis','r_arm'};
%options.visualize = true;
options.quiet = false;
options.collision_constraint_type = iktraj_collision_constraint_type;
options.min_distance = min_distance;
options.allow_ikoptions_modification = true;
options.position_cost = 1e-1;
options.acceleration_cost = 1e-6;

% Back fixed
%active_constraints = {qsc_constraint_0, position_constraint_1, quat_constraint_2, position_constraint_3, quat_constraint_4, posture_constraint_5, posture_constraint_6, posture_constraint_7, position_constraint_8, quat_constraint_9};

% Back free
active_constraints = {qsc_constraint_0, position_constraint_1, quat_constraint_2, position_constraint_3, quat_constraint_4, posture_constraint_6, posture_constraint_7, position_constraint_8, quat_constraint_9};

[xtraj,info,infeasible_constraint,xtraj_feasible,info_feasible] = collisionFreePlanner(r,tspan,q_seed_traj,q_nom_traj,options,active_constraints{:},s.ikoptions);
if (info > 10) 
  fprintf('collisionFreePlanner returned with info = %d\n',info);
  %if info == 13, display(infeasibleConstraintMsg(infeasible_constraint)); end;
  display(infeasibleConstraintMsg(infeasible_constraint));
end

%-----------------------------
% this code here just runs ik traj once
%ikoptions = s.ikoptions.setAdditionaltSamples(linspace(0,1,9));
%nt = 3;
%t = linspace(tspan(1), tspan(end), nt);
%disp('running inverseKinTraj...');
%[xtraj, info, infeasible_constraint] = inverseKinTraj(r, t, q_seed_traj, q_seed_traj, active_constraints{:}, ikoptions);
%if (info > 10) display(infeasibleConstraintMsg(infeasible_constraint)); end;
%-------------------------------

q_end = xtraj.eval(tspan(end));

if visualize
  xtraj = xtraj.setOutputFrame(v.getInputFrame());
  v.playback_speed = 0.25;
  playback(v, xtraj, struct('slider',true));
end
max_degrees_per_second = 30.000000;
plan_time = s.getPlanTimeForJointVelocity(xtraj, max_degrees_per_second);
s.publishTraj(plan_publisher, r, xtraj, info, plan_time);



return;



%--- pointwise ik --------

num_pointwise_time_points = num_collision_check_samples;
pointwise_time_points = linspace(t(1), t(end), num_pointwise_time_points);
q_seed_pointwise = xtraj.eval(pointwise_time_points);
q_seed_pointwise = q_seed_pointwise(1:nq,:);
[qtraj_pw, info_pw] = inverseKinPointwise(r, pointwise_time_points, q_seed_pointwise, q_seed_pointwise, active_constraints{:}, s.ikoptions);
xtraj_pw = [qtraj_pw; zeros(nq, length(pointwise_time_points))];

xtraj_pw = PPTrajectory(foh(pointwise_time_points, xtraj_pw));


xtraj_pw = xtraj_pw.setOutputFrame(v.getInputFrame());
playback(v, xtraj_pw, struct('slider',true));
s.publishTraj(plan_publisher, r, xtraj_pw, info_pw(end), plan_time);
