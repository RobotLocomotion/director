
fprintf('start...\n');


options.floating = true;
urdf = [getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model.urdf'];
% urdf = [getDrakePath(),'/systems/plants/constraint/test/model_simple_visuals.urdf'];
r = RigidBodyManipulator(urdf,options);
finger_joints = {...
    'right_f0_j0'
    'right_f0_j1'
    'right_f0_j2'
    'right_f1_j0'
    'right_f1_j1'
    'right_f1_j2'
    'right_f2_j0'
    'right_f2_j1'
    'right_f2_j2'
    'right_f3_j0'
    'right_f3_j1'
    'right_f3_j2'
    'left_f0_j0'
    'left_f0_j1'
    'left_f0_j2'
    'left_f1_j0'
    'left_f1_j1'
    'left_f1_j2'
    'left_f2_j0'
    'left_f2_j1'
    'left_f2_j2'
    'left_f3_j0'
    'left_f3_j1'
    'left_f3_j2'}';
for joint_name = finger_joints
  r = r.weldJoint(joint_name{:},1);
end
r = compile(r);
% ignored_bodies = {'ltorso','mtorso','r_talus','l_talus'};
ignored_bodies = 1:r.getNumBodies();
r = addLinksToCollisionFilterGroup(r,ignored_bodies,'no_collision',1);
not_ignored_bodies = [cellfun(@(name) r.findLinkInd(name),[r.collision_filter_groups('r_arm').getMembers(),{'world'}])'];
% not_ignored_bodies = [cellfun(@(name) r.findLinkInd(name),[{'r_hand'},{'world'}])'];
r = removeLinksFromCollisionFilterGroup(r,not_ignored_bodies,'no_collision',1);

r = compile(r);
r = r.addRobotFromURDF([getDrakePath() '/systems/plants/test/table.urdf'],[0;0;0],[0;0;0],struct('floating',false))


nom_data = load([getDrakePath() '/examples/Atlas/data/atlas_fp.mat']);
nq = r.getNumDOF();

q_nom = nom_data.xstar(1:nq);{'ltorso','mtorso','r_talus','l_talus'};
q_seed = q_nom+0.0*randn(nq,1);

% r.collision_filter_groups('table') = CollisionFilterGroup;
% r = r.addToIgnoredListOfCollisionFilterGroup({'l_leg','r_leg','core'},'table');
r = compile(r);

ikoptions = IKoptions(r);
cost = Point(r.getStateFrame(),1);
cost.base_x = 0;
cost.base_y = 0;
cost.base_z = 100;
cost.base_roll = 100;
cost.base_pitch = 100;
cost.base_yaw = 0;
cost.back_bkz = 1e4;
cost.back_bky = 1e4;
cost.back_bkx = 1e4;
cost.neck_ay =  100;
cost.l_arm_usy = 1;
cost.l_arm_shx = 1;
cost.l_arm_ely = 1;
cost.l_arm_elx = 1;
cost.l_arm_uwy = 1;
cost.l_arm_mwx = 1;
val = 100; % high cost on moving legs
cost.l_leg_hpz = val;
cost.l_leg_hpx = val;
cost.l_leg_hpy = val;
cost.l_leg_kny = val;
cost.l_leg_aky = val;
cost.l_leg_akx = val;
cost.r_arm_usy = cost.l_arm_usy;
cost.r_arm_shx = cost.l_arm_shx;
cost.r_arm_ely = cost.l_arm_ely;
cost.r_arm_elx = cost.l_arm_elx;
cost.r_arm_uwy = cost.l_arm_uwy;
cost.r_arm_mwx = cost.l_arm_mwx;
cost.r_leg_hpz = cost.l_leg_hpz;
cost.r_leg_hpx = cost.l_leg_hpx;
cost.r_leg_hpy = cost.l_leg_hpy;
cost.r_leg_kny = cost.l_leg_kny;
cost.r_leg_aky = cost.l_leg_aky;
cost.r_leg_akx = cost.l_leg_akx;
cost = double(cost);
ikoptions = ikoptions.setQ(diag(cost(1:nq)));
ikoptions = ikoptions.setQa(10*ikoptions.Qa);
ikoptions = ikoptions.setQv(10*ikoptions.Qv);
ikoptions = ikoptions.setMajorIterationsLimit(5e2);
ikmexoptions = ikoptions;
ikoptions = ikoptions.setMex(false);
ikmexoptions = ikmexoptions.setMex(true);

l_foot = r.findLinkInd('l_foot');
r_foot = r.findLinkInd('r_foot');
l_hand = r.findLinkInd('l_hand');
r_hand = r.findLinkInd('r_hand');
head = r.findLinkInd('head');
l_hand_pts = [0;0;0];
l_toes_x = 0.1;
r_hand_pts = [0;-0.1;0];
hand_goal = [0.5;-0.2;1.4];
tspan = [0,1];
l_foot_pts = [r.getBody(l_foot).getContactPoints('toe'),r.getBody(l_foot).getContactPoints('heel')];
r_foot_pts = [r.getBody(r_foot).getContactPoints('toe'),r.getBody(r_foot).getContactPoints('heel')];
% kc2l = WorldPositionConstraint(r,l_foot,l_foot_pts,[nan(2,4);1*ones(1,4)],[nan(2,4);nan(1,4)],tspan);
% kc2l = WorldPositionConstraint(r,l_foot,l_foot_pts,[nan(2,4);0*ones(1,4)],[nan(2,4);zeros(1,4)],tspan);
kc2l = WorldPositionConstraint(r,l_foot,l_foot_pts,[l_toes_x,l_toes_x,nan(1,2);nan(1,4);zeros(1,4)],[l_toes_x,l_toes_x,nan(1,2);nan(1,4);zeros(1,4)],tspan);
kc2r = WorldPositionConstraint(r,r_foot,r_foot_pts,[0,0,nan(1,2);0,nan(1,3);zeros(1,4)],[0,0,nan(1,2);0,nan(1,3);zeros(1,4)],tspan);
kc3 = WorldPositionConstraint(r,r_hand,r_hand_pts,hand_goal,hand_goal,[1 1]);
kc4 = WorldGazeDirConstraint(r,r_hand,[0;-0.5;1],[0;0;1],0.1,[1,1]);
scc = AllBodiesClosestDistanceConstraint(r,0.05,1e3,tspan);
qsc = QuasiStaticConstraint(r);
qsc = qsc.setShrinkFactor(0.5);
qsc = qsc.addContact(r_foot,r_foot_pts);
qsc = qsc.addContact(l_foot,l_foot_pts);
qsc = qsc.setActive(true);
%% Draw goal
lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'test');
lcmgl.glColor3f(0,1,0)
lcmgl.sphere(hand_goal,0.05,20,20);
lcmgl.glColor3f(0.5,0.5,0.5)
lcmgl.switchBuffers
timer_total = tic;
%% IK at start and end
timer1 = tic;
[q_start,info] = inverseKin(r,q_seed,q_nom,qsc,scc,kc2l,kc2r,ikmexoptions);

%error('stop')

kinsol = doKinematics(r,q_start);
l_foot_pts_start = r.forwardKin(kinsol,l_foot,l_foot_pts);
r_hand_pts_start = r.forwardKin(kinsol,r_hand,r_hand_pts);
kc2l = WorldPositionConstraint(r,l_foot,l_foot_pts,l_foot_pts_start,l_foot_pts_start,tspan);
% kc4 = WorldPositionConstraint(r,r_hand,r_hand_pts,(r_hand_pts_start+hand_goal)/2,(r_hand_pts_start+hand_goal)/2);
% [q_mid,info] = inverseKin(r,q_seed,q_nom,qsc,kc2l,kc2r,kc4,ikmexoptions);
tic;
[q_end,info] = inverseKin(r,q_seed,q_nom,qsc,kc2l,kc2r,kc3,kc4,ikmexoptions);
toc;
[q_end,info] = inverseKin(r,q_end,q_nom,qsc,scc,kc2l,kc2r,kc3,kc4,ikmexoptions);



fprintf('done.\n');

