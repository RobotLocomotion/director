classdef IKServer

    properties
        robot
        ikoptions
        q_nom
    end
        
    methods
        
        function obj = IKServer()
            obj.robot = RigidBodyManipulator()
        end

        function obj = loadNominalData(obj)
            nom_data = load([getDrakePath() '/examples/Atlas/data/atlas_fp.mat']);
            nq = obj.robot.getNumDOF();
            obj.q_nom = nom_data.xstar(1:nq);
        end
        
        function obj = setupCosts(obj)
            
            obj.ikoptions = IKoptions(obj.robot);
            obj.ikoptions = obj.ikoptions.setMajorIterationsLimit(5e2);
            obj.ikoptions = obj.ikoptions.setMex(true);
            
            val = 100; % high cost on moving legs
            nq = obj.robot.getNumDOF();
            
            cost = Point(obj.robot.getStateFrame(), 1);
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
            
            obj.ikoptions = obj.ikoptions.setQ(diag(cost(1:nq)));
            obj.ikoptions = obj.ikoptions.setQa(10*obj.ikoptions.Qa);
            obj.ikoptions = obj.ikoptions.setQv(10*obj.ikoptions.Qv);

        end
        
        
        function obj = addRobot(obj, modelName)
    
            filename = [getenv('DRC_PATH'), '/models/mit_gazebo_models/mit_robot_drake/', modelName, '.urdf'];
            options = struct('floating', true);
            xyz = zeros(3,1);
            rpy = zeros(3,1);
            obj.robot = obj.robot.addRobotFromURDF(filename , xyz, rpy, options);
            obj.robot = weldFingerJoints(obj.robot);
            obj.robot = compile(obj.robot);

        end
        
        
        function obj = addAffordance(obj, affordanceName)
            filename = [getenv('DRC_PATH'), '/drake/systems/plants/test/', affordanceName, '.urdf']
            options = struct('floating', false);
            xyz = zeros(3,1);
            rpy = zeros(3,1);
            obj.robot = obj.robot.addRobotFromURDF(filename , xyz, rpy, options);
            obj.robot = compile(obj.robot);
        end
        
        function linkNames = getLinkNames(obj)
            linkNames = {obj.robot.body.linkname}';
        end

        function pts = getLeftFootPoints(obj)
            bodyIndex = obj.robot.findLinkInd('l_foot');            
            toe = obj.robot.body(bodyIndex).getContactPoints('toe');
            heel = obj.robot.body(bodyIndex).getContactPoints('heel');
            pts = [toe, heel];
        end
      
        function pts = getRightFootPoints(obj)
            bodyIndex = obj.robot.findLinkInd('r_foot');
            toe = obj.robot.body(bodyIndex).getContactPoints('toe');
            heel = obj.robot.body(bodyIndex).getContactPoints('heel');
            pts = [toe, heel];
        end
    end

end
