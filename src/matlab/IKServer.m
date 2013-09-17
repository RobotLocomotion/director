classdef IKServer

    properties
        robot
        ikoptions
        q_nom
    end

    methods

        function obj = IKServer()
            obj.robot = RigidBodyManipulator();
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

            leftLegCost = 100;
            rightLegCost = 100;
            leftArmCost = 100;
            rightArmCost = 100;
            backCost = 1e4;
            neckCost = 100;

            nq = obj.robot.getNumDOF();

            cost = Point(obj.robot.getStateFrame(), 1);

            cost.base_x = 0;
            cost.base_y = 0;
            cost.base_z = 100;
            cost.base_roll = 100;
            cost.base_pitch = 100;
            cost.base_yaw = 0;

            cost.back_bkz = backCost;
            cost.back_bky = backCost;
            cost.back_bkx = backCost;

            cost.neck_ay =  neckCost;

            cost.l_arm_usy = leftArmCost;
            cost.l_arm_shx = leftArmCost;
            cost.l_arm_ely = leftArmCost;
            cost.l_arm_elx = leftArmCost;
            cost.l_arm_uwy = leftArmCost;
            cost.l_arm_mwx = leftArmCost;

            cost.r_arm_usy = rightArmCost;
            cost.r_arm_shx = rightArmCost;
            cost.r_arm_ely = rightArmCost;
            cost.r_arm_elx = rightArmCost;
            cost.r_arm_uwy = rightArmCost;
            cost.r_arm_mwx = rightArmCost;

            cost.l_leg_hpz = leftLegCost;
            cost.l_leg_hpx = leftLegCost;
            cost.l_leg_hpy = leftLegCost;
            cost.l_leg_kny = leftLegCost;
            cost.l_leg_aky = leftLegCost;
            cost.l_leg_akx = leftLegCost;

            cost.r_leg_hpz = rightLegCost;
            cost.r_leg_hpx = rightLegCost;
            cost.r_leg_hpy = rightLegCost;
            cost.r_leg_kny = rightLegCost;
            cost.r_leg_aky = rightLegCost;
            cost.r_leg_akx = rightLegCost;

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
            filename = [getenv('DRC_PATH'), '/drake/systems/plants/test/', affordanceName, '.urdf'];
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
