classdef SpaceRobot < handle
    %SpaceRobot Create a tree-structured robot
    %   ROBOT = SpaceRobot() creates a default model that contains no
    %   body.
    %
    %   SpaceRobot properties:
    %       NumBodies               - Number of bodies
    %       Bodies                 - Cell array of rigid bodies
    %       Base                   - Base of the robot
    %       BodyNames              - Cell array of Names of rigid bodies
    %       BaseName               - Name of robot base
    %
    %   SpaceRobot methods:
    %       getBody               - Get robot's body handle by name
    %       getTransform          - Get transform between two body frames
    %       homeConfiguration     - Return the home configuration for robot
    %       showdetails           - Display details of robot
    %       show                  - Plot robot body frames
    %       massMatrix            - Compute joint-space mass matrix
    %       inverseDynamics       - Compute required joint torques given desired motion
    %       forwardDynamics       - Compute resultant joint accelerations given joint torques and states
    %       centerOfMass          - Compute center of mass position and Jacobian
    %       externalForce         - Formulate external force matrix
    %
    %     Not Needed
    %       velocityProduct       - Compute joint torques that cancel velocity induced forces
    %       randomConfiguration   - Return a random configuration for robot
    %       copy                  - Copy robot
    %       subtree               - Get a subtree from robot as a new robot
    %       addSubtree            - Attach a subtree to current robot
    %       replaceJoint          - Replace the joint of one of robot's body
    %       replaceBody           - Replace a body in the robot
    %       addBody               - Add a body to robot
    %       removeBody            - Remove a body from robot
    %       gravityTorque         - Compute joint torques that compensate gravity
    %       checkCollision        - Check if robot is in collision
    %       writeAsFunction       - Create rigidBodyTree generating function

    %#codegen
    properties
        %Name Robot Name
        %
        %   Default: ""
        Name

        Bodies %  Cell array of robot bodies

        Base %  Base body of the robot, SpacecraftBase

        % Configuration
        q % Robot config (6+n x 1): [q0; qm]
        q0 % Base config (6x1): [Rx; Ry; Rz; r; p; y]
        qm % Manipulator config (Nx1): [qm1; ... ;qmN]

        q_dot % Robot speed config (6+n x 1): [q0_dot; qm_d0t]
        q0_dot % Base speed config (6x1): [Rx_dot; Ry_dot; Rz_dot; wx; wy; wz]
        qm_dot % Manipulator speed config (Nx1): [qm1_dot; ... ;qmN_dot]

        Logging % Verbose level: 'error', 'warning', 'info', 'debug'. Default:'warning'
    end

    % TODO: SetAccess = private
    properties % , SetAccess = private)
        Ttree % Forward kinematic transform tree (struct). Transform of each body to inertial frame
        Ttree_symb % Symbolic version of Ttree

        % Jacobs_symb                 % Jacobians of body joints  TODO: NOT IMPLEMENTED

        JacobsCoM_symb % Jacobians of body CoM, symbolic form
        JacobsCoM % Jacobians of body CoM, numeric form

        JacobsCoM_Base_symb % Jacobians of body CoM wrt to base frame

        Config % Struc with info about current config
        q_symb % Symbolic version of q
        q_dot_symb % Symbolic version of q_dot

        H_symb % Mass Matrix, symbolic form
        C_symb % NL Matrix, symbolic form
        Q_symb % Generlized forces matrix, symbolic form
        H
        C
        Q

        KinInitialized % bool, true if kin mats have been initialized
        DynInitialized % bool, true if dyn mats have been initialized
    end

    properties (SetAccess = private)
        %NumBodies Number of bodies in the robot
        %
        %   Default: 0
        NumBodies

        NumActiveJoints %  Number of active joints

        BodyNames

        BaseName
    end

    % TODO: Set and Get to private
    properties %(SetAccess = private , GetAccess = private)
        % Viz
        FastVizHelper % Helper class for fast visualization
        ShowTag

        % Symbolic Function handles
        matFuncHandle % Handle to symbolic function to compute matrices [H, C, Q] = matFuncHandle(q, q_dot)
        tTreeFuncHandle
        JacobsCoM_FuncHandle % Struct of symbolic function handle to convert symbolic body jacobian to numeric

        LogLevels = {'error', 'warning', 'info', 'debug'};
    end

    % Robot Representation methods
    methods

        function obj = SpaceRobot(varargin)
            initFuncs = false;

            if nargin == 1
                % From Struct
                assert(isa(varargin{1}, 'struct'), "Invalid input argument for SpaceRobot constructor")
                structModel = varargin{1};

                Name = structModel.Name;
                Base = structModel.Base;
                Bodies = structModel.Bodies;

                Ttree_symb = structModel.Ttree_symb;
                JacobsCoM_Base_symb = structModel.JacobsCoM_Base_symb;
                JacobsCoM_symb = structModel.JacobsCoM_symb;

                H_symb = structModel.H_symb;
                C_symb = structModel.C_symb;
                Q_symb = structModel.Q_symb;

                KinInitialized = structModel.KinInitialized;
                DynInitialized = structModel.DynInitialized;

                tTreeFuncHandle = structModel.tTreeFuncHandle;
                JacobsCoM_FuncHandle = structModel.JacobsCoM_FuncHandle;

            else
                Name = "";
                baseName = 'spacecraftBase';
                Base = SpacecraftBase(baseName);
                Bodies = cell(1, 0);

                Ttree_symb = struct(baseName, sym([]));
                JacobsCoM_Base_symb = struct(baseName, sym([]));
                JacobsCoM_symb = struct(baseName, sym([]));

                H_symb = sym([]);
                C_symb = sym([]);
                Q_symb = sym([]);

                qm_symb = [];
                qm_dot_symb = [];

                KinInitialized = false;
                DynInitialized = false;

                initFuncs = true;
            end

            % Set parameters
            obj.Name = Name;
            obj.Base = Base;
            obj.Bodies = Bodies;

            obj.Ttree_symb = Ttree_symb;
            obj.JacobsCoM_Base_symb = JacobsCoM_Base_symb;
            obj.JacobsCoM_symb = JacobsCoM_symb;

            obj.H_symb = H_symb;
            obj.C_symb = C_symb;
            obj.Q_symb = Q_symb;

            obj.KinInitialized = KinInitialized;
            obj.DynInitialized = DynInitialized;

            obj.Logging = 'warning';

            % Config
            syms 'Rx' 'Ry' 'Rz' 'r' 'p' 'y'
            syms 'Rx_d' 'Ry_d' 'Rz_d' 'wx' 'wy' 'wz'

            qm_symb = sym(zeros(obj.NumActiveJoints, 1));
            qm_dot_symb = sym(zeros(obj.NumActiveJoints, 1));

            for i = 1:obj.NumActiveJoints
                jnt = obj.findJointByConfigId(i);
                qm_symb(i) = jnt.SymbVar;
                qm_dot_symb(i) = sprintf('qm_dot%i', i);
            end

            obj.q_symb = [Rx; Ry; Rz; r; p; y; qm_symb];
            obj.q_dot_symb = [Rx_d; Ry_d; Rz_d; wx; wy; wz; qm_dot_symb];

            % Create function handle

            if initFuncs
                obj.tTreeFuncHandle = matlabFunction(struct2array(obj.Ttree_symb), 'Vars', {obj.q_symb});
                obj.matFuncHandle = matlabFunction(obj.H_symb, obj.C_symb, obj.Q_symb, 'Vars', {obj.q_symb, obj.q_dot_symb});

                f = fields(obj.JacobsCoM_symb);
                obj.JacobsCoM_FuncHandle = struct();

                for i = 1:length(f)
                    obj.JacobsCoM_FuncHandle.(f{i}) = matlabFunction(obj.JacobsCoM_symb.(f{i}), 'Vars', {obj.q_symb});
                end

            else
                obj.tTreeFuncHandle = tTreeFuncHandle;
                obj.matFuncHandle = [];
                obj.JacobsCoM_FuncHandle = JacobsCoM_FuncHandle;
            end

            % Viz
            obj.FastVizHelper = FastVizHelper;
            obj.ShowTag = ['SHOW_TAG_', randomString(5)];
        end

        addBody(obj, bodyIn, parentName)

        function initKin(obj)
            % Initialize kinematic tree (Ttree) and Jacobians

            obj.logger('Initializing kinematic matrices', 'info');

            obj.Base.BaseToParentTransform_symb = [rpy2r(obj.q_symb(4:6).'), obj.q_symb(1:3); zeros(1, 3), 1];

            % Forward kinematic tree
            obj.forwardKinematics('symbolic', true);

            % obj.computeJacobians('TargetFrame', 'base', 'symbolic', true);

            obj.computeJacobians('TargetFrame', 'inertial', 'symbolic', true);

            obj.KinInitialized = true;
        end

        function initDyn(obj, varargin)
            % Initialize H, C and Q Matrices
            %      'simplify'      - To simplify symbolic matrix. Takes a long time but makes
            %                        computing much faster after.
            %                        Default: true

            % Pars inputs
            parser = inputParser;
            parser.addParameter('simplify', true, ...
                @(x)validateattributes(x, {'logical', 'numeric'}, {'nonempty', 'scalar'}));

            parser.parse(varargin{:});

            simplify = parser.Results.simplify;

            % Start UI
            fig = uifigure;
            d = uiprogressdlg(fig, 'Title', 'Matrices Initialization', ...
                'Message', 'Initializing Matrices', ...
                'Indeterminate', 'on');
            fig.Position(3:4) = [410, 110];
            drawnow;

            % obj.initMassMat(d, simplify);
            obj.initCMat(d, simplify);
            obj.initQMat(d);

            d.Message = sprintf('Creating function handles...');
            obj.matFuncHandle = matlabFunction(obj.H_symb, obj.C_symb, obj.Q_symb, 'Vars', {obj.q_symb, obj.q_dot_symb});

            d.Message = sprintf('Done');

            close(fig);

            obj.DynInitialized = true;
        end

        logger(obj, msg, level)

        showDetails(obj)

        ax = show(obj, varargin)

        [ax, bodyDisplayObjArray] = showSimple(obj, parent, collisions, preserve, visuals, frames)

        ax = showFast(obj, parent, collisions, visuals, frames)

        animate(obj, ts, varargin)
    end

    % Kinematics Methods
    methods
        tTree = forwardKinematics(obj, varargin)

        T = getTransform(obj, bodyName1, varargin)

        comPositions = getCoMPosition(obj)

        JacM = computeJacobians(obj, varargin)

        JacM = comJacobiansBase(obj, varargin)

        AxisM = getAxisM(obj, bodyId, frame, varargin)

        res = kinetics(obj, q, q_dot, q_ddot)

    end

    % Dynamcics Methods
    methods
        initMassMat(obj, d, simplify) % OLD

        initCMat(obj, d, simplify) % OLD

        initQMat(obj, d) % OLD

        function [H, C, Q] = getMats(obj, q, q_dot) % OLD

            if nargin == 1
                q = obj.q;
                q_dot = obj.q_dot;
            end

            [H, C, Q] = obj.matFuncHandle(q, q_dot);
        end

        nOk = isNSkewSym(obj)

        cOk = isCOk(obj)

        D = MassMat(obj, varargin)

        [C, app_data] = CMat(obj, varargin)

        q_ddot = forwardDynamics(obj, F, q, q_dot)

        [tau_b, tau_m] = inverseDynamics(obj, varargin)

        function inertiaM = getInertiaM(obj, varargin) % OLD
            %getInertiaM Compute Inertia matrix of all the bodies in the inertial frame
            %   I_inertial = R * I_body * R'
            parser = inputParser;
            parser.StructExpand = false;

            parser.addParameter('symbolic', false, ...
                @(x)validateattributes(x, {'logical', 'numeric'}, {'nonempty', 'scalar'}));
            parser.parse(varargin{:});
            symbolic = parser.Results.symbolic;

            if symbolic
                tTree = obj.Ttree_symb;
            else
                tTree = obj.Ttree;
            end

            inertiaM = struct();

            % Base
            rotM = tTree.(obj.BaseName)(1:3, 1:3);
            inertiaM.(obj.BaseName) = rotM * obj.Base.InertiaM * rotM.';

            for i = 1:length(obj.BodyNames)
                rotM = tTree.(obj.BodyNames{i})(1:3, 1:3);
                inertiaM.(obj.BodyNames{i}) = rotM * obj.Bodies{i}.InertiaM * rotM.';
            end

        end

    end

    % Utilities
    methods %(Access = Private)

        function lId = findBodyIdxByName(obj, bodyName)
            % Returns idx of body with name 'bodyName'. Returns 0 for the base.
            % return -1 if name not found

            lId = -1;

            bodyName = convertStringsToChars(bodyName);

            if strcmp(obj.Base.Name, bodyName)
                lId = 0;
                return
            end

            for i = 1:obj.NumBodies

                if strcmp(obj.Bodies{i}.Name, bodyName)
                    lId = i;
                    break;
                end

            end

        end

        function lName = findBodyNameByIdx(obj, idx)
            % Returns name of body with idx.
            % return '' if idx not valid

            lName = '';

            if idx == 0
                lName = obj.BaseName;
                return
            end

            for i = 1:obj.NumBodies

                if obj.Bodies{i}.Id == idx
                    lName = obj.Bodies{i}.Name;
                    break;
                end

            end

        end

        function joint = findJointByName(obj, jntName)
            % Return joint corresponding to the name

            for i = 1:length(obj.Bodies)
                joint = obj.Bodies{i}.Joint;

                if strcmp(joint.Name, jntName)
                    return
                end

            end

            error("Invalid Joint name specified");
        end

        function joint = findJointByConfigId(obj, jointConfigId)
            % Return joint corresponding to id in configuration
            % jointConfigId (int): Id in config vector
            if jointConfigId > obj.NumActiveJoints || jointConfigId < 1
                error("Invalid configuration ID")
            end

            for i = 1:length(obj.Bodies)
                joint = obj.Bodies{i}.Joint;

                if jointConfigId == joint.Q_id;
                    return
                end

            end

            error("Invalid ID specified");
        end

    end

    % Setter/Getters
    methods
        % Properties
        function BaseName = get.BaseName(obj)
            BaseName = obj.Base.Name;
        end

        function BodyNames = get.BodyNames(obj)
            BodyNames = cell(1, obj.NumBodies);

            for i = 1:obj.NumBodies
                BodyNames{i} = obj.Bodies{i}.Name;
            end

        end

        function NumBodies = get.NumBodies(obj)
            NumBodies = length(obj.Bodies);
        end

        function NumActiveJoints = get.NumActiveJoints(obj)
            NumActiveJoints = 0;

            for i = 1:obj.NumBodies

                if ~strcmp(obj.Bodies{i}.Joint.Type, 'fixed')
                    NumActiveJoints = NumActiveJoints + 1;
                end

            end

        end

        % Matrices
        function H = get.H(obj)
            % get.H Get Mass Matrix at current config
            [H, ~, ~] = obj.matFuncHandle(obj.q, obj.q_dot);
        end

        function C = get.C(obj)
            % get.C Get C Matrix at current config
            [~, C, ~] = obj.matFuncHandle(obj.q, obj.q_dot);
        end

        function Q = get.Q(obj)
            % get.Q Get Q Matrix at current config
            [~, ~, Q] = obj.matFuncHandle(obj.q, obj.q_dot);
        end

        function tTree = get.Ttree(obj)
            tTree = struct();
            tTreeArray = obj.tTreeFuncHandle(obj.q);

            f = fields(obj.Ttree_symb);

            for i = 1:length(f)
                tTree.(f{i}) = tTreeArray(:, 1 + (i - 1) * 4:i * 4);
            end

        end

        function tTree = getTtreeNum(obj, q)
            % Get numerical value of tTree for given config
            tTree = struct();
            tTreeArray = obj.tTreeFuncHandle(q);

            f = fields(obj.Ttree_symb);

            for i = 1:length(f)
                tTree.(f{i}) = tTreeArray(:, 1 + (i - 1) * 4:i * 4);
            end

        end

        function JacobsCoMArray = get.JacobsCoM(obj)
            % Output struct with jacobians of all bodies at current SR config

            JacobsCoMArray = struct();

            JacobsCoMArray.(obj.BaseName) = obj.JacobsCoM_FuncHandle.(obj.BaseName)(obj.q);

            for i = 1:obj.NumBodies
                bodyName = obj.BodyNames{i};
                JacobsCoMArray.(bodyName) = obj.JacobsCoM_FuncHandle.(bodyName)(obj.q)
            end

        end

        function JacobBody = getJacobsCoMNum(obj, q, bodyName)
            % Output jacobian of CoM of bodyName given configuration q

            JacobBody = obj.JacobsCoM_FuncHandle.(bodyName)(obj.q);
        end

        % Config related
        function qm = get.qm(obj)
            qm = zeros(obj.NumActiveJoints, 1);

            for i = 1:obj.NumActiveJoints
                qm(i) = obj.findJointByConfigId(i).Position;
            end

        end

        function q0 = get.q0(obj)
            q0 = [obj.Base.R; obj.Base.Phi];
        end

        function q = get.q(obj)
            q = [obj.q0; obj.qm];
        end

        function set.qm(obj, qm)
            validateattributes(qm, {'numeric'}, {'nonempty', 'size', ...
                                        [obj.NumActiveJoints, 1]}, 'SpaceRobot', 'qm');

            for i = 1:obj.NumActiveJoints
                obj.findJointByConfigId(i).Position = qm(i);
            end

        end

        function set.q0(obj, q0)
            validateattributes(q0, {'numeric'}, {'nonempty', 'size', [6, 1]}, 'SpaceRobot', 'q0');

            obj.Base.R = q0(1:3);
            obj.Base.Phi = q0(4:6);
        end

        function set.q(obj, q)
            validateattributes(q, {'numeric'}, {'nonempty', 'size', [6 + obj.NumActiveJoints, 1]}, ...
                'SpaceRobot', 'q');
            obj.q0 = q(1:6);
            obj.qm = q(7:end);
        end

        function qm_dot = get.qm_dot(obj)
            qm_dot = zeros(obj.NumActiveJoints, 1);

            for i = 1:obj.NumActiveJoints
                qm_dot(i) = obj.findJointByConfigId(i).Speed;
            end

        end

        function q0_dot = get.q0_dot(obj)
            q0_dot = [obj.Base.R_dot; obj.Base.Omega];
        end

        function q_dot = get.q_dot(obj)
            q_dot = [obj.q0_dot; obj.qm_dot];
        end

        function set.qm_dot(obj, qm_dot)
            validateattributes(qm_dot, {'numeric'}, {'nonempty', 'size', ...
                                            [obj.NumActiveJoints, 1]}, 'SpaceRobot', 'qm_dot');

            for i = 1:obj.NumActiveJoints
                obj.findJointByConfigId(i).Speed = qm_dot(i);
            end

        end

        function set.q0_dot(obj, q0_dot)
            validateattributes(q0_dot, {'numeric'}, {'nonempty', 'size', [6, 1]}, 'SpaceRobot', 'q0_dot');

            obj.Base.R_dot = q0_dot(1:3);
            obj.Base.Omega = q0_dot(4:6);
        end

        function set.q_dot(obj, q_dot)
            validateattributes(q_dot, {'numeric'}, {'nonempty', 'size', [6 + obj.NumActiveJoints, 1]}, ...
                'SpaceRobot', 'q_dot');
            obj.q0_dot = q_dot(1:6);
            obj.qm_dot = q_dot(7:end);
        end

        function set.Logging(obj, logging)
            logging = validatestring(logging, obj.LogLevels, 'set.Logging', 'Logging');
            obj.Logging = logging;
        end

        function Config = get.Config(obj)
            Config = struct();

            % Base
            Config.(obj.BaseName).symbVar = obj.q_symb(1:6);
            Config.(obj.BaseName).symbVar_dot = obj.q_dot_symb(1:6);
            Config.(obj.BaseName).R = obj.q0(1:3);
            Config.(obj.BaseName).R_dot = obj.q0_dot(1:3);
            Config.(obj.BaseName).Phi = obj.q0(4:6);
            Config.(obj.BaseName).Omega = obj.q0_dot(4:6);

            for i = 1:obj.NumActiveJoints
                joint = obj.findJointByConfigId(i);
                Config.(joint.ChildBody.Name).symbVar = joint.SymbVar;
                Config.(joint.ChildBody.Name).Theta = joint.Position;
                Config.(joint.ChildBody.Name).Theta_dot = joint.Speed;
            end

        end

        function homeConfig(obj)
            %homeConfiguration Set robot to home configuration with zero joint speeds
            obj.q_dot = zeros(6 + obj.NumActiveJoints, 1);

            for i = 1:obj.NumActiveJoints
                joint = obj.findJointByConfigId(i);
                joint.Position = joint.HomePosition;
            end

            obj.q0 = obj.Base.HomeConf; % And updates Ttree
        end

    end

end

function s = randomString(n)
    %randomString Generate a random string with length N
    charset = char(['a':'z' '0':'9' 'A':'Z']);
    nset = length(charset);
    idx = randi(nset, 1, n);
    s = charset(idx);
end
