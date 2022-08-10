classdef SpaceRobot < handle
%SpaceRobot Create a tree-structured robot
%   ROBOT = SpaceRobot() creates a default model that contains no
%   link.
%
%   SpaceRobot properties:
%       NumLinks               - Number of linkss
%       Bodies                 - Cell array of rigid bodies
%       Base                   - Base of the robot
%       BodyNames              - Cell array of Names of rigid bodies
%       BaseName               - Name of robot base
%
%   SpaceRobot methods:
%       getBody               - Get robot's body handle by name
%       geometricJacobian     - Compute the geometric Jacobian
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

        %NumLinks Number of links in the robot
        %
        %   Default: 0
        NumLinks
        
        NumActiveJoints             %  Number of active joints

        Links                       %  Cell array of robot links

        LinkNames

        % Joints                    %  Cell array of robot joints
        
        Base                        %  Base link of the robot, SpacecraftBase
        BaseName
        
        Ttree                       % Forward kinematic transform tree (struct). Transform of each link to inertial frame                   
        BaseConfig                  % Base current configuration (struct)
        BaseSpeed                   % Base current speed (struct)
        JointsConfig                % Joints current configuration (struct)
        JointsSpeed                 % Joints current speed (struct)
    end

    % TODO: Change get and set to private
    properties % , GetAccess = private)
        q
        q_dot
        Hsym      
        Csym                           
        Qsym
    end

    properties(SetAccess = private , GetAccess = private)
    end
    
    % Robot Representation methods
    methods
        function obj = SpaceRobot(varargin)
            if nargin==1
                % TODO: Init from struct, urdf file, DH params
                % if isa(varargin{1}, 'struct')
                %     structModel = varargin{1};
                % elseif isa(varargin{1}, 'string') || isa(varargin{1}, 'char')
                %     [structModel, structKeys] = urdf2robot(varargin{1});
                % else
                %     error("Error creating SpaceRobot: Invalid robot model specified")
                % end

                % Name = structModel.name;
                % NumJoints = structModel.n_q; 
                % Links = structModel.links;                   
                % NumLinks = length(Links);
                % Joints = structModel.joints;           
                % Base = structModel.base_link;
                % Con = structModel.con;
                error("[SpaceRobot] Constructor from DH or urdf not yet implemented")                                
            else
                Name = "";
                NumLinks = 0;
                NumActiveJoints = 0;
                Links = cell(1, 0);                   
                LinkNames = cell(1, 0);
                BaseName = 'spacecraftBase';
                % Joints = {};           
                % Con = {};   
            end

            obj.Name = Name; 
            obj.NumLinks = NumLinks; 
            obj.NumActiveJoints = NumActiveJoints; 
            obj.Links = Links; 
            obj.LinkNames = LinkNames;
            obj.BaseName = BaseName;
            obj.Base = SpacecraftBase(BaseName);
            obj.JointsConfig = repmat(struct('JointName','', 'JointPosition', 0), 1, obj.NumActiveJoints);
            obj.JointsSpeed = repmat(struct('JointName','', 'JointSpeed', 0), 1, obj.NumActiveJoints);

            obj.q = [];
            obj.q_dot = [];
            obj.Hsym = [];
            obj.Csym = [];
            obj.Qsym = [];
        end
        
        addLink(obj, linkIn, parentName)
        
        function initMats(obj, simpM, simpC)
            % simpM: Simplify Mass Matrix. Takes a while
            % simpC: Simplify C Matrix. Takes a while
            if nargin == 1
                simpM = false;
                simpC = false;
            end
            if nargin == 2
                simpC = false;
            end

            fig = uifigure;
            d = uiprogressdlg(fig,'Title','Matrices Initialization',...
                'Message','Initializing Matrices', ...
                'Indeterminate','on');
            drawnow

            syms 'Rx' 'Ry' 'Rz' 'r' 'p' 'y'
            syms 'Rx_d' 'Ry_d' 'Rz_d' 'wx' 'wy' 'wz'

            qm = sym('qm',[obj.NumActiveJoints, 1],'real');
            qm_dot = sym('qm_dot',[obj.NumActiveJoints, 1],'real');

            obj.q = [Rx; Ry; Rz; r; p; y; qm];
            obj.q_dot = [Rx_d; Ry_d; Rz_d; wx; wy; wz; qm_dot];

            obj.JointsConfig = qm';
            obj.JointsSpeed = qm_dot';
            obj.BaseConfig = [Rx, Ry, Rz; r, p, y];
            obj.BaseSpeed = [Rx_d, Ry_d, Rz_d; wx, wy, wz];
            
            obj.initMassMat(d, simpM);    
            obj.initCMat(d, simpC);
            obj.initQMat(d);

            d.Message = sprintf('Done');

            close(d);
        end

        showDetails(obj)

        ax = show(obj, varargin)
        
        function parser = parseShowInputs(obj, varargin)
            %parseShowInputs Parse inputs to show method
            parser = inputParser;
            parser.StructExpand = false;
            parser.addParameter('Parent', [], ...
                @(x)robotics.internal.validation.validateAxesHandle(x));
            parser.addParameter('PreservePlot', true, ...
                @(x)validateattributes(x,{'logical', 'numeric'}, {'nonempty','scalar'}));
            parser.addParameter('FastUpdate', false, ...
                @(x)validateattributes(x,{'logical', 'numeric'}, {'nonempty','scalar'}));
            parser.addParameter('Visuals', 'on', ...
                @(x)any(validatestring(x, {'on', 'off'})));
            parser.addParameter('Collisions', 'off', ...
                @(x)any(validatestring(x, {'on', 'off'})));
            parser.addParameter('Frames', 'on', ...
                @(x)any(validatestring(x, {'on', 'off'})));
            parser.addParameter('Position', [0,0,0,0], ...
                @(x)(validateattributes(x, {'numeric'}, ...
                {'nonempty', 'real', 'nonnan', 'finite', 'vector', 'numel', 4})));
            
            parser.parse(varargin{:});
        end
    end

    % Kinematics Methods
    methods
        tTree = forwardKinematics(obj)

        T = getTransform(obj, linkName1, linkName2)

        comPositions = getCoMPosition(obj)

        % TODO
        Jac = geometricJacobian(obj, Q, endeffectorname)

        JacM = comJacobians(obj)

        JacM = comJacobiansBase(obj)

        AxisM = getAxisM(obj, linkId, frame)
    end

    % Dynamcics Methods
    methods
        initMassMat(obj, d, simpM)

        initCMat(obj, d, simpC)
        
        initQMat(obj, d)

        function H = getH(obj)
        % get.H Get Mass Matrix at current config
            q_val = [obj.BaseConfig.Position, obj.BaseConfig.Rot, [obj.JointsConfig.JointPosition]]';
            H = double(subs(obj.Hsym, obj.q, q_val));
        end

        function C = getC(obj)
            % get.C Get C Matrix at current config
                q_val = [obj.BaseConfig.Position, obj.BaseConfig.Rot, [obj.JointsConfig.JointPosition]]';
                q_dot_val = [obj.BaseSpeed.TSpeed, obj.BaseSpeed.ASpeed, [obj.JointsSpeed.JointSpeed]]';
                C = double(subs(obj.Csym, [obj.q; obj.q_dot], [q_val; q_dot_val]));
        end

        function Q = getQ(obj)
            % get.Q Get Q Matrix at current config
                q_val = [obj.BaseConfig.Position, obj.BaseConfig.Rot, [obj.JointsConfig.JointPosition]]';
                q_dot_val = [obj.BaseSpeed.TSpeed, obj.BaseSpeed.ASpeed, [obj.JointsSpeed.JointSpeed]]';
                Q = double(subs(obj.Qsym, [obj.q; obj.q_dot], [q_val; q_dot_val]));
        end

        nOk = isNSkewSym(obj)

        cOk = isCOk(obj, verbose)

        q_ddot = forwardDynamics(obj, F)

        tau = inverseDynamics(obj, varargin)
        
        function inertiaM = getInertiaM(obj)
        %getInertiaM Compute Inertia matrix of all the links in the inertial frame
        %   I_inertial = R * I_link * R'
            tTree = obj.Ttree;

            inertiaM = struct();
            
            % Base
            rotM = tTree.(obj.BaseName)(1:3, 1:3);
            inertiaM.(obj.BaseName) = rotM*obj.Base.InertiaM*rotM.';

            for i=1:length(obj.LinkNames)
                rotM = tTree.(obj.LinkNames{i})(1:3, 1:3);
                inertiaM.(obj.LinkNames{i}) = rotM*obj.Links{i}.InertiaM*rotM.';
            end
        end

    end
    
    % Utilities
    methods %(Access = Private)
        function lId = findLinkIdxByName(obj, linkName)
            % Returns idx of link with name 'linkName'. Returns 0 for the base.
            % return -1 if name not found
            
            lId = -1;

            linkName = convertStringsToChars(linkName);

            if strcmp(obj.Base.Name, linkName)
                lId = 0;
                return
            end

            for i = 1:obj.NumLinks
                if strcmp(obj.Links{i}.Name, linkName)
                    lId = i;
                    break;
                end
            end
        end

        function joint = findJointByName(obj, jntName)
            % Return joint id corresponding to the name
            
            for i = 1:length(obj.Links)
                joint = obj.Links{i}.Joint;
                if strcmp(joint.Name, jntName)
                    return 
                end
            end
            error("Invalid Joint name specified");
        end
    end

    % Setter/Getters
    methods
        function set.JointsConfig(obj, newConfig)
            %set JointsConfig

            validateattributes(newConfig, {'struct', 'numeric', 'sym'},...
                {'row'}, 'SpaceRobot', 'JointsConfig');
            
            if isa(newConfig,'struct')
                if length(newConfig) ~= obj.NumActiveJoints
                    error("Invalid config: Missing values")
                end
                obj.JointsConfig = newConfig;
            else
                if length(newConfig) ~= obj.NumActiveJoints
                    error("Invalid config: Missing values")
                end
                for i=1:length(newConfig)
                    obj.JointsConfig(i).JointPosition = newConfig(i);
                end
            end

            %Update all joints position
            for i=1:length(obj.JointsConfig)
                jntName = obj.JointsConfig(i).JointName;
                jntPosition = obj.JointsConfig(i).JointPosition;

                joint = obj.findJointByName(jntName);

                if i ~= joint.Q_id
                    error("Invalid joint idx while setting config")
                end
                joint.Position = jntPosition;
            end

            obj.forwardKinematics();

        end

        function set.JointsSpeed(obj, newSpeed)
            %set newSpeed: (1xN) real vector or struct

            validateattributes(newSpeed, {'struct', 'numeric', 'sym'},...
                {'row'}, 'SpaceRobot', 'JointsSpeed');
            
            if isa(newSpeed,'struct')
                if length(newSpeed) ~= obj.NumActiveJoints
                    error("Invalid config: Missing values")
                end
                obj.JointsSpeed = newSpeed;
            else
                if length(newSpeed) ~= obj.NumActiveJoints
                    error("Invalid config: Missing values")
                end
                for i=1:length(newSpeed)
                    obj.JointsSpeed(i).JointSpeed = newSpeed(i);
                end
            end
        end

        function homeConfig(obj)
            %homeConfiguration Set robot to home configuration
            %
            %   Q = homeConfiguration(ROBOT) returns the home
            %   configuration of ROBOT as predefined in the robot model.
            %   The configuration Q is returned as an array of structs.
            %   The structure array contains one struct for each non-fixed
            %   joint. Each struct contains two fields
            %       - JointName
            %       - JointPosition
            %   The sequence of structs in the array is the same as that
            %   displayed by SHOWDETAILS
            %
            %
            %   Example;
            %       % Load predefined robot models
            %       load exampleRobots
            %
            %       % Get the predefined home configuration for PUMA robot
            %       Q = homeConfiguration(puma1)
            %
            %   See also showdetails, randomConfiguration
    
            % Q = obj.TreeInternal.homeConfiguration();
            HomeConf = obj.JointsConfig;

            for i=1:length(HomeConf)
                jntName = HomeConf(i).JointName;
                joint = obj.findJointByName(jntName);
                
                HomeConf(i).JointPosition = joint.HomePosition;
            end

            obj.JointsConfig = [HomeConf.JointPosition];
            obj.JointsSpeed = [zeros(1, obj.NumActiveJoints)];
            obj.BaseConfig = [0, 0, 0; 0, 0, 0];
            obj.BaseSpeed = [0, 0, 0; 0, 0, 0];

            obj.forwardKinematics();
        end
        
        function baseConf = get.BaseConfig(obj)
            baseConf = struct;
            baseConf.Position = obj.Base.BasePosition;
            baseConf.Rot = obj.Base.BaseRot;
        end

        function baseSpeed = get.BaseSpeed(obj)
            baseSpeed = struct;
            baseSpeed.TSpeed = obj.Base.BaseTSpeed;
            baseSpeed.ASpeed = obj.Base.BaseASpeed;
        end

        function set.BaseConfig(obj, newConfig)
            % Set new base config
            % Can set by passing either a struct or array in the form [x, y, z; r, p, y]
            %
            %
            % Ex:
            %   newConfig = sc.BaseConfig
            %   newConfig.BasePosition = [1, 2, 3]
            %   sc.BaseConfig = newConfig
            %
            %   sc.BaseConfig = [1, 2, 3; pi/2, 0, 0]
            
            if isa(newConfig,'struct')
                validateattributes(newConfig, {'struct'},...
                {'nonempty'}, 'SpaceRobot', 'BaseConfig');

                % obj.BaseConfig = newConfig;
                obj.Base.BasePosition = newConfig.Position;
                obj.Base.BaseRot = newConfig.Rot;

            else
                
                validateattributes(newConfig, {'numeric', 'sym'},...
                {'nonempty', 'size', [2, 3]}, 'SpaceRobot', 'BaseConfig');
                
                obj.Base.BasePosition = newConfig(1, :);
                obj.Base.BaseRot = newConfig(2, :);
            end

            obj.forwardKinematics();
        end

        function set.BaseSpeed(obj, newConfig)
            % Set new base config
            % Can set by passing either a struct or array in the form [x, y, z; r, p, y]
            %
            %
            % Ex:
            %   newConfig = sc.BaseSpeed
            %   newConfig.BaseTSpeed = [1, 2, 3]
            %   sc.BaseSpeed = newConfig
            %
            %   sc.BaseSpeed = [1, 2, 3; pi/2, 0, 0]
                
            validateattributes(newConfig, {'numeric', 'sym'},...
                {'nonempty', 'size', [2, 3]}, 'SpaceRobot', 'BaseSpeed');
                
            obj.Base.BaseTSpeed = newConfig(1, :);
            obj.Base.BaseASpeed = newConfig(2, :);
        end
    end

end