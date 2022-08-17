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

        Base                        %  Base link of the robot, SpacecraftBase
        BaseName
        
        
        % Configuration
        q                           % Robot config (6+N x 1): [q0; qm]
        q0                          % Base config (6x1): [Rx; Ry; Rz; r; p; y]
        qm                          % Manipulator config (Nx1): [qm1; ... ;qmN] 

        q_dot                       % Robot speed config (6+N x 1): [q0_dot; qm_d0t]                       
        q0_dot                      % Base speed config (6x1): [Rx_dot; Ry_dot; Rz_dot; wx; wy; wz]
        qm_dot                      % Manipulator speed config (Nx1): [qm1_dot; ... ;qmN_dot] 
    end
    
    % TODO: SetAccess = private
    properties % , SetAccess = private)
        Ttree                       % Forward kinematic transform tree (struct). Transform of each link to inertial frame                   
        Ttree_symb                  % Symbolic version of Ttree    

        CoMJacobsBase_symb          % Jacobians of link CoM wrt to base frame
        Jacobs_symb                 % Jacobians of link joints

        Config                      % Struc with info about current config
        q_symb                      % Symbolic version of q
        q_dot_symb                  % Symbolic version of q_dot
        
        H_symb                      % Mass Matrix, symbolic form
        C_symb                      % NL Matrix, symbolic form
        Q_symb                      % Generlized forces matrix, symbolic form
        H
        C
        Q
    end
    
    % TODO: Set and Get to private
    properties %(SetAccess = private , GetAccess = private)
        % Viz
        FastVizHelper               % Helper class for fast visualization
        ShowTag
        
        % Symbolic Function handles
        matFuncHandle               % Handle to symbolic function to compute matrices [H, C, Q] = matFuncHandle(q, q_dot)
        tTreeFuncHandle
    end
    
    % Robot Representation methods
    methods
        function obj = SpaceRobot(varargin)
            if nargin==1
                % TODO: Init urdf file, DH params

                % From Struct
                if isa(varargin{1}, 'struct')                    
                    structModel = varargin{1};

                    Name = structModel.Name;
                    Base = structModel.Base;
                    Links = structModel.Links;
                    Ttree_symb = structModel.Ttree_symb;
                    CoMJacobsBase_symb = structModel.CoMJacobsBase_symb;
                    H_symb = structModel.H_symb;
                    C_symb = structModel.C_symb;
                    Q_symb = structModel.Q_symb;
                else
                    error("Error creating SpaceRobot: Invalid robot model specified")
                end

                     
            else
                Name = "";
                Base = SpacecraftBase('spacecraftBase');
                Links = cell(1, 0);    
                Ttree_symb = [];
                CoMJacobsBase_symb = [];
                H_symb = sym([]);
                C_symb = sym([]);
                Q_symb = sym([]);
                qm_symb = [];
                qm_dot_symb = [];
            end
            
            obj.Name = Name;
            obj.Base = Base;
            obj.Links = Links;
            obj.Ttree_symb = Ttree_symb;
            obj.CoMJacobsBase_symb = CoMJacobsBase_symb;
            obj.H_symb = H_symb;
            obj.C_symb = C_symb;
            obj.Q_symb = Q_symb;


            % Config
            syms 'Rx' 'Ry' 'Rz' 'r' 'p' 'y'
            syms 'Rx_d' 'Ry_d' 'Rz_d' 'wx' 'wy' 'wz'
            
            qm_symb = sym(zeros(obj.NumActiveJoints, 1));
            qm_dot_symb = sym(zeros(obj.NumActiveJoints, 1));
            for i=1:obj.NumActiveJoints
                jnt = obj.findJointByConfigId(i);
                qm_symb(i) = jnt.SymbVar;
                qm_dot_symb(i) = sprintf('qm_dot%i', i);
            end

            obj.q_symb = [Rx; Ry; Rz; r; p; y; qm_symb];
            obj.q_dot_symb = [Rx_d; Ry_d; Rz_d; wx; wy; wz; qm_dot_symb];
            
            % Create function handle
            obj.matFuncHandle = matlabFunction(obj.H_symb, obj.C_symb, obj.Q_symb, 'Vars', {obj.q_symb, obj.q_dot_symb});   
            obj.tTreeFuncHandle = matlabFunction(struct2array(obj.Ttree_symb), 'Vars', {obj.q_symb});

            % Viz
            obj.FastVizHelper = FastVizHelper;
            obj.ShowTag = ['SHOW_TAG_', randomString(5)];
        end
                
        addLink(obj, linkIn, parentName)
        
        function initMats(obj, varargin)
            % Initialize H, C and Q Matrices
            %      'simpH'         - To simplify mass matrix. Takes a long time but makes 
            %                        computing much faster after.
            %                        Default: false
            %
            %      'simpC'          - To simplify mass matrix. Takes a long time but makes 
            %                        computing much faster after.
            %                        Default: false
            
            % Pars inputs
            parser = inputParser;
            parser.StructExpand = false;

            parser.addParameter('simpH', false, ...
                @(x)validateattributes(x,{'logical', 'numeric'}, {'nonempty','scalar'}));
            parser.addParameter('simpC', false, ...
                @(x)validateattributes(x,{'logical', 'numeric'}, {'nonempty','scalar'}));

            parser.parse(varargin{:});
                
            simpH = parser.Results.simpH;
            simpC = parser.Results.simpC;
            
            % Start UI
            fig = uifigure;
            d = uiprogressdlg(fig,'Title','Matrices Initialization',...
                'Message','Initializing Matrices', ...
                'Indeterminate','on');
            fig.Position(3:4) = [410, 110];
            drawnow;

            obj.Base.BaseToParentTransform_symb = [rpy2r(obj.q_symb(4:6).'), obj.q_symb(1:3); zeros(1, 3), 1];
            
            d.Message = sprintf('Computing Symbolic Forward Kin...');
            obj.forwardKinematics('symbolic', true);

            d.Message = sprintf('Computing Symbolic CoM Jacobians...');
            obj.comJacobiansBase('symbolic', true);
            

            obj.initMassMat(d, simpH);    
            obj.initCMat(d, simpC);
            obj.initQMat(d);
            
            d.Message = sprintf('Creating function handles...');
            obj.matFuncHandle = matlabFunction(obj.H_symb, obj.C_symb, obj.Q_symb, 'Vars', {obj.q_symb, obj.q_dot_symb});
            obj.tTreeFuncHandle = matlabFunction(struct2array(obj.Ttree_symb), 'Vars', {obj.q_symb})

            d.Message = sprintf('Done');

            close(fig);
        end

        showDetails(obj)

        ax = show(obj, varargin)

        [ax, linkDisplayObjArray]  = showSimple(obj, parent, collisions, preserve, visuals, frames)

        ax = showFast(obj, parent, collisions, visuals, frames)
    end

    % Kinematics Methods
    methods
        tTree = forwardKinematics(obj, varargin)

        T = getTransform(obj, linkName1, linkName2)

        comPositions = getCoMPosition(obj)

        % TODO
        Jac = geometricJacobian(obj, Q, endeffectorname)

        JacM = comJacobians(obj)

        JacM = comJacobiansBase(obj, varargin)

        AxisM = getAxisM(obj, linkId, frame)
    end

    % Dynamcics Methods
    methods
        initMassMat(obj, d, simpM)

        initCMat(obj, d, simpC)
        
        initQMat(obj, d)

        function [H, C, Q] = getMats(obj, q, q_dot)
            if nargin==1
                q = obj.q;
                q_dot = obj.q_dot;
            end
            [H, C, Q] = obj.matFuncHandle(q, q_dot);
        end

        nOk = isNSkewSym(obj)

        cOk = isCOk(obj, verbose)

        q_ddot = forwardDynamics(obj, F, q, q_dot)

        tau = inverseDynamics(obj, varargin)
        
        function inertiaM = getInertiaM(obj, varargin)
        %getInertiaM Compute Inertia matrix of all the links in the inertial frame
        %   I_inertial = R * I_link * R'
            parser = inputParser;
            parser.StructExpand = false;

            parser.addParameter('symbolic', false, ...
                @(x)validateattributes(x,{'logical', 'numeric'}, {'nonempty','scalar'}));
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

        function lName = findLinkNameByIdx(obj, idx)
            % Returns name of link with idx.
            % return '' if idx not valid
            
            lName = '';

            if idx == 0
                lName = obj.BaseName;
                return
            end

            for i = 1:obj.NumLinks
                if obj.Links{i}.Id == idx
                    lName = obj.Links{i}.Name;
                    break;
                end
            end
        end    

        function joint = findJointByName(obj, jntName)
            % Return joint corresponding to the name
            
            for i = 1:length(obj.Links)
                joint = obj.Links{i}.Joint;
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
            
            for i = 1:length(obj.Links)
                joint = obj.Links{i}.Joint;
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

        function LinkNames = get.LinkNames(obj)
            LinkNames = cell(1, obj.NumLinks);
            for i = 1:obj.NumLinks
                LinkNames{i} = obj.Links{i}.Name;
            end
        end

        function NumLinks = get.NumLinks(obj)
            NumLinks = length(obj.Links);
        end

        function NumActiveJoints = get.NumActiveJoints(obj)
            NumActiveJoints = 0;
            for i=1:obj.NumLinks
                if ~strcmp(obj.Links{i}.Joint.Type, 'fixed')
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
            for i=1:length(f)
                tTree.(f{i}) = tTreeArray(:, 1+(i-1)*4: i*4);
            end
        end

        % Config related
        function qm = get.qm(obj)
            qm = zeros(obj.NumActiveJoints, 1);
            
            for i=1:obj.NumActiveJoints
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

            for i=1:obj.NumActiveJoints
                obj.findJointByConfigId(i).Position = qm(i);
            end
        end

        function set.q0(obj, q0)
            validateattributes(q0, {'numeric'}, {'nonempty', 'size', [6, 1]}, 'SpaceRobot', 'q0');

            obj.Base.R = q0(1:3);
            obj.Base.Phi = q0(4:6);
        end

        function set.q(obj, q)
            validateattributes(q, {'numeric'}, {'nonempty', 'size', [6+obj.NumActiveJoints, 1]}, ...
                               'SpaceRobot', 'q');
            obj.q0 = q(1:6);
            obj.qm = q(7:end);
        end

        function qm_dot = get.qm_dot(obj)
            qm_dot = zeros(obj.NumActiveJoints, 1);
            
            for i=1:obj.NumActiveJoints
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
            for i=1:obj.NumActiveJoints
                obj.findJointByConfigId(i).Speed = qm_dot(i);
            end
        end

        function set.q0_dot(obj, q0_dot)
            validateattributes(q0_dot, {'numeric'}, {'nonempty', 'size', [6, 1]}, 'SpaceRobot', 'q0_dot');

            obj.Base.R_dot = q0_dot(1:3);
            obj.Base.Omega = q0_dot(4:6);
        end

        function set.q_dot(obj, q_dot)
            validateattributes(q_dot, {'numeric'}, {'nonempty', 'size', [6+obj.NumActiveJoints, 1]}, ...
                               'SpaceRobot', 'q_dot');
            obj.q0_dot = q_dot(1:6);
            obj.qm_dot = q_dot(7:end);
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

            for i=1:obj.NumActiveJoints
                joint = obj.findJointByConfigId(i);
                Config.(joint.ChildLink.Name).symbVar = joint.SymbVar;
                Config.(joint.ChildLink.Name).Theta = joint.Position;
                Config.(joint.ChildLink.Name).Theta_dot = joint.Speed;
            end
        end

        function homeConfig(obj)
            %homeConfiguration Set robot to home configuration with zero joint speeds
            obj.q_dot = zeros(6+obj.NumActiveJoints, 1);
        
            for i=1:obj.NumActiveJoints
                joint = obj.findJointByConfigId(i);
                joint.Position = joint.HomePosition;
            end

            obj.q0 = obj.Base.HomeConf; % And updates Ttree
        end


    end

end

function s = randomString(n)
    %randomString Generate a random string with length N
        charset = char(['a':'z' '0':'9' 'A':'Z' ]);
        nset = length(charset);
        idx = randi(nset,1,n); 
        s = charset(idx);
    end