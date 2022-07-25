classdef SpaceRobot < handle
%SpaceRobot Create a tree-structured robot
%   ROBOT = SpaceRobot() creates a default model that contains no
%   rigid bodies.
%
%   rigidBodyTree properties:
%       NumLinks               - Number of linkss
%       Bodies                 - Cell array of rigid bodies
%       Base                   - Base of the robot
%       BodyNames              - Cell array of Names of rigid bodies
%       BaseName               - Name of robot base
%       DataFormat             - Input/output data format
%
%   rigidBodyTree methods:
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
        BaseConfig
        JointsConfig                % Joints current configuration (struct)

        % Con                       %  Structure with additional connectivity information.
    end

    properties(SetAccess = private)
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
%             obj.JointsConfig = struct('JointName', '', 'JointPosition', 0);
            obj.JointsConfig = repmat(struct('JointName','', 'JointPosition', 0), 1, obj.NumActiveJoints);
            
            % obj.Joints = Joints; 
            % obj.Con = Con;    
        end
        
        function addLink(obj, linkIn, parentName)
            narginchk(3,3);
            validateattributes(linkIn, {'Link'}, ...
                               {'scalar', 'nonempty'},'addBody', 'linkIn');

            % Check if link with same name exists already in robot
            linkId = obj.findLinkIdxByName(linkIn.Name); % Find link id from name
            
            if linkId > -1
                error("Invalid link name. Same name already exists in the robot")
            end

            % Check if parent exists in robot
            pId = obj.findLinkIdxByName(parentName);
            if pId == -1
                error("Invalid parent name")
            end
            
            % TODO: Check joint name collision

            % Update indexes
            obj.NumLinks = obj.NumLinks + 1;
            linkId = obj.NumLinks;

            obj.Links{linkId} = linkIn;
            obj.LinkNames{linkId} = linkIn.Name;
            linkIn.Id = linkId;
            
            if pId > 0
                parent = obj.Links{pId};
            else
                parent = obj.Base;
            end
            
            linkIn.ParentId = pId;
            linkIn.Parent = parent;
            parent.Children{end+1} = linkIn;

            % Add active joints to config and set joints to HomePosition
            if ~strcmp(linkIn.Joint.Type, 'fixed')
                obj.NumActiveJoints = obj.NumActiveJoints + 1;
                linkIn.Joint.Position = linkIn.Joint.HomePosition;
                linkIn.Joint.Q_id = obj.NumActiveJoints;
                obj.JointsConfig(obj.NumActiveJoints) = struct('JointName',linkIn.Joint.Name, 'JointPosition', linkIn.Joint.Position);                 

            end
        end
        
        function showdetails(obj)
        %showdetails Display details of the robot
        %   showdetails(ROBOT) displays details of each body in the
        %   robot including the body name, associated joint name and
        %   type, and its parent name and children names.
        %
        %   Example:
        %       % Display details of myRobot robot
        %       showdetails(myRobot);
        %
        %   See also show
            fprintf('--------------------\n');
            fprintf('Robot: (%d bodies)\n\n', int32(obj.NumLinks));
            
            widMaxBodyName = 10;
            widMaxJointName = 10;
            
            for i = 1:obj.NumLinks
                widMaxBodyName = ...
                    max(widMaxBodyName, length(obj.Links{i}.Name)+5);
                widMaxJointName = ...
                    max(widMaxJointName, length(obj.Links{i}.Joint.Name)+5);
            end
            
            fprintf('%4s   %*s   %*s   %*s   %*s(Idx)   Children Name(s)\n', ...
                'Idx', ...
                widMaxBodyName, 'Body Name',...
                widMaxJointName, 'Joint Name',...
                widMaxJointName, 'Joint Type',...
                widMaxBodyName+2, 'Parent Name' );
            fprintf('%4s   %*s   %*s   %*s   %*s-----   ----------------\n', ...
                '---', ...
                widMaxBodyName, '---------',...
                widMaxJointName, '----------',...
                widMaxJointName, '----------',...
                widMaxBodyName+2, '-----------');
            
            for i = 1:obj.NumLinks
                
                jointname = obj.Links{i}.Joint.Name;
                jointtype = obj.Links{i}.Joint.Type;
                linkName = obj.Links{i}.Name;
                
%                 fprintf('%4d', int32(obj.Links{i}.Index));
                fprintf('%4d', i);
                fprintf('   %*s', widMaxBodyName, linkName);
                fprintf('   %*s', widMaxJointName, jointname);
                fprintf('   %*s', widMaxJointName, jointtype);
                
                
                pid = obj.Links{i}.ParentId;
                if pid > 0
                    parent = obj.Links{pid};
                else
                    parent = obj.Base;
                end
                pid = obj.Links{i}.ParentId;
                
                % estimate the number of digits for a body index
                p = pid;
                widID = 0;
                while p > 0.2
                    p = floor(p/10);
                    widID = widID+1;
                end
                widID = max(1, widID);
                
                fprintf('%*s(%*d)   ', widMaxBodyName+8-widID, parent.Name, widID, int32(pid));
                
                childrenList = obj.Links{i}.Children;
                for j = 1:length(childrenList)
                    childrenName = childrenList{j}.Name;
                    childrenId = childrenList{j}.Id;
                    fprintf('%s(%d)  ', childrenName, int32(childrenId) );
                end
                
                fprintf('\n');
            end
            
            fprintf('--------------------\n');
        end
        
        %TODO
        function ax = show(obj, varargin)
        %SHOW Plot robot body frames
        %   SHOW(ROBOT) plots in MATLAB figure the body frames of
        %   ROBOT under the predefined home configuration.
        %
        %   SHOW(ROBOT, Q) plots in MATLAB figure the body frames of
        %   ROBOT under configuration Q.
        %
        %   AX = SHOW(ROBOT, ___) returns the axes handle under which
        %   the robot is plotted.
        %
        %   SHOW(___, Name, Value) provides additional options specified
        %   by one or more Name, Value pair arguments. Name must appear
        %   inside single quotes (''). You can specify several name-value
        %   pair arguments in any order as Name1, Value1, ..., NameN, ValueN:
        %
        %      'Parent'         - Handle of the axes in which the body
        %                         frames of the robot are to be rendered
        %
        %      'PreservePlot'   - When hold is on and show method is called
        %                         repeatedly, this Boolean parameter
        %                         determines whether the graphic
        %                         objects resulted from previous calls
        %                         of show method are preserved
        %                         (true) or cleared (false).
        %                         When 'PreservePlot' value is true,
        %                         'FastUpdate' value must be false.
        %
        %                         Default: true
        %
        %      'Frames'         - A char vector to turn on and off the
        %                         display of the body frames. The value
        %                         can be either 'on' or 'off'.
        %
        %                         Default: 'on'
        %
        %      'Visuals'        - A char vector to turn on and off the
        %                         display of the body visual meshes.
        %                         The value can be either 'on' or 'off'.
        %
        %                         Default: 'on'
        %
        %      'Collisions'     - A char vector to turn on and off the
        %                         display of the body collision geometries.
        %                         The value can be either 'on' or 'off'.
        %
        %                         Default: 'off'
        %
        %      'Position'       - Position of the robot [x,y,z,yaw].
        %
        %                         Default: [0,0,0,0]
        %
        %      'FastUpdate'     - Specify this parameter to improve the
        %                         performance, when show method is called
        %                         multiple times. The function uses
        %                         HGTransforms for performance improvement.
        %                         Set the value to true for faster
        %                         performance. Otherwise, set the value to
        %                         false. 'FastUpdate' preserves the visual
        %                         state of the robot until its value is
        %                         changed.
        %                         When 'FastUpdate' value is true,
        %                         'PreservePlot' value must be false.
        %
        %                         Default: false
        %
        %   Example:
        %       % Load predefined robot models
        %       load exampleRobots
        %
        %       % Render Baxter robot frames in 3D plot at home
        %       % configuration
        %       show(baxter, baxter.randomConfiguration);
        %
        %       % Open plot browser for a list of body names
        %       plotbrowser
        %
        %       % Plot Puma in the same plot as Baxter
        %       hold on
        %       show(puma1, puma1.homeConfiguration)
        %
        %
        %       % Plot LBR robot into two subplots with different configuration
        %       figure
        %       subplot(1,2,1);
        %       show(lbr, lbr.randomConfiguration);
        %       hold on
        %
        %       subplot(1,2,2);
        %       show(lbr, lbr.randomConfiguration);
        %       hold on
        %
        %       % After executing the following four lines. You should
        %       % still see one robot arm in both subplots.
        %       subplot(1,2,1)
        %       show(lbr, lbr.randomConfiguration, 'PreservePlot', false);
        %
        %       subplot(1,2,2)
        %       show(lbr, lbr.randomConfiguration, 'PreservePlot', false);
        %
        %
        %       % Playback a trajectory for a robot in two subplots
        %       % Make a copy of lbr robot
        %       lbrCpy = copy(lbr);
        %
        %       figure
        %
        %       % Start from home configuration
        %       Q = lbr.homeConfiguration;
        %       i = 1;
        %       for i = 1:50
        %           % Increment joint_a2 and joint_a4
        %           Q(2).JointPosition = Q(2).JointPosition + 0.02;
        %           Q(4).JointPosition = Q(4).JointPosition - 0.02;
        %
        %           % On the left subplot, preserve all previous
        %           % drawings, on the right subplot, only keep the
        %           % most recent drawing. Note the 'Parent' parameter
        %           % selects in which axis the robot is drawn
        %           show(lbr, Q, 'PreservePlot', false, 'Parent', subplot(1,2,1));
        %           show(lbrCpy, Q, 'Parent', subplot(1,2,2));
        %           hold on
        %           drawnow
        %       end
        %
        %       figure;
        %       robot = loadrobot("rethinkBaxter");
        %       show(robot, 'Collisions', 'on', 'Visuals', 'off');
        %
        %       %Animate robot configurations using FastUpdate
        %       figure
        %       robot = loadrobot("kinovaGen3","DataFormat","column");
        %       robotConfigs = trapveltraj([randomConfiguration(robot) randomConfiguration(robot)],100);
        %       for i = 1:100
        %           robot.show(robotConfigs(:,i),'PreservePlot',false,'FastUpdate',true);
        %           drawnow;
        %       end
        %
        %   See also showdetails

%             ax = obj.TreeInternal.show(varargin{:});
            warning('Not yet implemented')
        end
       
    end

    % Kinematics Methods
    methods
        function Ttree = forwardKinematics(obj)
            % Compute forwardKinematics of the robot. Output an array of the homogenous 
            % transformation matrix of inertial from to link: T_inertial_linkI

            n = obj.NumLinks;
            Ttree = repmat({eye(4)}, 1, n);             

            for i = 1:n
                link = obj.Links{i};
                
                % Find transform to parent
                TLink2Parent = link.Joint.transformLink2Parent; % Taking into account current config 

                % Find transform to inertial frame
                if link.ParentId > 0
                    Ttree{i} = Ttree{link.ParentId} * TLink2Parent;
                else % If parent is base
                    Ttree{i} = TLink2Parent;
                end
            
            end
        end

        % TODO
        function T = getTransform(obj, linkName1, linkName2)
        %getTransform Get the transform between two body frames
        %   T1 = getTransform(ROBOT, BODYNAME1) computes a
        %   transform T1 that converts points originally expressed in
        %   BODYNAME1 frame to be expressed in the robot's base frame.
        %
        %   T2 = getTransform(ROBOT, Q, BODYNAME1, BODYNAME2) computes
        %   a transform T2 that converts points originally expressed in
        %   BODYNAME1 frame to be expressed in BODYNAME2.
        
            narginchk(2,3);
            
            Ttree = obj.forwardKinematics(qvec);
            
            % 2-argument case: getTransform(ROBOT, linkName1)
            lId1 = findLinkIdxByName(obj, linkName1);
            if lId1 == 0
                T1 = eye(4);
            else
                T1 = Ttree{lId1};
            end
            
            T2 = eye(4);
            if nargin == 4
                % 4-argument case: getTransform(ROBOT, linkName1, linkName2)
                lId2 = findLinkIdxByName(obj, linkName2);
                if lId2 == 0
                    T2 = eye(4);
                else
                    T2 = Ttree{lId2};
                end
            end

            R = T2(1:3,1:3)';
            p = -R*T2(1:3,4);
            T = [R,p;[0 0 0 1]]*T1; % the first term is inv(T2)

        end
        
        % TODO
        function Jac = geometricJacobian(obj, Q, endeffectorname)
        %geometricJacobian Compute the geometric Jacobian
        %   JAC = geometricJacobian(ROBOT, Q, ENDEFFECTORNAME) computes
        %   the geometric Jacobian for the body ENDEFFECTORNAME in ROBOT
        %   under the configuration Q. The Jacobian matrix JAC is of size
        %   6xN, where N is the number of degrees of freedom. The
        %   Jacobian maps joint-space velocity to the Cartesian space
        %   end-effector velocity relative to the base coordinate frame.
        %
        %   Example:
        %       % Load predefined robot models
        %       load exampleRobots
        %
        %       % Get the Jacobian for right_wrist body in Baxter robot
        %       % under a random configuration
        %       jac = geometricJacobian(baxter,...
        %                      baxter.randomConfiguration,'right_wrist');

            Jac = obj.TreeInternal.geometricJacobian(Q, endeffectorname);
            warning('Not yep implemented');
        end
    
    end

    % Dynamcics Methods
    methods
         % TODO
        function H = massMatrix(obj, varargin)
        %massMatrix Compute the mass matrix for given configuration
        %   H = massMatrix(ROBOT) returns the joint-space mass
        %   matrix, H, of ROBOT for ROBOT's home configuration.
        %
        %   H = massMatrix(ROBOT, Q) returns the joint-space mass
        %   matrix, H, of ROBOT for the given configuration Q.
        %
        %   Joint configuration Q must be specified as a pNum-by-1 or
        %   an 1-by-pNum vector, depending on the DataFormat property
        %   of ROBOT, where pNum is the position number of ROBOT.
        %
        %   The returned mass matrix H is a positive-definite symmetric
        %   matrix with size vNum-by-vNum, where vNum is the velocity
        %   number of ROBOT (degrees of freedom).
        %
        %   Examples:
        %       % Load example robot
        %       load exampleRobots.mat
        %
        %       % Set lbr robot dynamics input data format to 'column'
        %       lbr.DataFormat = 'column';
        %
        %       % Generate a random configuration for lbr
        %       q = lbr.randomConfiguration
        %
        %       % Get the mass matrix at configuration q
        %       H = massMatrix(lbr, q);

%             narginchk(1,2);
%             q = validateDynamicsFunctionInputs(obj.TreeInternal, false, varargin{:});
%             H = robotics.manip.internal.RigidBodyTreeDynamics.massMatrix(obj.TreeInternal, q);
            warning('Not yet implemented')
        end
        
        % TODO
        function tau = inverseDynamics(obj, varargin)
        %inverseDynamics Compute required joint torques for desired motion.
        %   TAU = inverseDynamics(ROBOT) computes joint torques TAU
        %   required for ROBOT to statically hold its home
        %   configuration with no external forces applied.
        %
        %   TAU = inverseDynamics(ROBOT, Q) computes the required joint
        %   torques for ROBOT to statically hold the given
        %   configuration Q with no external forces applied.
        %
        %   TAU = inverseDynamics(ROBOT, Q, QDOT) computes the joint
        %   torques required for ROBOT given the joint configuration Q
        %   and joint velocities QDOT while assuming zero joint accelerations
        %   and no external forces. (Set Q = [] if the desired joint
        %   configuration is home configuration.)
        %
        %   TAU = inverseDynamics(ROBOT, Q, QDOT, QDDOT) computes the
        %   joint torques required for ROBOT given the joint
        %   configuration Q, joint velocities QDOT and joint accelerations
        %   QDDOT while assuming no external forces are applied. (Set
        %   QDOT = [] to indicate zero joint velocities.)
        %
        %   TAU = inverseDynamics(ROBOT, Q, QDOT, QDDOT, FEXT) computes
        %   the joint torques required for ROBOT given the joint
        %   configuration Q, joint velocities QDOT, joint accelerations
        %   QDDOT and the external forces FEXT. (Set QDDOT = [] to
        %   indicate zero joint accelerations.)
        %
        %   If ROBOT's DataFormat property is set to 'column', the
        %   input variables must be formatted as
        %   - Joint configuration, Q - pNum-by-1 vector
        %   - Joint velocities, QDOT - vNum-by-1 vector
        %   - Joint accelerations, QDDOT - vNum-by-1 vector
        %   - External forces, FEXT - 6-by-NB matrix
        %
        %   where:
        %   - pNum is the position number of ROBOT
        %   - vNum is the velocity number of ROBOT (degrees of freedom)
        %   - NB is the number of bodies in ROBOT
        %   - Each column of FEXT represents a wrench
        %     -- top 3 elements: moment
        %     -- bottom 3 elements: linear force
        %
        %   If the DataFormat property of ROBOT is set to 'row', then
        %   all the vectors/matrices above need to be transposed.
        %
        %   The returned joint torques TAU is either a vNum-by-1 or an
        %   1-by-vNum vector, depending on the DataFormat property.
        %
        %   Examples:
        %       % Load example robot
        %       load exampleRobots.mat
        %
        %       % Set lbr robot dynamics input data format to 'column'
        %       lbr.DataFormat = 'column';
        %
        %       % Generate a random configuration for lbr
        %       q = lbr.randomConfiguration
        %
        %       % Compute the required joint torques for lbr to
        %       % statically hold that configuration
        %       tau = inverseDynamics(lbr, q);
        %
        %   See also forwardDynamics, externalForce

%             narginchk(1,5);
%             [q, qdot, qddot, fext] = validateDynamicsFunctionInputs(obj.TreeInternal, true, varargin{:});
%             tau = robotics.manip.internal.RigidBodyTreeDynamics.inverseDynamics(obj.TreeInternal, q, qdot, qddot, fext);
%             tau = resultPostProcess(obj.TreeInternal, tau);
            warning('Not yet implemented')
        end

        % TODO
        function qddot = forwardDynamics(obj, varargin)
        %forwardDynamics Compute resultant joint acceleration given joint torques and states
        %   QDDOT = forwardDynamics(ROBOT) computes the resultant joint
        %   accelerations due to gravity when ROBOT is at its home
        %   configuration, with zero joint velocities and no external
        %   forces.
        %
        %   QDDOT = forwardDynamics(ROBOT, Q) computes the resultant
        %   joint accelerations due to gravity when ROBOT is at joint
        %   configuration Q, with zero joint velocities and no external
        %   forces.
        %
        %   QDDOT = forwardDynamics(ROBOT, Q, QDOT) computes the joint
        %   accelerations resulted from gravity and joint velocities
        %   QDOT when ROBOT is at Joint configuration Q. (Set Q = [] if
        %   the joint configuration is home configuration.)
        %
        %   QDDOT = forwardDynamics(ROBOT, Q, QDOT, TAU) computes the
        %   joint accelerations due to gravity, joint velocities QDOT
        %   and joint torques TAU when ROBOT is at joint configuration Q.
        %   (Set QDOT = [] to indicate zero joint velocities.)
        %
        %   QDDOT = forwardDynamics(ROBOT, Q, QDOT, TAU, FEXT) computes
        %   the joint accelerations due to gravity, joint velocities
        %   QDOT, torques applied to the joints TAU, and external forces
        %   applied to each body FEXT, when ROBOT is at configuration Q.
        %   (Set TAU = [] to indicate zero joint torques.)
        %
        %   If ROBOT's DataFormat property is set to 'column', the
        %   input variables must be formatted as
        %   - Joint configuration, Q - pNum-by-1 vector
        %   - Joint velocities, QDOT - vNum-by-1 vector
        %   - Joint torques, TAU - vNum-by-1 vector
        %   - External forces, FEXT - 6-by-NB matrix
        %
        %   where:
        %   - pNum is the position number of ROBOT
        %   - vNum is the velocity number of ROBOT (degrees of freedom)
        %   - NB is the number of bodies in ROBOT
        %   - Each column of FEXT represents a wrench
        %     -- top 3 elements: moment
        %     -- bottom 3 elements: linear force
        %
        %   If the DataFormat property of ROBOT is set to 'row', then
        %   all the vectors/matrices above need to be transposed.
        %
        %   The returned joint accelerations QDDOT is either a vNum-by-1
        %   or an 1-by-vNum vector, depending on the DataFormat property.
        %
        %   Examples:
        %       % Load example robot
        %       load exampleRobots.mat
        %
        %       % Set lbr robot dynamics input data format to 'row'
        %       lbr.DataFormat = 'row';
        %
        %       % Set the gravity
        %       lbr.Gravity = [0 0 -9.81];
        %
        %       % Get the home configuration for lbr
        %       q = lbr.homeConfiguration
        %
        %       % The end-effector (body 'tool0') experiences a wrench.
        %       % Use the 2 lines below to generate the corresponding
        %       % external force matrix, fext. Note that if the external
        %       % wrench is specified in the body 'tool0' frame, the
        %       % joint configuration, q, must be specified as the fourth
        %       % input argument for externalForce method.
        %       wrench = [0 0 0.5 0 0 0.3];
        %       fext = externalForce(lbr, 'tool0', wrench, q)
        %
        %       % Compute the resultant joint acceleration due to gravity
        %       % with the external force applied to the end-effector when
        %       % lbr is at its home configuration.
        %       qddot = forwardDynamics(lbr, q, [], [], fext);
        %
        %   See also inverseDynamics, externalForce

%             narginchk(1,5);
%             [q, qdot, tau, fext] = validateDynamicsFunctionInputs(obj.TreeInternal, false, varargin{:});
%             qddot = robotics.manip.internal.RigidBodyTreeDynamics.forwardDynamicsCRB(obj.TreeInternal, q, qdot, tau, fext);
%             qddot = resultPostProcess(obj.TreeInternal, qddot);
            warning('Not yet implemented')
        end
        
        % TODO
        function fext = externalForce(obj, bodyName, wrench, varargin)
        %externalForce Compose external force matrix relative to base
        %   FEXT = externalForce(ROBOT, BODYNAME, WRENCH) composes the
        %   external force matrix, FEXT, that applies an external WRENCH
        %   to body BODYNAME. WRENCH is assumed to be relative to
        %   the base frame.
        %
        %   FEXT = externalForce(ROBOT, BODYNAME, WRENCH, Q) is similar
        %   to the signature above, but WRENCH is assumed relative to
        %   the BODYNAME frame. The fourth input argument, joint
        %   configuration Q, is used to convert WRENCH to the base
        %   frame as required by FEXT.
        %
        %   Depending on the DataFormat property of ROBOT, FEXT is
        %   either a 6-by-vNum ('column') or vNum-by-6 ('row') matrix,
        %   where vNum is the velocity number of ROBOT (degrees of
        %   freedom). FEXT contains WRENCH in the correct column or row
        %   (relative to base frame) that corresponds to the given
        %   BODYNAME. The first 3 elements in WRENCH are assumed to be
        %   moment, the last 3 are assumed linear force.
        %
        %   Examples:
        %       % Load example robot
        %       load exampleRobots.mat
        %
        %       % Set lbr robot dynamics input data format to 'row'
        %       lbr.DataFormat = 'column';
        %
        %       % Get the home configuration for lbr
        %       q = lbr.homeConfiguration
        %
        %       % Set external force on link_4, wrench expressed in base frame
        %       fext1 = externalForce(lbr, 'link_1', [0 0 0.0 0.1 0 0]);
        %
        %       % Set external force on tool0, the end-effector, wrench
        %       % is expressed in tool0 frame
        %       fext2 = externalForce(lbr, 'tool0', [0 0 0.0 0.1 0 0], q);
        %
        %       % Compute the joint torques required to balance the
        %       % external forces
        %       tau = inverseDynamics(lbr, q, [], [], fext1+fext2);
        %
        %   See also inverseDynamics, forwardDynamics

%             narginchk(3,4);
%             fext = externalForce(obj.TreeInternal, bodyName, wrench, varargin{:});
            warning('Not yet implemented')
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
            for i=1:length(obj.Links)
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

            validateattributes(newConfig, {'struct', 'numeric'},...
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

        end

        function Q = homeConfiguration(obj)
            %homeConfiguration Return the home configuration for robot
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
                Q = obj.JointsConfig;

                for i=1:length(Q)
                    jntName = Q(i).JointName;
                    joint = obj.findJointByName(jntName);
                    
                    Q(i).JointPosition = joint.HomePosition;
                end
            end
        
        function baseConf = get.BaseConfig(obj)
            baseConf = struct;
            baseConf.Position = obj.Base.BasePosition;
            baseConf.Rot = obj.Base.BaseRot;
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

%                 obj.BaseConfig = newConfig;
                obj.Base.BasePosition = newConfig.Position;
                obj.Base.BaseRot = newConfig.Rot;

            else
                
                validateattributes(newConfig, {'numeric'},...
                {'nonempty', 'size', [2, 3]}, 'SpaceRobot', 'BaseConfig');
                
                obj.Base.BasePosition = newConfig(1, :);
                obj.Base.BaseRot = newConfig(2, :);
            end
        end
    end

end