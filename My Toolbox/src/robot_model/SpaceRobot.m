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
        %addLink Add link to robot config
        %   addLink(linkIn, parentName) add linkIn to robot setting parentName as parent link.
        %
        %   Example:
        %
        %   See also 
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
                obj.JointsConfig(obj.NumActiveJoints) = struct('JointName',linkIn.Joint.Name, ...
                    'JointPosition', linkIn.Joint.Position);                 
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
        %   ROBOT under current config
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

            parser = obj.parseShowInputs(varargin{:});
                        
            fast = parser.Results.FastUpdate;
            preserve = logical(parser.Results.PreservePlot);
            config = parser.Results.Config;
            position = parser.Results.Position;
            parent = parser.Results.Parent;
            collisions = parser.Results.Collisions;
            visuals = parser.Results.Visuals;
            frames = parser.Results.Frames;

            if fast && preserve
                error("Fast and preserve cannot be true at the same time");
            elseif ~fast
                % simpleShow
                % [ax, bodyDisplayObjArray] = obj.simpleShow(config, parent, collisions, position, preserve, visuals, frames);
                
                displayVisuals = strcmpi(visuals,'on');
                displayCollisions = strcmpi(collisions,'on');
                displayFrames = strcmpi(frames,'on');    
                
                if isempty(parent)
                    ax = newplot;
                else
                    ax = newplot(parent);
                end
                
                % if strcmp(ax.NextPlot, 'replace') || (isa(ax, 'matlab.ui.control.UIAxes') && ~ishold(ax))
                %     % When hold is off, for axes or uiaxes, respectively
                %     robotics.manip.internal.RigidBodyTreeVisualizationHelper.resetScene(obj, ax);
                %     % Adjust axis limits according to robot position
                %     ax.XLim = ax.XLim + basePosition(1);
                %     ax.YLim = ax.YLim + basePosition(2);
                %     ax.ZLim = ax.ZLim + basePosition(3);
                % else % when hold is on
                %     if preserve == false
                %         % If preserve flag is false,
                %         % remove previous drawn objects, if they could be found
                %         delete(findall(ax,'type','hgtransform','Tag', obj.ShowTag));
                %         delete(findall(ax,'type','patch','Tag', obj.ShowTag));
                %         delete(findall(ax,'type','line','Tag', obj.ShowTag));
                %     end
                % end
                
                % converting the six-element vector of translations and
                % orientations ([x,y,z,yaw,pitch,roll]), respectively, to a
                % homogeneous transformation matrix

                tTree = sc.forwardKinematics;
                
                [bodyDisplayObjArray, fmanager] = drawRobot(obj, ax, tTree, displayFrames, displayVisuals);

            else
                error("Fast not yet implemented")
                % ax = obj.fastShow(config, parent, collisions, position, visuals, frames);
            end
        end

        function [bodyDisplayObjArray, fmanager] = drawRobot(robot, ax, Ttree, displayFrames, displayVisuals)
            %drawRobotMemoryLess Display robot and output handles to the figure patches and lines
            %   This method draws the robot and provides a cell array
            %   output that contains the handles to the patches and lines
            %   used to draw the robot. The output cell array has dimension
            %   (N+1)x4. The dimensions correspond to the N rigidBodies and
            %   base of the associated rigidBodyTree (the base occupies the
            %   (N+1)th row). The first three columns of each cell array correspond to
            %   the handles to the associated objects for each body: visual
            %   patches, frame patches, and connecting lines, while the
            %   last column contains the child bodies corresponding to each
            %   of the connecting lines at that index (the parent body for
            %   each line is already given by the column index).
            
            % Configure the figure manager for the given axes
            figManagerOpts = struct(...
                'InitializeBannerWidgets', initBannerWidgets, ...
                'EnableClearInfoOnMousePress', enableClearInfoOnMousePress);

            
            % Configure the figure manager for the given axes
            fmanager = robotics.manip.internal.FigureManager(ax, figManagerOpts);
            fmanager.createCornerCoordinateFrame(ax);
            
            % Initialize output
            bodyDisplayObjArray = cell(robot.NumBodies+1, 4);
            
            % robotics.manip.internal.RigidBodyTreeVisualizationHelper.depositRobotShowTag(robot, ax.Parent);
            
            % body frame graphical parameters
            N = 16; % tessellation
            % magicLength, found to produce good visual for most common robots
            magicLength = 1.1;
            r = 0.005*magicLength; % axis arrow radius
            l = 15*r; % axis arrow length
            
            % frame visibility
            if displayFrames
                vis = 'on';
            else
                vis = 'off';
            end
            
            % Draw intertial frame ([0, 0, ,0])
            % TODO

            % Draw base frame (black axes)
            [F, V] = robotics.manip.internal.RigidBodyTreeVisualizationHelper.bodyFrameMesh(r*0.6, l*1.3, N, false);
            V = Ttree{end}*[V'; ones(1,length(V)) ];
            V = V(1:3,:)';
            
            hasVisuals = ~isempty(robot.Base.VisualsInternal);
            % Base frame patches
            bodyDisplayObjArray{end,2} = patch(ax, ...
                'Faces', F,...
                'Vertices', V, ...
                'FaceColor', 'k', ...
                'LineStyle', 'none', ...
                'Tag', robot.ShowTag, ...
                'Visible', vis, ...
                'DisplayName', robot.BaseName, ...
                'ButtonDownFcn', {@robotics.manip.internal.RigidBodyTreeVisualizationHelper.infoDisplay, robot, robot.NumBodies+1, Ttree{end}}, ...
                'UIContextMenu', robotics.manip.internal.RigidBodyTreeVisualizationHelper.getBodyContextMenu(robot.ShowTag, ax, robot.BaseName, hasVisuals, displayVisuals), ...
                'DeleteFcn', @robotics.manip.internal.RigidBodyTreeVisualizationHelper.deleteUIContextMenuUponPatchDestruction);
            
            % % draw regular body frames and connectors
            % [F, V, C] = robotics.manip.internal.RigidBodyTreeVisualizationHelper.bodyFrameMesh(r, l, N, false);
            % [Ff, Vf, Cf] = robotics.manip.internal.RigidBodyTreeVisualizationHelper.bodyFrameMesh(r, l, N, true);
            
            % for i = 1:robot.NumBodies
            %     if strcmp(robot.Bodies{i}.JointInternal.Type, 'fixed')
            %         Vi = Vf;
            %         Fi = Ff;
            %         Ci = Cf;
            %     else
            %         Vi = V;
            %         Fi = F;
            %         Ci = C;
            %     end
            %     Vi = Ttree{i}*[Vi'; ones(1,length(Vi)) ];
            %     Vi = Vi(1:3,:)';
            %     hasVisuals = ~isempty(robot.Bodies{i}.VisualsInternal);
            %     % Body frame patches
            %     bodyDisplayObjArray{i,2} = patch(ax, 'Faces', Fi,...
            %         'Vertices', Vi,...
            %         'FaceVertexCData', Ci,...
            %         'FaceColor', 'flat',...
            %         'LineStyle', 'none',...
            %         'Tag', robot.ShowTag, ...
            %         'Visible', vis, ...
            %         'DisplayName', robot.Bodies{i}.Name, ...
            %         'ButtonDownFcn', {@robotics.manip.internal.RigidBodyTreeVisualizationHelper.infoDisplay, robot, i, Ttree{i}},...
            %         'UIContextMenu', robotics.manip.internal.RigidBodyTreeVisualizationHelper.getBodyContextMenu(robot.ShowTag, ax, robot.Bodies{i}.Name, hasVisuals, displayVisuals), ...
            %         'DeleteFcn', @robotics.manip.internal.RigidBodyTreeVisualizationHelper.deleteUIContextMenuUponPatchDestruction);
                
            %     pid = robot.Bodies{i}.ParentIndex;
            %     if pid == 0
            %         % If the parent is the base, the transform of the
            %         % parent is the origin
            %         Tparent = Ttree{end};
            %     else
            %         Tparent = Ttree{pid};
            %     end
            %     Tcurr = Ttree{i};
            %     p = [Tparent(1:3,4)'; Tcurr(1:3,4)'];
                
            %     % Create the line and associate it with the parent object
            %     lineObject = line(ax, p(:,1), p(:,2), p(:,3),  'Tag', robot.ShowTag, 'Visible', vis);
                
            %     % Store the line object in the hgObject array. The lines
            %     % have to index to their parent bodies, since that is the
            %     % transform that will be associated with the line motion.
            %     if pid == 0
            %         % Base links, which have the origin as the parent, are
            %         % stored last
            %         lineIndex = robot.NumBodies + 1;
            %     else
            %         % All other lines are indexed by the parent
            %         lineIndex = pid;
            %     end
                
            %     % Since the line is associated with its parent, a frame
            %     % can be associated with multiple lines, which are stored
            %     % in a cell array. Index the line by its parent index. The
            %     % corresponding child body index is stored in the fourth
            %     % column of the bodyDisplayObjArray cell array.
            %     if isempty(bodyDisplayObjArray{lineIndex,3})
            %         % Create the cell array if it does not yet exist
            %         bodyDisplayObjArray{lineIndex,3} = {lineObject};
            %         bodyDisplayObjArray{lineIndex,4} = {i};
            %     else
            %         % Add the line to the cell array if it already exists
            %         bodyDisplayObjArray{lineIndex,3} = [bodyDisplayObjArray{lineIndex,3}; {lineObject}];
            %         bodyDisplayObjArray{lineIndex,4} = [bodyDisplayObjArray{lineIndex,4}; {i}];
            %     end
            % end
            
            % % body visuals visibility
            % if displayVisuals
            %     vis = 'on';
            % else
            %     vis = 'off';
            % end
            
            % for i = 1:robot.NumBodies+1
            %     if i > robot.NumBodies
            %         visGeom = robot.Base.VisualsInternal;
            %         T = Ttree{i};
            %         bname = robot.BaseName;
            %     else
            %         visGeom = robot.Bodies{i}.VisualsInternal;
            %         T = Ttree{i};
            %         bname = robot.Bodies{i}.Name;
            %     end
                
            %     meshPatchCell = cell(length(visGeom),1);
            %     for k = 1:length(visGeom)
            %         V = visGeom{k}.Vertices;
            %         F = visGeom{k}.Faces;
            %         TVis = visGeom{k}.Tform;
            %         color = visGeom{k}.Color;
                    
            %         numVertices = size(V,1);
            %         Vi = T*TVis*[V'; ones(1,numVertices) ];
            %         Vi = Vi(1:3,:)';
                    
            %         meshPatchCell{k} = patch(ax, ...
            %             'Faces', F,...
            %             'Vertices', Vi,...
            %             'FaceColor', color(1:3),...
            %             'LineStyle', 'none',...
            %             'Tag', robot.ShowTag, ...
            %             'DisplayName', [bname '_mesh'], ...
            %             'Visible', vis, ...
            %             'ButtonDownFcn', {@robotics.manip.internal.RigidBodyTreeVisualizationHelper.infoDisplay, robot, i, T}, ...
            %             'UIContextMenu', robotics.manip.internal.RigidBodyTreeVisualizationHelper.getBodyContextMenu(robot.ShowTag, ax, bname, 1, displayVisuals), ...
            %             'DeleteFcn', @robotics.manip.internal.RigidBodyTreeVisualizationHelper.deleteUIContextMenuUponPatchDestruction);
            %     end
            %     % Base and body visual patches
            %     bodyDisplayObjArray{i,1} = meshPatchCell;
            % end
            
        end

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
        function tTree = forwardKinematics(obj)
            % Compute forwardKinematics of the robot. Output an array of the homogenous 
            % transformation matrix of inertial from to link: T_inertial_linkI

            n = obj.NumLinks;
%             Ttree = repmat({eye(4)}, 1, n);
            tTree = struct;
            
            % Base
            baseTransform = obj.Base.BaseToParentTransform; % TODO: Relative to inertial frame
            tTree.(obj.BaseName) = struct('JointIdx', 0, 'Transform', baseTransform);

            for i = 1:n
                link = obj.Links{i};
                
                % Find transform to parent
                TLink2Parent = link.Joint.transformLink2Parent; % Taking into account current config 

                % Find transform to inertial frame
                parentT = tTree.(obj.Links{i}.Parent.Name).Transform;
                linkT = parentT * TLink2Parent;

                tTree.(obj.Links{i}.Name) = struct('LinkIdx', obj.Links{i}.Id, 'Transform', linkT);

                % if link.ParentId > 0
                %     Ttree{i} = Ttree{link.ParentId} * TLink2Parent;
                % else % If parent is base
                %     Ttree{i} = TLink2Parent;
                % end
            
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