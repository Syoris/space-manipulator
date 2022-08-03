classdef RobotVizHelper < handle
    % Helper function for SpaceRobot show method
    properties
        robot
    end

    properties (Constant)
        %NumTessellation
        NumTessellation = 16

        %CollisionMeshPatchColor
        %   Arbitrary choice of color for a collision mesh patch aside from the
        %   default black [0, 0, 0].
        CollisionMeshPatchColor = [0.2, 0.7, 0.2]
    end

    properties(SetAccess=private)
        magicLength  % magicLength, found to produce good visual for most common robots
        r
        l
    end

    methods
        function obj = RobotVizHelper(robot)
            obj.robot = robot;

            obj.magicLength = 1.1;
            obj.r = 0.005*obj.magicLength; % axis arrow radius
            obj.l = 15*obj.r; % axis arrow length

        end

        function [ax, linkDisplayObjArray, fmanager] = drawRobot(obj, ax, Ttree, displayFrames, displayVisuals)
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
            initBannerWidgets = true;
            enableClearInfoOnMousePress = true;
            figManagerOpts = struct(...
                'InitializeBannerWidgets', initBannerWidgets, ...
                'EnableClearInfoOnMousePress', enableClearInfoOnMousePress);
        
            
            % Configure the figure manager for the given axes
            fmanager = FigureManager(ax, figManagerOpts);
            fmanager.createCornerCoordinateFrame(ax);
            
            % Initialize output
            linkDisplayObjArray = cell(obj.robot.NumLinks + 2, 4);
            
            % robotics.manip.internal.RigidBodyTreeVisualizationHelper.depositRobotShowTag(robot, ax.Parent);
            
            % frame visibility
            if displayFrames
                vis = 'on';
            else
                vis = 'off';
            end
            
            % Draw intertial frame ([0, 0, ,0])
            [F, V] = obj.bodyFrameMesh(obj.r*0.6, obj.l*1.3, obj.NumTessellation, true); 
            linkDisplayObjArray{end,2} = patch(ax, ...
                'Faces', F,...
                'Vertices', V, ...
                'FaceColor', 'k', ...
                'LineStyle', 'none', ...
                'Visible', vis, ...
                'DisplayName', 'inertialFrame');

            % Draw base frame
            [F, V, C] = obj.bodyFrameMesh(obj.r*0.6, obj.l*1.3, obj.NumTessellation, false);
            V = Ttree.(obj.robot.BaseName).Transform*[V'; ones(1,length(V)) ];
            V = V(1:3,:)';
            
            hasVisuals = ~isempty(obj.robot.Base.Visuals);
            
            % Base frame patches
            linkDisplayObjArray{end - 1, 2} = patch(ax, ...
                'Faces', F,...
                'Vertices', V, ...
                'FaceVertexCData', C,...
                'FaceColor', 'flat',...
                'LineStyle', 'none', ...
                'Visible', vis, ...
                'DisplayName', obj.robot.BaseName);
                % , ...
                % 'ButtonDownFcn', {@robotics.manip.internal.RigidBodyTreeVisualizationHelper.infoDisplay, robot, robot.NumBodies+1, Ttree{end}}, ...
                % 'UIContextMenu', robotics.manip.internal.RigidBodyTreeVisualizationHelper.getBodyContextMenu(robot.ShowTag, ax, robot.BaseName, hasVisuals, displayVisuals), ...
                % 'DeleteFcn', @robotics.manip.internal.RigidBodyTreeVisualizationHelper.deleteUIContextMenuUponPatchDestruction);
            
            % draw regular body frames and connectors
            [F, V, C] = obj.bodyFrameMesh(obj.r, obj.l, obj.NumTessellation, false);  % Get free meshes
            [Ff, Vf, Cf] = obj.bodyFrameMesh(obj.r, obj.l, obj.NumTessellation, true); % Get fixed meshes
            

            for i = 1:obj.robot.NumLinks
                % Find link meshes
                if strcmp(obj.robot.Links{i}.Joint.Type, 'fixed')
                    Vi = Vf;
                    Fi = Ff;
                    Ci = Cf;
                else
                    Vi = V;
                    Fi = F;
                    Ci = C;
                end
                linkName = obj.robot.LinkNames{i};
                
                % Update link meshes position
                Vi = Ttree.(linkName).Transform*[Vi'; ones(1,length(Vi)) ];
                Vi = Vi(1:3,:)';
                
                hasVisuals = ~isempty(obj.robot.Links{i}.Visuals);
                
                % Body frame patches
                linkDisplayObjArray{i, 2} = patch(ax,...
                    'Faces', Fi,...
                    'Vertices', Vi,...
                    'FaceVertexCData', Ci,...
                    'FaceColor', 'flat',...
                    'LineStyle', 'none',...
                    'Visible', vis, ...
                    'DisplayName', obj.robot.LinkNames{i});
                
                pId = obj.robot.Links{i}.ParentId;
                if pId == 0
                    % If the parent is the base, the transform of the
                    % parent is the origin
                    Tparent = Ttree.(obj.robot.BaseName).Transform;
                else
                    Tparent = Ttree.(obj.robot.LinkNames{pId}).Transform;
                end
                Tcurr = Ttree.(obj.robot.LinkNames{i}).Transform;

                p = [Tparent(1:3,4)'; Tcurr(1:3,4)']; % Line starting and final points
                
                % Create the line and associate it with the parent object
                lineObject = line(ax, p(:,1), p(:,2), p(:,3), 'Visible', vis);
                
                % Store the line object in the hgObject array. The lines
                % have to index to their parent bodies, since that is the
                % transform that will be associated with the line motion.
                if pId == 0
                    % Base links, which have the origin as the parent, are
                    % stored last
                    lineIndex = obj.robot.NumLinks + 1;
                else
                    % All other lines are indexed by the parent
                    lineIndex = pId;
                end
                
                % Since the line is associated with its parent, a frame
                % can be associated with multiple lines, which are stored
                % in a cell array. Index the line by its parent index. The
                % corresponding child body index is stored in the fourth
                % column of the linkDisplayObjArray cell array.
                if isempty(linkDisplayObjArray{lineIndex,3})
                    % Create the cell array if it does not yet exist
                    linkDisplayObjArray{lineIndex,3} = {lineObject};
                    linkDisplayObjArray{lineIndex,4} = {i};
                else
                    % Add the line to the cell array if it already exists
                    linkDisplayObjArray{lineIndex,3} = [linkDisplayObjArray{lineIndex,3}; {lineObject}];
                    linkDisplayObjArray{lineIndex,4} = [linkDisplayObjArray{lineIndex,4}; {i}];
                end
            end
            
            % body visuals visibility
            if displayVisuals
                vis = 'on';
            else
                vis = 'off';
            end
            
            for i = 1:obj.robot.NumLinks+1
                % Find transform
                if i > obj.robot.NumLinks
                    visGeom = obj.robot.Base.Visuals;
                    linkName = obj.robot.BaseName;
                else
                    visGeom = obj.robot.Links{i}.Visuals;
                    linkName = obj.robot.LinkNames{i};
                end
                T = Ttree.(linkName).Transform;

                % Draw all visuals of current link
                meshPatchCell = cell(length(visGeom),1);
                for k = 1:length(visGeom)
                    V = visGeom{k}.Vertices;
                    F = visGeom{k}.Faces;
                    TVis = visGeom{k}.Tform;
                    color = visGeom{k}.Color;
                    
                    numVertices = size(V,1);
                    Vi = T*TVis*[V'; ones(1,numVertices) ];
                    Vi = Vi(1:3,:)';
                    
                    meshPatchCell{k} = patch(ax, ...
                        'Faces', F,...
                        'Vertices', Vi,...
                        'FaceColor', color(1:3),...
                        'LineStyle', 'none',...
                        'DisplayName', [linkName '_mesh'], ...
                        'Visible', vis);
                end
                % Base and body visual patches
                linkDisplayObjArray{i,1} = meshPatchCell;
            end
        end

        function resetScene(obj, ax)
            % PRIMARY axes
            axis(ax, 'vis3d');
            hFigure = ax.Parent;
            pan(hFigure,'on');
            rotate3d(hFigure,'on');
            
            ax.Visible = 'off';
            daspect(ax, [1 1 1]);
            
            
            % % estimate the size of workspace
            % if ~robot.IsMaxReachUpToDate
            %     robot.estimateWorkspace;
            %     robot.IsMaxReachUpToDate = true;
            % end
            % a = robot.EstimatedMaxReach; % updated by estimateWorkspace method
            a = 3;

            set(ax, 'xlim', [-a, a], 'ylim', [-a, a], 'zlim', [-a, a]);
            
            xlabel(ax, 'X');
            ylabel(ax, 'Y');
            zlabel(ax, 'Z');
            
            % set up view
            view(ax, [135 8]);
            set(ax, 'Projection', 'perspective');
            ax.CameraViewAngle = 8.0;
            ax.Tag = 'Primary';
            grid(ax, 'on');
            
            % set up lighting
            if isempty(findobj(ax,'Type','Light'))
                light('Position',[a, 0, a],'Style','local','parent',ax);
            end

            ax.Visible = 'on';
            
        end
    end

    methods (Static, Access = private)
        function [ F, V, C ] = bodyFrameMesh( r, l, N, isFixed )
            %BODYFRAMEMESH Create body frame mesh data. The data will be used as input
            %   for patch command.
            %   r - radius of each axis
            %   l - length of each axis
            %   N - number of side surfaces of the prism (used to approximate axis
            %       cylinder)
            %   isFixed - whether fixed body or not
            %
            %   F - Faces (array of vertex indices)
            %   V - Vertex (3D point)
            %   C - Color
            
            theta = linspace(0, 2*pi,N+1)';
            
            m = length(theta);
            d = 0.9;
            
            % z-axis
            Vz = [r*cos(theta), r*sin(theta), theta*0];
            
            if isFixed
                Vz = [Vz;Vz;Vz];
                Vz(m+1:2*m,3) = d*l*ones(m, 1);
                Vz(2*m+1:3*m, 3) = l*ones(m,1);
            else
                Vz = [Vz;Vz];
                Vz(m+1:2*m,3) = l*ones(m, 1);
            end
            
            
            Fz = [];
            for i = 1:m-1
                f = [i, i+1, m+i;
                    m+i, i+1, m+i+1];
                Fz = [Fz; f ]; %#ok<AGROW>
            end
            lf1 = size(Fz,1);
            
            if isFixed
                for i = m+1:2*m-1
                    f = [i, i+1, m+i;
                        m+i, i+1, m+i+1];
                    Fz = [Fz; f ]; %#ok<AGROW>
                end
                
                for i = (2*m+2):(3*m-2)
                    f = [i,i+1,3*m];
                    Fz = [Fz; f];  %#ok<AGROW>
                end
            end
            
            
            % y-axis
            Vy = [r*cos(theta), theta*0, r*sin(theta)];
            if isFixed
                Vy = [Vy;Vy;Vy];
                Vy(m+1:2*m,2) = d*l*ones(m, 1);
                Vy(2*m+1:3*m, 2) = l*ones(m,1);
            else
                Vy = [Vy;Vy];
                Vy(m+1:2*m,2) = l*ones(m, 1);
            end
            
            % x-axis
            Vx = [theta*0, r*cos(theta), r*sin(theta)];
            if isFixed
                Vx = [Vx;Vx;Vx];
                Vx(m+1:2*m, 1) = d*l*ones(m, 1);
                Vx(2*m+1:3*m, 1) = l*ones(m, 1);
            else
                Vx = [Vx;Vx];
                Vx(m+1:2*m, 1) = l*ones(m, 1);
            end
            
            % assemble
            V = [Vz; Vy; Vx];
            if isFixed
                Fy = 3*m + Fz;
                Fx = 6*m + Fz;
            else
                Fy = 2*m + Fz;
                Fx = 4*m + Fz;
            end
            F = [Fz; Fy; Fx];
            
            lf = length(Fz);
            if isFixed
                lf2 = lf - lf1;
                
                % fixed frame color scheme
                C =  [repmat([1,0,1],lf1,1);
                    repmat([0,0,1],lf2,1);
                    repmat([1,0,1],lf1,1);
                    repmat([0,1,0],lf2,1);
                    repmat([1,0,1],lf1,1);
                    repmat([1,0,0],lf2,1)];
            else
                
                % non-fixed frame color scheme
                C =  [repmat([0,0,1],lf,1);
                    repmat([0,1,0],lf,1);
                    repmat([1,0,0],lf,1)];
            end
            
            F = robotics.core.internal.PrimitiveMeshGenerator.flipFace(F);
        end

    end
end