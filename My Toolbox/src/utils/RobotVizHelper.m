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

    properties (SetAccess = private)
        magicLength % magicLength, found to produce good visual for most common robots
        r
        l
    end

    methods

        function obj = RobotVizHelper(robot)
            obj.robot = robot;

            obj.magicLength = 1.1;
            obj.r = 0.005 * obj.magicLength; % axis arrow radius
            obj.l = 15 * obj.r; % axis arrow length

        end

        function [ax, bodyDisplayObjArray, fmanager] = drawRobot(obj, ax, Ttree, displayFrames, displayVisuals)
            %drawRobotMemoryLess Display robot and output handles to the figure patches and lines
            %   This method draws the robot and provides a cell array
            %   output that contains the handles to the patches and lines
            %   used to draw the robot.
            %
            %   The output cell array has dimension (N+2)x4. The dimensions correspond to the N
            %   Body. Base at (N+1)th row, Inertial frame at (N+2)th row.
            %
            %   The first three columns of each cell array correspond to
            %   the handles to the associated objects for each body: visual
            %   patches, frame patches, and connecting lines, while the
            %   last column contains the child bodies corresponding to each
            %   of the connecting lines at that index (the parent body for
            %   each line is already given by the column index).

            % Configure the figure manager for the given axes
            initBannerWidgets = true;
            enableClearInfoOnMousePress = true;
            figManagerOpts = struct( ...
                'InitializeBannerWidgets', initBannerWidgets, ...
                'EnableClearInfoOnMousePress', enableClearInfoOnMousePress);

            % Configure the figure manager for the given axes
            fmanager = FigureManager(ax, figManagerOpts);
            fmanager.createCornerCoordinateFrame(ax);

            % Initialize output
            bodyDisplayObjArray = cell(obj.robot.NumBodies + 2, 4);

            obj.depositRobotShowTag(obj.robot, ax.Parent);

            % frame visibility
            if displayFrames
                vis = 'on';
            else
                vis = 'off';
            end

            % Draw intertial frame ([0, 0, ,0])
            [F, V] = obj.bodyFrameMesh(obj.r * 0.6, obj.l * 1.3, obj.NumTessellation, true);
            bodyDisplayObjArray{end, 2} = patch(ax, ...
                'Faces', F, ...
                'Vertices', V, ...
                'FaceColor', 'k', ...
                'LineStyle', 'none', ...
                'Visible', vis, ...
                'Tag', obj.robot.ShowTag, ...
                'DisplayName', 'inertialFrame');

            % Draw base frame
            [F, V, C] = obj.bodyFrameMesh(obj.r * 0.6, obj.l * 1.3, obj.NumTessellation, false);
            V = Ttree.(obj.robot.BaseName) * [V'; ones(1, length(V))];
            V = V(1:3, :)';

            hasVisuals = ~isempty(obj.robot.Base.Visuals);

            % Base frame patches
            bodyDisplayObjArray{end - 1, 2} = patch(ax, ...
            'Faces', F, ...
                'Vertices', V, ...
                'FaceVertexCData', C, ...
                'FaceColor', 'flat', ...
                'LineStyle', 'none', ...
                'Visible', vis, ...
                'Tag', obj.robot.ShowTag, ...
                'DisplayName', obj.robot.BaseName);
            % , ...
            % 'ButtonDownFcn', {@robotics.manip.internal.RigidBodyTreeVisualizationHelper.infoDisplay, robot, robot.NumBodies+1, Ttree{end}}, ...
            % 'UIContextMenu', robotics.manip.internal.RigidBodyTreeVisualizationHelper.getBodyContextMenu(robot.ShowTag, ax, robot.BaseName, hasVisuals, displayVisuals), ...
            % 'DeleteFcn', @robotics.manip.internal.RigidBodyTreeVisualizationHelper.deleteUIContextMenuUponPatchDestruction);

            % draw regular body frames and connectors
            [F, V, C] = obj.bodyFrameMesh(obj.r, obj.l, obj.NumTessellation, false); % Get free meshes
            [Ff, Vf, Cf] = obj.bodyFrameMesh(obj.r, obj.l, obj.NumTessellation, true); % Get fixed meshes

            for i = 1:obj.robot.NumBodies
                % Find body meshes
                if strcmp(obj.robot.Bodies{i}.Joint.Type, 'fixed')
                    Vi = Vf;
                    Fi = Ff;
                    Ci = Cf;
                else
                    Vi = V;
                    Fi = F;
                    Ci = C;
                end

                bodyName = obj.robot.BodyNames{i};

                % Update body meshes position
                Vi = Ttree.(bodyName) * [Vi'; ones(1, length(Vi))];
                Vi = Vi(1:3, :)';

                hasVisuals = ~isempty(obj.robot.Bodies{i}.Visuals);

                % Body frame patches
                bodyDisplayObjArray{i, 2} = patch(ax, ...
                'Faces', Fi, ...
                    'Vertices', Vi, ...
                    'FaceVertexCData', Ci, ...
                    'FaceColor', 'flat', ...
                    'LineStyle', 'none', ...
                    'Visible', vis, ...
                    'Tag', obj.robot.ShowTag, ...
                    'DisplayName', obj.robot.BodyNames{i});

                pId = obj.robot.Bodies{i}.ParentId;

                if pId == 0
                    % If the parent is the base, the transform of the
                    % parent is the origin
                    Tparent = Ttree.(obj.robot.BaseName);
                else
                    Tparent = Ttree.(obj.robot.BodyNames{pId});
                end

                Tcurr = Ttree.(obj.robot.BodyNames{i});

                p = [Tparent(1:3, 4)'; Tcurr(1:3, 4)']; % Line starting and final points

                % Create the line and associate it with the parent object
                lineObject = line(ax, p(:, 1), p(:, 2), p(:, 3), 'Tag', obj.robot.ShowTag, 'Visible', vis);

                % Store the line object in the hgObject array. The lines
                % have to index to their parent bodies, since that is the
                % transform that will be associated with the line motion.
                if pId == 0
                    % Base bodies, which have the origin as the parent, are
                    % stored last
                    lineIndex = obj.robot.NumBodies + 1;
                else
                    % All other lines are indexed by the parent
                    lineIndex = pId;
                end

                % Since the line is associated with its parent, a frame
                % can be associated with multiple lines, which are stored
                % in a cell array. Index the line by its parent index. The
                % corresponding child body index is stored in the fourth
                % column of the bodyDisplayObjArray cell array.
                if isempty(bodyDisplayObjArray{lineIndex, 3})
                    % Create the cell array if it does not yet exist
                    bodyDisplayObjArray{lineIndex, 3} = {lineObject};
                    bodyDisplayObjArray{lineIndex, 4} = {i};
                else
                    % Add the line to the cell array if it already exists
                    bodyDisplayObjArray{lineIndex, 3} = [bodyDisplayObjArray{lineIndex, 3}; {lineObject}];
                    bodyDisplayObjArray{lineIndex, 4} = [bodyDisplayObjArray{lineIndex, 4}; {i}];
                end

            end

            % body visuals visibility
            if displayVisuals
                vis = 'on';
            else
                vis = 'off';
            end

            for i = 1:obj.robot.NumBodies + 1
                % Find transform
                if i > obj.robot.NumBodies
                    visGeom = obj.robot.Base.Visuals;
                    bodyName = obj.robot.BaseName;
                else
                    visGeom = obj.robot.Bodies{i}.Visuals;
                    bodyName = obj.robot.BodyNames{i};
                end

                T = Ttree.(bodyName);

                % Draw all visuals of current body
                meshPatchCell = cell(length(visGeom), 1);

                for k = 1:length(visGeom)
                    V = visGeom{k}.Vertices;
                    F = visGeom{k}.Faces;
                    TVis = visGeom{k}.Tform;
                    color = visGeom{k}.Color;

                    numVertices = size(V, 1);
                    Vi = T * TVis * [V'; ones(1, numVertices)];
                    Vi = Vi(1:3, :)';

                    meshPatchCell{k} = patch(ax, ...
                        'Faces', F, ...
                        'Vertices', Vi, ...
                        'FaceColor', color(1:3), ...
                        'LineStyle', 'none', ...
                        'DisplayName', [bodyName '_mesh'], ...
                        'Tag', obj.robot.ShowTag, ...
                        'Visible', vis);
                end

                % Base and body visual patches
                bodyDisplayObjArray{i, 1} = meshPatchCell;
            end

        end

        function resetScene(obj, ax)
            % PRIMARY axes
            axis(ax, 'vis3d');
            hFigure = ax.Parent;
            pan(hFigure, 'on');
            rotate3d(hFigure, 'on');

            ax.Visible = 'off';
            daspect(ax, [1 1 1]);

            %TODO estimate 'a' value, axis range
            a = 3;

            set(ax, 'xlim', [-a, a], 'ylim', [-a, a], 'zlim', [-a, a]);

            xlabel(ax, 'X');
            ylabel(ax, 'Y');
            zlabel(ax, 'Z');

            % set up view
            view(ax, [45 60]); % [135, 8]
            set(ax, 'Projection', 'perspective');
            % ax.view(0, 90); % Set view to above
            ax.CameraViewAngle = 8.0;
            ax.Tag = 'Primary';
            grid(ax, 'on');

            % set up lighting
            if isempty(findobj(ax, 'Type', 'Light'))
                light('Position', [a, 0, a], 'Style', 'local', 'parent', ax);
            end

            ax.Visible = 'on';

            ax.DeleteFcn = @RobotVizHelper.clearCornerAxes;
        end

    end

    methods (Static, Access = private)

        function clearCornerAxes(src, ~)
            %clearCornerAxes
            axArray = findall(src.Parent, 'Type', 'Axes', 'Tag', 'CornerCoordinateFrame');

            for idx = 1:numel(axArray)
                delete(axArray(idx));
            end

        end

        function [F, V, C] = bodyFrameMesh(r, l, N, isFixed)
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

            theta = linspace(0, 2 * pi, N + 1)';

            m = length(theta);
            d = 0.9;

            % z-axis
            Vz = [r * cos(theta), r * sin(theta), theta * 0];

            if isFixed
                Vz = [Vz; Vz; Vz];
                Vz(m + 1:2 * m, 3) = d * l * ones(m, 1);
                Vz(2 * m + 1:3 * m, 3) = l * ones(m, 1);
            else
                Vz = [Vz; Vz];
                Vz(m + 1:2 * m, 3) = l * ones(m, 1);
            end

            Fz = [];

            for i = 1:m - 1
                f = [i, i + 1, m + i;
                    m + i, i + 1, m + i + 1];
                Fz = [Fz; f]; %#ok<AGROW>
            end

            lf1 = size(Fz, 1);

            if isFixed

                for i = m + 1:2 * m - 1
                    f = [i, i + 1, m + i;
                        m + i, i + 1, m + i + 1];
                    Fz = [Fz; f]; %#ok<AGROW>
                end

                for i = (2 * m + 2):(3 * m - 2)
                    f = [i, i + 1, 3 * m];
                    Fz = [Fz; f]; %#ok<AGROW>
                end

            end

            % y-axis
            Vy = [r * cos(theta), theta * 0, r * sin(theta)];

            if isFixed
                Vy = [Vy; Vy; Vy];
                Vy(m + 1:2 * m, 2) = d * l * ones(m, 1);
                Vy(2 * m + 1:3 * m, 2) = l * ones(m, 1);
            else
                Vy = [Vy; Vy];
                Vy(m + 1:2 * m, 2) = l * ones(m, 1);
            end

            % x-axis
            Vx = [theta * 0, r * cos(theta), r * sin(theta)];

            if isFixed
                Vx = [Vx; Vx; Vx];
                Vx(m + 1:2 * m, 1) = d * l * ones(m, 1);
                Vx(2 * m + 1:3 * m, 1) = l * ones(m, 1);
            else
                Vx = [Vx; Vx];
                Vx(m + 1:2 * m, 1) = l * ones(m, 1);
            end

            % assemble
            V = [Vz; Vy; Vx];

            if isFixed
                Fy = 3 * m + Fz;
                Fx = 6 * m + Fz;
            else
                Fy = 2 * m + Fz;
                Fx = 4 * m + Fz;
            end

            F = [Fz; Fy; Fx];

            lf = length(Fz);

            if isFixed
                lf2 = lf - lf1;

                % fixed frame color scheme
                C = [repmat([1, 0, 1], lf1, 1);
                        repmat([0, 0, 1], lf2, 1);
                        repmat([1, 0, 1], lf1, 1);
                        repmat([0, 1, 0], lf2, 1);
                        repmat([1, 0, 1], lf1, 1);
                        repmat([1, 0, 0], lf2, 1)];
            else

                % non-fixed frame color scheme
                C = [repmat([0, 0, 1], lf, 1);
                        repmat([0, 1, 0], lf, 1);
                        repmat([1, 0, 0], lf, 1)];
            end

            F = robotics.core.internal.PrimitiveMeshGenerator.flipFace(F);
        end

        function depositRobotShowTag(robot, parent)
            %depositRobotShowTag
            name = FigureManager.RobotShowTagsIdentifier;
            retObj = getappdata(parent, name);

            if isempty(retObj)
                setappdata(parent, name, {robot.ShowTag});
            else
                tmp = [retObj, robot.ShowTag];
                setappdata(parent, name, unique(tmp));
            end

        end

    end

    methods (Static)

        function hgArray = addHGTransforms(bodyDisplayObjArray, parentAxes)
            %addHGTransforms Add HGTransforms to the figure objects
            %   The SpaceRobot figure display contains patches
            %   representing the frames, visual patches, and lines.
            %
            %   This method assigns hgTransform object associations to these objects
            %   so that they can be moved in a figure without needing to
            %   redefine them. The method accepts:
            %       - bodyDisplayObjArray
            %           An (N+2)x3 cell array corresponding to the N bodies of a
            %           SpaceRobot, the base and the inertial frame
            %
            %       - parentAxes
            %           Axes handle that specifies the parent of the hgtransform objects
            %
            %   The three columns contain cell arrays of visual patches, patch objects
            %   for each frame, and cell arrays of lines, respectively. The
            %   method assigns one hgTransform for each Body, and
            %   then assigns all the figure objects associated with that body
            %   as its children. For lines, which connect two bodies, the
            %   hgTransforms are not used since they are shared over all the
            %   objects and are only used to affect orientation, not scale (the
            %   issue of scale is unique to the lines in this application).

            hgArray = cell(size(bodyDisplayObjArray, 1), 1);

            for i = 1:size(bodyDisplayObjArray, 1)
                % For each Body, there is one associated
                % hgtransform object. The transform is explicitly parented
                % to the axes handle given as input.
                hgArray{i} = hgtransform(parentAxes);

                % Cell arrays of patches for each visual
                visualPatchArray = bodyDisplayObjArray{i, 1};

                for j = 1:length(visualPatchArray)
                    visualPatchArray{j}.Parent = hgArray{i};
                end

                % Patch objects for each frame
                framePatch = bodyDisplayObjArray{i, 2};

                if ~isempty(framePatch)
                    framePatch.Parent = hgArray{i};
                end

                % HGTransforms are not attached to lines because lines are
                % defined by their end points. Since this approach uses one
                % HGTransform per body (that controls the motion of all the
                % associated frames, meshes, etc.), HGTransforms can't also
                % be set to control line scale, because that would
                % adversely affect the scale of other items that we want to
                % keep the same size. As a result, lines are updated by
                % changing the endpoints, rather than by associating them
                % with an hgTransform.
            end

        end

        function fastVisualizationUpdate(hgDataArray, rbtLineData, Ttree, TtreeBaseline)

            % Update the HGTransforms
            RobotVizHelper.updateBodyHGTransforms(hgDataArray, Ttree, TtreeBaseline);

            hgArrayLength = size(hgDataArray, 1);
            TtreeLength = size(Ttree, 2);
            rbtLineDataLength = size(rbtLineData, 1);

            % Update the lines connecting the frames, if applicable (skip inertial frame)
            for i = 1:size(rbtLineData, 1) - 1
                bodyChildLines = rbtLineData{i, 1};
                bodyChildIndices = rbtLineData{i, 2};

                parentBodyTransform = Ttree{i};

                for j = 1:numel(bodyChildLines)
                    % Get the line
                    rigidBodyLine = bodyChildLines{j};

                    % Get the child transform
                    childBodyTransform = Ttree{bodyChildIndices{j}};

                    lineOrigin = parentBodyTransform(1:3, 4);

                    % Re-assign the line data points
                    rigidBodyLine.XData = [lineOrigin(1) childBodyTransform(1, 4)];
                    rigidBodyLine.YData = [lineOrigin(2) childBodyTransform(2, 4)];
                    rigidBodyLine.ZData = [lineOrigin(3) childBodyTransform(3, 4)];
                end

            end

        end

        function updateBodyHGTransforms(hgArray, Ttree, TtreeBaseline)
            %updateBodyHGTransforms Update HGTransform matrices
            %   This method accepts and N+2 cell array of HGTransform
            %   objects (corresponding to the N bodies of an associated
            %   SpaceRobot, the base and the inertial frame), a transform tree representing
            %   the poses of each of those bodies (in inertial frame), and a base
            %   transform tree, indicating the poses of each of those
            %   bodies at the time when the HGTransforms were initialized.
            %   The method updates the HGTransform matrices, which are 4x4
            %   homogeneous transform matrices, to represent the current
            %   desired positions given by Ttree. Note that the poses in
            %   Ttree are given with respect to inertial frame, which is defined
            %   with respect to the figure origin, whereas the HGTransform
            %   pose is defined with respect to the poses they were in when
            %   the HGTransform was initialized. These poses are defined by
            %   TtreeBaseline.

            hgArrayLength = size(hgArray, 1);
            TtreeLength = size(Ttree, 2);

            if hgArrayLength == TtreeLength
                % In RBT visualization support, the base can move, thus
                % Ttree and hgArray length are same.
                N = hgArrayLength;
            else
                % In the interactive use cases the rigid bodytree is fixed,
                % thus Ttree and hgArray length are different.
                N = hgArrayLength - 1;
            end

            % Iterate through the movable bodies and base
            for i = 1:N
                T = Ttree{i};
                % The HGTransforms are initialized in a position
                % other than the base, so the transform must be provided
                % with respect to that position. However, the rigidBodyTree
                % position is only known with respect to the origin, so it
                % is necessary to post multiply by the inverse of the
                % initial position of the rigidBodyTree (i.e. the position
                % it was in when the HGTransform was initialized).
                TBaseline = TtreeBaseline{i};
                hgMatrix = T * robotics.manip.internal.tforminv(TBaseline);

                % Update the HGTransformation matrix
                hgArray{i}.Matrix = hgMatrix;
            end

        end

    end

end
