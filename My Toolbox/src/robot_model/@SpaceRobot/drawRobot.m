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