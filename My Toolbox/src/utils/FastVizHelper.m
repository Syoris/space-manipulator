classdef FastVizHelper < handle
%FASTVIZHELPER Fast visualization helper for SpaceRobot
%   This class acts as an interface between the figure axes, the
%   HGTransform objects, and the SpaceRobot class.
    properties
        %HGTransforms - Cell array of HG Transforms
        HGTransforms

        %RBTLineData - (N+1)-by-2 Cell array of line data
        %   The RBTLineData cell array has N+1 rows (for the N rigid bodies
        %   and base in the associated RBT) and 2 columns. The first column
        %   is a cell array of line handles, and the second column is a
        %   cell array of child body indices. For the ith row, the cell
        %   array in the first column contains all the lines that start at
        %   the rigidBody with index i. The second column contains the
        %   indices of all the child bodies to which these lines connect.
        %   For the base, the index N+1 is used. This data is used when
        %   lines are updated during visualization updates.
        RBTLineData

        %BaselineConfig - Configuration of the rigidBodyTree when it is first plotted and hgTransforms are initialized
        BaselineConfig

        %AxesHandle - Handle to the figure axes
        AxesHandle

        %IsUpdated - Flag to check if properties have been initialized
        IsUpdated
        
        %MeshAndFrameOptions - Array to store mesh and frame options
        %   3-by-1 double array to hold options for collision
        %   meshes, visual meshes or frames in that order. Value '1' 
        %   corresponds to 'true' and '0' to 'false'. Used to check if the 
        %   options have changed
        MeshAndFrameOptions
    end

    methods
        function obj = FastVizHelper()
            obj.IsUpdated = false;
            obj.MeshAndFrameOptions = [0, 1, 1];
        end

        function fastUpdate(obj, scObj, parent, collisions, visuals, frames)
        %fastUpdate Update position and orientation of HGTransforms, Lines          
            displayCollisions = strcmpi(collisions,'on');
            displayVisuals = strcmpi(visuals,'on');
            displayFrames = strcmpi(frames,'on');

            %Determine if reinitialization needed
            obj.checkIfReinitializationRequired(scObj, parent, [displayCollisions, displayVisuals, displayFrames]);
            obj.MeshAndFrameOptions = [displayCollisions, displayVisuals, displayFrames];
            
            if ~obj.IsUpdated
                obj.initializeAxesChildrenAndTransforms(scObj, parent, collisions, visuals, frames);
                obj.BaselineConfig = obj.kinStruct2Cell(scObj, scObj.Ttree);
                obj.IsUpdated = true;

            else
                tTree = obj.kinStruct2Cell(scObj, scObj.Ttree);
                RobotVizHelper.fastVisualizationUpdate(obj.HGTransforms, obj.RBTLineData, tTree, obj.BaselineConfig);
            end
        end
    end

    methods (Access = private)

        function initializeAxesChildrenAndTransforms(obj, scObj, parent, collisions, visuals, frames)
        %initializeAxesChildrenAndTransforms Initialize Axes, plot objects HGTransform array, LineData
            [obj.AxesHandle, axesObjects] = showSimple(scObj, parent, collisions, false, visuals, frames);
            obj.HGTransforms = RobotVizHelper.addHGTransforms(axesObjects, obj.AxesHandle);
            obj.RBTLineData = axesObjects(:,3:4);

            grid(obj.AxesHandle, 'on');
            % rotate3d(obj.AxesHandle, 'off');
        end

        function checkIfReinitializationRequired(obj, scObj, parent, newMeshAndFrameOptions)
        %checkIfReinitializationRequired Check if HGTransform reinitialization required
        %   Check if there is a need to reinitialize HGTransforms which may
        %   be if there is any change to the SpaceRobot or any 
        %   visualization options or if either axis, parent or HGTransforms
        %   are not defined properly
        %   Change IsUpdated flag. 

            if obj.IsUpdated == false
                return;
            else
                %if the collisions, visuals or frames option has changed
                haveMeshFrameOptionsChanged = any(obj.MeshAndFrameOptions ~= newMeshAndFrameOptions);
                
                %if "Parent" is defined and Parent is not equal to
                %obj.AxesHandle
                isNewParent = ~isempty(parent) && ~isequal(parent, obj.AxesHandle);

                %If HGtransforms/ axes has been deleted
                isDeletedHGT = isempty(obj.HGTransforms) || ~isvalid(obj.HGTransforms{1});

                %if a parent is not defined and the gca doesn't match the
                %obj.AxesHandle
                isAxesChanged = isempty(parent) && ~isequal(gca, obj.AxesHandle);

                %If axis is undefined or invalid
                isAxesUndefined = isempty(obj.AxesHandle) || ~isvalid(obj.AxesHandle);
                
                %If bodies have been added or removed
                %Number of HGTransforms equals the number of bodies +1
                hasTreeSizeChanged = ~isequal(scObj.NumLinks+2, size(obj.HGTransforms, 1));
                
                %Needs re-initialization when IsUpdated is false
                obj.IsUpdated = ~(isAxesUndefined || isNewParent || isDeletedHGT || isAxesChanged || haveMeshFrameOptionsChanged || hasTreeSizeChanged);
            end
        end
    end

    methods(Static)
        function tTree = kinStruct2Cell(scObj, treeStruct)
            % tTree: 1x(N+1) Homogeneous transform matrix cell array.
            % ith col -> for link i
            % last col -> for base
            
            tTree = cell(1, scObj.NumLinks + 1);
            tTree{end} = treeStruct.(scObj.BaseName);

            for i=1:scObj.NumLinks
                tTree{i} = treeStruct.(scObj.findLinkNameByIdx(i));
            end            
        end
    end

    

end
