classdef Link < handle
    properties
        Name
        Id              % Id in the spacecraft. -1 if not part of one.
        Joint           % Link joint
        
        Mass            % Link mass
        CenterOfMass    % CoM of link specified as translation vector from frame origin [x, y, z]
        Inertia         % Link Inertia relative to the body frame. [Ixx Iyy Izz Iyz Ixz Ixy]. Unit: kilogram-meter-squared (kg*m^2)
        % T               % Homogeneous transformation matrix from parent joint [4x4].
        
        Parent          % Link parent. Joint specifies how this link can move relative to parent
        ParentId
        Children        % Link children. Array of __
        
        Visuals
    end

    properties(SetAccess=private)
        InertiaM        % Link inertia matrix in body frame
    end

    methods
        function obj = Link(linkName)
            obj.Name = linkName;
            obj.Id = -1;              % Id in the spacecraft
            obj.ParentId = -1;
            obj.Joint = Joint(strcat(linkName, '_jnt'), 'fixed');
            
            obj.Mass = 1;
            obj.CenterOfMass = [0, 0, 0];
            obj.Inertia = [1, 1, 1, 0, 0, 0];
        
            obj.Children = cell(1, 0);
        
            obj.Visuals = {};
            
        end

        function addVisual(obj, type, parameter, tform, color)
            %addVisual Add visual to a body link
            %   type: Geometry type (box, cylinder, sphere)
            %   parameter: Geometry parameter. Depends on type.
            %               box: [xl, yl, zl]
            %               cylinder: [radius, length]
            %               sphere: [radius]
            %   tform: Homogeneous matrix to link frame
            %   color: [r g b a]
            
            basicGeo = BasicGeometry();
            if nargin > 3
                basicGeo.Tform = tform;
            end
            if (nargin > 4) && ~isempty(color) % if color is specified, replace the default color
                basicGeo.Color = color;
            end

            switch type
                case 'Box'
                    [F, V] = robotics.core.internal.PrimitiveMeshGenerator.boxMesh(parameter);
                    scale = [1 1 1];
                    basicGeo.SourceData = {'box', parameter};
                case 'Cylinder'
                    [F, V] = robotics.core.internal.PrimitiveMeshGenerator.cylinderMesh(parameter);
                    scale = [1 1 1];
                    basicGeo.SourceData = {'cylinder', parameter};
                case 'Sphere'
                    [F, V] = robotics.core.internal.PrimitiveMeshGenerator.sphereMesh(parameter);
                    scale = [1 1 1];
                    basicGeo.SourceData = {'sphere', parameter};
            end

            if ~isempty(basicGeo.SourceData)
                basicGeo.Faces = F;
                basicGeo.Vertices = V;
                basicGeo.Scale=scale;
                obj.Visuals{end+1} = basicGeo;
            end
        end
            
    end

    methods
        function set.CenterOfMass(obj, com)
            % Set CenterOfMass to com.
            validateattributes(com, {'numeric'},...
            {'nonempty', 'size', [1, 3]}, 'Link', 'CenterOfMass');

            obj.CenterOfMass = com;
        end

        function set.Inertia(obj, inertia)
            % Set CenterOfMass to com.
            validateattributes(inertia, {'numeric'},...
            {'nonempty', 'size', [1, 6]}, 'Link', 'Inertia');

            obj.Inertia = inertia;
        end

        function inertiaM = get.InertiaM(obj)
            inertiaM = [obj.Inertia(1), obj.Inertia(6), obj.Inertia(5);
                        obj.Inertia(6), obj.Inertia(2), obj.Inertia(4);
                        obj.Inertia(5), obj.Inertia(4), obj.Inertia(3)];
        end
    end
end