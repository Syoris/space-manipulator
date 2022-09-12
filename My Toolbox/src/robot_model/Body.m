classdef Body < handle
    %Body To represent a rigid body.
    %
    %   body = Body('bodyName') creates a default rigid body
    %
    %   Body properties:
    %       Name                - Number of bodies
    %       Id                  - Cell array of rigid bodies
    %       Joint               - Base of the robot
    %       Mass                - Cell array of Names of rigid bodies
    %       CenterOfMass        - Name of robot base
    %       Inertia             -

    %   SetAccess=private
    %       Children            -
    %       Parent              -
    %       Visuals             -
    %       ParentId            -
    %       InertiaM            -
    %       MassM               - Body mass matrix
    %       ParentRotM          - Rotation matrix to parent body (R_i_i-1)
    %       A                   - Twist propagation matrix
    %       P                   - Joint rate propagation matrix
    %       Length              - Vector from body origin to body end position, in body frame

    %   Body methods:
    %       addVisual           - Add a visual to body
    %

    properties
        Name
        Id % Id in the appendage. -1 if not part of one.
        Joint % Body joint

        Mass % Body mass
        CenterOfMass % CoM of body specified as translation vector from frame origin [x, y, z]
        Inertia % Body Inertia relative to the body frame. [Ixx Iyy Izz Iyz Ixz Ixy]. Unit: kilogram-meter-squared (kg*m^2)

        Parent % Body parent. Joint specifies how this body can move relative to parent
        ParentId
        Children % Body children. Array of Bodies
    end

    properties % TODO: (SetAccess = private)
        Visuals % Body visual representation

        InertiaM % Body inertia matrix in body frame
        ParentRotM % Rotation matrix to parent body (R_i_i-1). In symbolic form.
        RotM % Rotation matrix to parent body: diag(R_i_i-1). Evaluated at current config.

        A % Twist propagation matrix, in frame i-1
        Length % Vector from body origin to body end position, in body frame
        P % Joint rate propagation matrix.

        M % Body mass matrix. [m_i*eye(3), zeros(3, 3); zeros(3, 3), InertiaM]
    end

    methods

        function obj = Body(bodyName)
            obj.Name = bodyName;
            obj.Id = -1; % Id in the spacecraft
            obj.ParentId = -1;
            obj.Joint = Joint(strcat(bodyName, '_jnt'), 'fixed');

            obj.Mass = 1;
            obj.CenterOfMass = [0, 0, 0];
            obj.Inertia = [1, 1, 1, 0, 0, 0];

            obj.Children = cell(1, 0);

            obj.Visuals = {};

            obj.ParentRotM = eye(4);
            obj.RotM = eye(6);
            obj.A = eye(6);
            obj.Length = zeros(3, 1);
            obj.P = zeros(6, 1);
            obj.M = zeros(6, 6);
        end

        function initBody(obj)
            %initBody Init body matrices: ParentRotM, A, P and M

            % Rotation matrix and parent length
            T = obj.Joint.transformBody2ParentSymb;
            [R, L] = tr2rt(T);
            obj.ParentRotM = R;
            obj.Parent.Length = double(L);

            % A - Twist propagation matrix
            obj.A = [eye(3), -skew(obj.Parent.Length); zeros(3, 3), eye(3)];

            % P - Joint rate propagation matrix
            obj.P = [zeros(3, 1); obj.Joint.Axis.'];

            % M - Body mass matrix
            obj.M = [obj.Mass * eye(3), zeros(3, 3); zeros(3, 3), obj.InertiaM];
        end

        function addVisual(obj, type, parameter, tform, color)
            %addVisual Add visual to a body body
            %   type: Geometry type (box, cylinder, sphere)
            %   parameter: Geometry parameter. Depends on type.
            %               box: [xl, yl, zl]
            %               cylinder: [radius, length]
            %               sphere: [radius]
            %   tform: Homogeneous matrix to body frame
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
                basicGeo.Scale = scale;
                obj.Visuals{end + 1} = basicGeo;
            end

        end

    end

    methods

        function set.Joint(obj, joint)
            obj.Joint = joint;
            joint.ChildBody = obj;
        end

        function set.CenterOfMass(obj, com)
            % Set CenterOfMass to com.
            validateattributes(com, {'numeric'}, ...
            {'nonempty', 'size', [1, 3]}, 'Body', 'CenterOfMass');

            obj.CenterOfMass = com;
        end

        function set.Inertia(obj, inertia)
            % Set CenterOfMass to com.
            validateattributes(inertia, {'numeric'}, ...
            {'nonempty', 'size', [1, 6]}, 'Body', 'Inertia');

            obj.Inertia = inertia;

            obj.InertiaM = [obj.Inertia(1), obj.Inertia(6), obj.Inertia(5);
                        obj.Inertia(6), obj.Inertia(2), obj.Inertia(4);
                        obj.Inertia(5), obj.Inertia(4), obj.Inertia(3)];
        end

        function rotM = get.RotM(obj)
            rotM_val = double(subs(obj.ParentRotM, obj.Joint.SymbVar, obj.Joint.Position));
            rotM = [rotM_val, zeros(3, 3); zeros(3, 3), rotM_val];
        end

    end

end
