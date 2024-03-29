classdef SpacecraftBase < Body
    % SpacecraftBase Object representing the spacecraft base
    %
    % Currently, RotM correponds to the rotation matrix between Base and Manipulator anchor point

    properties
        R % Base position wrt parent frame    [Rx; Ry; Rz]
        Phi % Base rotation wrt parent frame    [r; p; y]
        R_dot % Base translational vel            [Rx_dot; Ry_dot; Rz_dot]
        Omega % Base angular vel                  [wx; wy; wz]

        HomeConf % Base home config                  [Rx; Ry; Rz; r; p; y]

        BaseToParentTransform_symb % Transform from base to inertial frame in symbolic. Initialize in SpaceRobot.initDyn().
        BaseToParentTransform % Transform from base to inertial frame
        ManipToBaseTransform % Transform from manipulator first joint to base CoM

        P_base % Spacecraft twist propagation
    end

    methods

        function obj = SpacecraftBase(baseName)
            obj@Body(baseName);

            obj.Id = 0;

            obj.R = [0; 0; 0];
            obj.Phi = [0; 0; 0];
            obj.R_dot = [0; 0; 0];
            obj.Omega = [0; 0; 0];

            obj.HomeConf = zeros(6, 1);

            obj.BaseToParentTransform_symb = sym(zeros(3, 3));
            obj.ManipToBaseTransform = eye(4);

            obj.P_base = eye(6);
        end

    end

    % Internal functions
    methods

        function updateTransform(obj)

            if ~isempty(obj.Phi)
                rotM = rpy2r(obj.Phi.');
            else
                rotM = eye(3);
            end

            if ~isempty(obj.R)
                trans = obj.R;
            else
                trans = zeros(3, 1);
            end

            obj.BaseToParentTransform = [rotM, trans; zeros(1, 3), 1];
        end

        function initBase(obj)
            [R, L] = tr2rt(obj.ManipToBaseTransform);

            obj.ParentRotM = R;

            % A - Twist propagation matrix
            obj.A = [eye(3), -skew(L); zeros(3, 3), eye(3)];

            % P - Joint rate propagation matrix
            obj.P = eye(6);

            % M - Body mass matrix
            obj.M = [obj.Mass * eye(3), zeros(3, 3); zeros(3, 3), obj.InertiaM];

            % Rotation mat
            obj.RotM_symb = [obj.ParentRotM, zeros(3, 3); zeros(3, 3), obj.ParentRotM];

            if ~isempty(obj.Joint.SymbVar)
                obj.RotM_handle = matlabFunction(obj.RotM_symb, 'Vars', {obj.Joint.SymbVar});
            end

        end

    end

    % Setter/Getter
    methods

        function set.R(obj, newR)
            validateattributes(newR, {'numeric'}, ...
                {'nonempty', 'size', [3, 1]}, 'SpacecraftBase', 'R');

            obj.R = newR;
            obj.updateTransform;
        end

        function set.Phi(obj, newPhi)
            validateattributes(newPhi, {'numeric'}, ...
                {'nonempty', 'size', [3, 1]}, 'SpacecraftBase', 'Phi');

            obj.Phi = newPhi;
            obj.updateTransform;
        end

        function set.R_dot(obj, newR_dot)
            validateattributes(newR_dot, {'numeric'}, ...
                {'nonempty', 'size', [3, 1]}, 'SpacecraftBase', 'R_dot');
            obj.R_dot = newR_dot;
        end

        function set.Omega(obj, newOmega)
            validateattributes(newOmega, {'numeric'}, ...
                {'nonempty', 'size', [3, 1]}, 'SpacecraftBase', 'Omega');
            obj.Omega = newOmega;
        end

        function set.HomeConf(obj, newHomeConf)
            validateattributes(newHomeConf, {'numeric'}, ...
                {'nonempty', 'size', [6, 1]}, 'SpacecraftBase', 'HomeConf');

            obj.HomeConf = newHomeConf;
        end

    end

end
