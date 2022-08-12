classdef SpacecraftBase < Link
    properties
       R                    % Base position wrt parent frame    [Rx; Ry; Rz]
       Phi                  % Base rotation wrt parent frame    [r; p; y]
       R_dot                % Base translational vel            [Rx_dot; Ry_dot; Rz_dot]
       Omega                % Base angular vel                  [wx; wy; wz]

       HomeConf             % Base home config                  [Rx; Ry; Rz; r; p; y]

       BaseToParentTransform_symb       % Transform from base to inertial frame in symbolic. Initialize in SpaceRobot.initMats().
       BaseToParentTransform            % Transform from base to inertial frame
    end

    methods
        function obj = SpacecraftBase(baseName)
            obj@Link(baseName);
            obj.R = [0; 0; 0];
            obj.Phi = [0; 0; 0];
            obj.R_dot = [0; 0; 0];
            obj.Omega = [0; 0; 0];

            obj.HomeConf = zeros(6, 1);

            obj.BaseToParentTransform_symb = sym(zeros(3, 3));
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
    end

    % Setter/Getter
    methods    
        function set.R(obj, newR)
            validateattributes(newR, {'numeric'},...
                {'nonempty', 'size', [3, 1]}, 'SpacecraftBase', 'R');

            obj.R = newR;
            obj.updateTransform;
        end

        function set.Phi(obj, newPhi)
            validateattributes(newPhi, {'numeric'},...
                {'nonempty', 'size', [3, 1]}, 'SpacecraftBase', 'Phi');

            obj.Phi = newPhi;
            obj.updateTransform;
        end

        function set.R_dot(obj, newR_dot)
            validateattributes(newR_dot, {'numeric'},...
                {'nonempty', 'size', [3, 1]}, 'SpacecraftBase', 'R_dot');
            obj.R_dot = newR_dot;
        end

        function set.Omega(obj, newOmega)
            validateattributes(newOmega, {'numeric'},...
                {'nonempty', 'size', [3, 1]}, 'SpacecraftBase', 'Omega');
            obj.Omega = newOmega;
        end

        function set.HomeConf(obj, newHomeConf)
            validateattributes(newHomeConf, {'numeric'},...
                {'nonempty', 'size', [6, 1]}, 'SpacecraftBase', 'HomeConf');
            
            obj.HomeConf = newHomeConf;
        end

    end
end