classdef SpacecraftBase < Link
    properties
       BasePosition         % Base position wrt parent frame, [x, y, z]
       BaseRot              % Base rotation wrt parent frame, [r, p, y]
       BaseTSpeed           % Base translational vel [x_dot, y_dot, z_dot]
       BaseASpeed           % Base angular vel [wx, wy, wz]

    end
    properties(SetAccess=private)
        BaseToParentTransform
    end

    methods
        function obj = SpacecraftBase(baseName)
            obj@Link(baseName);
            obj.BasePosition = [0, 0, 0];
            obj.BaseRot = [0, 0, 0];
        end
    end
    % Internal functions
    methods        
        function updateTransform(obj)
            if ~isempty(obj.BaseRot)
                rotM = rpy2r(obj.BaseRot);
            else
                rotM = eye(3);
            end
            
            if ~isempty(obj.BasePosition)
                trans = obj.BasePosition';
            else
                trans = zeros(3, 1);
            end

            obj.BaseToParentTransform = [rotM, trans; zeros(1, 3), 1];
        end
    end

    % Setter/Getter
    methods
        function set.BasePosition(obj, newPosition)
            obj.BasePosition = newPosition;
            obj.updateTransform;
        end

        function set.BaseRot(obj, newRot)
            obj.BaseRot = newRot;
            obj.updateTransform;
        end

        function set.BaseTSpeed(obj, newTSpeed)
            obj.BaseTSpeed = newTSpeed;
        end

        function set.BaseASpeed(obj, newASpeed)
            obj.BaseASpeed = newASpeed;
        end

        
    end
end