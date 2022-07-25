classdef SpacecraftBase < Link
    properties
       BasePosition         % Base position wrt parent frame, [x, y, z]
       BaseRot              % Base rotation wrt parent frame, [r, p, y]

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
                rotM = eul2rotm(obj.BaseRot);
            else
                rotM = eye(3);
            end
            
            if ~isempty(obj.BasePosition)
                trans = obj.BasePosition';
            else
                trans = zeros(3, 1);
            end

            obj.BaseToParentTransform = [rotM, trans; zeros(1, 4)];
        end
    end

    % Setter/Getter
    methods
        function set.BasePosition(obj, newPosition)
            obj.BasePosition = newPosition;
            obj.updateTransform;
        end

        function set.BaseRot(obj, newRot)
            obj.BaseRot  = newRot;
            obj.updateTransform;
        end
        
    end
end