classdef Joint < handle
    properties
        % Identificaiton
        Name
        Id
        Type                        % Either revolute or fixed
        Q_id                        % Active joint id (-1 if fixed)
        
        % Config
        SymbVar                         % Configuration symbolic variable associated with joint   
        Position                    % Current Joint Positition
        Speed                       % Current Joint Speed
        
        PositionLimits              % Position limits of the joint. Specified as [min max].
        HomePosition                % Joint home position

        % Kin
        ParentLink
        ChildLink
        Axis        
    end

    properties(SetAccess = private)
        JointToParentTransform      % Transfrom from joints to parent link
    end

    methods
        function obj = Joint(jntName, jntType)
            if nargin < 2
                error("Invalid parameters")
            end
            obj.Name = jntName;
            
            % Check type is valid
            if nargin == 2
                jntType = validatestring(jntType, ...
                    {'revolute', 'fixed'}, ...
                     'rigidBodyJoint','jntType');
            else
                jntType = 'fixed'; 
            end
            obj.Type = jntType;

            obj.Id = 0;
            obj.Q_id = -1;
            obj.SymbVar = '';
            obj.PositionLimits = [-pi, pi];

            obj.ParentLink = '';
            obj.ChildLink = '';
            obj.Axis = [0 0 1];
            
            obj.HomePosition = 0;
            obj.Position = 0;
            obj.Speed = 0;
            
            obj.JointToParentTransform = eye(4);
        end
        
        function T = transformLink2Parent(obj)
            % Find current config to initial config transform
            switch(obj.Type)
                case 'fixed'
                    TJ = eye(4);
                case 'revolute'
                    TJ = angvec2tr(obj.Position, obj.Axis);
                otherwise
                    error("Wrong Type")
            end
            T = obj.JointToParentTransform*TJ;
        end

        function T = transformLink2ParentSymb(obj)
            % Find current config to initial config transform
            switch(obj.Type)
                case 'fixed'
                    TJ = eye(4);
                case 'revolute'
                    TJ = angvec2tr(obj.SymbVar, obj.Axis);
                otherwise
                    error("Wrong Type")
            end
            T = obj.JointToParentTransform*TJ;
        end

    end

    % Setter/Getter
    methods
        function setFixedTransform(obj, input)
            %setFixedTransform Set fixed transform properties of joint
            %   Note that for a revolute joint, theta is a joint variable
            %   and thus not considered part of the joint's fixed transform.
            %   Similarly, for a prismatic joint, d is a joint variable.
            %   Joint variables in DH/MDH parameters will be ignored during
            %   setFixedTransform calls. Check the methods in RigidBodyTree
            %   class (such as getTransform) to see how joint variables are
            %   specified.
            %
            %   setFixedTransform(JOINT, T) sets JointToParentTransform to
            %   the user-supplied 4x4 homogeneous transform matrix T and 
            %   ChildToJointTransform to an identity matrix.
            %
            validateattributes(input, {'double'},...
                {'nonnan', 'finite', 'real','nonempty', 'size',[4, 4]}, ...
                 'setFixedTransform', 'input'); 
            if ~isequal(double(input(4,:)),[0 0 0 1])
                error("Last row of Homogeneous Transfrom matrix is invalid");
            end

            obj.JointToParentTransform = double(input);
        end

        function set.Position(obj, newPosition)
            if newPosition > obj.PositionLimits(2)
                obj.Position = obj.PositionLimits(2);
                obj.Speed = 0;
            elseif newPosition < obj.PositionLimits(1)
                obj.Position = obj.PositionLimits(1);
                obj.Speed = 0;
            else
                obj.Position = newPosition;
            end
        end
    end
end