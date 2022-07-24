classdef Joint < handle
    properties
        % Identificaiton
        Name
        Id
        Type                        % Either revolute or fixed
        Q_id                        % Active joint id (-1 if fixed)
        
        % Config
        Position                    % Current Joint Positition
        PositionLimits              % Position limits of the joint. Specified as [min max].
        HomePosition                % Joint home position

        % Kin
        ParentLink
        ChildLink
        Axis
        
        % Might add later
        % T1                          %  Homogeneous transformation matrix from parent link [4x4].
        

    end

    properties(SetAccess = private)
        JointToParentTransform      % Transfrom from joints to parent link
%         ChildToJointTransform
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
            obj.PositionLimits = [-pi, pi];

            obj.ParentLink = '';
            obj.ChildLink = '';
            obj.Axis = [0 0 1];
            obj.HomePosition = 0;
            obj.Position = 0;
            obj.JointToParentTransform = eye(4);

%             obj.T1 = eye(4);
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
%             obj.ChildToJointTransform = eye(4);
        end
    end
end