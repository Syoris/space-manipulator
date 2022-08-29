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
        %JointToParentTransform The first fixed transform property of a joint 
        %   JointToParentTransform represents the homogeneous transform
        %   that converts points originally expressed in the joint predecessor
        %   frame to the parent body frame. This property is read-only.
        %
        %   Note joint predecessor frame is fixed on the parent body.
        %
        %   Default: eye(4)  
        JointToParentTransform      % Transfrom from joints to parent link

        %ChildToJointTransform The second fixed transform property of a joint 
        %   ChildToJointTransform represents the homogeneous transform that
        %   converts points originally expressed in the child body frame to
        %   the joint successor frame. This property is read-only. 
        %
        %   Note the rigid body who owns the joint is the child body in this
        %   context. And the joint successor frame is fixed on the child
        %   body.
        %   
        %   Default: eye(4)  
        ChildToJointTransform
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

            % obj.ParentLink = '';
            obj.ChildLink = '';
            obj.Axis = [0 0 1];
            
            obj.HomePosition = 0;
            obj.Position = 0;
            obj.Speed = 0;
            
            obj.JointToParentTransform = eye(4);
            obj.ChildToJointTransform = eye(4);
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

            TP = obj.ParentLink.Joint.ChildToJointTransform; % Transform from previous joint to current joint
            
            % T = obj.JointToParentTransform*TJ*obj.ChildToJointTransform;
            T = TP * obj.JointToParentTransform * TJ;
            
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
            T = obj.JointToParentTransform*TJ*obj.ChildToJointTransform;
        end

    end

    % Setter/Getter
    methods
        function setFixedTransform(obj, input, notation)
            %setFixedTransform Set fixed transform properties of joint
            %   Note that for a revolute joint, theta is a joint variable
            %   and thus not considered part of the joint's fixed transform.
            %
            %   setFixedTransform(JOINT, T) sets JointToParentTransform to
            %   the user-supplied 4x4 homogeneous transform matrix T and 
            %   ChildToJointTransform to an identity matrix.
            %
            %   setFixedTransform(JOINT, DHPARAMS, 'dh') sets 
            %   ChildToJointTransform using Denavit-Hartenberg parameters
            %   DHPARAMS and JointToParentTransform to an identity matrix. 
            %   DHPARAMS are given in the order [a alpha d theta].
            %
            %   NOTE: The first joint (the one connected to the base) JointToParentTransform will be
            %         set when adding the joint to the SpaceRobot using `addLink`
            
            narginchk(2,3);

            if nargin < 3
                notation = 'matrix';
            end

            switch(notation)
                case 'dh'
                    extractFixedTransformFromDH(obj, input); 

                case 'matrix'                    
                    validateattributes(input, {'double'},...
                        {'nonnan', 'finite', 'real','nonempty', 'size',[4, 4]}, ...
                         'setFixedTransform', 'input'); 
                    if ~isequal(double(input(4,:)),[0 0 0 1])
                        error("Last row of Homogeneous Transfrom matrix is invalid");
                    end

                    obj.JointToParentTransform = double(input);
                    obj.ChildToJointTransform = eye(4);

                otherwise
                    error("Wrong Type")
            end                               
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

        % function set.ParentLink(obj, parentLink)
        %     obj.ParentLink = parentLink;

        %     % If parent is base, set JointToParentTransform to ManipToBaseTransform
        %     if obj.ParentLink.Id == 0
        %         obj.JointToParentTransform = obj.ParentLink.ManipToBaseTransform;
        %     end
        % end
    end

    methods (Access = private) 
        function extractFixedTransformFromDH(obj, dhparams)
            %extractFixedTransformFromDH 
            validateattributes(dhparams, {'double'}, { 'nonnan', ...
                'finite', 'real', 'nonempty', 'vector',...
                'numel', 4}, 'extractFixedTransformFromDH', 'dhparams'); 
            
            a = dhparams(1);
            alpha = dhparams(2);
            d = dhparams(3);
            theta = dhparams(4);

            Ta = [eye(3), [a, 0, 0]'; [0, 0, 0, 1]];
            Talpha = [1, 0, 0, 0; 0, cos(alpha), -sin(alpha), 0;...
                        0, sin(alpha), cos(alpha), 0; 0, 0, 0, 1];
            Td = [eye(3), [0, 0, d]'; [0, 0, 0, 1]];
            Ttheta = [cos(theta), -sin(theta), 0, 0; ...
                   sin(theta), cos(theta), 0, 0; 0, 0, 1, 0;0, 0, 0, 1];

            TL = eye(4);
            switch(obj.Type)
                case 'revolute'
                    TL = Td*Ta*Talpha; 
                case 'fixed'
                    TL = Ttheta*Td*Ta*Talpha; 
                otherwise
                    error("Wrong Type");
            end

            obj.ChildToJointTransform = TL;  
            obj.JointToParentTransform = eye(4); 

        end
    end
end