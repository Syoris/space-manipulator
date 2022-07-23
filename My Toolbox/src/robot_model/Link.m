classdef Link < handle
    properties
        Name
        Id              % Id in the spacecraft. -1 if not part of one.
        Joint           % Link joint
        
        Mass            % Link mass
        CenterOfMass    % CoM of link specified as 
        Inertia         % Link Inertia relative to the body frame. [Ixx Iyy Izz Iyz Ixz Ixy]. Unit: kilogram-meter-squared (kg*m^2)
        T               % Homogeneous transformation matrix from parent joint [4x4].
        
        Parent          % Link parent. Joint specifies how this link can move relative to parent
        ParentId
        Children        % Link children. Array of __
        
        Visuals
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
        
            obj.Visuals = cell(0, 0);
            
        end
    end
end