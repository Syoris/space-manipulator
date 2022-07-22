classdef Joint
    properties
        Name
        Id
        Type
        Q_id        % Active joint id (-1 if fixed)
        ParentLink
        ChildLink
        Axis
        T1          %  Homogeneous transformation matrix from parent link [4x4].
    end
end