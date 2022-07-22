classdef Link
    properties
        Name
        Id
        ParentJoint     % Parent joint id.
        T               % Homogeneous transformation matrix from parent joint [4x4].
        Mass
        Inertia         % Link’s inertia matrix [3x3], projected in the body-fixed link’s CCS.
        Visuals
    end
end