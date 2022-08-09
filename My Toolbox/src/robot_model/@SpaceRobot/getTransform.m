function T = getTransform(obj, linkName1, linkName2)
%getTransform Get the transform between two link frames
%   T1 = getTransform(ROBOT, linkName1) computes a
%   transform T1 that converts points originally expressed in
%   linkName1 frame to be expressed in the robot's base frame.
%
%   T2 = getTransform(ROBOT, linkName1, linkName2) computes
%   a transform T2 that converts points originally expressed in
%   linkName1 frame to be expressed in linkName2.

    narginchk(2,3);
    
    tTree = obj.forwardKinematics();
    
    % 2-argument case: getTransform(ROBOT, linkName1)
    T1 = tTree.(linkName1);
    
    T2 = tTree.(obj.BaseName);
    % 4-argument case: getTransform(ROBOT, linkName1, linkName2)
    if nargin == 3
        T2 = tTree.(linkName2);
    end
    
    % Compute transform:
    %   T_1_2: frame 2 represented in frame 1
    %   T_b_1, T_b_2: Frames i repsrented in base frame
    %   T_1_2 = T_b_1^-1 * T_b_2
    R = T2(1:3,1:3)';
    p = -R*T2(1:3,4);
    T = [R,p;[0 0 0 1]]*T1; % the first term is inv(T2)

end