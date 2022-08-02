function T = getTransform(obj, linkName1, linkName2)
%getTransform Get the transform between two body frames
%   T1 = getTransform(ROBOT, BODYNAME1) computes a
%   transform T1 that converts points originally expressed in
%   BODYNAME1 frame to be expressed in the robot's base frame.
%
%   T2 = getTransform(ROBOT, Q, BODYNAME1, BODYNAME2) computes
%   a transform T2 that converts points originally expressed in
%   BODYNAME1 frame to be expressed in BODYNAME2.

    narginchk(2,3);
    
    Ttree = obj.forwardKinematics(qvec);
    
    % 2-argument case: getTransform(ROBOT, linkName1)
    lId1 = findLinkIdxByName(obj, linkName1);
    if lId1 == 0
        T1 = eye(4);
    else
        T1 = Ttree{lId1};
    end
    
    T2 = eye(4);
    if nargin == 4
        % 4-argument case: getTransform(ROBOT, linkName1, linkName2)
        lId2 = findLinkIdxByName(obj, linkName2);
        if lId2 == 0
            T2 = eye(4);
        else
            T2 = Ttree{lId2};
        end
    end

    R = T2(1:3,1:3)';
    p = -R*T2(1:3,4);
    T = [R,p;[0 0 0 1]]*T1; % the first term is inv(T2)

end