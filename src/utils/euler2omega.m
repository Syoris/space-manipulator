function [w] = euler2omega(euler, eulerRate)
    %OMEGA2EULER Transform euler rates to angular velocities in body frame
    %   euler: rpy
    %   w: [wx; wy; wz]. In body frame

    % In body frame
    sx = sin(euler(1));
    cx = cos(euler(1));
    sy = sin(euler(2));
    cy = cos(euler(2));

    RotM = [1,      0,      -sy;
            0,      cx,     sx*cy;
            0,      -sx,    cx*cy];

%     % In Inertial frame
%     sy = sin(euler(2));
%     cy = cos(euler(2));
%     sz = sin(euler(3));
%     cz = cos(euler(3));
% 
%     RotM_inv = [cz/cy,      sz/cy,      0;
%                 -sz,        cz,         0;
%                 sy*cz/cy,   sy*sz/cy,   1];

    % Conversion
    w = RotM * eulerRate;
end
