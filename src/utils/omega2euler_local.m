function [eulerRate] = omega2euler_local(euler, w)
    %OMEGA2EULER_LOCAL Transform angular rate expressed in local BODY FRAME to euler rates
    %   euler: rpy
    %   w: [wx; wy; wz], in BODY FRAME

    sx = sin(euler(1));
    cx = cos(euler(1));
    sy = sin(euler(2));
    cy = cos(euler(2));

    RotM_inv = [1,  sy*sx/cy,   sy*cx/cy;
                0,  cx,         -sx;
                0,  sx/cy,      cx/cy];
   
    % Conversion
    eulerRate = RotM_inv * w;
end
