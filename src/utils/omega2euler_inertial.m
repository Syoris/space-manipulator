function [eulerRate] = omega2euler_inertial(euler, w)
    %OMEGA2EULER_INERTIAL Transform angular rate expressed in INERTIAL FRAME to euler rates
    %   euler: rpy
    %   w: [wx; wy; wz], in INERTIAL FRAME
    
    sy = sin(euler(2));
    cy = cos(euler(2));
    sz = sin(euler(3));
    cz = cos(euler(3));

    RotM_inv = [cz/cy,      sz/cy,      0;
                -sz,        cz,         0;
                sy*cz/cy,   sy*sz/cy,   1];

    eulerRate = RotM_inv * w;
end
