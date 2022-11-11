function psi_ddot = omega2euler_accel_inertial(psi, psi_dot, wb_dot)
% Tranform angular accelartion (in INERTIAL FRAME) to psi angle acceleration
%
%   psi: Euler angles (ZYX convention)
%   psi_dot: Euler rates
%   wb_dot: Body angular accelration, in INERTIAL frame

    % In body frame
    sy = sin(psi(2));
    cy = cos(psi(2));
    sz = sin(psi(3));
    cz = cos(psi(3));

    RotM_inv = [cz/cy,      sz/cy,      0;
                -sz,        cz,         0;
                sy*cz/cy,   sy*sz/cy,   1];

    Ry = [-cz*sy,   0,  0;
          -sz*sy,   0,  0;
          -cy,      0,  0];

    Rz = [-sz*cy,   -cz,    0;
          cz*cy,    -sz,    0;
          0,        0,      0];
    
    R_dot = psi_dot(2)*Ry + psi_dot(3)*Rz;

    psi_ddot = RotM_inv*(wb_dot - R_dot*psi_dot);
end