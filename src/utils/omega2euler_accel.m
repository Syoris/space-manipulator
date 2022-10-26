function psi_ddot = omega2euler_accel(psi, psi_dot, wb_dot)
% Converts angular accelartion (in body frame) to euler angle acceleration
%
%   psi: Euler angles (ZYX convention)
%   psi_dot: Euler rates
%   wb_dot: Body angular accelration, in local frame

    % In body frame
    sx = sin(psi(1));
    cx = cos(psi(1));
    sy = sin(psi(2));
    cy = cos(psi(2));

    RotM_inv = [1,  sy*sx/cy,   sy*cx/cy;
                0,  cx,         -sx;
                0,  sx/cy,      cx/cy];

    Rx = [0,    0,      0;
          0,    -sx,    cx*cy;
          0,    -cx,     -sx*cy];

    Ry = [0,    0,  -cy;
          0,    0,  -sx*sy;
          0,    0,  -cx*sy];
    
    R_dot = psi_dot(1)*Rx + psi_dot(2)*Ry;

    psi_ddot = RotM_inv*(wb_dot - R_dot*psi_dot);

end