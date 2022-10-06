function [eulerRate] = omega2euler(euler, w)
    %OMEGA2EULER Transfert body (??) angular rate to euler rates
    %   euler: rpy
    %   w: [wx; wy; wz]. In body frame

    % In body frame
    sx = sin(euler(1));
    cx = cos(euler(1));
    sy = sin(euler(2));
    cy = cos(euler(2));

    %     RotM = [1,  0,  -sy;
    %             0,  cx, sx*cy;
    %             0,  -sx cx*cy];

    RotM_inv = [1, sy * sx / cy, sy * cx / cy;
            0, cx, -sx;
            0, sx / cy cx / cy];

    % % In Inertial frame
    % sy = sin(euler(2));
    % cy = cos(euler(2));
    % sz = sin(euler(3));
    % cz = cos(euler(3));

    % RotM_inv = [cz/cy,      sz/cy,      0;
    %             -sz,        cz,         0;
    %             sy*cz/cy,   sy*sz/cy,   1];
    % Conversion
    eulerRate = RotM_inv * w;
end
