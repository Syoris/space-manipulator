function [w] = euler2omega_local(euler, eulerRate)
    %OMEGA2EULER_LOCAL Transform euler rates to angular velocities in LOCAL frame
    %   euler: rpy
    %   w: [wx; wy; wz]. In LOCAL BODY frame
    

    % In body frame
    sx = sin(euler(1));
    cx = cos(euler(1));
    sy = sin(euler(2));
    cy = cos(euler(2));

    RotM = [1,      0,      -sy;
            0,      cx,     sx*cy;
            0,      -sx,    cx*cy];
    
    % Conversion
    w = RotM * eulerRate;
end
