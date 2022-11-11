function [w] = euler2omega_inertial(euler, eulerRate)
    %OMEGA2EULER_INERTIAL Transform euler rates to angular velocities in INERTIAL frame
    %   euler: rpy
    %   w: [wx; wy; wz]. In INERTIAL frame
    
    
    % In Inertial frame
    sy = sin(euler(2));
    cy = cos(euler(2));
    sz = sin(euler(3));
    cz = cos(euler(3));

    RotM = [cz*cy,      -sz,    0;
            sz*cy,      cz,     0;
            -sy,        0,      1];   

    % Conversion
    w = RotM * eulerRate;
end
