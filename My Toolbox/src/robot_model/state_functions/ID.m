function h = ID(sr_info, q, q_dot, q_ddot)
    % ID  Compute Inverse Dynamics without depending on class object
    %
    %   Parameters
    %   - srInfo    SpaceRobot Infos. Struct
    %
    %   - q         Config
    %
    %   - q_dot     Speed
    %
    %   - q_ddot    Accels

    %% Initialization
    N = sr_info.N;
    n = sr_info.n;
    nk = sr_info.nk; % Number of bodies in the appendage

    qb = q(1:6);
    qm = q(7:end);

    qb_dot = q_dot(1:6);
    qm_dot = q_dot(7:end);

    qb_ddot = q_ddot(1:6);
    qm_ddot = q_ddot(7:end);

    %% Twist propagation
    % Need to get
    %   A_array
    %   t
    %   t_dot

end
