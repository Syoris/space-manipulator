function dx = sr_state_func(x, u, sr_info) % #codegen
    % dx = [q_dot; q_ddot]
    % x = [q; q_dot]
    %
    % q = [rb; psi_b; qm]
    % q_dot = [vb; wb; qm_dot]
    % q_ddot = [vb_dot; wb_dot; qm_ddot]
    %
    % sr_info   SpaceRobot static information

    N = sr_info.N;
    %     n = sr_info.n;
    nk = sr_info.nk;

    %%
    dx = zeros(2 * N, 1);
    q = x(1:N); % [r_bx, r_by, r_bz, psi_b_x, psi_b_y, psi_b_z, q1, ..., qn]
    %     qb = q(1:6);
    %     qm = q(7:8);

    q_dot = x(N + 1:end); % [r_bx_dot, r_by_dot, r_bz_dot, w_b_x, w_b_y, w_b_z, q1_dot, ..., qn_dot]
    %     qb_dot = q_dot(1:6);
    %     qm_dot = q_dot(7:8);
    %%
    % 1 - Kinematics
    Rb = zeros(3, 3);
    Ra = zeros(6, 6);
    Rm = zeros(3, 3 * nk);
    [Rb, Ra, Rm] = feval(sr_info.RFunc, q);
    Rm = reshape(Rm, 3, 3, nk); % Split Rm to nk 3x3 arrays

    % 2 - Kinetics
    [t, t_dot, Omega, A, A_dot] = Kin(sr_info, q, q_dot, zeros(N, 1), {Rb, Ra, Rm});

    % 3 - ID
    [tau, ~] = ID(sr_info, t, t_dot, Omega, A, A_dot);
    tau_b = blkdiag(Rb, zeros(3, 3)) * tau{1}; % Express base torque in intertial frame
    h = [tau_b; tau{2}];

    %     wb = t{1}(4:6);
    %     C = CorMat(sr_info, wb, Omega, A, A_dot, {Rb, Ra, Rm});
    %     h = C*q_dot;

    % 4 - Mass Mat
    D = MassM(sr_info, q, A);

    % 5 - FD
    q_ddot = D^ - 1 * (u - h);

    %% Assign outputs
    % x1_dot, x1 = [xb; qm]
    dx(1:3) = q_dot(1:3); % rb_dot = rb
    dx(4:6) = omega2euler_local(q(4:6), q_dot(4:6)); % psi_b_dot = R_psi^-1*w_b
    dx(7:N) = q_dot(7:end); % qm_d0t

    % x2_dot, x3 = [qb_dot, qm_dot]
    dx(N + 1:end) = q_ddot;
end
