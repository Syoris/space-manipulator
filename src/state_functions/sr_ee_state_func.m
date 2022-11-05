function dx = sr_ee_state_func(x, u, sr_info) %#codegen
    % x = [x; x_dot]
    % dx = [x_dot; x_ddot]

    % Where  *** x_dot = [vb, psi_b_dot, qm_dot] ***
    % Where  *** x_ddot = [vb_dot, psi_b_ddot, qm_ddot] ***

    % q = x = [rb; psi_b; qm; ree; psi_ee], 6+6+n
    % q_dot = [vb; wb; qm_dot; vee; wee], 6+6+n
    % q_ddot = [vb_dot; wb_dot; qm_ddot; vee_dot; wee_dot], 6+6+n

    N = sr_info.N;
    %     n = sr_info.n;
    nk = sr_info.nk;

    %% --- Initialize states ---
    dx = zeros((N + 6) * 2, 1);

    q = x(1:N);
    q_ee = x(N + 1:N + 6);

    x_dot = x(N + 7:2 * N + 6);
    x_ee_dot = x(2 * N + 7:end);

    q_dot = x_dot;
    q_dot(4:6) = euler2omega_local(q(4:6), x_dot(4:6)); % wb = R_psi*psi_b_dot

%     q_ee_dot = x_ee_dot;
%     q_ee_dot(4:6) = euler2omega_inertial(q_ee(4:6), x_ee_dot(4:6)); % wee = R_psi*psi_ee_dot

    %% --- Compute accel ---
    % 1 - Kinematics
    Rb = zeros(3, 3);
    Ra = zeros(6, 6);
    Rm = zeros(3, 3 * nk);
    Tee = zeros(4, 4);

    [Rb, Ra, Rm, Tee] = feval(sr_info.RFunc, q);
    Rm = reshape(Rm, 3, 3, nk); % Split Rm to nk 3x3 arrays

    % 2 - Kinetics
    [t, t_dot, Omega, A, A_dot] = Kin(sr_info, q, q_dot, zeros(N, 1), {Rb, Ra, Rm});

    % 3 - C
    wb = t{1}(4:6);
    %     C = CorMat(sr_info, wb, Omega, A, A_dot, {Rb, Ra, Rm});
    %     h = C*q_dot;
    [tau, ~] = ID(sr_info, t, t_dot, Omega, A, A_dot);
    tau_b = blkdiag(Rb, zeros(3, 3)) * tau{1}; % Express base torque in intertial frame
    h = [tau_b; tau{2}];

    % 4 - Mass Mat
    D = MassM(sr_info, q, A);
    D_inv = D^ - 1;

    % 5 - FD
    q_ddot = D_inv * (u - h);

    % 6 - End Effector
    Ree = Tee(1:3, 1:3);
    Ree = blkdiag(Ree, Ree);
    J = Ree * Jacobian('endeffector', sr_info, A, {Rb, Ra});
    J_dot = Ree * Jacobian_dot('endeffector', sr_info, A, A_dot, {Rb, Ra}, wb, Omega);

    q_ee_ddot = J * q_ddot + J_dot * q_dot;

    %% --- Convert states ---
    x_ddot = q_ddot;
    x_ddot(4:6) = omega2euler_accel_local(q(4:6), x_dot(4:6), q_ddot(4:6)); % psi, psi_dot, wb_dot

    x_ee_ddot = q_ee_ddot;
    x_ee_ddot(4:6) = omega2euler_accel_inertial(q_ee(4:6), x_ee_dot(4:6), q_ee_ddot(4:6)); % omega2euler_accel(psi, psi_dot, wb_dot)

    % TODO
    % x1_dot, x1 = [xb; qm]
    dx(1:N) = x_dot;

    % x2_dot, x2 = [xee]
    dx(N + 1:N + 6) = x_ee_dot;

    % x3_dot, x3 = [qb_dot, qm_dot]
    dx(N + 7:2 * N + 6) = x_ddot;

    % x4_dot, x4 = [q_ee_dot]
    dx(2 * N + 7:2 * N + 12) = x_ee_ddot;
end
