function dx = sr_state_func(x, u) %#codegen
    % dx = [q_dot; q_ddot]
    % x = [q; q_dot]
    %
    % q = [rb; psi_b; qm]
    % q_dot = [vb; wb; qm_dot]
    % q_ddot = [vb_dot; wb_dot; qm_ddot]
    %
    % params: SpaceRobot

    %% Load params
    sr_info = SR2_info();

    %%
    dx = zeros(16, 1);
    q = x(1:8);
    %     qb = q(1:6);
    %     qm = q(7:8);

    q_dot = x(9:16);
    %     qb_dot = q_dot(1:6);
    %     qm_dot = q_dot(7:8);
    %%
    % 1 - Kinematics
    [Rb, Ra, Rm] = RFunc_SR2(q);
    Rm = reshape(Rm, 3, 3, []); % Split Rm to nk 3x3 arrays

    % 2 - Kinetics
    [t, t_dot, Omega, A, A_dot] = Kin(sr_info, q, q_dot, zeros(8, 1), {Rb, Ra, Rm});

    % 3 - ID
    wb = t{1}(4:6);
    C = CorMat(sr_info, wb, Omega, A, A_dot, {Rb, Ra, Rm});
%     [tau, ~] = ID(sr_info, t, t_dot, Omega, A, A_dot);
%     h = [tau{1}; tau{2}];
    h = C*q_dot;

    % 4 - Mass Mat
    D = MassM(sr_info, q, A);

    % 5 - FD
    q_ddot = D^ - 1 * (u - h);

    %% Assign outputs
    % Speed
    dx(1:8) = q_dot;
    dx(9:16) = q_ddot;

end
