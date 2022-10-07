function dx = sr_ee_state_func(x, u) %#codegen
    % x = [q; q_dot]
    % dx = [q_dot; q_ddot]
    
    %
    % q = [rb; psi_b; qm; ree; psi_ee]
    % q_dot = [vb; wb; qm_dot; vee; wee]
    % q_ddot = [vb_dot; wb_dot; qm_ddot; vee_dot; wee_dot]

    %% Load params
    sr_info = SR2_info();

    %%
    dx = zeros(28, 1);

    q = x(1:8);
    q_ee = x(9:14);

    %     qb = q(1:6);
    %     qm = q(7:8);

    q_dot = x(15:22);
    q_ee_dot = x(23:28);

    %     qb_dot = q_dot(1:6);
    %     qm_dot = q_dot(7:8);
    %%
    % 1 - Kinematics
    [Rb, Ra, Rm] = RFunc_SR2(q);
    Rm = reshape(Rm, 3, 3, []); % Split Rm to nk 3x3 arrays

    % 2 - Kinetics
    [t, t_dot, Omega, A, A_dot] = Kin(sr_info, q, q_dot, zeros(8, 1), {Rb, Ra, Rm});

    % 3 - ID
    [tau, ~] = ID(sr_info, t, t_dot, Omega, A, A_dot);
    h = [tau{1}; tau{2}];

    % 4 - Mass Mat
    D = MassM(sr_info, q, A);

    % 5 - FD
    q_ddot = D^ - 1 * (u - h);

    % 6 - End Effector
    J = Jacobian('endeffector', sr_info, A, {Rb, Ra});
    J_inv = J^-1;
    J_dot = Jacobian_dot('endeffector', sr_info, A, A_dot, {Rb, Ra}, qb_dot(4:6), Omega);
    
    A = D*J_inv;
%     B = 


    %% Assign outputs
    % Speed
    dx(1:8) = q_dot;
    dx(9:16) = q_ddot;

end
