function dx = sr_ee_state_func(x, u) %#codegen
    % x = [q; q_dot]
    % dx = [q_dot; q_ddot]
    
    %
    % q = [rb; psi_b; qm; ree; psi_ee], 6+6+n
    % q_dot = [vb; wb; qm_dot; vee; wee], 6+6+n
    % q_ddot = [vb_dot; wb_dot; qm_ddot; vee_dot; wee_dot], 6+6+n

    %% Load params
    sr_info = SR2_info();

    %%
    dx = zeros(28, 1);

    q = x(1:8);
%     q_ee = x(9:14);

    q_dot = x(15:22);
    q_ee_dot = x(23:28);

    %%
    % 1 - Kinematics
    [Rb, Ra, Rm, Ree] = RFunc_SR2(q);
    Rm = reshape(Rm, 3, 3, []); % Split Rm to nk 3x3 arrays

    % 2 - Kinetics
    [t, t_dot, Omega, A, A_dot] = Kin(sr_info, q, q_dot, zeros(8, 1), {Rb, Ra, Rm});

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
    Ree = Ree(1:3, 1:3);
    Ree = blkdiag(Ree, Ree);
    J = Ree*Jacobian('endeffector', sr_info, A, {Rb, Ra});
%     J_inv = pinv(J);
    J_dot = Ree*Jacobian_dot('endeffector', sr_info, A, A_dot, {Rb, Ra}, wb, Omega);
    
    A_inv = J*D_inv;   
    q_ee_ddot = A_inv * (u - h) + J_dot*q_dot;


    %% Assign outputs
    dx(1:8) = q_dot;
    dx(9:14) = q_ee_dot;

    dx(15:22) = q_ddot;
    dx(23:28) = q_ee_ddot;
end
