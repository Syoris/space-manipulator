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
    [Rb, Ra, Rm] = RFunc_SR2(q);
    Rm = reshape(Rm, 3, 3, []); % Split Rm to nk 3x3 arrays

    % 2 - Kinetics
    [t, ~, Omega, A, A_dot] = Kin(sr_info, q, q_dot, zeros(8, 1), {Rb, Ra, Rm});

    % 3 - C
    wb = t{1}(4:6);
    C = CorMat(sr_info, wb, Omega, A, A_dot, {Rb, Ra, Rm});
    h = C*q_dot;

    % 4 - Mass Mat
    D = MassM(sr_info, q, A);

    % 5 - FD
    q_ddot = D^ - 1 * (u - h);

    % 6 - End Effector
    J = Jacobian('endeffector', sr_info, A, {Rb, Ra});
    J_inv = pinv(J);
    J_dot = Jacobian_dot('endeffector', sr_info, A, A_dot, {Rb, Ra}, wb, Omega);
    
    A = J*D;
    B = C*J_inv - D*J_inv*J_dot*J_inv;
    q_ee_ddot = A * (u - B*q_ee_dot);


    %% Assign outputs
    dx(1:8) = q_dot;
    dx(9:14) = q_ee_dot;

    dx(15:22) = q_ddot;
    dx(23:28) = q_ee_ddot;
end
