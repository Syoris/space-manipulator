function dx = sr_ee_state_func(x, u, sr_info) %#codegen
    % x = [q; q_dot]
    % dx = [q_dot; q_ddot]
    % Where  *** q_dot = [vb, psi_b_dot] ***
    
    %
    % q = [rb; psi_b; qm; ree; psi_ee], 6+6+n
    % q_dot = [vb; wb; qm_dot; vee; wee], 6+6+n
    % q_ddot = [vb_dot; wb_dot; qm_ddot; vee_dot; wee_dot], 6+6+n


    N = sr_info.N;
%     n = sr_info.n;
    nk = sr_info.nk;   

    %%
    dx = zeros((N+6)*2, 1);

    q = x(1:N);
    q_ee = x(N+1:N+6);

    q_dot = x(N+7:2*N+6);
    q_ee_dot = x(end-5:end);

    q_dot(4:6) = euler2omega(q(4:6), q_dot(4:6)); % wb = R_psi*psi_b_dot
    q_ee_dot(4:6) = euler2omega(q_ee(4:6), q_ee_dot(4:6)); % wee = R_psi*psi_ee_dot

    %%
    % 1 - Kinematics
    Rb = zeros(3, 3);
    Ra = zeros(6, 6);
    Rm = zeros(3, 3*nk);
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
    J = Ree*Jacobian('endeffector', sr_info, A, {Rb, Ra});
%     J_inv = pinv(J);
    J_dot = Ree*Jacobian_dot('endeffector', sr_info, A, A_dot, {Rb, Ra}, wb, Omega);
    
    A_inv = J*D_inv;   
    q_ee_ddot = A_inv * (u - h) + J_dot*q_dot;


    %% Assign outputs
    % x1_dot, x1 = [xb; qm]
    dx(1:3) = q_dot(1:3); % rb_dot = rb
    dx(4:6) = omega2euler(q(4:6), q_dot(4:6)); % psi_b_dot = R_psi^-1*w_b
    dx(7:N) = q_dot(7:N); % qm_d0t

    % x2_dot, x2 = [xee]
    dx(N+1:N+3) = q_ee_dot(1:3);
    dx(N+4:N+6) = omega2euler(q_ee(4:6), q_ee_dot(4:6)); 
    
    % x3_dot, x3 = [qb_dot, qm_dot]
    dx(N+7:2*N+6) = q_ddot;

    % x4_dot, x4 = [q_ee_dot]  
    dx(end-5:end) = q_ee_ddot;
end