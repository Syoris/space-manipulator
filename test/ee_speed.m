function [Xee, Xee_dot] = ee_speed(sr_info, q, q_dot)
    % Computes Xee and Xee_dot from q and q_dot
    % Xee_dot angular vel express in omega.

    % 1 - EE Position
    [Rb, Ra, Rm, Tee] = feval(sr_info.RFunc, q);
    Rm = reshape(Rm, 3, 3, []); % Split Rm to nk 3x3 arrays

    [Ree, ree] = tr2rt(Tee);
    Ree_blk = blkdiag(Ree, Ree);

    psi_ee = tr2rpy(Ree, 'zyx');
    Xee = [ree; psi_ee.'];

    % 2 - Kinetics
    [t, ~, Omega, A, A_dot] = Kin(sr_info, q, q_dot, zeros(sr_info.N, 1), {Rb, Ra, Rm});
    wb = t{1}(4:6);

    % 3 - J
    J = Ree_blk * Jacobian('endeffector', sr_info, A, {Rb, Ra});
    % J_dot = Ree_blk*Jacobian_dot('endeffector', sr_info, A, A_dot, {Rb, Ra}, wb, Omega);

    % 4 - Xee_dot
    Xee_dot = J * q_dot; % Omega
    % Xee_ddot = J*q_ddot + J_dot*q_dot;

%     Xee_dot_psi = Xee_dot;

    Xee_dot(4:6) = omega2euler_inertial(Xee(4:6), Xee_dot(4:6));

end
