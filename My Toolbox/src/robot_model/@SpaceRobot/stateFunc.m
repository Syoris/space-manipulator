function z = stateFunc(obj, x, u)
    % z = [q_dot; q_ddot]
    % x = [q; q_dot]
    %
    % q = [rb; psi_b; qm]
    % q_dot = [vb; wb; qm_dot]
    % q_ddot = [vb_dot; wb_dot; qm_ddot]

    z = zeros(16, 1);
    q = x(1:8);
    qb = q(1:6);
    qm = q(7:8);

    q_dot = x(9:16);
    qb_dot = q_dot(1:6);
    qm_dot = q_dot(7:8);

    % Speed
    z(1:6) = qb_dot; % [vb; wb]
    z(7:8) = qm_dot; % [qm_dot]

    % Accel
    q_ddot = obj.forwardDynamics(u, q, q_dot);
    z(9:16) = q_ddot;

end
