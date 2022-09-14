% Position
sc.q0 = diag([1; 1; 1; 1; 0; 0]) * rand(6, 1);
sc.qm = diag([1; 1]) * rand(2, 1);

% Speed
sc.q0_dot = diag([0; 0; 0; 0; 0; 0]) * rand(6, 1);
sc.qm_dot = diag([0; 0]) * rand(2, 1);

% Accel
q0_ddot = diag([0; 0; 0; 0; 0; 0]) * rand(6, 1);
qm_ddot = diag([0; 0]) * rand(2, 1);

% Set values
q = sc.q;
q_dot = sc.q_dot;

qb = q(1:6);
qm = q(7:end);

qb_dot = q_dot(1:6);
qm_dot = q_dot(7:end);