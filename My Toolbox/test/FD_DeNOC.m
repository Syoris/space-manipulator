%% Forward Dyn
load SR2.mat
clc
% --- Conf ---
% Position
sr2.q0 = diag([1; 1; 1; 0; 0; 0]) * rand(6, 1);
sr2.qm = diag([1; 1]) * rand(2, 1);

% Speed
sr2.q0_dot = diag([1; 1; 1; 0; 0; 0]) * rand(6, 1);
sr2.qm_dot = diag([1; 1]) * rand(2, 1);

% Accel
q0_ddot = diag([0; 0; 0; 0; 0; 0]) * rand(6, 1);
qm_ddot = diag([0; 0]) * rand(2, 1);

% Forces
f0 = [1; 1; 1]; % Force on baseC
n0 = [1; 1; 1]; % Torque on base
tau_b = [f0; n0];
tau_qm = [1; 1]; % Joints torque

tau = [tau_b; tau_qm];

% Run Spart kin
run spart_twist_test.m

[u0dot_S, umdot_S] = FD([n0; f0], tau_qm, zeros(6, 1), zeros(6, 3), t0_S, tm_S, P0, pm, I0, Im, Bij, Bi0, sr2.q0_dot, sr2.qm_dot, robotSpart);
q_ddot_S = [u0dot_S(4:6); u0dot_S(1:3); umdot_S];

% DeNOC
% M = sr2.MassMat;
% [h_b, h_m] = sr2.inverseDynamics(sr2.q, sr2.q_dot, zeros(8, 1));
% h = [h_b; h_m];

q_ddot = sr2.forwardDynamics(tau, sr2.q, sr2.q_dot);

fprintf('--- FD ---\n')
fprintf('[\tSPART\t  DeNOC\t Symb]\n')
disp([q_ddot_S, q_ddot])






%% 6 DoF

% load SR6.mat

clc
% --- Conf ---
% Position
sr6.q0 = diag([1; 1; 1; 0; 0; 0]) * rand(6, 1);
sr6.qm = diag([1; 1; 1; 1; 1; 1]) * rand(6, 1);

% Speed
sr6.q0_dot = diag([1; 1; 1; 1; 0; 0]) * rand(6, 1);
sr6.qm_dot = diag([1; 1; 1; 1; 0; 0]) * rand(6, 1);

% Accel
q0_ddot = diag([0; 0; 0; 0; 0; 0]) * rand(6, 1);
qm_ddot = diag([0; 0; 0; 0; 0; 0]) * rand(6, 1);

% Forces
f0 = [1; 1; 1]; % Force on baseC
n0 = [1; 1; 1]; % Torque on base
tau_b = [f0; n0];

tau_qm = [1; 1; 0; 0; 0; 0]; % Joints torque

tau = [tau_b; tau_qm];

% DeNOC
tic
q_ddot = sr6.forwardDynamics(tau, sr6.q, sr6.q_dot);
toc

fprintf('--- FD ---\n')
disp(q_ddot)