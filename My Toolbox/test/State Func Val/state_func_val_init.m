clc
% load 'SR2.mat'
load 'SR6.mat'

sr.homeConfig();
q0 = sr.q;
q_dot0 = sr.q_dot;

[Ree, xee0] = tr2rt(sr.Ttree.endeffector);
xee0 = [xee0; tr2rpy(Ree).'];
xee_dot0 = zeros(6, 1);

% Forces
fx = 0;
fy = 0;
fz = 0;

nx = 0;
ny = 2;
nz = 0;

tau1 = 0;
tau2 = 0;
tau = [tau1; tau2];

tau = zeros(6, 1);


%% Xee ddot test
q = eye(8) * rand(8, 1);
q_dot = eye(8) * rand(8, 1);
% q_ddot = eye(8) * rand(8, 1);
u = eye(8) * rand(8, 1);

sr_info = SR2_info();

% 1 - Kinematics
[Rb, Ra, Rm, Ree] = RFunc_SR2(q);
Rm = reshape(Rm, 3, 3, []); % Split Rm to nk 3x3 arrays

% 2 - Kinetics
[t, t_dot, Omega, A, A_dot] = Kin(sr_info, q, q_dot, zeros(8, 1), {Rb, Ra, Rm});

% 3 - C
wb = t{1}(4:6);
C = CorMat(sr_info, wb, Omega, A, A_dot, {Rb, Ra, Rm});
% h = C*q_dot;
[tau, ~] = ID(sr_info, t, t_dot, Omega, A, A_dot);
tau_b = blkdiag(Rb, zeros(3, 3)) * tau{1}; % Express base torque in intertial frame
h = [tau_b; tau{2}];

% 4 - Mass Mat
D = MassM(sr_info, q, A);
q_ddot = D^ - 1 * (u - h);

% Jacobs
Ree = Ree(1:3, 1:3);
Ree = blkdiag(Ree, Ree);
J = Ree*Jacobian('endeffector', sr_info, A, {Rb, Ra});
J_inv = pinv(J);
J_dot = Ree*Jacobian_dot('endeffector', sr_info, A, A_dot, {Rb, Ra}, wb, Omega);

% --- EE Dyn ---
Xee_dot = J*q_dot;
Xee_ddot = J*q_ddot + J_dot * q_dot;

A = D*J_inv;
B = C*J_inv - D*J_inv*J_dot*J_inv;
X_ee_ddot = pinv(A) * (u - B*Xee_dot);

% X_ee_ddot2 = J*D^-1 * u + J_dot*q_dot - J*D^-1*C*q_dot;


A_inv = J*D^-1;

X_ee_ddot2 = A_inv * (u - h) + J_dot*q_dot;


fprintf("Xee Accel\n")
fprintf("[From Jac \t From Dyn \t test]\n")
disp([Xee_ddot, X_ee_ddot2, X_ee_ddot])




