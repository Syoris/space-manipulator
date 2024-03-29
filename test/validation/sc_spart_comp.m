%% Space Robot Dynamics
% To compare dynamic functions with the one obtained with SPART
clc
load 'SC_2DoF.mat'

scTest = sc;

% Spacecraft State
qm_val = [pi / 6; -pi / 4];
r0_val = [0.5; 0.2; 1];
delta0_val = [0; 0; 0];

r0_dot_val = [0; 0; 0];
w0_val = [0; 0; 0];
qm_dot_val = [4; -1] * pi / 180; %Joint velocities

% Initial condition
syms 't' 'Rx' 'Ry' 'Rz' 'r' 'p' 'y' 'qm1(t)' 'qm2(t)'
syms 'Rx_d' 'Ry_d' 'Rz_d' 'wx' 'wy' 'wz' 'qm1_d' 'qm2_d'

R0_val = rpy2r(delta0_val');
r0 = [Rx; Ry; Rz];
delta0 = [r; p; y];
R0 = rpy2r(transpose(delta0));
qm = [qm1(t); qm2(t)];

q = [r0; delta0; qm];

q_dot = [Rx_d; Ry_d; Rz_d; wx; wy; wz; qm1_d; qm2_d];
qm_dot = [qm1_d; qm2_d];

q_val = [r0_val; delta0_val; qm_val];
q_dot_val = [r0_dot_val; w0_val; qm_dot_val];

%
scTest.q = q_val;
scTest.q_dot = q_dot_val;

% SPART
filename = 'SC_2DoF.urdf';
[robotSpart, robot_keys] = urdf2robot(filename);

%% --- Jacobians ---
% SPART
[RJ, RL, rJ, rL, e, g] = Kinematics(R0, r0, qm, robotSpart);
%Differential kinematics
[Bij, Bi0, P0, pm] = DiffKinematics(R0, r0, rL, e, g, robotSpart);

%Jacobians
[J00, Jm0] = Jacob(r0, r0, rL, P0, pm, 0, robotSpart);
[J01, Jm1] = Jacob(rL(1:3, 1), r0, rL, P0, pm, 1, robotSpart);
[J02, Jm2] = Jacob(rL(1:3, 2), r0, rL, P0, pm, 2, robotSpart);
[J03, Jm3] = Jacob(rL(1:3, 3), r0, rL, P0, pm, 3, robotSpart);
J_S = {[[J01(4:6, 4:6), J01(4:6, 1:3); J01(1:3, 4:6), J01(1:3, 1:3)], [Jm1(4:6, :); Jm1(1:3, :)]];
    [[J02(4:6, 4:6), J02(4:6, 1:3); J02(1:3, 4:6), J02(1:3, 1:3)], [Jm2(4:6, :); Jm2(1:3, :)]];
    [[J03(4:6, 4:6), J03(4:6, 1:3); J03(1:3, 4:6), J03(1:3, 1:3)], [Jm3(4:6, :); Jm3(1:3, :)]]};
J_S_ori = {[J01, Jm1], [J02, Jm2], [J03, Jm3]};

%Velocities
[t0, tm] = Velocities(Bij, Bi0, P0, pm, [r0_dot_val; w0_val], qm_dot, robotSpart);

% % Toolbox
comPoses = scTest.getCoMPosition();
Jacobians = scTest.JacobsCoM;

for i = 1:scTest.NumBodies
    bodyName = scTest.BodyNames{i};
    J_i = Jacobians.(bodyName);

    fprintf('\n##### Body %i #####\n', i);
    fprintf('Mine:\n')
    fprintf('\n')
    disp(J_i);

    fprintf('SPART:\n')
    %     disp(J_S{i});
    J_i_S = double(subs(J_S{i}, q, q_val));
    disp(J_i_S);

    assert(isequal(round(J_i, 5), round(J_i_S, 5)), 'Jacobians not matching for body %s', bodyName);
end

%% --- H - Mass Matrix ---
% SPART
%Inertias in inertial frames
[I0, Im] = I_I(R0, RL, robotSpart);

%Mass Composite Body matrix
[M0_tilde, Mm_tilde] = MCB(I0, Im, Bij, Bi0, robotSpart);

%Generalized Inertia matrix
[H0, H0m, Hm] = GIM(M0_tilde, Mm_tilde, Bij, Bi0, P0, pm, robotSpart);
H_spart_ori = [H0, H0m; H0m', Hm];
H_spart = [[H0(4:6, 4:6), H0(4:6, 1:3); H0(1:3, 4:6), H0(1:3, 1:3)], [H0m(4:6, :); H0m(1:3, :)];
                                                                [H0m(4:6, :)', H0m(1:3, :)'], Hm];

%Generalized Convective Inertia matrix
[C0, C0m, Cm0, Cm] = CIM(t0, tm, I0, Im, M0_tilde, Mm_tilde, Bij, Bi0, P0, pm, robotSpart);
C_spart_ori = [C0, C0m; Cm0, Cm];
C_spart = [[C0(4:6, 4:6), C0(4:6, 1:3); C0(1:3, 4:6), C0(1:3, 1:3)], [C0m(4:6, :); C0m(1:3, :)];
                                                                [Cm0(:, 4:6), Cm0(:, 1:3)], Cm];

% Comparison
fprintf('\n##### H - Mass Matrix #####\n')
fprintf('--- Computed ---\n');
tic
disp(scTest.H);
toc

fprintf('--- SPART ---\n');
tic
H_spart_val = double(subs(H_spart, q, q_val));
disp(H_spart_val)
toc
%% --- C - Non-Linear Effect ---
fprintf('\n##### C Matrix #####\n')
fprintf('--- Computed ---\n');
tic
disp(scTest.C);
toc

fprintf('--- SPART ---\n');
tic
disp(double(subs(C_spart, [q; q_dot], [q_val; q_dot_val])))
toc

% % C Matrix Check
assert(scTest.isNSkewSym());
assert(scTest.isCOk(true));
%% --- Forward Dyn ---
% FORCES
f0 = [0; 0; 0.5]; % Force on baseC
n0 = [0; 0; 0]; % Torque on base
tau_qm = [0; 0]; % Joints torque

% SPART
%Gravity
g = 9.8; %[m s-2]

%External forces (includes gravity and assumes z is the vertical direction)
wF0 = [n0; f0];
wFm = [zeros(6, robotSpart.n_bodys_joints)];

%Joint torques
tauq0 = zeros(6, 1);

t0 = double(subs(t0, [q, q_dot], [q_val, q_dot_val]));
tm = double(subs(tm, [q, q_dot], [q_val, q_dot_val]));
P0 = double(subs(P0, [q, q_dot], [q_val, q_dot_val]));
pm = double(subs(pm, [q, q_dot], [q_val, q_dot_val]));
I0 = double(subs(I0, [q, q_dot], [q_val, q_dot_val]));
Im = double(subs(Im, [q, q_dot], [q_val, q_dot_val]));
Bij = double(subs(Bij, [q, q_dot], [q_val, q_dot_val]));
Bi0 = double(subs(Bi0, [q, q_dot], [q_val, q_dot_val]));

fprintf("\n### Foward Dynamics ###\n")
fprintf('-- Computed --\n')
tic
F = [f0; n0; tau_qm];
q_ddot = scTest.forwardDynamics(F);
disp(q_ddot)
toc

fprintf('-- SPART -- \n')
tic
[u0dot_FD, umdot_FD] = FD(tauq0, tau_qm, wF0, wFm, t0, tm, P0, pm, I0, Im, Bij, Bi0, q_dot_val(1:6), qm_dot_val, robotSpart);
disp([u0dot_FD(4:6); u0dot_FD(1:3); umdot_FD])
toc

fprintf("Same result: %i\n", all(round(q_ddot, 5) == round([u0dot_FD(4:6); u0dot_FD(1:3); umdot_FD], 5)))
return
%% --- Inverse Dynamics ---
%Accelerations
u0dot = zeros(6, 1);
umdot = zeros(robotSpart.n_q, 1);

%Accelerations
[t0dot, tmdot] = Accelerations(t0, tm, P0, pm, Bi0, Bij, q_dot_val(1:6), qm_dot_val, u0dot, umdot, robotSpart);

%Inverse Dynamics - Flying base
[tau0, taum] = ID(wF0, wFm, t0, tm, t0dot, tmdot, P0, pm, I0, Im, Bij, Bi0, robotSpart);
