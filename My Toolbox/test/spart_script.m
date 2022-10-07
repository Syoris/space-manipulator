function spart_res = spart_script(sr, fileName, q, q_dot, q_ddot)
%% spart_script.m
% To compute main space robot parametres using SPART library
%
%   sr: SpaceRobot
%   fileName: File containing robot info
%   q: 
%   q_dot: 
%   q_ddot: 

% SPART
[robotSpart, ~] = urdf2robot(fileName);
robotSpart.links(3).mass = sr.Bodies{3}.Mass;
robotSpart.links(3).inertia = sr.Bodies{3}.InertiaM;

qb = q(1:6);
qm = q(7:end);

qb_dot = q_dot(1:6);
qm_dot = q_dot(7:end);

qb_ddot = q_ddot(1:6);
qm_ddot = q_ddot(7:end);

R0 = rpy2r(q(4:6).');
u0 = [qb_dot(4:6); qb_dot(1:3)];
um = qm_dot;

u0_dot = [qb_ddot(4:6); qb_ddot(1:3)];
um_dot = qm_ddot(1:2);

[RJ, RL, rJ, rL, e, g] = Kinematics(R0, qb(1:3), qm, robotSpart);

%Differential kinematics
[Bij, Bi0, P0, pm] = DiffKinematics(R0, qb(1:3), rL, e, g, robotSpart);

%Velocities of CoM, Expressed in inertial frame
[t0_S, tm_S] = Velocities(Bij, Bi0, P0, pm, u0, um, robotSpart);
spart_res.twist.tb = t0_S;
spart_res.twist.tm = tm_S;

% Jacobians, com. Inertial frame
[J00_com, Jm0_com] = Jacob(qb(1:3), qb(1:3), rL, P0, pm, 0, robotSpart);
[J01_com, Jm1_com] = Jacob(rL(1:3, 1), qb(1:3), rL, P0, pm, 1, robotSpart);
[J02_com, Jm2_com] = Jacob(rL(1:3, 2), qb(1:3), rL, P0, pm, 2, robotSpart);
[J03_com, Jm3_com] = Jacob(rL(1:3, 3), qb(1:3), rL, P0, pm, 3, robotSpart);
J_S_com = {[[J01_com(4:6, 4:6), J01_com(4:6, 1:3); J01_com(1:3, 4:6), J01_com(1:3, 1:3)], [Jm1_com(4:6, :); Jm1_com(1:3, :)]];
        [[J02_com(4:6, 4:6), J02_com(4:6, 1:3); J02_com(1:3, 4:6), J02_com(1:3, 1:3)], [Jm2_com(4:6, :); Jm2_com(1:3, :)]];
        [[J03_com(4:6, 4:6), J03_com(4:6, 1:3); J03_com(1:3, 4:6), J03_com(1:3, 1:3)], [Jm3_com(4:6, :); Jm3_com(1:3, :)]]};
% J_S_ori = {[J01, Jm1], [J02, Jm2], [J03, Jm3]};
spart_res.J_com = J_S_com;

% Jacobians, joint
J_I = cell(3, 1);
J_i = cell(3, 1);
for i=1:3
    [J0i, Jmi] = Jacob(rJ(1:3, i), qb(1:3), rL, P0, pm, i, robotSpart);

    Ji_I = [[J0i(4:6, 4:6), J0i(4:6, 1:3); J0i(1:3, 4:6), J0i(1:3, 1:3)], [Jmi(4:6, :); Jmi(1:3, :)]];

    J_I{i} = Ji_I;
    
    [Ri, ~] = tr2rt(sr.getTransform(sr.BodyNames{i}, 'TargetFrame', 'inertial', 'Symbolic', false));
    J_i{i} = blkdiag(Ri.', Ri.') * Ji_I;
end
spart_res.J_I = J_I;
spart_res.J_i = J_i;


% Jacobians derivative
J_dot_I = cell(3, 1);
J_dot_i = cell(3, 1);
for i=1:3
    tJi = J_I{i}*q_dot;
    tJi = [tJi(4:6); tJi(1:3)];
    [J0pdot, Jmpdot] = Jacobdot(rJ(1:3, i),tJi, qb(1:3), t0_S, rL, tm_S, P0, pm, i, robotSpart);
    J_dot_i_S = [[J0pdot(4:6, 4:6), J0pdot(4:6, 1:3); J0pdot(1:3, 4:6), J0pdot(1:3, 1:3)], [Jmpdot(4:6, :); Jmpdot(1:3, :)]];
    J_dot_I{i} = J_dot_i_S;
    
    [Ri, ~] = tr2rt(sr.getTransform(sr.BodyNames{i}, 'TargetFrame', 'inertial', 'Symbolic', false));
    J_dot_i{i} = blkdiag(Ri.', Ri.') * J_dot_i_S;
end
spart_res.J_dot_I = J_dot_I;
spart_res.J_dot_i = J_dot_i;

%Accelerations of CoM, twist-rate. Inertial frame
[t0_dot_S, tm_dot_S] = Accelerations(t0_S, tm_S, P0, pm, Bi0, Bij, u0, um, u0_dot, um_dot, robotSpart);
spart_res.accel.tb_dot = t0_dot_S;
spart_res.accel.tm_dot = tm_dot_S;

%Inertias projected in the inertial frame
[I0, Im] = I_I(R0, RL, robotSpart);

%Inverse Dynamics - Flying base
wF0 = zeros(6, 1);
wFm = zeros(6, 3);

[tau0_S, taum_S, wq_tilde, wq_tilde0] = ID_test(wF0, wFm, t0_S, tm_S, t0_dot_S, tm_dot_S, P0, pm, I0, Im, Bij, Bi0, robotSpart);
tau0_S = [tau0_S(4:6); tau0_S(1:3)];

spart_res.tau.tau_b = tau0_S;
spart_res.tau.tau_m = taum_S;

spart_res.wen.wen_b = wq_tilde;
spart_res.wen.wen_m = wq_tilde0;

%Mass Composite Body matrix
[M0_tilde, Mm_tilde] = MCB(I0, Im, Bij, Bi0, robotSpart);

%Generalized Inertia matrix
[H0, H0m, Hm] = GIM(M0_tilde, Mm_tilde, Bij, Bi0, P0, pm, robotSpart);
H_spart_ori = [H0, H0m; H0m', Hm];
H_spart = [[H0(4:6, 4:6), H0(4:6, 1:3); H0(1:3, 4:6), H0(1:3, 1:3)], [H0m(4:6, :); H0m(1:3, :)];
                                                                [H0m(4:6, :)', H0m(1:3, :)'], Hm];
spart_res.H = H_spart;

%Generalized Convective Inertia matrix
[C0, C0m, Cm0, Cm, M0_tilde, child_con_C0, Bi0, Hi0_tilde, Mdot0_tilde, P0, Mdot_tilde] = CIM_test(t0_S, tm_S, I0, Im, M0_tilde, Mm_tilde, Bij, Bi0, P0, pm, robotSpart);
C_spart_ori = [C0, C0m; Cm0, Cm];
C_spart = [[C0(4:6, 4:6), C0(4:6, 1:3); C0(1:3, 4:6), C0(1:3, 1:3)], [C0m(4:6, :); C0m(1:3, :)];
                                                                [Cm0(:, 4:6), Cm0(:, 1:3)], Cm];

spart_res.C = C_spart;

end