% SPART
filename = 'SC_2DoF.urdf';
[robotSpart, robot_keys] = urdf2robot(filename);
robotSpart.links(3).mass = sr.Bodies{3}.Mass;
robotSpart.links(3).inertia = sr.Bodies{3}.InertiaM;

R0 = rpy2r(q(4:6).');
u0 = [q0_dot(4:6); q0_dot(1:3)];
um = qm_dot;

u0_dot = [q0_ddot(4:6); q0_ddot(1:3)];
um_dot = qm_ddot(1:2);

[RJ, RL, rJ, rL, e, g] = Kinematics(R0, q0(1:3), qm, robotSpart);

%Differential kinematics
[Bij, Bi0, P0, pm] = DiffKinematics(R0, q0(1:3), rL, e, g, robotSpart);

%Velocities
[t0_S, tm_S] = Velocities(Bij, Bi0, P0, pm, u0, um, robotSpart);

%Accelerations, twist-rate
[t0_dot_S,tm_dot_S] = Accelerations(t0_S,tm_S,P0,pm,Bi0,Bij,u0,um,u0_dot,um_dot,robotSpart);

%Inertias projected in the inertial frame
[I0,Im]=I_I(R0,RL,robotSpart);

%Inverse Dynamics - Flying base
wF0=zeros(6,1);
wFm=zeros(6,3);

[tau0_S,taum_S, wq_tilde, wq_tilde0] = ID_test(wF0,wFm,t0_S,tm_S,t0_dot_S,tm_dot_S,P0,pm,I0,Im,Bij,Bi0,robotSpart);

%Mass Composite Body matrix
[M0_tilde, Mm_tilde] = MCB(I0, Im, Bij, Bi0, robotSpart);

%Generalized Inertia matrix
[H0, H0m, Hm] = GIM(M0_tilde, Mm_tilde, Bij, Bi0, P0, pm, robotSpart);
H_spart_ori = [H0, H0m; H0m', Hm];
H_spart = [[H0(4:6, 4:6), H0(4:6, 1:3); H0(1:3, 4:6), H0(1:3, 1:3)], [H0m(4:6, :); H0m(1:3, :)];
                                                                [H0m(4:6, :)', H0m(1:3, :)'], Hm];

%Generalized Convective Inertia matrix
[C0, C0m, Cm0, Cm, M0_tilde, child_con_C0, Bi0, Hi0_tilde, Mdot0_tilde, P0, Mdot_tilde] = CIM_test(t0_S, tm_S, I0, Im, M0_tilde, Mm_tilde, Bij, Bi0, P0, pm, robotSpart);
C_spart_ori = [C0, C0m; Cm0, Cm];
C_spart = [[C0(4:6, 4:6), C0(4:6, 1:3); C0(1:3, 4:6), C0(1:3, 1:3)], [C0m(4:6, :); C0m(1:3, :)];
                                                                [Cm0(:, 4:6), Cm0(:, 1:3)], Cm];


