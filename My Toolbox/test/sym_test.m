%% Symbolic expressions test and comp with SPART
%% --- Load Robots ---
load 'SC_2DoF.mat'

% SPART
filename='SC_2DoF.urdf';
[robotSpart,robot_keys] = urdf2robot(filename);

% Initial condition
qm=sym('qm',[data.n,1],'real');
R0=sym(eye(3));
r0=sym([0;0;0]);

u0=sym('u0',[6,1],'real');
um=sym('um',[data.n,1],'real');

sc.JointsConfig = qm';
sc.BaseConfig = [r0'; sym([0;0;0]')];

%% --- Kin SPART ---
% Spart
R0=sym(eye(3));
r0=sym([0;0;0]);

[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robotSpart);
[Bij,Bi0,P0,pm]=DiffKinematics(R0,r0,rL,e,g,robotSpart);

[J00, Jm0]=Jacob(r0,r0,rL,P0,pm,0,robotSpart);
[J01, Jm1]=Jacob(rL(1:3,1),r0,rL,P0,pm,1,robotSpart);
[J02, Jm2]=Jacob(rL(1:3,2),r0,rL,P0,pm,2,robotSpart);
[J03, Jm3]=Jacob(rL(1:3,3),r0,rL,P0,pm,3,robotSpart);
J_S = {[[J01(4:6, 4:6), J01(4:6, 1:3); J01(1:3, 4:6), J01(1:3, 1:3)], [Jm1(4:6, :); Jm1(1:3, :)]]; 
       [[J02(4:6, 4:6), J02(4:6, 1:3); J02(1:3, 4:6), J02(1:3, 1:3)], [Jm2(4:6, :); Jm2(1:3, :)]];
       [[J03(4:6, 4:6), J03(4:6, 1:3); J03(1:3, 4:6), J03(1:3, 1:3)], [Jm3(4:6, :); Jm3(1:3, :)]]};
J_S_ori = {[J01, Jm1], [J02, Jm2], [J03, Jm3]};

[t0,tm]=Velocities(Bij,Bi0,P0,pm,u0,um,robotSpart);

%% --- Dyn SPART--- 
% Spart
%Inertias in inertial frames
[I0,Im]=I_I(R0,RL,robotSpart);

%Mass Composite Body matrix
[M0_tilde,Mm_tilde]=MCB(I0,Im,Bij,Bi0,robotSpart);

%Generalized Inertia matrix
[H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robotSpart);
H_spart_ori = [H0, H0m; H0m', Hm];
H_spart = [[H0(4:6, 4:6), H0(4:6, 1:3); H0(1:3,4:6), H0(1:3, 1:3)], [H0m(4:6, :); H0m(1:3, :)]; 
           [H0m(4:6, :)', H0m(1:3, :)'], Hm];

%Generalized Convective Inertia matrix
[C0, C0m, Cm0, Cm] = CIM(t0,tm,I0,Im,M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robotSpart);
%% --- Kin ----
tTree = sc.forwardKinematics;