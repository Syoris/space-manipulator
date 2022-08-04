%% SPART Tutorial
%URDF filename
filename='SC_2DoF.urdf';

%Create robot model
[robotSpart,robot_keys] = urdf2robot(filename);
% robotTlbx = importrobot(filename);

%% --- Kinematics ---%
%Base position
R0=eye(3);  %Rotation from Base-spacecraft to inertial
r0=[0.5;0.5;0]; %Position of the base-spacecraft

%Joint variables [rad]
qm=[pi/4;-pi/2];

%Velocities
u0=zeros(6,1); %Base-spacecraft velocity
um=[4;-1]*pi/180; %Joint velocities

[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robotSpart);
%End-Effector
% TEE=[RL(1:3,1:3,end),rL(1:3,end);zeros(1,3),1]*T_Ln_EE;

%% --- Differential Kinematics ---%
%Differential kinematics
[Bij,Bi0,P0,pm]=DiffKinematics(R0,r0,rL,e,g,robotSpart);

%Jacobian of the Link 3
[J01, Jm1]=Jacob(rL(1:3,1),r0,rL,P0,pm,1,robotSpart);
[J02, Jm2]=Jacob(rL(1:3,2),r0,rL,P0,pm,2,robotSpart);
[J03, Jm3]=Jacob(rL(1:3,3),r0,rL,P0,pm,3,robotSpart);

%End-effector Jacobian
% [J0EE, JmEE]=Jacob(TEE(1:3,4),r0,rL,P0,pm,robotSpart.n_links_joints,robotSpart);

%Velocities
[t0,tm]=Velocities(Bij,Bi0,P0,pm,u0,um,robotSpart);
pass
%% --- Inertia Matrices ---%
%Inertias in inertial frames
[I0,Im]=I_I(R0,RL,robotSpart);
%Mass Composite Body matrix
[M0_tilde,Mm_tilde]=MCB(I0,Im,Bij,Bi0,robotSpart);
%Generalized Inertia matrix
[H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robotSpart);
%Generalized Convective Inertia matrix
[C0, C0m, Cm0, Cm] = CIM(t0,tm,I0,Im,M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robotSpart);

%% --- Forward Dynamics ---%
%Gravity
g=9.8; %[m s-2]

%External forces (includes gravity and assumes z is the vertical direction)
wF0=[0;0;0;0;0;-robotSpart.base_link.mass*g];
wFm=[zeros(5,robotSpart.n_q);
    -robotSpart.links(1).mass*g,-robotSpart.links(2).mass*g,-robotSpart.links(3).mass*g,-robotSpart.links(4).mass*g,-robotSpart.links(5).mass*g];

%Joint torques
tauq0=zeros(6,1);
tauqm=zeros(robotSpart.n_q,1);

%Forward Dynamics
[u0dot_FD,umdot_FD] = FD(tauq0,tauqm,wF0,wFm,t0,tm,P0,pm,I0,Im,Bij,Bi0,u0,um,robotSpart);

%% --- Inverse Dynamics - Flying ---%

%Accelerations
u0dot=zeros(6,1);
umdot=zeros(robotSpart.n_q,1);

%Accelerations
[t0dot,tmdot]=Accelerations(t0,tm,P0,pm,Bi0,Bij,u0,um,u0dot,umdot,robotSpart);

%Inverse Dynamics - Flying base
[tau0,taum] = ID(wF0,wFm,t0,tm,t0dot,tmdot,P0,pm,I0,Im,Bij,Bi0,robotSpart);