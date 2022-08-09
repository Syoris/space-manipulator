%% SPART Tutorial
%URDF filename
filename='SC_2DoF.urdf';

%Create robot model
[robotSpart,robot_keys] = urdf2robot(filename);
% robotTlbx = importrobot(filename);

%% --- Kinematics ---%
%Base position
R0=eye(3);  %Rotation from Base-spacecraft to inertial
r0_S=[0; 0; 0]; %Position of the base-spacecraft

%Joint variables [rad]
qm_S=[pi/6; -pi/4];

%Velocities
u0_S=zeros(6,1); %Base-spacecraft velocity
um_S=[4;-1]*pi/180; %Joint velocities

[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0_S,qm_S,robotSpart);
%End-Effector
% TEE=[RL(1:3,1:3,end),rL(1:3,end);zeros(1,3),1]*T_Ln_EE;

%% --- Differential Kinematics ---%
%Differential kinematics
[Bij,Bi0,P0,pm]=DiffKinematics(R0,r0_S,rL,e,g,robotSpart);

%Jacobians
[J00, Jm0]=Jacob(r0_S,r0_S,rL,P0,pm,0,robotSpart);
[J01, Jm1]=Jacob(rL(1:3,1),r0_S,rL,P0,pm,1,robotSpart);
[J02, Jm2]=Jacob(rL(1:3,2),r0_S,rL,P0,pm,2,robotSpart);
[J03, Jm3]=Jacob(rL(1:3,3),r0_S,rL,P0,pm,3,robotSpart);
J_S = {[[J01(4:6, 4:6), J01(4:6, 1:3); J01(1:3, 4:6), J01(1:3, 1:3)], [Jm1(4:6, :); Jm1(1:3, :)]]; 
       [[J02(4:6, 4:6), J02(4:6, 1:3); J02(1:3, 4:6), J02(1:3, 1:3)], [Jm2(4:6, :); Jm2(1:3, :)]];
       [[J03(4:6, 4:6), J03(4:6, 1:3); J03(1:3, 4:6), J03(1:3, 1:3)], [Jm3(4:6, :); Jm3(1:3, :)]]};
J_S_ori = {[J01, Jm1], [J02, Jm2], [J03, Jm3]};

%End-effector Jacobian
% [J0EE, JmEE]=Jacob(TEE(1:3,4),r0,rL,P0,pm,robotSpart.n_links_joints,robotSpart);

%Velocities
[t0,tm]=Velocities(Bij,Bi0,P0,pm,u0_S,um_S,robotSpart);
%% --- Inertia Matrices ---%
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
C_spart_ori = [C0, C0m; Cm0, Cm];
C_spart = [[C0(4:6, 4:6), C0(4:6, 1:3); C0(1:3,4:6), C0(1:3, 1:3)], [C0m(4:6, :); C0m(1:3, :)]; 
           [Cm0(:, 4:6), Cm0(:, 1:3)], Cm];
return
%% --- Inertia Matrix Test ---%
H_test = zeros(6+2);

% Base
Jv_B = [J00(4:6, :), Jm0(4:6, :)]; % Base speed jacobian
Jw_B = [J00(1:3, :), Jm0(1:3, :)]; % Base rotation jacobian
inertias = sc.getInertiaM();

bV = sc.Base.Mass*(Jv_B'*Jv_B);
% bW = Jw_B'*inertias.(sc.BaseName)*Jw_B;
bW = Jw_B'*I0*Jw_B;

H_test = H_test + bV + bW;

for i=1:sc.NumLinks
    link = sc.Links{i};
    joint = link.Joint;
    
    % Compute for active joints
    if joint.Q_id ~= -1   
%         Jv_i = Jacobians.(link.Name)(1:3, :);
%         Jw_i = Jacobians.(link.Name)(4:6, :);
        Jv_i = J_S_ori{i}(4:6, :);
        Jw_i = J_S_ori{i}(1:3, :);

%         I_i = inertias.(link.Name);
        I_i = Im(:, :, i);
        
        iv_1 = link.Mass*(Jv_B'*Jv_B);
        iv_2 = 2*link.Mass*(Jv_B'*Jv_i);
        iv_3 = link.Mass*(Jv_i'*Jv_i);
        iw = Jw_i' * I_i * Jw_i;
        
        H_test = H_test + iv_1 + iv_2 + iv_3 + iw;
    end
end

% Spart comp
fprintf('\n##### Mass Matrix #####\n')
fprintf('--- Computed ---');
H_test

fprintf('--- SPART ---');
H_spart

return

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
[u0dot_FD,umdot_FD] = FD(tauq0,tauqm,wF0,wFm,t0,tm,P0,pm,I0,Im,Bij,Bi0,u0_S,um_S,robotSpart);

%% --- Inverse Dynamics - Flying ---%

%Accelerations
u0dot=zeros(6,1);
umdot=zeros(robotSpart.n_q,1);

%Accelerations
[t0dot,tmdot]=Accelerations(t0,tm,P0,pm,Bi0,Bij,u0_S,um_S,u0dot,umdot,robotSpart);

%Inverse Dynamics - Flying base
[tau0,taum] = ID(wF0,wFm,t0,tm,t0dot,tmdot,P0,pm,I0,Im,Bij,Bi0,robotSpart);