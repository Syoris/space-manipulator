%% Symbolic expressions test and comp with SPART
%% Load Robots
clc
load 'SC_2DoF.mat'

% SPART
filename='SC_2DoF.urdf';
[robotSpart,robot_keys] = urdf2robot(filename);

% Initial condition
syms 'Rx' 'Ry' 'Rz' 'r' 'p' 'y' 'qm1' 'qm2'
syms 'Rx_d' 'Ry_d' 'Rz_d' 'wx' 'wy' 'wz' 'qm1_d' 'qm2_d'

r0= [Rx;Ry;Rz];
delta0 = [r;p;y];
R0=diag(delta0);
qm= [qm1; qm2];

q = [r0; delta0; qm];

q_dot = [Rx_d; Ry_d; Rz_d; wx; wy; wz; qm1_d; qm2_d];
qm_dot = [qm1_d; qm2_d];

u0=sym('u0',[6,1],'real');
um=sym('um',[sc.NumActiveJoints,1],'real');

sc.JointsConfig = qm';
sc.BaseConfig = [r0'; delta0'];
% %% SPART
% % Kinematics
% [RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robotSpart);
% [Bij,Bi0,P0,pm]=DiffKinematics(R0,r0,rL,e,g,robotSpart);
% 
% [J00, Jm0]=Jacob(r0,r0,rL,P0,pm,0,robotSpart);
% [J01, Jm1]=Jacob(rL(1:3,1),r0,rL,P0,pm,1,robotSpart);
% [J02, Jm2]=Jacob(rL(1:3,2),r0,rL,P0,pm,2,robotSpart);
% [J03, Jm3]=Jacob(rL(1:3,3),r0,rL,P0,pm,3,robotSpart);
% J_S = {[[J01(4:6, 4:6), J01(4:6, 1:3); J01(1:3, 4:6), J01(1:3, 1:3)], [Jm1(4:6, :); Jm1(1:3, :)]]; 
%        [[J02(4:6, 4:6), J02(4:6, 1:3); J02(1:3, 4:6), J02(1:3, 1:3)], [Jm2(4:6, :); Jm2(1:3, :)]];
%        [[J03(4:6, 4:6), J03(4:6, 1:3); J03(1:3, 4:6), J03(1:3, 1:3)], [Jm3(4:6, :); Jm3(1:3, :)]]};
% J_S_ori = {[J01, Jm1], [J02, Jm2], [J03, Jm3]};
% 
% [t0,tm]=Velocities(Bij,Bi0,P0,pm,u0,um,robotSpart);
% % Dynamics
% 
% %Inertias in inertial frames
% [I0,Im]=I_I(R0,RL,robotSpart);
% 
% %Mass Composite Body matrix
% [M0_tilde,Mm_tilde]=MCB(I0,Im,Bij,Bi0,robotSpart);
% 
% %Generalized Inertia matrix
% [H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robotSpart);
% H_spart_ori = [H0, H0m; H0m', Hm];
% H_spart = [[H0(4:6, 4:6), H0(4:6, 1:3); H0(1:3,4:6), H0(1:3, 1:3)], [H0m(4:6, :); H0m(1:3, :)]; 
%            [H0m(4:6, :)', H0m(1:3, :)'], Hm];
% H_spart = double(subs(H_spart, q, [0, 0, 0, 0, 0, 0, pi/6, -pi/4]'));
% 
% 
% %Generalized Convective Inertia matrix
% [C0, C0m, Cm0, Cm] = CIM(t0,tm,I0,Im,M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robotSpart);
%% My Toolbox
% Kin
tTree = sc.forwardKinematics;
inertias = sc.getInertiaM();

% Dynamics
H_sym = sym(zeros(6+sc.NumActiveJoints, 6+sc.NumActiveJoints));

% Vectors
Mtot = 17.5;
Rc0 = [q(1); q(2); q(3)]; % Vector to base CoM

% Link CoM
comPoses = sc.getCoMPosition();
rk = sym(zeros(3, 1, sc.NumActiveJoints));

T_I_B = sc.Base.BaseToParentTransform; % Transform of base to I
R_temp = T_I_B(1:3,1:3)';
T_B_I = [R_temp, -R_temp*T_I_B(1:3,4) ;[0 0 0 1]]; % Inverse

for i=1:sc.NumActiveJoints
    T_I_i = comPoses.(sc.LinkNames{i}); % Transform of i to I
    
    T_B_i = T_B_I * T_I_i;

    rk(:, :, i) = T_B_i(1:3, 4);
end
% rk = subs(rk, [Rx, Ry, Rz, r, p, y], [0, 0, 0, 0, 0, 0]);

% Omega Computation
w0 = [q_dot(4); q_dot(5); q_dot(6)]; % Omega computation
wk = sym(zeros(3, 1, sc.NumActiveJoints));
for i=1:sc.NumActiveJoints
    T = sc.getTransform(sc.BaseName, sc.LinkNames{i});
    wk(:,:,i) = wk(:,:,i) + T(1:3, 1:3)*w0;

    for j=1:i-1
        T = sc.getTransform(sc.LinkNames{j}, sc.LinkNames{i});
        wk(:,:,i) = wk(:,:,i) + T(1:3, 1:3)*qm_dot(j)*sc.Links{j}.Joint.Axis';
    end
    wk(:,:,i) = wk(:,:,i) + qm_dot(i)*sc.Links{i}.Joint.Axis';
end
% wk = subs(wk, [Rx, Ry, Rz, r, p, y], [0, 0, 0, 0, 0, 0]);

% Mass Matrix
fprintf('Computing Derivatives...\n')
for i=1:6+sc.NumActiveJoints
    for j=1:6+sc.NumActiveJoints
%         fprintf('H_%i_%i\n', i, j)
        t1 = Mtot * diff(Rc0, q(i))'*diff(Rc0, q(j));
        t2 = diff(w0, q_dot(i))' * inertias.(sc.BaseName) * diff(w0, q_dot(j));
        H_sym(i, j) = H_sym(i, j) + t1 + t2;

        for k=1:sc.NumActiveJoints
            link = sc.Links{k};
            t3 = link.Mass * diff(rk(:, :, k), q(i))' * diff(rk(:, :, k), q(j));
            t4 = diff(wk(:, :, k), q_dot(i))' * inertias.(link.Name) * diff(wk(:, :, k), q_dot(j));

            H_sym(i, j) = H_sym(i, j) + t3 + t4;
        end
        
        t5 = 0;
        for k=1:sc.NumActiveJoints
            link = sc.Links{k};
            
            t5 = t5 + link.Mass * diff(rk(:, :, k), q(i))';
        end
        H_sym(i, j) = H_sym(i, j) + t5 * diff(Rc0, q(j));
        
        t6 = 0;
        for k=1:sc.NumActiveJoints
            link = sc.Links{k};
            
            t6 = t6 + link.Mass * diff(rk(:, :, k), q(j))';
        end
        H_sym(i, j) = H_sym(i, j) + t6 * diff(Rc0, q(i));
    end
end

r0 = [0;0;0];
delta0 = [0;0;0];
qm = [pi/6;-pi/4];
H2 = double(subs(H_sym, q, [ r0', delta0', qm' ]'));