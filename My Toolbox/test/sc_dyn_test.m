%% Space Robot Dynamics
% To compare dynamic functions with the one obtained with SPART
clc
if ~exist('sc', 'var')
    fprintf("Loading SpaceRobot...\n")
    load 'SC_2DoF.mat'
end    

% Spacecraft State
qm_val=[pi/6; -pi/4];
r0_val = [0.5; 0.2; 1];
delta0_val = [0; 0; 0];

r0_dot_val = [0; 0; 0];
w0_val = [0; 0; 0];
qm_dot_val = [4;-1]*pi/180; %Joint velocities



% Initial condition
syms 't' 'Rx' 'Ry' 'Rz' 'r' 'p' 'y' 'qm1(t)' 'qm2(t)'
syms 'Rx_d' 'Ry_d' 'Rz_d' 'wx' 'wy' 'wz' 'qm1_d' 'qm2_d'

R0_val = rpy2r(delta0_val');
r0= [Rx;Ry;Rz];
delta0 = [r;p;y];
R0 = rpy2r(transpose(delta0));
qm= [qm1(t); qm2(t)];

q = [r0; delta0; qm];

q_dot = [Rx_d; Ry_d; Rz_d; wx; wy; wz; qm1_d; qm2_d];
qm_dot = [qm1_d; qm2_d];

q_val = [r0_val; delta0_val; qm_val];
q_dot_val = [r0_dot_val; w0_val; qm_dot_val];

% 
sc.JointsConfig = qm_val';
sc.JointsSpeed = qm_dot_val';
sc.BaseConfig = [r0_val'; delta0_val'];
sc.BaseSpeed = [r0_dot_val'; w0_val'];

% SPART
filename='SC_2DoF.urdf';
[robotSpart,robot_keys] = urdf2robot(filename);

%% --- Jacobians ---
% SPART
[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robotSpart);
%Differential kinematics
[Bij,Bi0,P0,pm]=DiffKinematics(R0,r0,rL,e,g,robotSpart);

%Jacobians
[J00, Jm0]=Jacob(r0,r0,rL,P0,pm,0,robotSpart);
[J01, Jm1]=Jacob(rL(1:3,1),r0,rL,P0,pm,1,robotSpart);
[J02, Jm2]=Jacob(rL(1:3,2),r0,rL,P0,pm,2,robotSpart);
[J03, Jm3]=Jacob(rL(1:3,3),r0,rL,P0,pm,3,robotSpart);
J_S = {[[J01(4:6, 4:6), J01(4:6, 1:3); J01(1:3, 4:6), J01(1:3, 1:3)], [Jm1(4:6, :); Jm1(1:3, :)]]; 
       [[J02(4:6, 4:6), J02(4:6, 1:3); J02(1:3, 4:6), J02(1:3, 1:3)], [Jm2(4:6, :); Jm2(1:3, :)]];
       [[J03(4:6, 4:6), J03(4:6, 1:3); J03(1:3, 4:6), J03(1:3, 1:3)], [Jm3(4:6, :); Jm3(1:3, :)]]};
J_S_ori = {[J01, Jm1], [J02, Jm2], [J03, Jm3]};

%Velocities
[t0,tm]=Velocities(Bij,Bi0,P0,pm, [r0_dot_val; w0_val], qm_dot, robotSpart);


% % Toolbox
% comPoses = sc.getCoMPosition();
% Jacobians = sc.comJacobians();
% for i=1:sc.NumLinks
%     linkName = sc.LinkNames{i};
%     J_i = Jacobians.(linkName);
%     
%     fprintf('\n##### Link %i #####\n', i);
%     fprintf('Mine:\n')
% %     disp(J_i);
%     fprintf('\n')
%     disp(double(subs(J_i, q, q_val)));
% 
%     fprintf('SPART:\n')
% %     disp(J_S{i});
%     disp(double(subs(J_S{i}, q, q_val)));
% end

%% H - Mass Matrix
% SPART
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

% Comparison
H = sc.getH();

fprintf('\n##### H - Mass Matrix #####\n')
fprintf('--- Computed ---\n');
% disp(H);
% fprintf('\n')
H_val = double(subs(H, q, q_val));
disp(H_val);

fprintf('--- SPART ---\n');
H_spart_val = double(subs(H_spart, q, q_val));
disp(H_spart_val)
%% C - Non-Linear Effect 
C = sc.getC();

fprintf('\n##### C Matrix #####\n')
fprintf('--- Computed ---\n');
disp(C);

fprintf('--- SPART ---\n');
disp(double(subs(C_spart, [q; q_dot], [q_val; q_dot_val])))

% C Matrix Check
assert(sc.isNSkewSym());
assert(sc.isCOk(true));

% % Skew Sym
% H2 = subs(H, q(1:6), q_val(1:6));
% H_d = diff(H2, t);
% H_d = subs(H_d, [diff(qm1(t), t), diff(qm2(t), t)], [qm1_d, qm2_d]);
% H_d = subs(H_d, q(7:8), q_val(7:8));
% 
% N = H_d - 2*subs(C, [q; q_dot(1:6)], [q_val; q_dot_val(1:6)]);
% N_spart = H_d - 2*subs(C_spart, [q; q_dot(1:6)], [q_val; q_dot_val(1:6)]);
% 
% N_val = double(subs(N, q_dot(7:8), q_dot_val(7:8)));
% N_spart_val = double(subs(N_spart, q_dot(7:8), q_dot_val(7:8)));
% 
% fprintf('--- Skew Symmetric Check ---\n')
% disp(N_val)
% 
% % Validity
% % h_ijk computing
% K = sc.NumActiveJoints + 6;
% h = sym(zeros(K, K, K));
% for i=1:K
%     fprintf('\t i=%i\n', i);
%     for j=1:K
% 
%         for k=1:K         
%             h_ijk = ( diff(H(i, j), q(k)) - 0.5*diff(H(j, k), q(i)) ) * q_dot(k);
%             h(i, j, k) = h_ijk;
%         end
%     end
% end
% 
% % Check C Matrix
% fprintf('--- C Matrix Check ---')
% for i =1:K
%     t1 = 0;
%     t1_spart = 0;
%     t2 = 0;
% 
%     for j=1:K   
%         t1 = t1 + C(i, j)*q_dot(j);
%         t1_spart = t1_spart + C_spart(i, j)*q_dot(j);
% 
%         for k=1:K
%             t2 = t2 + h(i, j, k)*q_dot(j);
%         end
%     end
% 
%     t1 = double(subs(t1, [q; q_dot], [q_val; q_dot_val]));
%     t1_spart = double(subs(t1_spart, [q; q_dot], [q_val; q_dot_val]));
%     t2 = double(subs(t2, [q; q_dot], [q_val; q_dot_val]));
% 
%     fprintf(['\nRow %i:\n ' ...
%              '\t-Computed: %f\n' ...
%              '\t-SPART: %f\n' ...
%              '\t-Check: %f\n'], i, t1, t1_spart, t2);
% end

%% --- Forward Dyn ---
% FORCES
f0 = [0; 0; 0]; % Force on baseC
n0 = [0; 0; 0.1]; % Torque on base
tau_qm=[0; 0]; % Joints torque

% SPART
%Gravity
g=9.8; %[m s-2]

%External forces (includes gravity and assumes z is the vertical direction)
wF0=[n0; f0];
wFm=[zeros(6,robotSpart.n_links_joints)];

%Joint torques
tauq0=zeros(6,1);

t0 = double(subs(t0, [q, q_dot], [q_val, q_dot_val]));
tm = double(subs(tm, [q, q_dot], [q_val, q_dot_val]));
P0 = double(subs(P0, [q, q_dot], [q_val, q_dot_val]));
pm = double(subs(pm, [q, q_dot], [q_val, q_dot_val]));
I0 = double(subs(I0, [q, q_dot], [q_val, q_dot_val]));
Im = double(subs(Im, [q, q_dot], [q_val, q_dot_val]));
Bij = double(subs(Bij, [q, q_dot], [q_val, q_dot_val]));
Bi0 = double(subs(Bi0, [q, q_dot], [q_val, q_dot_val]));

%Forward Dynamics
[u0dot_FD,umdot_FD] = FD(tauq0,tau_qm,wF0,wFm,t0,tm,P0,pm,I0,Im,Bij,Bi0,q_dot_val(1:6),qm_dot_val,robotSpart);


% TOOLBOX
F = [f0;n0;tau_qm];
q_ddot = sc.forwardDynamics(F);


fprintf("\n### Foward Dynamics ###\n")
fprintf('-- Computed --\n')
disp(q_ddot)

fprintf('-- SPART -- \n')
disp([u0dot_FD(4:6); u0dot_FD(1:3); umdot_FD])

fprintf("Same result: %i\n", all(round(q_ddot, 5) == round([u0dot_FD(4:6); u0dot_FD(1:3); umdot_FD], 5)))



