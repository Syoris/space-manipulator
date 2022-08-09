%% Computing of jacobians
% To compare jacobians with the ones computed by SPART. Run SPART_example.m
% to compare. Make sure initial configs are the same
run 'SPART_example.m'
load 'SC_2DoF.mat'

% Initial condition
syms 'Rx' 'Ry' 'Rz' 'r' 'p' 'y' 'qm1' 'qm2'
syms 'Rx_d' 'Ry_d' 'Rz_d' 'wx' 'wy' 'wz' 'qm1_d' 'qm2_d'

r0= [Rx;Ry;Rz];
delta0 = [r;p;y];
qm= [qm1; qm2];

q = [r0; delta0; qm];

q_dot = [Rx_d; Ry_d; Rz_d; wx; wy; wz; qm1_d; qm2_d];
qm_dot = [qm1_d; qm2_d];

q_val = [r0_S; zeros(3, 1); qm_S];
q_dot_val = [u0_S; um_S];

% qm=[pi/6; -pi/4];
% R0=eye(3);  %Rotation from Base-spacecraft to inertial
% r0=[0; 0; 0]; %Position of the base-spacecraft

sc.JointsConfig = qm';
sc.BaseConfig = [r0'; 0, 0, 0];

%% Jacobians
comPoses = sc.getCoMPosition();
Jacobians = sc.comJacobians();
for i=1:sc.NumLinks
    linkName = sc.LinkNames{i};
    J_i = Jacobians.(linkName);
    
    fprintf('\n##### Link %i #####\n', i);
    fprintf('Mine:\n')
    disp(J_i);
    fprintf('\n')
    disp(double(subs(J_i, q, q_val)));

    fprintf('SPART:\n')
    disp(J_S{i});
   
%     fprintf('--- J_%i1', i)
%     J_i(1:3, 4:6)
%     J_S{i}(1:3, 4:6)
% 
%     fprintf('--- J_%i2', i)
%     J_i(1:3, 7:7+sc.NumActiveJoints-1)
%     J_S{i}(1:3, 7:7+sc.NumActiveJoints-1)
% 
%     % J_i3
%     fprintf('--- J_%i3', i)
%     J_i(4:6, 7:7+sc.NumActiveJoints-1)
%     J_S{i}(4:6, 7:7+sc.NumActiveJoints-1)
end

%% H - Mass Matrix
H = sc.massMatrix();

% Spart comp
fprintf('\n##### H - Mass Matrix #####\n')
fprintf('--- Computed ---\n');
disp(H);
fprintf('\n')
H_val = double(subs(H, q, q_val));
disp(H_val);

fprintf('--- SPART ---\n');
disp(H_spart)
%% C - Non-Linear Effect 
K = 6+sc.NumActiveJoints;
C = sym(zeros(K));

fprintf('Computing C...\n');
for i=1:K
    fprintf('\t i=%i\n', i);
    for j=1:K
        c_ij = 0;

        for k=1:K         

            c_ijk = 1/2 * ( diff(H(i, j), q(k)) + diff(H(i, k), q(j)) - diff(H(j, k), q(i)) ) * q_dot(k);
            
            c_ij = c_ij + c_ijk;
        end

        C(i, j) = c_ij;
    end
end

fprintf('\n##### C Matrix #####\n')
fprintf('--- Computed ---\n');
disp(C);
fprintf('\n')
C_val = double(subs(C, [q; q_dot], [q_val; q_dot_val]));
disp(C_val);

fprintf('--- SPART ---\n');
disp(C_spart)
