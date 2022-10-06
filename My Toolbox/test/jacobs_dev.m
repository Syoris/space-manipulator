%% jacobs_dev.m
% Jacobians and their derivatives development using DeNOC

NEW_CONF = 1;
qb_conf = [1; 1; 1; 1; 1; 1];
qm_conf = [1; 1];

qb_dot_conf = [1; 1; 1; 1; 1; 1]; % [0; 0; 0; 0; 0; 0], [1; 1; 1; 1; 1; 1]
qm_dot_conf = [1; 1]; % [0; 0], [1; 1]

qb_ddot_conf = [1; 1; 1; 0; 0; 0];
qm_ddot_conf = [1; 1];

%% --- Config ---
clc
close all

if ~exist('sr', 'var')
    fprintf("Loading SR\n")
    load 'SR2.mat'
end

if NEW_CONF
    % Position
    sr.q0 = diag(qb_conf) * rand(6, 1);
    sr.qm = diag(qm_conf) * rand(2, 1);
    
    % Speed
    sr.q0_dot = diag(qb_dot_conf) * rand(6, 1);
    sr.qm_dot = diag(qm_dot_conf) * rand(2, 1);

    % Accel
    q0_ddot = diag(qb_ddot_conf) * rand(6, 1);
    qm_ddot = diag(qm_ddot_conf) * rand(2, 1);
    
    % Set values
    q = sr.q;
    q_dot = sr.q_dot;
    
    qb = q(1:6);
    qm = q(7:end);
    
    qb_dot = q_dot(1:6);
    qm_dot = q_dot(7:end);

    q_ddot = [q0_ddot; qm_ddot];
end

n = sr.NumActiveJoints;
nk = sr.NumBodies;
N = n + 6;

run spart_script
%% Compute J
J = cell(nk, 1);
for i=1:nk
    bodyName = sr.BodyNames{i};
    J{i} = sr.JacobsCoM_FuncHandle.(bodyName)(sr.q);
end

%% Compute J, DeNOC
sr_info = SR2_info();
% 1 - Kinematics
[Rb, Ra, Rm] = RFunc_SR2(q);
Rm = reshape(Rm, 3, 3, []); % Split Rm to nk 3x3 arrays

% 2 - Kinetics
% blkdiag(Rb.', Rb.')*[t0_dot_S(4:6); t0_dot_S(1:3)]

[t, t_dot, Omega, A, A_dot] = Kin(sr_info, q, q_dot, q_ddot, {Rb, Ra, Rm});

% --- Nkl ---
Nkl = eye(6*nk, 6*nk);
for i=2:nk
    A_i = A{2}(:, :, i) ;% A_i_(i-1)
    
    for j=i-1:-1:1       
        if j==i-1
            blkMat = A_i; % Set lower diag to A_i_(i-1)
        else        
            blkMat = Nkl(6*i-5:6*i, 6*(j+1)-5:6*(j+1))*Nkl(6*(j+1)-5:6*(j+1), 6*j-5:6*j);
        end

        Nkl(6*i-5:6*i, 6*j-5:6*j) = blkMat;
    end
end

% --- Nd ---
Nd = zeros(6*nk, nk);
for i=1:nk
    Nd(6*i-5:6*i, i) = sr_info.P{2}(:, :, i);
end


% --- Nbl ---
Ab = zeros(6*nk, 6);
[R_b_I, ~] = tr2rt(sr.getTransform('spacecraftBase', 'TargetFrame', 'inertial', 'Symbolic', false));

A_0b_b = sr_info.A{1} * [R_b_I.', zeros(3, 3); zeros(3, 3), eye(3)]; % Base to anchor twist propagation matrix, base frame
A_0b_k = Ra.' * A_0b_b;  % Base to anchor twist propagation matrix, Appendage frame

A_1b = A{2}(:, :, 1) * A_0b_k; % A_1b = A_10 * A_0b_k
Ab(1:6, :) = A_1b;

Nbl = Nkl*Ab;

% --- Ndb ---
Ndb = sr_info.P{1};

% --- Jacobians ---
J_denoc = cell(3, 1);
J_denoc_I = cell(3, 1);
J_denoc_B = cell(3, 1);

for i=1:3
    Jk = Nkl(6*i-5:6*i, :) * Nd;
    Jb = Nbl(6*i-5:6*i, :) * Ndb;

    Jk = Jk(:, 1:end-1); % TODO, handle fixed joints
    
    J_denoc{i} = [Jb, Jk]; 

    [R_I, ~] = tr2rt(sr.getTransform(sr.BodyNames{i}, 'TargetFrame', 'inertial', 'Symbolic', false));
    J_denoc_I{i} = blkdiag(R_I, R_I)*[Jb, Jk]; 
end

%% Compute J_dot
% --- Nkl_dot ---
Nkl_dot = zeros(6*nk, 6*nk);
for i=2:nk
    A_dot_i = A_dot{2}(:, :, i) ;% A_dot_i_(i-1)
    
    for j=i-1:-1:1       
        if j==i-1
            blkMat = A_dot_i; % Set lower diag to A_i_(i-1)
        else        
            t1 = Nkl_dot(6*i-5:6*i, 6*(j+1)-5:6*(j+1))*Nkl(6*(j+1)-5:6*(j+1), 6*j-5:6*j); % A_dot_i_(j+1) * A_(j+1)_(j)
            t2 = Nkl(6*i-5:6*i, 6*(j+1)-5:6*(j+1))*Nkl_dot(6*(j+1)-5:6*(j+1), 6*j-5:6*j); % A_i_(j+1) * A_dot_(j+1)_(j)
            blkMat = t1 + t2;
        end

        Nkl_dot(6*i-5:6*i, 6*j-5:6*j) = blkMat;
    end
end

% --- Nbl_dot ---
Ab_dot = zeros(6*nk, 6); % Matrix

w_b = qb_dot(4:6); % Base angular rate

A_0b_dot_b = zeros(6, 6);
P_0b = [0.5; 0; 0];

A_0b_dot_b(1:3, 4:6) = -skew(sr_info.A{1}(1:3, 4:6)*w_b);
A_0b_dot_a = Ra.' * A_0b_dot_b;

A_10 = A{2}(:, :, 1);
A_10_dot = A_dot{2}(:, :, 1);

A_1b_dot = A_10_dot * A_0b_k + A_10 * A_0b_dot_a; % A_1b_dot = A_10_dot * A_0b_k + A_10 * A_0b_dot_a

Ab_dot(1:6, :) = A_1b_dot;

Nbl_dot = Nkl_dot*Ab + Nkl*Ab_dot;

% --- Nd_dot ---
Nd_dot = zeros(6*nk, nk);
for i=1:3
    Nd_dot(6*i-5:6*i, i) = Omega{2}(:, :, i)*sr_info.P{2}(:, :, i);
end

% Ndb_dot = Omega{1}*sr_info.P{1};
Ndb_dot = blkdiag(zeros(3, 3), skew(w_b))*sr_info.P{1};

% --- Jacobians derivative ---
J_dot_denoc = cell(3, 1);
J_dot_denoc_I = cell(3, 1);

for i=1:3
    Jb_dot = Nbl_dot(6*i-5:6*i, :) * Ndb + Nbl(6*i-5:6*i, :) * Ndb_dot;

    Jk_dot = Nkl_dot(6*i-5:6*i, :) * Nd + Nkl(6*i-5:6*i, :)*Nd_dot;
    

    Jk_dot = Jk_dot(:, 1:end-1); % TODO, handle fix joints
    
    J_dot_denoc{i} = [Jb_dot, Jk_dot]; 

    [R_I, ~] = tr2rt(sr.getTransform(sr.BodyNames{i}, 'TargetFrame', 'inertial', 'Symbolic', false));
    J_dot_denoc_I{i} = blkdiag(R_I, R_I)*[Jb_dot, Jk_dot]; 
end
%% Check Jacobians
% J, comp w/ symbolic
% compare_jacs(sr, J_S_com, J);

% J, computed with DeNOC
% compare_jacs(sr, J_S, J_denoc);
% compare_jacs(sr, J_S, J_denoc_I);

% J_dot
% compare_jacs(sr, J_dot_S, J_dot_denoc);
compare_jacs(sr, J_dot_S, J_dot_denoc_I);

%% ### Check Velocities and Accel ###
fprintf("\n\n ### Comparing velocities ###\n")
for i=1:3
    accelOk = true;
    speedOk = true;

    ti = t{2}(:, :, i);
    ti_dot = t_dot{2}(:, :, i);

%     ti_S = [tm_S(4:6, i); tm_S(1:3, i)];
%     ti_dot_S = [tm_dot_S(4:6, i); tm_dot_S(1:3, i)];

    ti_S = J_S{i}*q_dot;
    ti_dot_S = J_S{i}*q_ddot + J_dot_S{i} * q_dot;

    [R, ~] = tr2rt(sr.getTransform(sr.BodyNames{i}, 'TargetFrame', 'inertial', 'Symbolic', false));

    ti_S_b = blkdiag(R, R).'*ti_S;
    ti_dot_S_b = blkdiag(R, R).'*ti_dot_S;


    ti_denoc_i = J_denoc{i}*q_dot; % In frame i
    ti_denoc_I = J_denoc_I{i}*q_dot; % In inertial frame

    ti_dot_denoc_i = J_denoc{i}*q_ddot + J_dot_denoc{i}*q_dot;
  
    fprintf("\n### %s ###\n", sr.BodyNames{i})
    fprintf("\t --- Velocities ---\n")
    fprintf('\t Kin     DeNOC  SPART (in body frame)\n')
    disp([ti, ti_denoc_i, ti_S_b])
    
    if ~isequal(round(ti_denoc_i, 5), round(ti, 5))
        fprintf("ERROR, Speed not matching between DeNOC and SPART\n")
        speedOk = false;
    end

    fprintf("\t --- Accel ---\n")
    fprintf('\t Kin     DeNOC      SPART (in body frame)\n')
    disp([ti_dot, ti_dot_denoc_i, ti_dot_S_b])

    if ~isequal(round(ti_dot, 5), round(ti_dot_denoc_i, 5))
        fprintf("ERROR, Accels not matching\n")
        accelOk = false;
    end
end

if speedOk
    fprintf("!!! Speeds ok !!!\n")
else
    fprintf("### Speed error ###\n")
end

if accelOk
    fprintf("!!! Accels ok !!!\n")
else
    fprintf("### Accel error ###\n")
end

%% Compare Results
function compare_jacs(sr, J_1, J_2)
    allOk = true;
    errList = {};

    for i = 1:sr.NumBodies
        bodyName = sr.BodyNames{i};
        J_i1 = J_1{i};
        J_i2 = J_2{i};
        
        fprintf('##### %s #####\n', bodyName);
        fprintf('Jacob 1:\n')      
        disp(J_i1);
        
        fprintf('\n')
        fprintf('Jacob 2:\n')
        disp(J_i2);
        
        if ~isequal(round(J_i1, 5), round(J_i2, 5))
            fprintf("ERROR: Jacobians not matching for body %s\n", bodyName)
            errList{end+1} = bodyName;
            allOk = false;
        else
            fprintf("Jacobians ok for %s\n", bodyName)
        end
        
    end
    if allOk
        fprintf("\n--- All jacobians matching ---\n")
    else
        fprintf("\nERROR: Jacobians not matching for bodies:\n")
        for i=1:length(errList)
            fprintf("\t-%s\n", errList{i})
        end
    end
end
    