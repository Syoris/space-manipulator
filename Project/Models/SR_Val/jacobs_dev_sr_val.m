%% jacobs_dev.m
% Jacobians and their derivatives development using DeNOC

NEW_CONF = 1;
qb_conf = [1; 1; 1; 1; 1; 1];
qm_conf = [1; 1];

qb_dot_conf = [1; 1; 1; 1; 1; 1]; % [0; 0; 0; 0; 0; 0], [1; 1; 1; 1; 1; 1]
qm_dot_conf = [1; 1]; % [0; 0], [1; 1]

qb_ddot_conf = [0; 0; 0; 0; 0; 0];
qm_ddot_conf = [0; 0];

% Sections to run
RUN_JAC = 1;
RUN_VEL = 0;
RUN_DYN = 0;

%% --- Config ---
clc
close all

if ~exist('sr', 'var')
    fprintf("Loading SR\n")
    load 'SR_Val.mat'
end

n = sr.NumActiveJoints;
nk = sr.NumBodies;
N = n + 6;

if NEW_CONF
    % Position
    sr.q0 = diag(qb_conf) * rand(6, 1);
    sr.qm = diag(qm_conf) * rand(n, 1);
    
    % Speed
    sr.q0_dot = diag(qb_dot_conf) * rand(6, 1);
    sr.qm_dot = diag(qm_dot_conf) * rand(n, 1);

    % Accel
    q0_ddot = diag(qb_ddot_conf) * rand(6, 1);
    qm_ddot = diag(qm_ddot_conf) * rand(n, 1);
    
    % Set values
    q = sr.q;
    q_dot = sr.q_dot;
    
    qb = q(1:6);
    qm = q(7:end);
    
    qb_dot = q_dot(1:6);
    qm_dot = q_dot(7:end);

    q_ddot = [q0_ddot; qm_ddot];
end

spart_res = spart_script(sr, 'SR_Val.urdf', q, q_dot, q_ddot);

%% Initialize vals
sr_info = SR_Val_info();
% 1 - Kinematics
[Rb, Ra, Rm] = RFunc_SR_Val(q);
Rm = reshape(Rm, 3, 3, []); % Split Rm to nk 3x3 arrays

% 2 - Kinetics
[t, t_dot, Omega, A, A_dot] = Kin(sr_info, q, q_dot, q_ddot, {Rb, Ra, Rm});

%% Jacobians
if RUN_JAC    
    % --- J ---
    R_arr = cell(nk, 1);
    for i=1:nk
        [R_arr{i}, ~] = tr2rt(sr.getTransform(sr.BodyNames{i}, 'TargetFrame', 'inertial', 'Symbolic', false));
    end
    
    J_i = cell(4, 1);
    J_I = cell(4, 1);
    
    for i=1:nk
        J_I{i} = blkdiag(R_arr{i}, R_arr{i})*Jacobian(sr.BodyNames{i}, sr_info, A, {Rb, Ra});

        J_i{i} = Jacobian(sr.BodyNames{i}, sr_info, A, {Rb, Ra});
    end
      
    fprintf('\n\n--- COMPARING J ---\n')
    compare_jacs(sr, spart_res.J_i, J_i);
    % compare_jacs(sr, spart_res.J_I, J_I);
    
    % --- J_dot ---
    J_dot_i = cell(nk, 1);
    for i=1:nk
        J_dot_i{i} = Jacobian_dot(sr.BodyNames{i}, sr_info, A, A_dot, {Rb, Ra}, qb_dot(4:6), Omega);
    end    
    fprintf('\n\n--- COMPARING J_DOT ---\n')
    compare_jacs(sr, spart_res.J_dot_i, J_dot_i);
end

%% Velocities and Accel
if RUN_VEL
    fprintf("\n\n ### Comparing velocities ###\n")
    for i=1:3
        accelOk = true;
        speedOk = true;
    
        ti = t{2}(:, :, i);
        ti_dot = t_dot{2}(:, :, i);
    
        
        % Spart speed, body frame
        ti_S = spart_res.J_i{i}*q_dot;
        ti_dot_S = spart_res.J_i{i}*q_ddot + spart_res.J_dot_i{i} * q_dot;
    % 
    %     [R, ~] = tr2rt(sr.getTransform(sr.BodyNames{i}, 'TargetFrame', 'inertial', 'Symbolic', false));
    % 
    %     ti_S_b = blkdiag(R, R).'*ti_S;
    %     ti_dot_S_b = blkdiag(R, R).'*ti_dot_S;
    
    
        ti_denoc_i = J_i{i}*q_dot; % In frame i
        ti_denoc_I = J_I{i}*q_dot; % In inertial frame
    
        ti_dot_denoc_i = J_i{i}*q_ddot + J_dot_i{i}*q_dot;
      
        fprintf("\n### %s ###\n", sr.BodyNames{i})
        fprintf("\t --- Velocities ---\n")
        fprintf('\t Kin     DeNOC  SPART (in body frame)\n')
        disp([ti, ti_denoc_i, ti_S])
        
        if ~isequal(round(ti_denoc_i, 5), round(ti, 5))
            fprintf("ERROR, Speed not matching between DeNOC and SPART\n")
            speedOk = false;
        end
    
        fprintf("\t --- Accel ---\n")
        fprintf('\t Kin     DeNOC      SPART (in body frame)\n')
        disp([ti_dot, ti_dot_denoc_i, ti_dot_S])
    
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
end

%% Dynamics
if RUN_DYN
    fprintf('\n\n--- DYNAMICS ---\n')

    % --- D - Mass Matrix ---
    D = MassM(sr_info, q, A);
    if isequal(round(D, 2), round(spart_res.H, 2))
        fprintf("MASS MATRIX MATCHING\n")
    else
        fprintf("ERROR, Mass mat not matching\n")
        disp(D)
        disp(spart_res.H)
    end
    
    % --- Inverse Dyn ---
    [tau, ~] = ID(sr_info, t, t_dot, Omega, A, A_dot);
    tau_b = blkdiag(Rb, eye(3))*tau{1};
    tau_m = tau{2};
    if isequal(round(tau_b, 2), round(spart_res.tau.tau_b, 2))
        fprintf("BASE TORQUE MATCHING\n")
    else
        fprintf("ERROR: base torque not matching\n")
        disp([tau_b, spart_res.tau.tau_b])
    end
    
    if isequal(round(tau_m, 2), round(spart_res.tau.tau_m, 2))
        fprintf("MANIP TORQUE MATCHING\n")
    else
        fprintf("ERROR: manip torque not matching\n")
        disp([tau_m, spart_res.tau.tau_m])
    end

    % --- C - Cor Mat ---
    C = CorMat(sr_info, qb_dot(4:6), Omega, A, A_dot, {Rb, Ra});
    
%     if isequal(round(C, 2), round(spart_res.C, 2))
%         fprintf("C MATRIX MATCHING\n")
%         fprintf("C\n")
%         disp(C)
%         fprintf("SPART\n")
%         disp(spart_res.C)
%     else
%         fprintf("ERROR: C matrices not matching\n")
%         fprintf("C\n")
%         disp(C)
%         fprintf("SPART\n")
%         disp(spart_res.C)
%     end
    
    h_S = spart_res.C*q_dot;
    h = C*q_dot;
    
    [tau, ~] = ID(sr_info, t, t_dot, Omega, A, A_dot);
    h2 = [blkdiag(Rb, eye(3))*tau{1}; tau{2}];
    
    hS2 = [spart_res.tau.tau_b; spart_res.tau.tau_m];
%     fprintf("[FromC \t\t FromID \t C, SPART\t ID, SPART]\n")
%     disp([h, h2, h_S, hS2])
    
    if isequal(round(h, 2), round(h_S, 2))
        fprintf("h MATRIX MATCHING\n")
    else
        fprintf("ERROR: h MATS NOK\n")
    end
    fprintf('ORBOT\t\t SPART\n')
    disp([h, h_S])
    
    
    % if isequal(round(h, 2), round(h_S, 2))
    %     fprintf("h MATRIX MATCHING\n")
    %     disp([h, h_S])
    % else    
    %     fprintf("ERROR: h matrices not matching\n")
    %     fprintf("\th\t\tSPART\n")
    %     disp([h, h_S])
    % end
end


%% Functions
% --- Compare Jacobians ---
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
        fprintf("\nAll jacobians matching\n")
    else
        fprintf("\nERROR: Jacobians not matching for bodies:\n")
        for i=1:length(errList)
            fprintf("\t-%s\n", errList{i})
        end
    end
end
    