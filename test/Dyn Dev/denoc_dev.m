%% DeNOC Dev - DeNOC alog development
load 'SR2.mat'
sr2.homeConfig;

%% Inverse Dynamics
clc
[tau_b, tau_m, app_data_1] = sr2.inverseDynamics(sr2.q, sr2.q_dot, [q0_ddot; qm_ddot]);

tb = app_data_1.base.t;
tb_dot = app_data_1.base.t_dot;
wen_base = app_data_1.base.wen;

% run spart_twist_test.m

fprintf('### TWIST ###\n')
fprintf('--- Base ---\n')
t0_S = [t0_S(4:6); t0_S(1:3)];
fprintf('[SPART, DeNOC (CoM in I)]\n')  
disp([t0_S, tb, tb_2])  
if ~isequal(round(t0_S, 5), round(tb, 5))
        fprintf('ERROR\n')
else
    fprintf('OK\n')
end    

for i=1:3
    body = sr2.Bodies{i};  
    fprintf('--- Body: %s ---\n', body.Name)
    
    t_S = [tm_S(4:6, i); tm_S(1:3, i)];

    [R, ~] = tr2rt(sr2.getTransform(body.Name, 'symbolic', false, 'targetFrame', 'inertial'));
    t_D = app_data_2.t_array(:, :, i);
    t_com = t_D(1:3) + skew(t_D(4:6)) * (body.CenterOfMass.');

    t_com_I = blkdiag(R, R) * [t_com; t_D(4:6)];
    
    fprintf('[SPART, DeNOC (CoM in I)]\n')  
    disp([t_S, t_com_I])  
    if ~isequal(round(t_S, 5), round(t_com_I, 5))
        fprintf('ERROR\n')  
    else
        fprintf('OK\n')
    end               
end

% fprintf('### ACCEL ###\n')
% fprintf('--- Base ---\n')
% t0_dot_S = [t0_dot_S(4:6); t0_dot_S(1:3)];
% if ~isequal(round(t0_dot_S, 5), round(tb_dot, 5))
%     fprintf('ERROR\n')
%     fprintf('[SPART, DeNOC]\n')  
%     disp([t0_dot_S, tb_dot])    
% else
%     fprintf('OK\n')
%     fprintf('[SPART, DeNOC]\n')  
%     disp([t0_dot_S, tb_dot])   
% end    
% 
% for i=1:3
%     body = sr2.Bodies{i};  
%     fprintf('--- Body: %s ---\n', body.Name)
%     
%     t_dot_S = [tm_dot_S(4:6, i); tm_dot_S(1:3, i)];
%     r = body.CenterOfMass.';
% 
%     [R, ~] = tr2rt(sr2.getTransform(body.Name, 'symbolic', false, 'targetFrame', 'inertial'));
%     t_o_dot = app_data_2.t_dot_array(:, :, i);
%     t_o = app_data_2.t_array(:, :, i);
% 
%     t_o_dot_I = blkdiag(R, R) * t_o_dot;
% 
%     t_com_dot = t_o_dot(1:3) + skew(t_o_dot(4:6)) * r + cross(t_o(4:6), cross(t_o(4:6), r));
%     t_com_dot_I = blkdiag(R, R) * [t_com_dot; t_o_dot(4:6)];
%     
%     fprintf('[SPART, DeNOC (CoM in I)]\n')  
%     disp([t_dot_S, t_com_dot_I])  
% 
%     if ~isequal(round(t_dot_S, 5), round(t_com_dot_I, 5))
%         fprintf('ERROR\n')       
%     else
%         fprintf('OK\n')    
%     end        
% end
% 
% fprintf('--- Wrenches ---\n')
% fprintf('Base\n')
% wq_tilde0 = [wq_tilde0(4:6); wq_tilde0(1:3)];
% [R, ~] = tr2rt(sr2.getTransform(sr2.BaseName, 'symbolic', false, 'targetFrame', 'inertial'));
% wen_base_I = blkdiag(R, R) * wen_base;
% 
% fprintf('[SPART, DeNOC]\n')  
% disp([wq_tilde0, wen_base_I])   
% if ~isequal(round(wq_tilde0, 5), round(wen_base_I, 5))
%     fprintf('ERROR\n')
% else
%     fprintf('OK\n') 
% end    
% 
% for i=1:3
%     body = sr2.Bodies{i};  
%     fprintf('%s\n', body.Name)
%     
%     w_S = [wq_tilde(4:6, i); wq_tilde(1:3, i)];
%     
%     r = body.CenterOfMass.';
%     [R, ~] = tr2rt(sr2.getTransform(body.Name, 'symbolic', false, 'targetFrame', 'inertial'));
%     
%     w_o = app_data_2.wen_array(:, :, i);
%     w_o_I = blkdiag(R, R) * w_o;
%     
%     w_com = w_o;
%     w_com(4:6) = w_o(4:6) + skew(-r)*w_o(1:3);
%     w_com_I = blkdiag(R, R) * w_com;
%     
%     fprintf('[SPART, DeNOC (CoM), DeNOC (o)]\n')  
%     disp([w_S, w_com_I, w_o_I])    
% end
% 
% fprintf('--- Torques ---\n')
% fprintf('Base\n')
% tau0_S = [tau0_S(4:6); tau0_S(1:3)];
% 
% [R, ~] = tr2rt(sr2.getTransform(sr2.BaseName, 'symbolic', false, 'targetFrame', 'inertial'));
% tau_b_I = blkdiag(R, eye(3)) * tau_b;
% 
% fprintf('[SPART, DeNOC]\n')
% disp([tau0_S, tau_b_I])
% 
% fprintf('[SPART, DeNOC]\n')
% disp([taum_S, tau_m])



%% Mass Matrix
clc

% % --- Mat initialization ---
% Ma = zeros(6, 6);
% nk = sr2.NumBodies;
% n = sr2.NumActiveJoints;
% 
% Dbb = zeros(6, 6);
% Dba = zeros(6, n);
% Da = zeros(n, n);
% 
% % --- For each appendage ---
% app_data = struct(); % Data for the whole appendage
% 
% app_data.A_array = zeros(6, 6, nk + 1); % A_array(:, :, i): A_i_(i-1), in i frame. Updated at the end of for loop
% % app_data.R_array = zeros(3, 3, nk+1); % R_array(:, :, i): R_i_(i-1), rotation mat from i -> i-1
% app_data.M_array = zeros(6, 6, nk + 1); % Mass_hat array
% 
% app_data.A_array(:, :, end) = zeros(6, 6); % A matrix from payload to ee
% % app_data.R_array(:, :, end) = eye(3); % R matrix from payload to ee
% app_data.M_array(:, :, end) = zeros(6, 6); % Payload mass matrix
% 
% for i = nk:-1:1
%     body_i = sr2.Bodies{i};
%     jnt_idx_i = body_i.Joint.Q_id;
% 
%     M_next = app_data.M_array(:, :, i + 1); % Next link M_hat matrix
% 
%     %     R = app_data.R_array(:, :, nk+1).'; % Rot from i+1 -> i. Diag form
% 
%     A = app_data.A_array(:, :, i + 1); % A_(i+1)_i, in i+1 frame
%     %     A = R * A; % A_(i+1)_i, in i+1 frame
% 
%     M_ik = body_i.M + A.' * M_next * A;
% 
%     P_i = body_i.P;
% 
%     if jnt_idx_i > 0
%         % Compute Dak
%         A_i_j = eye(6);
% 
%         for j = i:-1:1
%             body_j = sr2.Bodies{j};
%             jnt_idx_j = body_j.Joint.Q_id;
% 
%             if jnt_idx_j > 0
%                 Da(jnt_idx_i, jnt_idx_j) = P_i.' * M_ik * A_i_j * body_j.P;
%                 Da(jnt_idx_j, jnt_idx_i) = Da(jnt_idx_i, jnt_idx_j).';
% 
%                 R_j = body_j.RotM.'; % R from j-1 -> j
%                 A_j = body_j.A; % A_j_j-1. j-1 frame
% 
%                 A_i_j = A_i_j * (R_j * A_j); % Compute A_i_(j-1)
%             end
% 
%         end
% 
%         % Compute Dba
%         Dba(:, jnt_idx_i) = sr2.Base.P.' * sr2.Base.A.' * A_i_j.' * M_ik * P_i;
%     end
% 
%     % Update matrices
%     app_data.A_array(:, :, i) = body_i.RotM.' * body_i.A;
%     app_data.M_array(:, :, i) = M_ik; % Payload mass matrix    (Not needed to save??)
%     %     app_data.R_array(:, :, nk) = ;
% end
% 
% A_1_b = app_data.A_array(:, :, 1) * sr2.Base.A;
% 
% Ma = Ma + A_1_b.' * M_ik * A_1_b;
% 
% % Base matrix
% Dbb = sr2.Base.P.' * (sr2.Base.M + Ma) * sr2.Base.P;
% 
% D = [Dbb, Dba; Dba.', Da];
D = sr2.MassMat;
H = sr2.H;

run spart_twist_test.m

fprintf("--- Mass Matrix ---\n")
fprintf('SPART:\n')
disp(H_spart)
fprintf('DeNOC\n')
disp(D)
fprintf('Symb\n')
disp(H)


if isequal(round(H_spart, 5), round(D, 5)) && isequal(round(H_spart, 5), round(H, 5))
    fprintf('All Mat equal\n')
elseif ~isequal(round(D, 5), round(H, 5))
    fprintf('Symb and DeNOC diff ERROR\n')
else
    fprintf('DeNOC and Symb same, Spart diff')
end
    


%% C Matrix
clc

nk = sr2.NumBodies;
n = sr2.NumActiveJoints;

% --- A_i_i-1, A_i-i-1_dot, Omega_i ---
% Base twist
w_b = qb_dot(4:6); % Base angular rate
Omega_b = blkdiag(skew(w_b), skew(w_b)); % blkdiag(skew(w_b), skew(w_b)) TODO IMPORTANT: CHECK DEFINITION

tb = sr2.Base.P * qb_dot; % Base twist

Ab_b = sr2.Base.A; % Base twist propagation matrix, base frame
Ab_k = sr2.Base.RotM.' * Ab_b; % Base twist propagation matrix, Appendage frame

Ab_dot_k = sr2.Base.RotM.' * (Omega_b * Ab_b - Ab_b * Omega_b); % Appendage frame

% Anchor point
t0k = Ab_k * tb; % Anchor point speed
w_0k = skew(t0k(4:6));
Omega_0k = blkdiag(w_0k, w_0k); % Anchor angular speed

% Init mats
app_data = struct(); % Data for the whole appendage
app_data.t_array = zeros(6, 1, nk);
app_data.Omega_array = zeros(6, 6, nk);

app_data.A_array = zeros(6, 6, nk); % A_array(:, :, i): A_i_i-1
app_data.A_dot_array = zeros(6, 6, nk); % A_dot_array(:, :, i): A_dot_i_i-1

% Init prev data for first joint
A_dot_prev = Omega_0k * sr2.Bodies{1}.A - sr2.Bodies{1}.A * Omega_0k;
t_prev = t0k;

for i = 1:nk
    body = sr2.Bodies{i};

    R = body.RotM.'; % Rotation matrix from parent to current

    % Joint vals. Set to 0 if joint is fixed
    jnt_idx = body.Joint.Q_id;

    if jnt_idx > 0
        qi = qm(jnt_idx);
        qi_dot = qm_dot(jnt_idx);
    else
        qi = 0;
        qi_dot = 0;
    end

    % Propagation matrices
    A_i = R * body.A; % twist propagation, frame i
    A_dot_i = R * A_dot_prev; % accel propagation, frame i
    P_i = body.P; % Joint rate propagation matrix

    % twist
    ti = A_i * t_prev + P_i * qi_dot;
    wi = ti(4:6);
    wi_skew = skew(wi);
    Omega_i = blkdiag(wi_skew, wi_skew);

    % Update matrices
    app_data.t_array(:, :, i) = ti;
    app_data.Omega_array(:, :, i) = Omega_i;

    app_data.A_array(:, :, i) = A_i;
    app_data.A_dot_array(:, :, i) = A_dot_i;

    % Update prev values
    if i < nk
        A_dot_prev = Omega_i * sr2.Bodies{i + 1}.A - sr2.Bodies{i + 1}.A * Omega_i;
        t_prev = ti;
    end

end

% --- Mi_h, Mi_dot_h, Hi ---
Ca = zeros(n, n);
Cbb = zeros(6, 6);
Cba = zeros(6, n);
Cab = zeros(n, 6);
Ha = zeros(6, 6);

Ev = blkdiag(zeros(3, 3), eye(3));

app_data.A_array(:, :, end + 1) = zeros(6, 6);
app_data.A_dot_array(:, :, end + 1) = zeros(6, 6);

app_data.M_h_array = zeros(6, 6, nk + 1);
app_data.M_dot_h_array = zeros(6, 6, nk + 1);
app_data.H_array = zeros(6, 6, nk + 1);

% Specify payload
app_data.M_h_array(:, :, end) = zeros(6, 6);
app_data.M_dot_h_array(:, :, end) = zeros(6, 6);
app_data.H_array(:, :, end) = zeros(6, 6);

% Body 1 to base
A_1_b = app_data.A_array(:, :, 1) * Ab_k;

% For each body
for i = nk:-1:1
    body = sr2.Bodies{i};

    % Body vars
    jnt_idx_i = body.Joint.Q_id;
    Omega_i = app_data.Omega_array(:, :, i);
    ti = app_data.t_array(:, :, i);
    Ai = app_data.A_array(:, :, i + 1); % A_(i+1)_i in frame i+1
    Ai_dot = app_data.A_dot_array(:, :, i + 1); % Adot_(i+1)_i in frame i+1
    Pi = body.P;

    % Next body effect
    Mi_h_next = app_data.M_h_array(:, :, i + 1);
    Mi_dot_h_next = app_data.M_dot_h_array(:, :, i + 1);
    Hi_next = app_data.H_array(:, :, i + 1);
    
    Mi = body.M;
    Mi(4:6, 4:6) = body.InertiaM;

    % Mi_h
    Mi_h = body.M + Ai.' * Mi_h_next * Ai;

    % Mi_dot_h
    
    Mi_dot = Omega_i * Mi * Ev;
    Mi_dot_h = Mi_dot + Ai.' * Mi_dot_h_next * Ai;

    % Hi
    Hi = Ai.' * (Mi_h_next * Ai_dot + Hi_next * Ai);

    % Update struct
    app_data.M_h_array(:, :, i) = Mi_h;
    app_data.M_dot_h_array(:, :, i) = Mi_dot_h;
    app_data.H_array(:, :, i) = Hi;

    % Ca, LHS OK
    A_i_j = eye(6);
    Adot_i_j = zeros(6);

    for j = i - 1:-1:1
        body_j = sr2.Bodies{j};
        jnt_idx_j = body_j.Joint.Q_id;
        Omega_j = app_data.Omega_array(:, :, j);
        Pj = body_j.P;

        A_j = app_data.A_array(:, :, j + 1); % A_(j+1)_j
        Adot_j = app_data.A_dot_array(:, :, j + 1); % Adot_(j+1)_j

        Adot_i_j = Adot_i_j * A_j + A_i_j * Adot_j;

        A_i_j = A_i_j * A_j; % A_i_(j+1) *  A_(j+1)_j

        if jnt_idx_i > 0
            Ca(jnt_idx_i, jnt_idx_j) = Pi.' * ((Hi + Mi_dot_h) * A_i_j + Mi_h * (Adot_i_j + A_i_j * Omega_j)) * Pj;
        end

    end

    % Ca, RHS OK
    A_j_i = eye(6);

    for j = i:nk
        body_j = sr2.Bodies{j};
        jnt_idx_j = body_j.Joint.Q_id;
        Omega_j = app_data.Omega_array(:, :, j);
        Pj = body_j.P;

        Mj_h = app_data.M_h_array(:, :, j);
        Mj_dot_h = app_data.M_dot_h_array(:, :, j);
        Hj = app_data.H_array(:, :, j);

        if jnt_idx_j > 0
            Ca(jnt_idx_i, jnt_idx_j) = Pi.' * A_j_i.' * (Hj + Mj_h * Omega_j + Mj_dot_h) * Pj;
        end

        A_j_i = app_data.A_array(:, :, j + 1) * A_j_i;
    end
    
    % Cba, Cab, NOK for base angular speed
    if jnt_idx_i > 0
        Pb = sr2.Base.P;
        Cba(:, jnt_idx_i) = Pb.' * A_1_b.' * (A_i_j.' * Hi + A_i_j.' * Mi_h * Omega_i + A_i_j.' * Mi_dot_h) * Pi;

        Cab(jnt_idx_i, :) = Pi.' * ((Mi_h * Adot_i_j + (Hi + Mi_dot_h) * A_i_j) * A_1_b ...
            + Mi_h * A_i_j * (Omega_b * A_1_b + A_1_b * Omega_b)) * Pb;
    end

end

A_1_0 = app_data.A_array(:, :, 1);
Adot_1_0 = app_data.A_dot_array(:, :, 1);
Adot_1_b = Adot_1_0 * Ab_k + A_1_0 * Ab_dot_k;

H1 = app_data.H_array(:, :, 1);
M1_h = app_data.M_h_array(:, :, 1);
M1_dot_h = app_data.M_dot_h_array(:, :, 1);

Ha = Ha + A_1_b.' * (H1 * A_1_b + M1_h * (Adot_1_b + A_1_b * Omega_b) + M1_dot_h * A_1_b);

Mb = sr2.Base.M;
Mb_dot = Omega_b * Mb * Ev;
Cbb = Pb.' * (Mb * Omega_b + Mb_dot + Ha) * Pb;

Mb_h = Mb + A_1_b' * M1_h * A_1_b;
Mb_dot_h = Mb_dot + A_1_b' * M1_dot_h * A_1_b;
H1b = A_1_b' * (M1_h*Adot_1_b + H1*A_1_b);

C = [Cbb, Cba; Cab, Ca];
h = C * q_dot;

% run spart_twist_test.m
% Cbb_spart = C_spart(1:6, 1:6);
% 
% M0_tilde_s = [M0_tilde(4:6, 4:6), M0_tilde(4:6, 1:3); M0_tilde(1:3, 4:6), M0_tilde(1:3, 1:3)]; 
% H_s = [child_con_C0(4:6, 4:6), child_con_C0(4:6, 1:3); child_con_C0(1:3, 4:6), child_con_C0(1:3, 1:3)];
% Mdot0_tilde_s = [Mdot0_tilde(4:6, 4:6), Mdot0_tilde(4:6, 1:3); Mdot0_tilde(1:3, 4:6), Mdot0_tilde(1:3, 1:3)];
% 
% fprintf("Base Mass Hat\n")
% fprintf("SPART\n")
% disp(M0_tilde_s)
% fprintf("DeNOC\n")
% disp(Mb_h)
% 
% fprintf("Base Mass Dot Hat\n")
% fprintf("SPART\n")
% disp(Mdot0_tilde_s)
% fprintf("DeNOC\n")
% disp(Mb_dot_h)
% 
% 
% fprintf("H_1b\n")
% fprintf("SPART\n")
% disp(H_s)
% fprintf("DeNOC\n")
% disp(H1b)



% h_S = C_spart * q_dot;
% 
% fprintf("--- h ---\n")
% fprintf("SPART\n")
% disp(h_S)
% fprintf("DeNOC\n")
% disp(h)
% 
% fprintf("--- C ---\n")
% fprintf("SPART\n")
% disp(C_spart)
% fprintf("DeNOC\n")
% disp(C)