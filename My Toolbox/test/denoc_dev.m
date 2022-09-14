load 'SC2_2DoF.mat'
sc.homeConfig;

% Init mats
sc.Base.initBase();

for i = 1:length(sc.Bodies)
    sc.Bodies{i}.initBody();
end

%% Inverse Dynamics
newConfig = true;

% --- Initial cond ---
if newConfig
    % Position
    sc.q = zeros(8, 1);
    sc.qm = diag([1; 1]) * rand(2, 1);

    % Speed
    sc.q0_dot = diag([1; 1; 0; 0; 0; 0]) * rand(6, 1);
    sc.qm_dot = diag([1; 1]) * rand(2, 1);

    % Accel
    q0_ddot = diag([0; 0; 0; 0; 0; 0]) * rand(6, 1);
    qm_ddot = diag([0; 0]) * rand(2, 1);
end

[tau_b, tau_m] = sc.inverseDynamics(sc.q, sc.q_dot, [q0_ddot; qm_ddot]);

run spart_twist_test.m

fprintf('--- Torques ---\n')
fprintf('Base\n')
tau0 = [tau0_S(4:6); tau0_S(1:3)];
fprintf('[SPART, DeNOC]\n')
disp([tau0, tau_b])

fprintf('[SPART, DeNOC]\n')
disp([taum_S, tau_m])

%% --- Initialize ---
% Used for development of algorithm

% Add ee coord
qm = [sc.qm; 0];
qm_dot = [sc.qm_dot; 0];
qm_ddot = [qm_ddot; 0];

% Base twist
w_b = sc.q_dot(4:6); % Base angular rate
Omega_b = blkdiag(skew(w_b), skew(w_b)); % blkdiag(skew(w_b), skew(w_b)) TODO IMPORTANT: CHECK DEFINITION

tb = sc.Base.P * sc.q0_dot;
tb_dot = sc.Base.P * q0_ddot + Omega_b * sc.Base.P * sc.q0_dot;
w_cstr = zeros(6, 1); % Base wrench

% --- For each appendage ---
app_data = struct(); % Data for the whole appendage

Ab_b = sc.Base.A; % Base twist propagation matrix, base frame
Ab_k = sc.Base.RotM.' * Ab_b; % Base twist propagation matrix, Appendage frame

Omega_b = blkdiag(skew(w_b), skew(w_b));
Ab_dot_k = sc.Base.RotM.' * (Omega_b * Ab_b - Ab_b * Omega_b); % Appendage frame

t0 = Ab_k * tb; % Anchor point speed
t0_dot = Ab_k * tb_dot + Ab_dot_k * tb; % Anchor point accel

app_data.anchor.Ab_k = Ab_k; % Base twist propagation matrix, Appendage frame
app_data.anchor.Ab_dot_k = Ab_dot_k; % Appendage frame
app_data.anchor.t = t0; % Anchor point speed
app_data.anchor.t_dot = t0_dot; % Anchor point accel
omega_0k = skew(app_data.anchor.t(4:6));
app_data.anchor.Omega = blkdiag(omega_0k, omega_0k); % Anchor point accel

% Init mats
nk = 3;
app_data.t_array = zeros(6, 1, nk);
app_data.t_dot_array = zeros(6, 1, nk);
app_data.Omega_array = zeros(6, 6, nk);

app_data.A_array = zeros(6, 6, nk); % A_array(:, :, i): A_i_i-1
app_data.A_dot_array = zeros(6, 6, nk); % A_dot_array(:, :, i): A_dot_i_i-1

% Init prev data for first joint
A_dot_prev = app_data.anchor.Omega * sc.Bodies{1}.A - sc.Bodies{1}.A * app_data.anchor.Omega;
t_prev = app_data.anchor.t;
t_dot_prev = app_data.anchor.t_dot;

% --- Twist propagation ---
for i = 1:nk
    body = sc.Bodies{i};

    R = body.RotM.'; % Rotation matrix from parent to current

    % Joint vals
    qi = qm(i);
    qi_dot = qm_dot(i);
    qi_ddot = qm_ddot(i);

    % Propagation matrices
    A_i = R * body.A; % twist propagation, frame i
    A_dot_i = R * A_dot_prev; % accel propagation, frame i
    P_i = body.P; % Joint rate propagation matrix

    % twist
    ti = A_i * t_prev + P_i * qi_dot; % TODO: Check how to handle specified speed and fix joints
    wi = ti(4:6);
    wi_skew = skew(wi);
    Omega_i = blkdiag(wi_skew, wi_skew);

    ti_dot = A_i * t_dot_prev + A_dot_i * t_prev + P_i * qi_ddot + Omega_i * P_i * qi_dot;

    % Update matrices
    app_data.t_array(:, :, i) = ti;
    app_data.t_dot_array(:, :, i) = ti_dot;
    app_data.Omega_array(:, :, i) = Omega_i;

    app_data.A_array(:, :, i) = A_i;
    app_data.A_dot_array(:, :, i) = A_dot_i;

    % Update prev values
    if i < nk
        A_dot_prev = Omega_i * sc.Bodies{i + 1}.A - sc.Bodies{i + 1}.A * Omega_i;

        t_prev = ti;
        t_dot_prev = ti_dot;
    end

end

%
% % --- Force propagation ---
% app_data.w_array = zeros(6, 1, nk); % Wrench array
% app_data.tau_array = zeros(1, nk); % Torque array. tau_array(:, 2) = torque of Joint 2
%
% w_next = zeros(6, 1); % Wrench at the end-effect
% A_next = app_data.A_array(:, :, 3).'; % From i to i+1
% Ev = blkdiag(zeros(3, 3), eye(3));
%
% for i = nk:-1:1
%     body = sc.Bodies{i};
%
%     Omega_i = app_data.Omega_array(:, :, i);
%     ti = app_data.t_array(:, :, i);
%     ti_dot = app_data.t_dot_array(:, :, i);
%     Mi = body.M;
%
%     P_i = body.P; % Joint rate propagation matrix
%
%     gamma = Omega_i * Mi * Ev * ti;
%     w = Mi * ti_dot + gamma;
%     w_i = w + A_next.' * w_next;
%
%     tau_i = P_i' * w_i;
%
%     % Update mats
%     app_data.w_array(:, :, i) = w_i;
%     app_data.tau_array(:, i) = tau_i;
%
%     % Next values
%     A_next = app_data.A_array(:, :, i); % From i to i-1
%     w_next = w_i;
% end
%
% % Base torque
% w_cstr = w_cstr - (app_data.A_array(:, :, 1) * app_data.anchor.Ab_k).' * app_data.w_array(:, :, 1);
%
% w_base = sc.Base.M * tb_dot + Omega_b * sc.Base.M * Ev * tb - w_cstr;
% tau_base = sc.Base.P.' * w_base;

%% Mass Matrix
clc
% --- Mat initialization ---
Ma = zeros(6, 6);
nk = sc.NumBodies;
n = sc.NumActiveJoints;

Dbb = zeros(6, 6);
Dba = zeros(6, n);
Da = zeros(n, n);

% --- For each appendage ---
app_data = struct(); % Data for the whole appendage

app_data.A_array = zeros(6, 6, nk + 1); % A_array(:, :, i): A_i_(i-1), in i frame. Updated at the end of for loop
% app_data.R_array = zeros(3, 3, nk+1); % R_array(:, :, i): R_i_(i-1), rotation mat from i -> i-1
app_data.M_array = zeros(6, 6, nk + 1); % Mass_hat array

app_data.A_array(:, :, end) = zeros(6, 6); % A matrix from payload to ee
% app_data.R_array(:, :, end) = eye(3); % R matrix from payload to ee
app_data.M_array(:, :, end) = zeros(6, 6); % Payload mass matrix

for i = nk:-1:1
    body_i = sc.Bodies{i};
    jnt_idx_i = body_i.Joint.Q_id;

    M_next = app_data.M_array(:, :, i + 1); % Next link M_hat matrix

    %     R = app_data.R_array(:, :, nk+1).'; % Rot from i+1 -> i. Diag form

    A = app_data.A_array(:, :, i + 1); % A_(i+1)_i, in i+1 frame
    %     A = R * A; % A_(i+1)_i, in i+1 frame

    M_ik = body_i.M + A.' * M_next * A;

    P_i = body_i.P;

    if jnt_idx_i > 0
        % Compute Dak
        A_i_j = eye(6);

        for j = i:-1:1
            body_j = sc.Bodies{j};
            jnt_idx_j = body_j.Joint.Q_id;

            if jnt_idx_j > 0
                Da(jnt_idx_i, jnt_idx_j) = P_i.' * M_ik * A_i_j * body_j.P;
                Da(jnt_idx_j, jnt_idx_i) = Da(jnt_idx_i, jnt_idx_j).';

                R_j = body_j.RotM.'; % R from j-1 -> j
                A_j = body_j.A; % A_j_j-1. j-1 frame

                A_i_j = A_i_j * (R_j * A_j); % Compute A_i_(j-1)
            end

        end

        % Compute Dba
        Dba(:, jnt_idx_i) = sc.Base.P.' * sc.Base.A.' * A_i_j.' * M_ik * P_i;
    end

    % Update matrices
    app_data.A_array(:, :, i) = body_i.RotM.' * body_i.A;
    app_data.M_array(:, :, i) = M_ik; % Payload mass matrix    (Not needed to save??)
    %     app_data.R_array(:, :, nk) = ;
end

A_1_b = app_data.A_array(:, :, 1) * sc.Base.A;

Ma = Ma + A_1_b.' * M_ik * A_1_b;

% Base matrix
Dbb = sc.Base.P.' * (sc.Base.M + Ma) * sc.Base.P;

D = [Dbb, Dba; Dba.', Da];

D = sc.MassMat;
fprintf("--- Mass Matrix ---\n")
fprintf('SPART:\n')
disp(H_spart)
fprintf('DeNOC\n')
disp(D)

if isequal(round(H_spart, 5), round(D, 5))
    fprintf('Mass Matrix OK\n')
else
    fprintf('Mass matrix ERROR\n')
end

%% C Matrix
clc

newConfig = false;

% --- Configuration ---
if newConfig
    % Position
    sc.q = zeros(8, 1);
    sc.qm = diag([1; 1]) * rand(2, 1);

    % Speed
    sc.q0_dot = diag([1; 1; 1; 1; 1; 1]) * rand(6, 1);
    sc.qm_dot = diag([1; 1]) * rand(2, 1);
end

q = sc.q;
q_dot = sc.q_dot;

qb = q(1:6);
qm = q(7:end);

qb_dot = q_dot(1:6);
qm_dot = q_dot(7:end);

nk = sc.NumBodies;
n = sc.NumActiveJoints;

% --- A_i_i-1, A_i-i-1_dot, Omega_i ---
% Base twist
w_b = qb_dot(4:6); % Base angular rate
Omega_b = blkdiag(skew(w_b), skew(w_b)); % blkdiag(skew(w_b), skew(w_b)) TODO IMPORTANT: CHECK DEFINITION

tb = sc.Base.P * qb_dot; % Base twist

Ab_b = sc.Base.A; % Base twist propagation matrix, base frame
Ab_k = sc.Base.RotM.' * Ab_b; % Base twist propagation matrix, Appendage frame

Ab_dot_k = sc.Base.RotM.' * (Omega_b * Ab_b - Ab_b * Omega_b); % Appendage frame

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
A_dot_prev = Omega_0k * sc.Bodies{1}.A - sc.Bodies{1}.A * Omega_0k;
t_prev = t0k;

for i = 1:nk
    body = sc.Bodies{i};

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
        A_dot_prev = Omega_i * sc.Bodies{i + 1}.A - sc.Bodies{i + 1}.A * Omega_i;
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
    body = sc.Bodies{i};

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

    % Mi_h
    Mi_h = body.M + Ai.' * Mi_h_next * Ai;

    % Mi_dot_h
    Mi_dot = Omega_i * body.M * Ev;
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
        body_j = sc.Bodies{j};
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
        body_j = sc.Bodies{j};
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
        Pb = sc.Base.P;
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

Mb = sc.Base.M;
Mb_dot = Omega_b * Mb * Ev;
Cbb = Pb.' * (Mb * Omega_b + Mb_dot + Ha) * Pb;

C = [Cbb, Cba; Cab, Ca];
h = C * q_dot;

run spart_twist_test.m
h_S = C_spart * q_dot;

fprintf("--- h ---\n")
fprintf("SPART\n")
disp(h_S)
fprintf("DeNOC\n")
disp(h)

fprintf("--- C ---\n")
fprintf("SPART\n")
disp(C_spart)
fprintf("DeNOC\n")
disp(C)