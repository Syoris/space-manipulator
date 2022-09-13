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

% --- Initialize ---
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
omega_skew = skew(app_data.anchor.t(4:6));
app_data.anchor.Omega = blkdiag(omega_skew, omega_skew); % Anchor point accel

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

% --- Force propagation ---
app_data.w_array = zeros(6, 1, nk); % Wrench array
app_data.tau_array = zeros(1, nk); % Torque array. tau_array(:, 2) = torque of Joint 2

w_next = zeros(6, 1); % Wrench at the end-effect
A_next = app_data.A_array(:, :, 3).'; % From i to i+1
Ev = blkdiag(zeros(3, 3), eye(3));

for i = nk:-1:1
    body = sc.Bodies{i};

    Omega_i = app_data.Omega_array(:, :, i);
    ti = app_data.t_array(:, :, i);
    ti_dot = app_data.t_dot_array(:, :, i);
    Mi = body.M;

    P_i = body.P; % Joint rate propagation matrix

    gamma = Omega_i * Mi * Ev * ti;
    w = Mi * ti_dot + gamma;
    w_i = w + A_next.' * w_next;

    tau_i = P_i' * w_i;

    % Update mats
    app_data.w_array(:, :, i) = w_i;
    app_data.tau_array(:, i) = tau_i;

    % Next values
    A_next = app_data.A_array(:, :, i); % From i to i-1
    w_next = w_i;
end
 
% Base torque
w_cstr = w_cstr - (app_data.A_array(:, :, 1) * app_data.anchor.Ab_k).' * app_data.w_array(:, :, 1);

w_base = sc.Base.M * tb_dot + Omega_b * sc.Base.M * Ev * tb - w_cstr;
tau_base = sc.Base.P.' * w_base;


run spart_twist_test.m
