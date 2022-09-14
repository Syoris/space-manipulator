function [tau_b, tau_m] = inverseDynamics(obj, varargin)
    %inverseDynamics Compute required joint torques for desired motion.
    %   TAU = inverseDynamics(SR) computes joint torques TAU
    %   required for SR add current configuration. Considers joints position and speed. Assume
    %   acceleration to be zeros
    %
    %   TAU = inverseDynamics(SR, Q) computes the required joint
    %   torques for SR add given configuration Q. Assume joint speed and acceleration to be zero.
    %
    %   TAU = inverseDynamics(SR, Q, QDOT) computes the joint
    %   torques required for SR given the joint configuration Q
    %   and joint velocities QDOT while assuming zero joint accelerations
    %   and no external forces.
    %
    %   TAU = inverseDynamics(SR, Q, QDOT, QDDOT) computes the
    %   joint torques required for SR given the joint
    %   configuration Q, joint velocities QDOT and joint accelerations
    %   QDDOT while assuming no external forces are applied.
    %
    %   Input format expected:
    %   - Joint configuration, Q - N-by-1 vector
    %   - Joint velocities, QDOT - N-by-1 vector
    %   - Joint accelerations, QDDOT - N-by-1 vector
    %
    %   Output:
    %       tau_b: Base torque in base frame [fx; fy; fz; nx; ny; nz]
    %       tau_m: Joint torques [tau_1; ... ; tau_n]

    % --- Input ---
    narginchk(1, 4);
    N = 6 + obj.NumActiveJoints;
    n = obj.NumActiveJoints;

    q = zeros(N, 1);
    q_dot = zeros(N, 1);
    q_ddot = zeros(N, 1);

    if nargin == 1
        q = obj.q;
        q_dot = obj.q_dot;
    end

    if nargin >= 2
        q = varargin{1};
    end

    if nargin >= 3
        q_dot = varargin{2};
    end

    if nargin == 4
        q_ddot = varargin{3};
    end

    qb = q(1:6);
    qm = q(7:end);

    qb_dot = q_dot(1:6);
    qm_dot = q_dot(7:end);

    qb_ddot = q_ddot(1:6);
    qm_ddot = q_ddot(7:end);

    % --- Base Twist ---
    w_b = qb_dot(4:6); % Base angular rate
    Omega_b = blkdiag(skew(w_b), skew(w_b)); % blkdiag(skew(w_b), skew(w_b)) TODO IMPORTANT: CHECK DEFINITION

    tb = obj.Base.P * qb_dot; % Base twist
    tb_dot = obj.Base.P * qb_ddot + Omega_b * obj.Base.P * qb_dot; % Base accel

    wen_cstr = zeros(6, 1); % Base constraint wrench

    % --- Appendage k Dynamic --- % TODO Repeat for each appendage
    app_data = struct(); % Data for the whole appendage

    Ab_b = obj.Base.A; % Base twist propagation matrix, base frame
    Ab_k = obj.Base.RotM.' * Ab_b; % Base twist propagation matrix, Appendage frame

    Ab_dot_k = obj.Base.RotM.' * (Omega_b * Ab_b - Ab_b * Omega_b); % Appendage frame

    t0 = Ab_k * tb; % Anchor point speed
    t0_dot = Ab_k * tb_dot + Ab_dot_k * tb; % Anchor point accel

    app_data.anchor.Ab_k = Ab_k; % Base twist propagation matrix, Appendage frame
    app_data.anchor.Ab_dot_k = Ab_dot_k; % Appendage frame
    app_data.anchor.t = t0; % Anchor point speed
    app_data.anchor.t_dot = t0_dot; % Anchor point accel
    omega_skew = skew(app_data.anchor.t(4:6));
    app_data.anchor.Omega = blkdiag(omega_skew, omega_skew); % Anchor angular speed

    % Init mats
    nk = obj.NumBodies; % Number of bodies in the appendage

    app_data.t_array = zeros(6, 1, nk);
    app_data.t_dot_array = zeros(6, 1, nk);
    app_data.Omega_array = zeros(6, 6, nk);

    app_data.A_array = zeros(6, 6, nk); % A_array(:, :, i): A_i_i-1
    app_data.A_dot_array = zeros(6, 6, nk); % A_dot_array(:, :, i): A_dot_i_i-1

    % Init prev data for first joint
    A_dot_prev = app_data.anchor.Omega * obj.Bodies{1}.A - obj.Bodies{1}.A * app_data.anchor.Omega;
    t_prev = app_data.anchor.t;
    t_dot_prev = app_data.anchor.t_dot;

    % --- Twist propagation ---
    for i = 1:nk
        body = obj.Bodies{i};

        R = body.RotM.'; % Rotation matrix from parent to current

        % Joint vals. Set to 0 if joint is fixed
        jnt_idx = body.Joint.Q_id;

        if jnt_idx > 0
            qi = qm(jnt_idx);
            qi_dot = qm_dot(jnt_idx);
            qi_ddot = qm_ddot(jnt_idx);
        else
            qi = 0;
            qi_dot = 0;
            qi_ddot = 0;
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

        ti_dot = A_i * t_dot_prev + A_dot_i * t_prev + P_i * qi_ddot + Omega_i * P_i * qi_dot;

        % Update matrices
        app_data.t_array(:, :, i) = ti;
        app_data.t_dot_array(:, :, i) = ti_dot;
        app_data.Omega_array(:, :, i) = Omega_i;

        app_data.A_array(:, :, i) = A_i;
        app_data.A_dot_array(:, :, i) = A_dot_i;

        % Update prev values
        if i < nk
            A_dot_prev = Omega_i * obj.Bodies{i + 1}.A - obj.Bodies{i + 1}.A * Omega_i;

            t_prev = ti;
            t_dot_prev = ti_dot;
        end

    end

    % --- Force propagation ---
    app_data.wen_array = zeros(6, 1, nk); % Wrench array
    app_data.tau_array = zeros(1, n); % Torque array. tau_array(:, 2) = torque of Joint 2. Only contains active joints

    wen_next = zeros(6, 1); % Wrench at the end-effect
    A_next = app_data.A_array(:, :, 3).'; % From i to i+1
    Ev = blkdiag(zeros(3, 3), eye(3));

    for i = nk:-1:1
        body = obj.Bodies{i};
        jnt_idx = body.Joint.Q_id;

        Omega_i = app_data.Omega_array(:, :, i);
        ti = app_data.t_array(:, :, i);
        ti_dot = app_data.t_dot_array(:, :, i);
        Mi = body.M;

        P_i = body.P; % Joint rate propagation matrix

        gamma = Omega_i * Mi * Ev * ti;
        wen_i = Mi * ti_dot + gamma;
        wen_i = wen_i + A_next.' * wen_next;

        tau_i = P_i' * wen_i;

        % Update mats
        app_data.wen_array(:, :, i) = wen_i;

        if jnt_idx > 0
            app_data.tau_array(:, jnt_idx) = tau_i;
        end

        % Next values
        A_next = app_data.A_array(:, :, i); % From i to i-1
        wen_next = wen_i;
    end

    % Base torque
    wen_cstr = wen_cstr - (app_data.A_array(:, :, 1) * app_data.anchor.Ab_k).' * app_data.wen_array(:, :, 1);

    w_base = obj.Base.M * tb_dot + Omega_b * obj.Base.M * Ev * tb - wen_cstr;
    tau_b = obj.Base.P.' * w_base;

    tau_m = app_data.tau_array.';
end
