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

    nk = obj.NumBodies; % Number of bodies in the appendage

    % --- Twist Propagation ---
    app_data = obj.kinetics(q, q_dot, q_ddot);

    tb = app_data.base.t;
    tb_dot = app_data.base.t_dot;

    w_b = tb(4:6); % Base angular rate
    Omega_b = blkdiag(skew(w_b), skew(w_b)); % blkdiag(skew(w_b), skew(w_b)) TODO IMPORTANT: CHECK DEFINITION

    % --- Force propagation ---
    wen_cstr = zeros(6, 1); % Base constraint wrench

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

    app_data.base.wen = w_base;
    app_data.base.tau = tau_b;

    tau_m = app_data.tau_array.';
end
