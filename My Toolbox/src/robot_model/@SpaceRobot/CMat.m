function [C, app_data] = CMat(obj, varargin)
    %CMat Compute the space robot non-linear forces mat
    %   C = MassMat(SR) computes mass matrix for SR current configuration and speeds
    %
    %   C = MassMat(SR, Q) computes mass matrix for SR given configuration Q. Assume joint speed
    %   to be zero.
    %
    %   C = MassMat(SR, Q, QDOT) computes mass matrix for SR given configuration Q and speeds QDOT.
    %
    %   Input format expected:
    %   - Joint configuration, Q - N-by-1 vector
    %   - Speed configuration, QDOT - N-by-1 vector
    %
    %   Output:
    %       C: Non-Linear Effect Matrix (NxN)s

    % --- Input ---
    narginchk(1, 3);
    N = 6 + obj.NumActiveJoints;
    n = obj.NumActiveJoints;
    nk = obj.NumBodies;

    q = zeros(N, 1);
    q_dot = zeros(N, 1);

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

    qb = q(1:6);
    qm = q(7:end);

    qb_dot = q_dot(1:6);
    qm_dot = q_dot(7:end);

    app_data = struct(); % Data for the whole appendage

    % --- A_i_i-1, A_i-i-1_dot, Omega_i ---
    % Base twist
    w_b = qb_dot(4:6); % Base angular rate
    Omega_b = blkdiag(skew(w_b), skew(w_b)); % blkdiag(skew(w_b), skew(w_b)) TODO IMPORTANT: CHECK DEFINITION
%     Omega_b(1:3, 1:3) = zeros(3, 3); 

    tb = obj.Base.P * qb_dot; % Base twist

    app_data.base.t = tb;

    Ab_b = obj.Base.A; % Base twist propagation matrix, base frame
    Ab_k = obj.Base.RotM.' * Ab_b; % Base twist propagation matrix, Appendage frame

    Ab_dot_k = obj.Base.RotM.' * (Omega_b * Ab_b - Ab_b * Omega_b); % Appendage frame

    % Anchor point
    t0k = Ab_k * tb; % Anchor point speed
    w_0k = skew(t0k(4:6));
    Omega_0k = blkdiag(w_0k, w_0k); % Anchor angular speed

    app_data.anchor.Ab_k = Ab_k; % Base twist propagation matrix, Appendage frame
    app_data.anchor.Ab_dot_k = Ab_dot_k; % Appendage frame
    app_data.anchor.t = t0k; % Anchor point speed
    omega_skew = skew(app_data.anchor.t(4:6));
    app_data.anchor.Omega = blkdiag(omega_skew, omega_skew); % Anchor angular speed

    % Init mats
    app_data.t_array = zeros(6, 1, nk);
    app_data.Omega_array = zeros(6, 6, nk);

    app_data.A_array = zeros(6, 6, nk); % A_array(:, :, i): A_i_i-1
    app_data.A_dot_array = zeros(6, 6, nk); % A_dot_array(:, :, i): A_dot_i_i-1

    % Init prev data for first joint
    A_dot_prev = Omega_0k * obj.Bodies{1}.A - obj.Bodies{1}.A * Omega_0k;
    t_prev = t0k;

    for i = 1:nk
        body = obj.Bodies{i};

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
            A_dot_prev = Omega_i * obj.Bodies{i + 1}.A - obj.Bodies{i + 1}.A * Omega_i;
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
    
    A_1_b  = obj.Bodies{1}.RotM.' * [eye(3), -skew([0.5; 0; 0]); zeros(3, 3), eye(3)];


    % For each body
    for i = nk:-1:1
        body = obj.Bodies{i};

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
            body_j = obj.Bodies{j};
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
            body_j = obj.Bodies{j};
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
            Pb = obj.Base.P;
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

    Mb = obj.Base.M;
    Mb_dot = Omega_b * Mb * Ev;
    Cbb = Pb.' * (Mb * Omega_b + Mb_dot + Ha) * Pb;

    Mb_h = Mb + A_1_b' * M1_h * A_1_b;
    Mb_dot_h = Mb_dot + A_1_b' * M1_dot_h * A_1_b;
    H1b = A_1_b' * (M1_h * Adot_1_b + H1 * A_1_b);

    C = [Cbb, Cba; Cab, Ca];
