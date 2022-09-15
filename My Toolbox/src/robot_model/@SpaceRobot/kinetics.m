function res = kinetics(obj, q, q_dot, q_ddot)
    %kinetics Compute the velocity of all the body inertial frame, in body frame

    qb = q(1:6);
    qm = q(7:end);

    qb_dot = q_dot(1:6);
    qm_dot = q_dot(7:end);

    qb_ddot = q_ddot(1:6);
    qm_ddot = q_ddot(7:end);

    % --- Base Twist ---
    app_data = struct(); % Data for the whole appendage

    w_b = qb_dot(4:6); % Base angular rate
    Omega_b = blkdiag(skew(w_b), skew(w_b)); % blkdiag(skew(w_b), skew(w_b)) TODO IMPORTANT: CHECK DEFINITION

    tb = obj.Base.P * qb_dot; % Base twist
    tb_dot = obj.Base.P * qb_ddot + Omega_b * obj.Base.P * qb_dot; % Base accel

    wen_cstr = zeros(6, 1); % Base constraint wrench

    app_data.base.t = tb;
    app_data.base.t_dot = tb_dot;

    % --- Appendage k Dynamic --- % TODO Repeat for each appendage
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

    res = app_data;

end
