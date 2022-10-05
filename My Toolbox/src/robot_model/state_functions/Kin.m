function [t, t_dot, Omega, A, A_dot] = Kin(sr_info, q, q_dot, q_ddot, R)
    % Kin   Compute kinetics of the SR
    %
    %   -sr_info    Struct w/ all fix parameter of SR w/ fields
    %
    %
    %                   - jnt_idx: Idx of each joint generalized coord. -1 if fixed.
    %
    %                   - N: SR otal # of DoF (=n+6)
    %
    %                   - nk: Manipulator # of bodies
    %
    %                   - n: Manipulator # of DoF
    %
    %                   - A: Cell Array w/ twist propagation matrices {Ab, Am}.
    %                           - Ab: (6, 6, 1); Base to Anchor point, base frame
    %                           - Am: (6, 6, nk); A_i_(i-1), (i-1) frame
    %                   - M: Cell Array w/ mass matrices {Mb, Mm}.
    %                           - Mb: (6, 6, 1); Base mass mat
    %                           - Mm: (6, 6, nk); Body i mass mat, body i frame
    %
    %                   - P: Cell Array w/ joint rate propagation matrices {Pb, Pm}.
    %                            - Pb: (6, 6, 1); Base, base frame
    %                            - Pm: (6, 1, nk); Body i, body i frame
    %
    %   -R           Cell Array w/ roation matrices {Rb, Rm}.
    %                           - Rb: (6, 6, 1); Rot matrix from appendage anchor to base frame
    %                           - Rm: (6, 1, nk); Rot matrix from frame i to frame i-1
    %   OUTPUT
    %   -t          Space Robot twist {tb, tm}. tm: (6, 1, nk)
    %
    %   -t_dot      Space Robot accel {tb_dot, tm_dot}. tm_dot: (6, 1, nk)
    %
    %   -Omega      Omega array {Omega_b, Omega_m}. Omega_m: (6, 6, nk)
    %
    %   -A          Twist propaagtion matrices {Ab, Am}. A_i_(i-1) in frame i
    %                   -Ab: Manipulator anchor to base, anchor frame
    %                   -Am: Array w/ manip matrices
    %
    %   -A_dot      Derivative of twist propagtion matrices {Ab_dot, Am_dot}. A_i_(i-1) in frame i
    %                   -Ab_dot: Manipulator anchor to base, anchor frame
    %                   -Am_dot: Array w/ manip matrices

    N = sr_info.N;
    n = sr_info.n;
    nk = sr_info.nk; % Number of bodies in the appendage

    qb = q(1:6);
    qm = q(7:end);

    qb_dot = q_dot(1:6);
    qm_dot = q_dot(7:end);

    qb_ddot = q_ddot(1:6);
    qm_ddot = q_ddot(7:end);

    % --- Base Twist ---
    w_b = qb_dot(4:6); % Base angular rate
    Omega_b = zeros(6, 6);  % blkdiag(skew(w_b), skew(w_b)) TODO IMPORTANT: CHECK DEFINITION
    Omega_b(1:3, 1:3) = skewSym(w_b);
    Omega_b(4:6, 4:6) = Omega_b(1:3, 1:3);

    Pb = sr_info.P{1};
    tb = Pb * qb_dot; % Base twist
    tb_dot = Pb * qb_ddot + Omega_b * Pb * qb_dot; % Base accel

    % --- Appendage k Dynamic --- % TODO Repeat for each appendage
    % Anchor point
    Ab_b = sr_info.A{1}; % Base to anchor twist propagation matrix, base frame
    Rb = R{1}; % Rotation matrix from appendage to base
    Ab_k = Rb.' * Ab_b; % Base to appendage twist propagation matrix, Appendage frame

    Ab_dot_k = Rb.' * (Omega_b * Ab_b - Ab_b * Omega_b); % Appendage frame

    t0 = Ab_k * tb; % Anchor point speed
    t0_dot = Ab_k * tb_dot + Ab_dot_k * tb; % Anchor point accel

    w_0 = t0(4:6);

    Omega_0 = zeros(6, 6);  % blkdiag(skew(w_b), skew(w_b)) TODO IMPORTANT: CHECK DEFINITION
    Omega_0(1:3, 1:3) = skewSym(w_0);
    Omega_0(4:6, 4:6) = Omega_0(1:3, 1:3);

    % Setup arrays
    tm_array = zeros(6, 1, nk);
    tm_dot_array = zeros(6, 1, nk);
    Omegam_array = zeros(6, 6, nk);

    Am_array = zeros(6, 6, nk); % Am_array(:, :, i): A_i_i-1, in i frame
    Am_dot_array = zeros(6, 6, nk); % Am_dot_array(:, :, i): A_dot_i_i-1, in i frame

    % Init prev data for first joint
    Am = sr_info.A{2}; % A array, in frame (i-1)
    Rm = R{2}; % Manipulator R array
    Pm = sr_info.P{2}; % Pm_i array, in frame (i)

    A_dot_prev = Omega_0 * Am(:, :, 1) - Am(:, :, 1) * Omega_0;
    t_prev = t0;
    t_dot_prev = t0_dot;

    % --- Twist propagation ---
    for i = 1:nk
        % Body values
        jnt_idx = sr_info.jnt_idx(i);

        r_i = Rm(:, :, i).'; % Rotation matrix from (i) to (i-1)
        
        R_i = zeros(6, 6);
        R_i(1:3, 1:3) = r_i;
        R_i(4:6, 4:6) = r_i;

%         R_i = blkdiag(R_i, R_i).'; % Rotation matrix from (i-1) to (i)

        A_i = Am(:, :, i); % Propagation matrix, (i-1) frame
        A_i = R_i * A_i; % twist propagation, frame i

        A_dot_i = R_i * A_dot_prev; % accel propagation, frame i

        P_i = Pm(:, :, i); % Joint rate propagation matrix

        % Config
        if jnt_idx > 0
            qi = qm(jnt_idx);
            qi_dot = qm_dot(jnt_idx);
            qi_ddot = qm_ddot(jnt_idx);
        else
            qi = 0;
            qi_dot = 0;
            qi_ddot = 0;
        end

        % twist
        ti = A_i * t_prev + P_i * qi_dot;
        wi = ti(4:6);
%         wi_skew = skew(wi);

        Omega_i = zeros(6, 6);
        Omega_i(1:3, 1:3) = skewSym(wi);
        Omega_i(4:6, 4:6) = Omega_i(1:3, 1:3);
        
%         Omega_i = blkdiag(wi_skew, wi_skew);

        ti_dot = A_i * t_dot_prev + A_dot_i * t_prev + P_i * qi_ddot + Omega_i * P_i * qi_dot;

        % Update Arrays
        tm_array(:, :, i) = ti;
        tm_dot_array(:, :, i) = ti_dot;
        Omegam_array(:, :, i) = Omega_i;

        Am_array(:, :, i) = A_i;
        Am_dot_array(:, :, i) = A_dot_i;

        % Update prev values
        if i < nk
            A_next = Am(:, :, i + 1); % Twist prop. matrice from (i+1) to i, i frame
            A_dot_prev = Omega_i * A_next - A_next * Omega_i;

            t_prev = ti;
            t_dot_prev = ti_dot;
        end

    end

    t = {tb, tm_array};
    t_dot = {tb_dot, tm_dot_array};

    Omega = {Omega_b, Omegam_array};

    A = {Ab_k, Am_array};
    A_dot = {Ab_dot_k, Am_dot_array};
end
