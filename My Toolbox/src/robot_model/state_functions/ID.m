function [tau, wen] = ID(sr_info, t, t_dot, Omega, A, A_dot)
    % ID  Compute Inverse Dynamics without depending on class object
    %
    %   Parameters
    %   -sr_info    Struct w/ all fix parameter of SR w/ fields
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
    %
    %   OUTPUT
    %
    %   -tau        Torques {tb, tm}. tb: (6x1); tm (nx1)
    %
    %   -wen        Wrenches applied to joints {wen_b, wen_m}. wen_b: (6x1); wen_m (6x1xn)
    %

    %% Initialization
    N = sr_info.N;
    n = sr_info.n;
    nk = sr_info.nk; % Number of bodies in the appendage

    % Base Parameters
    tb = t{1};
    tb_dot = t_dot{1};
    Omega_b = Omega{1}; % blkdiag(skew(w_b), skew(w_b)) TODO IMPORTANT: CHECK DEFINITION

    Mb = sr_info.M{1};
    Pb = sr_info.P{1};
    A_0b = A{1}; % Twist propagation matrix, base to anchor, anchor frame

    wen_cstr = zeros(6, 1); % Base constraint wrench
    wen_array = zeros(6, 1, nk); % Wrench array
    tau_array = zeros(1, n); % Torque array. tau_array(:, 2) = torque of Joint 2. Only contains active joints

    wen_next = zeros(6, 1); % Wrench at the end-effect
    A_next = zeros(6, 6); % ee load to ee, set to zeros
    Ev = blkdiag(zeros(3, 3), eye(3));

    tm = t{2};
    tm_dot = t_dot{2};
    Omega_m = Omega{2};
    Am = A{2};
    Am_dot = A_dot{2};

    Mm = sr_info.M{2};
    Pm = sr_info.P{2};

    % Force Propagation
    for i = nk:-1:1
        % Body values
        jnt_idx = sr_info.jnt_idx(i);
        Omega_i = Omega_m(:, :, i);

        ti = tm(:, :, i);
        ti_dot = tm_dot(:, :, i);
        Mi = Mm(:, :, i);
        P_i = Pm(:, :, i);

        gamma = Omega_i * Mi * Ev * ti;
        wen_i = Mi * ti_dot + gamma;
        wen_i = wen_i + A_next.' * wen_next;

        tau_i = P_i' * wen_i;

        % Update mats
        wen_array(:, :, i) = wen_i;

        if jnt_idx > 0
            tau_array(:, jnt_idx) = tau_i;
        end

        % Next values
        A_next = Am(:, :, i); % From i to i-1
        wen_next = wen_i;
    end

    % Base torque
    A_10 = Am(:, :, 1); % Twist propagation from body 1 to manip anchor, body 1 frame

    wen_cstr = wen_cstr - (A_10 * A_0b).' * wen_array(:, :, 1);

    wen_b = Mb * tb_dot + Omega_b * Mb * Ev * tb - wen_cstr;
    tau_b = Pb.' * wen_b;

    tau_m = tau_array.';

    tau = {tau_b, tau_m};

    wen = {wen_b, wen_array};
end
