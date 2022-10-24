function C = CorMat(sr_info, wb, Omega, A, A_dot, R)
    % CORMAT  Compute C matrix of coriolis effect and non-linear terms
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
    %   -wb         Base angular speed
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
    %   -R         Rot Matrice: {Rb, Ra}
    %                   Rb: from base to inertial frame
    %                   Ra: from anchor to base frame
    %
    %   OUTPUT
    %
    %   -tau        Torques {tb, tm}. tb: (6x1); tm (nx1)
    %
    %   -wen        Wrenches applied to joints {wen_b, wen_m}. wen_b: (6x1); wen_m (6x1xn)

    %% Initialization
    n = sr_info.n;
    nk = sr_info.nk; % Number of bodies in the appendage

    Rb = R{1};
    Ra = R{2};

    % Base Parameters
    Omega_b = Omega{1}; % blkdiag(skew(w_b), skew(w_b)) TODO IMPORTANT: CHECK DEFINITION
    % Omega_b(1:3, 1:3) = Omega_b(4:6, 4:6);

    Mb = sr_info.M{1};
    Pb = sr_info.P{1};

    Mm = sr_info.M{2};
    Pm = sr_info.P{2};

    % --- Propagation matrices ---
    % Anchor to base
    A_ab_b = sr_info.A{1} * [Rb.', zeros(3, 3); zeros(3, 3), eye(3)]; % Base to anchor twist propagation matrix, base frame
    A_ab_a = Ra.' * A_ab_b; % Base to anchor twist propagation matrix, Appendage frame

    A_ab_dot_b = zeros(6, 6);
    A_ab_dot_b(1:3, 4:6) = -skewSym(sr_info.A{1}(1:3, 4:6) * wb);
    A_ab_dot_a = Ra.' * A_ab_dot_b;

    % Body 1 to base
    A_1a = A{2}(:, :, 1);
    A_1a_dot = A_dot{2}(:, :, 1);

    A_1b = A_1a * A_ab_a; % A_1b = A_1a * A_ab_a
    A_1b_dot = A_1a_dot * A_ab_a + A_1a * A_ab_dot_a; % A_1b_dot = A_1a_dot * A_ab_a + A_1a * A_ab_dot_a

    % --- Mi_h, Mi_dot_h, Hi ---
    Ca = zeros(n, n);
    Cbb = zeros(6, 6);
    Cba = zeros(6, n);
    Cab = zeros(n, 6);
    Ha = zeros(6, 6);

    Ev = zeros(6, 6);
    Ev(4:6, 4:6) = [1 0 0; 0 1 0; 0 0 1];

    % app_data.A_array(:, :, end + 1) = zeros(6, 6);
    % app_data.A_dot_array(:, :, end + 1) = zeros(6, 6);

    % Payload to ee propagation
    A_payload = zeros(6, 6);
    A_dot_payload = zeros(6, 6);

    Mi_h_payload = zeros(6, 6);
    Mi_dot_h_payload = zeros(6, 6);
    Hi_payload = zeros(6, 6);

    % Set payload as next body
    Ai_next = A_payload;
    Ai_dot_next = A_dot_payload;

    Mi_h_next = Mi_h_payload;
    Mi_dot_h_next = Mi_dot_h_payload;
    Hi_next = Hi_payload;

    % Arrays to store vals
    M_h_array = zeros(6, 6, nk); % Mass_hat
    M_dot_h_array = zeros(6, 6, nk); %Mass_dot_hat:
    H_array = zeros(6, 6, nk);

    Omega_m = Omega{2};
    Am = A{2};
    Am_dot = A_dot{2};

    % For each body
    for i = nk:-1:1
        % --- Body Values ---
        jnt_idx_i = sr_info.jnt_idx(i);

        Pi = Pm(:, :, i);
        Mi = Mm(:, :, i);
        Omega_i = Omega_m(:, :, i);

        % Mi_h
        Mi_h = Mi + Ai_next.' * Mi_h_next * Ai_next;

        % Mi_dot_h
        Mi_dot = Omega_i * Mi * Ev;
        Mi_dot_h = Mi_dot + Ai_next.' * Mi_dot_h_next * Ai_next;

        % Hi
        Hi = Ai_next.' * (Mi_h_next * Ai_dot_next + Hi_next * Ai_next);

        % Update Arrays
        M_h_array(:, :, i) = Mi_h;
        M_dot_h_array(:, :, i) = Mi_dot_h;
        H_array(:, :, i) = Hi;

        % --- Ca, LHS ---
        A_ij = eye(6);
        A_dot_ij = zeros(6);

        for j = i - 1:-1:1
            jnt_idx_j = sr_info.jnt_idx(j);

            Pj = Pm(:, :, j);
            Omega_j = Omega_m(:, :, j);

            A_j = Am(:, :, j + 1); % A_(j+1)_j
            Adot_j = Am_dot(:, :, j + 1); % Adot_(j+1)_j

            A_dot_ij = A_dot_ij * A_j + A_ij * Adot_j;

            A_ij = A_ij * A_j; % A_i_(j+1) *  A_(j+1)_j

            if jnt_idx_i > 0
                Ca(jnt_idx_i, jnt_idx_j) = Pi.' * ((Hi + Mi_dot_h) * A_ij + Mi_h * (A_dot_ij + A_ij * Omega_j)) * Pj;
            end

        end

        % --- Ca, RHS ---
        A_ji = eye(6);

        for j = i:nk
            jnt_idx_j = sr_info.jnt_idx(j);
            Pj = Pm(:, :, j);
            Omega_j = Omega_m(:, :, j);

            Mj_h = M_h_array(:, :, j);
            Mj_dot_h = M_dot_h_array(:, :, j);
            Hj = H_array(:, :, j);

            if jnt_idx_j > 0
                Ca(jnt_idx_i, jnt_idx_j) = Pi.' * A_ji.' * (Hj + Mj_h * Omega_j + Mj_dot_h) * Pj;
            end

            if j == nk
                A_j = A_payload;
            else
                A_j = Am(:, :, j + 1);
            end

            A_ji = A_j * A_ji; % A_(j+1)_i = A_(j+1)_j * A_j_i
        end

        % --- Cba, Cab ---
        if jnt_idx_i > 0
            A_bj = A_1b.' * A_ij.';
            
            A_i1 = A_ij;
            A_i1_dot = A_dot_ij;
            
%             A_1b;
%             A_1b_dot;
            
            A_ib = A_i1*A_1b;
            A_ib_dot = A_i1_dot*A_1b + A_i1*A_1b_dot;

            Cba(:, jnt_idx_i) = Pb.' * A_bj * (Hi + Mi_h * Omega_i + Mi_dot_h) * Pi;           
                  
            Cab(jnt_idx_i, :) = Pi.' * ( Mi_h*A_ib_dot + Mi_h*A_ib*Omega_b + Mi_dot_h*A_ib + Hi*A_ib);
           
             
        end

        % Update values
        Ai_next = Am(:, :, i); % From i to i-1
        Ai_dot_next = Am_dot(:, :, i); % From i to i-1

        Mi_h_next = Mi_h;
        Mi_dot_h_next = Mi_dot_h;
        Hi_next = Hi;
    end

    % Impact on base
    H1 = H_array(:, :, 1);
    M1_h = M_h_array(:, :, 1);
    M1_dot_h = M_dot_h_array(:, :, 1);

    Ha = Ha + A_1b.' * (H1 * A_1b + M1_h * (A_1b_dot + A_1b * Omega_b) + M1_dot_h * A_1b);

    % C Mat
    Mb_dot = Omega_b * Mb * Ev;
    Cbb = Pb.' * (Mb * Omega_b + Mb_dot + Ha) * Pb;

    C = [Cbb, Cba; Cab, Ca];

end
