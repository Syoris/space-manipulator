function D = MassM(sr_info, q, A)
    %MassM Computes SR Mass Matrix

    q0 = q(1:6);
    qm = q(7:end);

    % --- Initialization ---
    N = sr_info.N;
    n = sr_info.n;
    nk = sr_info.nk; % Number of bodies in the appendage

    A_0b = A{1}; % Twist propagation matrix, base to anchor, anchor frame
    Am = A{2};

    Mb = sr_info.M{1};
    Mm = sr_info.M{2};

    Pb = sr_info.P{1};
    Pm = sr_info.P{2};

    % Init mats
    Ma = zeros(6, 6);
    Dbb = zeros(6, 6);
    Dba = zeros(6, n);
    Da = zeros(n, n);
    Ev = blkdiag(zeros(3, 3), eye(3));

    M_array = zeros(6, 6, nk); % Mass_hat array

    % Payload
    M_next = zeros(6, 6); % Payload mass matrix, Next link M_hat matrix
    A_next = zeros(6, 6); % A matrix from payload to ee, % A_(i+1)_i, in i+1 frame

    % --- Computation ---
    for i = nk:-1:1
        jnt_idx_i = sr_info.jnt_idx(i);

        Mi = Mm(:, :, i);
        Pi = Pm(:, :, i);

        M_ik = Mi + A_next.' * M_next * A_next;

        if jnt_idx_i > 0
            % Compute Dak
            A_i_j = eye(6);

            for j = i:-1:1
                jnt_idx_j = sr_info.jnt_idx(j);

                Pj = Pm(:, :, j);

                if jnt_idx_j > 0
                    Da(jnt_idx_i, jnt_idx_j) = Pi.' * M_ik * A_i_j * Pj;
                    Da(jnt_idx_j, jnt_idx_i) = Da(jnt_idx_i, jnt_idx_j).';

                    A_j = Am(:, :, jnt_idx_j); % A_j_(j-1) in j frame

                    A_i_j = A_i_j * A_j; % Compute A_i_(j-1)
                end

            end

            % Compute Dba
            Dba(:, jnt_idx_i) = Pb.' * A_0b.' * A_i_j.' * M_ik * Pi;
        end

        M_array(:, :, i) = M_ik; % Payload mass matrix

        M_next = M_ik;
        A_next = Am(:, :, i);
    end

    A_1b = Am(:, :, 1) * A_0b;

    Ma = Ma + A_1b.' * M_ik * A_1b;

    % Base matrix
    Dbb = Pb.' * (Mb + Ma) * Pb;

    D = [Dbb, Dba; Dba.', Da];

end
