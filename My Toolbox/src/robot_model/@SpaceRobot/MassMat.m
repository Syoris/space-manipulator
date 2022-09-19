function D = MassMat(obj, varargin)
    %MassMat Compute the space robot mass matrix
    %   D = MassMat(SR) computes mass matrix for SR current configuration
    %
    %   D = MassMat(SR, Q) computes mass matrix for SR given configuration Q
    %
    %   Input format expected:
    %   - Joint configuration, Q - N-by-1 vector
    %
    %   Output:
    %       D: Mass Matrix (NxN)

    narginchk(1, 2);

    if nargin > 1
        q = varargin{1}
    else
        q = obj.q;
    end

    q0 = q(1:6);
    qm = q(7:end);

    % --- Mat initialization ---
    Ma = zeros(6, 6);
    nk = obj.NumBodies;
    n = obj.NumActiveJoints;

    Dbb = zeros(6, 6);
    Dba = zeros(6, n);
    Da = zeros(n, n);

    % --- For each appendage ---
    app_data = struct(); % Data for the whole appendage

    app_data.A_array = zeros(6, 6, nk + 1); % A_array(:, :, i): A_i_(i-1), in i frame. Updated at the end of for loop
    app_data.M_array = zeros(6, 6, nk + 1); % Mass_hat array

    app_data.A_array(:, :, end) = zeros(6, 6); % A matrix from payload to ee
    app_data.M_array(:, :, end) = zeros(6, 6); % Payload mass matrix

    for i = nk:-1:1
        body_i = obj.Bodies{i};
        jnt_idx_i = body_i.Joint.Q_id;

        M_next = app_data.M_array(:, :, i + 1); % Next link M_hat matrix

        A = app_data.A_array(:, :, i + 1); % A_(i+1)_i, in i+1 frame

        M_ik = body_i.M + A.' * M_next * A;

        P_i = body_i.P;

        if jnt_idx_i > 0
            % Compute Dak
            A_i_j = eye(6);

            for j = i:-1:1
                body_j = obj.Bodies{j};
                jnt_idx_j = body_j.Joint.Q_id;

                if jnt_idx_j > 0
                    Da(jnt_idx_i, jnt_idx_j) = P_i.' * M_ik * A_i_j * body_j.P;
                    Da(jnt_idx_j, jnt_idx_i) = Da(jnt_idx_i, jnt_idx_j).';

                    R_j = body_j.getRotM(qm(jnt_idx_j)).'; % R from j-1 -> j
                    A_j = body_j.A; % A_j_j-1. j-1 frame

                    A_i_j = A_i_j * (R_j * A_j); % Compute A_i_(j-1)
                end

            end

            % Compute Dba
            Dba(:, jnt_idx_i) = obj.Base.P.' * obj.Base.A.' * A_i_j.' * M_ik * P_i;
        end
        
        if jnt_idx_i > 0
            RotMat = body_i.getRotM(qm(jnt_idx_i));
        else
            RotMat = body_i.getRotM(0);
        end
        
        % Update matrices
        app_data.A_array(:, :, i) = RotMat.' * body_i.A;
        app_data.M_array(:, :, i) = M_ik; % Payload mass matrix    (Not needed to save??)
    end

    A_1_b = app_data.A_array(:, :, 1) * obj.Base.A;

    Ma = Ma + A_1_b.' * M_ik * A_1_b;

    % Base matrix
    Dbb = obj.Base.P.' * (obj.Base.M + Ma) * obj.Base.P;

    D = [Dbb, Dba; Dba.', Da];
end
