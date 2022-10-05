function q_ddot = forwardDynamics(obj, F, q, q_dot)
    % Compute forward Dynamics using the algo.

    % TODO: COMPLETE

    %% --- Initialize ---
    nb = zeros(6, 1);
    Mb = obj.Base.M;

    %% For each appendage
    nk = obj.NumBodies;
    n = obj.NumActiveJoints;
    N = 6 + n;

    % Computation of A
    app_data = obj.kinetics(q, q_dot, zeros(N, 1));
    A_array = app_data.A_array;

    A_array(:, :, end + 1) = eye(6, 6); % A_ee+1_ee
    n_array = zeros(6, 1, nk + 1);
    M_bar_array = zeros(6, 6, nk + 1);
    phi_bar_array = zeros(6, 1, nk + 1);

    for i = 1:nk
        A_in_i = A_array(:, :, i + 1); % A_(i+1)_i
        n_next = n_array(:, :, i + 1);

        n_i = A_in_i.' * n_next;

    end

end
