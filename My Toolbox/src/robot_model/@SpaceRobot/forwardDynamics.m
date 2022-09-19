function q_ddot = forwardDynamics(obj, F, q, q_dot)
    %forwardDynamics Compute resultant joint acceleration given joint torques and states
    %   QDDOT = forwardDynamics(ROBOT) computes the resultant joint
    %   accelerations due to joint speed and applied torque
    %
    %   F = [f0; n0; tau_m]
    %       f0: Forces applied on base, in base frame (3x1)
    %       n0: Torque applied to base, in base frame (3x1)
    %       tau_m: Joint torques (Nx1)

    assert(obj.DynInitialized, ['Dynamics needs to be initialized for calling this function.' ...
                            ' Call SpaceRobot.initDyn() first']);

    % % Symbolic method, depracated
    % if nargin > 2
    %     [H, C, Q] = obj.getMats(q, q_dot);
    % else
    %     [H, C, Q] = obj.getMats();
    %     q_dot = obj.q_dot;
    % end

    % q_ddot = H^-1*(Q*F - C*q_dot);

    N = obj.NumActiveJoints + 6;

    if nargin == 2
        q = obj.q;
        q_dot = obj.q_dot;
    end

    if nargin == 3
        q_dot = zeros(N, N);
    end

    M = obj.MassMat;
    [h_b, h_m] = obj.inverseDynamics(q, q_dot, zeros(N, 1));
    h = [h_b; h_m];

    q_ddot = M^ - 1 * (F - h);

end
