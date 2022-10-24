function q_ddot = forwardDynamics(obj, F, q, q_dot)
    %forwardDynamics Compute resultant joint acceleration given joint torques and states
    %   QDDOT = forwardDynamics(ROBOT, F) computes the resultant joint
    %   accelerations due to applied torque at current SR config (position and speed)
    %
    %   QDDOT = forwardDynamics(ROBOT, F, q) computes the resultant joint
    %   accelerations due to applied torque at q config. Assumes joint speed to be null.
    %
    %   QDDOT = forwardDynamics(ROBOT, F, q, q_dot) computes the resultant joint
    %   accelerations due to joint speed (q_dot) and applied torque at given q config.
    %
    %
    %   F = [f0; n0; tau_m]
    %       f0: Forces applied on base, in base frame (3x1)
    %       n0: Torque applied to base, in base frame (3x1)
    %       tau_m: Joint torques (Nx1)
    %
    %

    assert(obj.DynInitialized, ['Dynamics needs to be initialized for calling this function.' ...
                            ' Call SpaceRobot.initDyn() first']);

    narginchk(2, 4);

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
