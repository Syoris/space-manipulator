function q_ddot = forwardDynamics(obj, F)
    %forwardDynamics Compute resultant joint acceleration given joint torques and states
    %   QDDOT = forwardDynamics(ROBOT) computes the resultant joint
    %   accelerations due to joint speed and applied torque
    %
    %   F = [f0; n0; tau_m]
    %       f0: Forces applied on base, in base frame (3x1)
    %       n0: Torque applied to base, in base frame (3x1)
    %       tau_m: Joint torques (Nx1)

    % TODO: Possibility to specify q_dot and q
    tic
    H = obj.getH();
    toc
    tic
    C = obj.getC();
    toc
    tic
    Q = obj.getQ();
    toc

    q_dot_val = [obj.BaseSpeed.TSpeed, obj.BaseSpeed.ASpeed, [obj.JointsSpeed.JointSpeed]]';

    q_ddot = H^-1*(Q*F - C*q_dot_val);
end