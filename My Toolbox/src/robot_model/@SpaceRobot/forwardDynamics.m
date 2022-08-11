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
    H = obj.getH();
    C = obj.getC();
    Q = obj.getQ();

    q_ddot = H^-1*(Q*F - C*obj.q_dot);
end