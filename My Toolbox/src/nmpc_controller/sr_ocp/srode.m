function [ dx ] = srode(t, x, u, p, w )      

%% Load params
    sr_info = SR2_info();

%%    
    dx = zeros(16, 1);
    q = x(1:8);
    qb = q(1:6);
    qm = q(7:8);

    q_dot = x(9:16); 
    qb_dot = q_dot(1:6);
    qm_dot = q_dot(7:8);
%% 
    % 1 - Kinematics
    [Rb, Rm] = RFunc_SR2(q);
    Rm = reshape(Rm, 3, 3, []); % Split Rm to nk 3x3 arrays
    
    % 2 - Kinetics
    [t, t_dot, Omega, A, A_dot] = Kin(sr_info, q, q_dot, zeros(8, 1), {Rb, Rm});
    
    % 3 - ID
    [tau, ~] = ID(sr_info, t, t_dot, Omega, A, A_dot);
    h = [tau{1}; tau{2}];
    
    % 4 - Mass Mat
    D = MassM(sr_info, q, A);
    
    % 5 - FD
    q_ddot = D^-1 * (u - h);

    %% Assign outputs
    % Speed
    dx(1:8) = q_dot;
    dx(9:16) = q_ddot;

end
