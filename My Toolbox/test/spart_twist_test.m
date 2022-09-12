% SPART
filename = 'SC_2DoF.urdf';
[robotSpart, robot_keys] = urdf2robot(filename);

R0 = rpy2r(sc.q(4:6).');
u0 = [sc.q0_dot(4:6); sc.q0_dot(1:3)];
um = sc.qm_dot;

u0_dot = [q0_ddot(4:6); q0_ddot(1:3)];
um_dot = qm_ddot(1:2);

[RJ, RL, rJ, rL, e, g] = Kinematics(R0, sc.q0(1:3), sc.qm, robotSpart);

%Differential kinematics
[Bij, Bi0, P0, pm] = DiffKinematics(R0, sc.q0(1:3), rL, e, g, robotSpart);

%Velocities
[t0, tm] = Velocities(Bij, Bi0, P0, pm, u0, um, robotSpart);

%Accelerations, twist-rate
[t0_dot,tm_dot] = Accelerations(t0,tm,P0,pm,Bi0,Bij,u0,um,u0_dot,um_dot,robotSpart);

%Inertias projected in the inertial frame
[I0,Im]=I_I(R0,RL,robotSpart);

%Inverse Dynamics - Flying base
wF0=zeros(6,1);
wFm=zeros(6,3);

[tau0,taum, wq_tilde, wq_tilde0] = ID_test(wF0,wFm,t0,tm,t0_dot,tm_dot,P0,pm,I0,Im,Bij,Bi0,robotSpart);

%%
clc
fprintf('### TWIST ###\n')
for i=1:3
    body = sc.Bodies{i};  
    fprintf('--- Body: %s ---\n', body.Name)
    
    t_S = [tm(4:6, i); tm(1:3, i)];

    [R, ~] = tr2rt(sc.getTransform(body.Name, 'symbolic', false));
    t_D = app_data.t_array(:, :, i);
    t_D = blkdiag(R, R) * t_D;
    
    fprintf('[SPART, DeNOC]\n')  
    disp([t_S, t_D])    
end

fprintf('### ACCEL ###\n')
for i=1:3
    
    fprintf('--- Body: %s ---\n', body.Name)
    
    t_dot_S = [tm_dot(4:6, i); tm_dot(1:3, i)];

    [R, ~] = tr2rt(sc.getTransform(body.Name, 'symbolic', false));
    t_dot_D = app_data.t_dot_array(:, :, i);
    t_dot_D = blkdiag(R, R) * t_dot_D;
    
    fprintf('[SPART, DeNOC]\n')  
    disp([t_dot_S, t_dot_D])   
end

% Inverse Dyn

fprintf('### Inverse Dyn ###\n')
fprintf('--- Torques ---\n')
fprintf('[SPART, DeNOC]\n')  
disp([taum, app_data.tau_array(1, 1:2)'])

fprintf('--- Wrenches ---\n')
for i=1:3
    body = sc.Bodies{i};  
    fprintf('%s\n', body.Name)
    
    w_S = [wq_tilde(4:6, i); wq_tilde(1:3, i)];

    [R, ~] = tr2rt(sc.getTransform(body.Name, 'symbolic', false));
    w_D = app_data.w_array(:, :, i);
%     w_D = blkdiag(R, R) * w_D;
    
    fprintf('[SPART, DeNOC]\n')  
    disp([w_S, w_D])    
end

