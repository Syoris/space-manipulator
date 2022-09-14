% SPART
filename = 'SC_2DoF.urdf';
[robotSpart, robot_keys] = urdf2robot(filename);
robotSpart.links(3).mass = sc.Bodies{3}.Mass;
robotSpart.links(3).inertia = sc.Bodies{3}.InertiaM;

R0 = rpy2r(sc.q(4:6).');
u0 = [sc.q0_dot(4:6); sc.q0_dot(1:3)];
um = sc.qm_dot;

u0_dot = [q0_ddot(4:6); q0_ddot(1:3)];
um_dot = qm_ddot(1:2);

[RJ, RL, rJ, rL, e, g] = Kinematics(R0, sc.q0(1:3), sc.qm, robotSpart);

%Differential kinematics
[Bij, Bi0, P0, pm] = DiffKinematics(R0, sc.q0(1:3), rL, e, g, robotSpart);

%Velocities
[t0_S, tm_S] = Velocities(Bij, Bi0, P0, pm, u0, um, robotSpart);

%Accelerations, twist-rate
[t0_dot_S,tm_dot_S] = Accelerations(t0_S,tm_S,P0,pm,Bi0,Bij,u0,um,u0_dot,um_dot,robotSpart);

%Inertias projected in the inertial frame
[I0,Im]=I_I(R0,RL,robotSpart);

%Inverse Dynamics - Flying base
wF0=zeros(6,1);
wFm=zeros(6,3);

[tau0_S,taum_S, wq_tilde, wq_tilde0] = ID_test(wF0,wFm,t0_S,tm_S,t0_dot_S,tm_dot_S,P0,pm,I0,Im,Bij,Bi0,robotSpart);

%Mass Composite Body matrix
[M0_tilde, Mm_tilde] = MCB(I0, Im, Bij, Bi0, robotSpart);

%Generalized Inertia matrix
[H0, H0m, Hm] = GIM(M0_tilde, Mm_tilde, Bij, Bi0, P0, pm, robotSpart);
H_spart_ori = [H0, H0m; H0m', Hm];
H_spart = [[H0(4:6, 4:6), H0(4:6, 1:3); H0(1:3, 4:6), H0(1:3, 1:3)], [H0m(4:6, :); H0m(1:3, :)];
                                                                [H0m(4:6, :)', H0m(1:3, :)'], Hm];

%Generalized Convective Inertia matrix
[C0, C0m, Cm0, Cm, M0_tilde, child_con_C0, Bi0, Hi0_tilde, Mdot0_tilde, P0, Mdot_tilde] = CIM_test(t0_S, tm_S, I0, Im, M0_tilde, Mm_tilde, Bij, Bi0, P0, pm, robotSpart);
C_spart_ori = [C0, C0m; Cm0, Cm];
C_spart = [[C0(4:6, 4:6), C0(4:6, 1:3); C0(1:3, 4:6), C0(1:3, 1:3)], [C0m(4:6, :); C0m(1:3, :)];
                                                                [Cm0(:, 4:6), Cm0(:, 1:3)], Cm];

   
%     
%         
% end
% 
% fprintf('### ACCEL ###\n')
% fprintf('--- Base ---\n')
% t0_dot_S = [t0_dot_S(4:6); t0_dot_S(1:3)];
% if ~isequal(round(t0_dot_S, 5), round(tb_dot, 5))
%     fprintf('ERROR\n')
%     fprintf('[SPART, DeNOC]\n')  
%     disp([t0_dot_S, tb_dot])    
% else
%     fprintf('OK\n')
%     fprintf('[SPART, DeNOC]\n')  
%     disp([t0_dot_S, tb_dot])   
% end    
% 
% for i=1:3
%     body = sc.Bodies{i};  
%     fprintf('--- Body: %s ---\n', body.Name)
%     
%     t_dot_S = [tm_dot_S(4:6, i); tm_dot_S(1:3, i)];
%     r = body.CenterOfMass.';
% 
%     [R, ~] = tr2rt(sc.getTransform(body.Name, 'symbolic', false));
%     t_o_dot = app_data.t_dot_array(:, :, i);
%     t_o = app_data.t_array(:, :, i);
% 
%     t_o_dot_I = blkdiag(R, R) * t_o_dot;
% 
%     t_com_dot = t_o_dot(1:3) + skew(t_o_dot(4:6)) * r + cross(t_o(4:6), cross(t_o(4:6), r));
%     t_com_dot_I = blkdiag(R, R) * [t_com_dot; t_o_dot(4:6)];
%     
%     if ~isequal(round(t_dot_S, 5), round(t_com_dot_I, 5))
%         fprintf('ERROR\n')
%         fprintf('[SPART, DeNOC (CoM in I)]\n')  
%         disp([t_dot_S, t_com_dot_I])    
%     else
%         fprintf('OK\n')
%         fprintf('[SPART, DeNOC (CoM in I)]\n')  
%         disp([t_dot_S, t_com_dot_I])  
%     end    
% 
%     
% end
% 
% % Inverse Dyn
% fprintf('--- Wrenches ---\n')
% fprintf('Base\n')
% wq_tilde0 = [wq_tilde0(4:6); wq_tilde0(1:3)];
% if ~isequal(round(wq_tilde0, 5), round(w_base, 5))
%     fprintf('ERROR\n')
%     fprintf('[SPART, DeNOC]\n')  
%     disp([wq_tilde0, w_base])    
% else
%     fprintf('OK\n')
%     fprintf('[SPART, DeNOC]\n')  
%     disp([wq_tilde0, w_base])   
% end    
% 
% for i=1:3
%     body = sc.Bodies{i};  
%     fprintf('%s\n', body.Name)
%     
%     w_S = [wq_tilde(4:6, i); wq_tilde(1:3, i)];
%     
%     r = body.CenterOfMass.';
%     [R, ~] = tr2rt(sc.getTransform(body.Name, 'symbolic', false));
%     
%     w_o = app_data.w_array(:, :, i);
%     w_o_I = blkdiag(R, R) * w_o;
%     
%     w_com = w_o;
%     w_com(4:6) = w_o(4:6) + skew(-r)*w_o(1:3);
%     w_com_I = blkdiag(R, R) * w_com;
%     
%     fprintf('[SPART, DeNOC (CoM), DeNOC (o)]\n')  
%     disp([w_S, w_com_I, w_o_I])    
% end
% 
% fprintf('--- Torques ---\n')
% fprintf('Base\n')
% tau0 = [tau0(4:6); tau0(1:3)];
% fprintf('[SPART, DeNOC]\n')  
% disp([tau0, tau_base])   
% 
% fprintf('[SPART, DeNOC]\n')  
% disp([taum, app_data.tau_array(1, 1:2)'])

