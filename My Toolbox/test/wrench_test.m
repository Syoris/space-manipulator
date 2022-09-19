%% BODY 2
body = sc.Bodies{2};
r = body.CenterOfMass.'; % From O to CoM
l = body.Length;

% Theoretical forces
w_S_com = [wq_tilde(4:6, 2); wq_tilde(1:3, 2)]; % Theoretical force at CoM  
w_S_o = w_S_com;
w_S_o(4:6) = w_S_com(4:6) + skew(r)*w_S_com(1:3);

w_o2 = app_data.w_array(:, :, 2);

W_e = app_data.w_array(:, :, 3);
fe = W_e(1:3);
ne = W_e(4:6);

% My comp
to = app_data.t_array(:, :, 2);
v0 = to(1:3);
omega0 = to(4:6);

to_dot = app_data.t_dot_array(:, :, 2);
vo_dot = to_dot(1:3);
omega0_dot = to_dot(4:6);

Icom = body.InertiaM;
Io = body.InertiaM;
Iz = body.InertiaM(3, 3) + body.Mass * 0.25^2;
Io(3, 3) = Iz;


c = r*body.Mass;

%force
f0 = body.Mass * vo_dot - skew(c)*omega0_dot - skew(omega0) * skew(c) * omega0 + fe;

% torque
ni = skew(c) * vo_dot +  Io * omega0_dot + skew(omega0) * (Io * omega0);

ni2 = (Icom - m*skew(r)*skew(r))*omega0_dot + m*skew(r)*vo_dot;

fprintf("COMP: [ni_goal, ni, ni2]\n")
disp([ni_goal, ni, ni2])

% no = ni + ne + skew(l) * fe;
% 
% fprintf("COMP: [SPART, DeNOC, old]\n")
% disp([w_S_o(4:6), no, w_o2(4:6)])


%% WRT CoM
body = sc.Bodies{2};

% Theoretical forces
w_S_com = [wq_tilde(4:6, 2); wq_tilde(1:3, 2)]; % Theoretical force at CoM  

W_e = app_data.w_array(:, :, 3);
fe = W_e(1:3);
ne = W_e(4:6);

% Speed
t_S = [tm(4:6, 2); tm(1:3, 2)];
t_dot_S = [tm_dot(4:6, 2); tm_dot(1:3, 2)];

v = t_S(1:3);
omega = t_S(4:6);

v_dot = t_dot_S(1:3);
omega_dot = t_dot_S(4:6);

Io = body.InertiaM;
% Io = body.InertiaM * body.Mass * 0.25^2;
% Io = diag([0, 0, Iz]);

r = zeros(3, 1);
c = r*body.Mass;
l = body.CenterOfMass.';

%force
fi = body.Mass * v_dot - skew(c)*omega0_dot - skew(omega0) * skew(c) * omega0;
f_com = fi + fe;

% torque
ni = skew(c) * vo_dot +  Io * omega0_dot + skew(omega0) * (Io * omega0);
n_com = ni + ne + skew(l) * fe;

fprintf("COMP: [SPART, DeNOC]\n")
disp([w_S_com, [f_com; n_com]])

%%

% wq =
% 
%          0         0         0
%          0         0         0
%     0.0530    0.0106    0.0011
%          0         0         0
%     0.2648    0.3310    0.1589
%          0         0         0
% 
%  wq_tilde =
% 
%          0         0         0
%          0         0         0
%     0.4717    0.0514    0.0011
%          0         0         0
%     0.7547    0.4899    0.1589
%          0         0         0