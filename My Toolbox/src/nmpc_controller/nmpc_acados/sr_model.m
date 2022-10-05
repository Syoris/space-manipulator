function model = sr_model()

import casadi.*

%% system dimensions
nx = 16;
nu = 8;

%% system parameters

%% named symbolic variables
p = SX.sym('p');         % horizontal displacement of cart [m]
theta = SX.sym('theta'); % angle of rod with the vertical [rad]
v = SX.sym('v');         % horizontal velocity of cart [m/s]
dtheta = SX.sym('dtheta'); % angular velocity of rod [rad/s]
F = SX.sym('F');         % horizontal force acting on cart [N]

%% (unnamed) symbolic variables
sym_x = vertcat(p, theta, v, dtheta);
sym_xdot = SX.sym('xdot', nx, 1);
sym_u = F;

%% dynamics
%expr_f_expl = vertcat(v, ...
%                      dtheta, ...
%                      (- l*m*sin(theta)*dtheta.^2 + F + g*m*cos(theta)*sin(theta))/(M + m - m*cos(theta).^2), ...
%                      (- l*m*cos(theta)*sin(theta)*dtheta.^2 + F*cos(theta) + g*m*sin(theta) + M*g*sin(theta))/(l*(M + m - m*cos(theta).^2)));
sin_theta = sin(theta);
cos_theta = cos(theta);
denominator = M + m - m*cos_theta.^2;
expr_f_expl = vertcat(v, ...
                      dtheta, ...
                      (- l*m*sin_theta*dtheta.^2 + F + g*m*cos_theta*sin_theta)/denominator, ...
                      (- l*m*cos_theta*sin_theta*dtheta.^2 + F*cos_theta + g*m*sin_theta + M*g*sin_theta)/(l*denominator));
expr_f_impl = expr_f_expl - sym_xdot;

%% constraints
expr_h = sym_u;

%% cost
W_x = diag([1e3, 1e3, 1e-2, 1e-2]);
W_u = 1e-2;
expr_ext_cost_e = sym_x'* W_x * sym_x;
expr_ext_cost = expr_ext_cost_e + sym_u' * W_u * sym_u;
% nonlinear least sqares
cost_expr_y = vertcat(sym_x, sym_u);
W = blkdiag(W_x, W_u);
model.cost_expr_y_e = sym_x;
model.W_e = W_x;



%% populate structure
model.nx = nx;
model.nu = nu;
model.sym_x = sym_x;
model.sym_xdot = sym_xdot;
model.sym_u = sym_u;
model.expr_f_expl = expr_f_expl;
model.expr_f_impl = expr_f_impl;
model.expr_h = expr_h;
model.expr_ext_cost = expr_ext_cost;
model.expr_ext_cost_e = expr_ext_cost_e;

model.cost_expr_y = cost_expr_y;
model.W = W;

end