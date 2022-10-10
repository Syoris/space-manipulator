%% Generate mex for matlab nmpc
x = zeros(16, 1);
u = zeros(8, 1);

codegen -report sr_state_func.m -args {x, u}

%% Generate mex for matlab ee nmpc
x = zeros(28, 1);
u = zeros(8, 1);

codegen -report sr_ee_state_func.m -args {x, u}

%% Generate mex for acado ocp
t = 0;
x = zeros(16, 1);
u = zeros(8, 1);
p = zeros(0, 1);
w = zeros(0, 1);

% codegen -report srode_ocp_acado.m -args {t, x, u, p, w}

