%% Generate mex for matlab nmpc
clc
maxDoF = 14;
N = 14+6;
nk = 1;
funcHandle = 'SR6_info';
% x = zeros(16, 1);
% u = zeros(8, 1);

states = coder.typeof(1, [2*N 1], [true false]);
input = coder.typeof(1, [N 1], [true false]);
% funcHandle = coder.typeof(1, [1 20], [false true]);

codegen -report sr_state_func.m -args {states, input, nk, funcHandle} -o 'My Toolbox'\src\state_functions\sr_state_func_mex.mexw64


%% Generate mex for matlab ee nmpc
x = zeros(28, 1);
u = zeros(8, 1);

codegen -report sr_ee_state_func.m -args {x, u} -o 'My Toolbox'\src\state_functions\sr_ee_state_func_mex.mexw64

%% Generate mex for SR6
clc
% maxDoF = 14;
% N = 14+6;
% nk = 1;
% funcHandle = 'SR6_info';
% x = zeros(16, 1);
% u = zeros(8, 1);

% states = coder.typeof(1, [2*N 1], [true false]);
% input = coder.typeof(1, [N 1], [true false]);
% funcHandle = coder.typeof(1, [1 20], [false true]);

x = zeros(24, 1);
u = zeros(12, 1);

codegen -report SR6_state_func.m -args {x, u} -o Project\Models\SR6\SR6_state_func_mex.mexw64


%% Generate mex for acado ocp
t = 0;
x = zeros(16, 1);
u = zeros(8, 1);
p = zeros(0, 1);
w = zeros(0, 1);

% codegen -report srode_ocp_acado.m -args {t, x, u, p, w}

