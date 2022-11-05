%% Generate mex for matlab nmpc
clc
maxDoF = 14;
N = 14 + 6;
nk = 1;
funcHandle = 'SR6_info';
% x = zeros(16, 1);
% u = zeros(8, 1);

states = coder.typeof(1, [2 * N 1], [true false]);
input = coder.typeof(1, [N 1], [true false]);
% funcHandle = coder.typeof(1, [1 20], [false true]);

codegen -report sr_state_func.m -args {states, input, nk, funcHandle} -o src\state_functions\sr_state_func_mex.mexw64

%% Generate mex for matlab ee nmpc
x = zeros(28, 1);
u = zeros(8, 1);

codegen -report sr_ee_state_func.m -args {x, u} -o src\state_functions\sr_ee_state_func_mex.mexw64

%% Generate mex for matlab ee nmpc
x = zeros(36, 1);
u = zeros(12, 1);
sr_info = SR6_info();

codegen -report sr_ee_state_func.m -args {x, u, sr_info} -o src\state_functions\sr_ee_state_func_mex.mexw64