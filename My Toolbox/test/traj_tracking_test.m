%% Initialization
clc
load 'SC_2DoF.mat'

sc.homeConfig();

p0 = sc.qm;

traj = squareTraj([0; 0; 0], 0.5, 10, 101);