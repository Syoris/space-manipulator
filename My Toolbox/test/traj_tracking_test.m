%% Initialization
clc
load 'SC_2DoF.mat'

f0 = [0; 0; 0];     % Forces on base [fx, fy, fz], in base frame
n0 = [0; 0; 0];     % Torques on base [nx, ny, nz], in base frame
tau_qm = [0; 0.1];      % Joint torques

sc.homeConfig();

tTree = sc.forwardKinematics();
[~, p0] = tr2rt(tTree.endeffector);

traj = squareTraj(p0, 0.5, 10, 101, 'plane', 'xy');