%% ctrl_comp.m
% To load past tests results and compare them
tablePath = fullfile('results\controllers\table_ctrl.mat');
load(tablePath);
load SR6.mat

% Ctrl overview
% 1: Shit
% 2: Unstable
% 3: Smooth, error a bit too big
% 4: Quite good
% 5: Too aggressive, noisy action
% 6: Too aggressive, noisy action
% 7: Good
% 8: Really good
% 9: 8 but with base mass

% Best ones: 3, 4, 7

%% Get Test Data
close all

ctrlName = 'Ctrl8';
ctrlInfo = ctrl_table(strcmp(ctrl_table.Controller, ctrlName), :);
load(ctrlInfo.("Save Path"));
fprintf('Results of %s\n', ctrlName);

logsout = ctrl_struct.Res.logsout;
simTime = ctrl_struct.Res.simTime;
n = sr.NumActiveJoints;
N = n + 6;

% Animate
rate = 2;
saveName = '';
% animateFunc(sr, logsout, rate, saveName);

% Plots
plotFunc(sr, logsout, simTime, n, N);

