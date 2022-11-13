%% ctrl_comp.m
% To load past tests results and compare them
tablePath = fullfile('results\controllers\table_ctrl.mat');
load(tablePath);
load SR6.mat

%% Get Test Data
close all

ctrlName = 'Ctrl5';
ctrlInfo = ctrl_table(strcmp(ctrl_table.Controller, ctrlName), :);
load(ctrlInfo.("Save Path"));

logsout = ctrl_struct.Res.logsout;
simTime = ctrl_struct.Res.simTime;
n = sr.NumActiveJoints;
N = n + 6;

% Animate
rate = 2;
saveName = '';
animateFunc(sr, logsout, rate, saveName);

% Plots
plotFunc(sr, logsout, simTime, n, N);

