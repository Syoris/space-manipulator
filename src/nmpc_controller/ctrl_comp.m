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
%
% NEW BASE, 1.5m cube
% 13: Good action;  Unc: None;              Base Cstr: 45
        
% NEW BASE, 2m cube
% 14: Unc: Mass(-0.2%);                  Base Cstr: 30 Res: Good                
% 15: Unc: None                          Base Cstr: 30
% 16: Unc: Mass(-0.4%), Inertia(-0.4%)   Base Cstr: 30 Res: LETS GO

% 17: Test w/ Ts = 0.25 and rk4(M=5)
                    
% Best ones: 3, 4, 7

%% Get Test Data
close all

ctrlName = 'Ctrl15';
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
% plotFunc(sr, logsout, simTime, n, N);

% Matrices
fprintf("Model Base Mass Mat\b")
ctrl_struct.Res.modBaseM

fprintf("Sim Base Mass Mat\n")
ctrl_struct.Res.simBaseM

