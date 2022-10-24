%% Trajectory Tracking
clc
close all

if ~exist('sr6', 'var')
    fprintf("Loading SR6\n")
    load 'SR6.mat'
end

sr.homeConfig();

n = sr.NumActiveJoints;
N = n + 6;

% Parameters
% sr.qm = [0; -pi/4];
dA = 4;
Gb = ones(6, 1) * 1;
Gm = ones(n, 1) * 5;

simTime = '5.0';
Nsamp = 205; % Make sure (Nsamp - 5) is multiple of 4
squareLength = 0.5;

% Run controller design if gains not present
if ~exist('Kp_b', 'var') || ~exist('Kd_b', 'var') || ~exist('Kp_m', 'var') || ~exist('Kd_m', 'var')
    fprintf("Finding controller gains...\n")
    run('ctrl_synth')
end

% Compute Traj
[~, p0] = tr2rt(sr.getTransform('endeffector', 'TargetFrame', 'inertial', 'symbolic', false));
traj = squareTraj(p0, squareLength, str2double(simTime), Nsamp, 'plane', 'xy');

%% Launch Sim
% profile on
tic
fprintf("Simulating...\n")
set_param('traj_tracking', 'StopTime', simTime)
simRes = sim('traj_tracking');
fprintf("Done\n")
toc
% profile viewer
% profile off

%% Animation
clc
close all
folder = 'results/videos/';
fileName = '';

if ~strcmp(fileName, '')
    savePath = strcat(folder, fileName);
else
    savePath = '';
end

trajRes = struct();
trajRes.ref = traj;
trajRes.Xee = simRes.Xee;

tic
sr.animate(simRes.q, 'fps', 17, 'rate', 0.5, 'traj', trajRes, 'fileName', savePath);
toc
