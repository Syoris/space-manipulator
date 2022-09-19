%% Trajectory Tracking
clc
close all
load 'SR2.mat'
sr.homeConfig();

% Parameters
% sr.qm = [0; -pi/4];
dA = 1;
Gb = ones(6, 1)*10;
Gm = ones(2, 1)*25;
simTime = '11.0';
trajTime = 10;
startTime = 1;

Nsamp = 205;  % Make sure (Nsamp - 5) is multiple of 4
squareLength = 0.5;
circleRadius = 0.25;

% Run controller design if gains not present
if ~exist('Kp_b', 'var') || ~exist('Kd_b', 'var') || ~exist('Kp_m', 'var') || ~exist('Kd_m', 'var')
    fprintf("Finding controller gains...\n")
    run('ctrl_synth')
end


% Compute Traj
[~, p0] = tr2rt(sr.getTransform('endeffector', 'TargetFrame',  'inertial', 'symbolic', false));
% traj = squareTraj(p0, squareLength, str2double(simTime), Nsamp, 'plane', 'xy');
traj = circleTraj(p0, circleRadius, trajTime, Nsamp, 'plane', 'xy', 'tStart', startTime);

% Launch Sim
tic
fprintf("Simulating...\n")
set_param('traj_tracking', 'StopTime', num2str(trajTime + startTime))
simRes = sim('traj_tracking');
fprintf("Done\n")
toc

%% Animation
clc
close all
folder = 'Project/Videos/';
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
sr.animate(simRes.q, 'fps', 17, 'rate', 1, 'traj', trajRes, 'fileName', savePath); 
toc