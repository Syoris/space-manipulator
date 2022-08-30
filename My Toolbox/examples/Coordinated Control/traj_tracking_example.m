%% Trajectory Tracking
clc
close all
load 'SC_2DoF.mat'
sc.homeConfig();

% Parameters
% sc.qm = [0; -pi/4];
dA = 1;
Gb = ones(6, 1)*10;
Gm = ones(2, 1)*25;
simTime = '10.0';
Nsamp = 205;  % Make sure (Nsamp - 5) is multiple of 4
squareLength = 0.5;

% Run controller design if gains not present
if ~exist('Kp_b', 'var') || ~exist('Kd_b', 'var') || ~exist('Kp_m', 'var') || ~exist('Kd_m', 'var')
    fprintf("Finding controller gains...\n")
    run('ctrl_synth')
end


% Compute Traj
[~, p0] = tr2rt(sc.getTransform('endeffector', 'TargetFrame',  'inertial', 'symbolic', false));
traj = squareTraj(p0, squareLength, str2double(simTime), Nsamp, 'plane', 'xy');

% Launch Sim
fprintf("Simulating...\n")
set_param('traj_tracking', 'StopTime', simTime)
simRes = sim('traj_tracking');
fprintf("Done\n")

%% Animation
clc
close all
folder = 'Project/Videos/';
fileName = 'coordinated_control_traj';

trajRes = struct();
trajRes.ref = traj;
trajRes.Xee = simRes.Xee;

tic
sc.animate(simRes.q, 'fps', 17, 'rate', 1, 'traj', trajRes, 'fileName', strcat(folder, fileName)); 
toc