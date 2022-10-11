%% nmpc_matlab.m  NMPC First tests
% Design and test of NMPC controller for SR
%
% Following Pendulum SwingUp examples
% To open: openExample('mpc/SwingupControlOfPendulumUsingNMPCExample')
%

% ### OPTIONS ###
GEN_MEX = 0;
SIM = 1;

simTime = '7.0'; 
tStep = 0.2;

%% Config
clc
close all

if ~exist('sr', 'var')
    fprintf("Loading SR\n")
    load 'SR2.mat'
end
sr.homeConfig();

conf1 = [0; 0; 0; 0; 0; 0; 0; 0];
conf2 = [0; 0; 0; 0; 0; 0; pi/4; -pi/2];

sr.q = conf1;

mdl = 'nmpc_sim_ee'; 

q0 = sr.q;
q_dot_0 = sr.q_dot;


[~, xee0] = tr2rt(sr.Ttree.endeffector);
xee0 = [xee0; zeros(3, 1)];
xee0_dot = zeros(6, 1);

% xee_ref = [xee0(1)+0.2; 0; 0];
xee_ref = [xee0(1)+0.2; 0; 0];

fprintf('SR initial state set to:\n')
fprintf('\t-q0:')
disp(q0.')
fprintf('\t-Xee0:')
disp(xee0.')
fprintf('\t-Xee_ref:')
disp(xee_ref.')

%% NMPC Controller
fprintf('--- Creating NMPC Controller\n ---')
% Parameters
Ts = 0.1;
Tp = 20; % # of prediction steps
Tc = 10; % # of ctrl steps

fprintf('Parameters:\n')
fprintf('\tTs: %.2f\n', Ts)
fprintf('\tTp: %i (%.2f sec)\n', Tp, Tp*Ts)
fprintf('\tTc: %i (%.2f sec)\n', Tc, Tc*Ts)

% 
n = sr.NumActiveJoints;
N = n+6;
nx = 12 + 2*n + 12; % Change to 24 + 2*n w/ EE
ny = 3; % EE states
nu = n+6;

nlmpc_ee = nlmpc(nx,ny,nu);


nlmpc_ee.Ts = Ts;
nlmpc_ee.PredictionHorizon = Tp;
nlmpc_ee.ControlHorizon = Tc;

% Prediction Model
nlmpc_ee.Model.NumberOfParameters = 0;
nlmpc_ee.Model.StateFcn = "sr_ee_state_func";
nlmpc_ee.Model.OutputFcn = "sr_ee_output_func";

nlmpc_ee.Model.IsContinuousTime = true;

% Cost Function
r_ee_W = 1; % Position position weight
psi_ee_W = 0; % Position orientation weight

fb_rate_W = 1;
nb_rate_W = 1;
taum_rate_W = 1; 

nlmpc_ee.Weights.OutputVariables = ones(1, 3)*r_ee_W; %[ones(1, 3)*r_ee_W, ones(1, 3)*psi_ee_W]; % [ree_x ree_y ree_z psi_ee_x psi_ee_y psi_ee_z]

% nlobj.Weights.ManipulatedVariables = [ones(1, 3)*fb_W, ones(1, 3)*nb_W, ones(1, 2)*taum_W];

% nlobj.Weights.ManipulatedVariablesRate = [ones(1, 3)*fb_rate_W, ones(1, 3)*nb_rate_W, ones(1, 2)*qm_W];


% --- Constraints ---
% Joint positions
for i=1:sr.NumActiveJoints
    jnt = sr.findJointByConfigId(i);

    nlmpc_ee.States(i+6).Min = jnt.PositionLimits(1);
    nlmpc_ee.States(i+6).Max = jnt.PositionLimits(2);
end

% Torques
baseMaxForce = 5;
baseMaxTorque = 5;
motorMaxTorque = 200;
maxTorques = [ones(3, 1)*baseMaxForce; ones(3, 1)*baseMaxTorque; ones(n, 1)*motorMaxTorque];

% Torque limits
for i=1:N
    nlmpc_ee.MV(i).Min = -maxTorques(i);
    nlmpc_ee.MV(i).Max = maxTorques(i);
end

% % Validate
x0 = [sr.q; xee0; sr.q_dot; xee0_dot];
u0 = zeros(8, 1);
% validateFcns(nlmpc_ee, x0, u0, []);
% 
% yref = [x0(1:6)', 0, 0];

%% Generate MEX file
if GEN_MEX
    fprintf('--- MEX file generation ---\n')
    path = fullfile('My Toolbox/src/nmpc_controller/');
    ctrl_save_name = 'nlmpc_ee_mex';

    save_path = fullfile(path, ctrl_save_name);
    
    set_param([mdl, '/NMPC Controller'],'UseMEX', 'on')
    set_param([mdl, '/NMPC Controller'],'mexname', ctrl_save_name)

    % path = []
    [coreData,onlineData] = getCodeGenerationData(nlmpc_ee,x0,u0);
    mexFcn = buildMEX(nlmpc_ee, save_path, coreData, onlineData);
else
%     set_param([mdl, '/NMPC Controller'],'UseMEX', 'off')
end


%% State Estimation
% TODO: ADD EKF for Xe



%% Simulink
if SIM
    fprintf('\n--- SIM ---\n')
    set_param(mdl, 'StopTime', simTime)
    
    % Sim Time Timer
    fprintf('Launching Simulation...\n')
    fprintf('\tCurrent simulation time: 0.00');
    t = timer;
    t.Period = 2;
    t.ExecutionMode = 'fixedRate';
    t.TimerFcn = @(myTimerObj, thisEvent)fprintf('\b\b\b\b%.2f', get_param(mdl, 'SimulationTime'));
    start(t)
    
    % Start Sim    
    simRes = sim(mdl);      
    stop(t);
    delete(t);
    fprintf('\nTotal Sim Time (min): %.2f\n', simRes.getSimulationMetadata.TimingInfo.TotalElapsedWallTime/60);
end

%% Animate
data = simRes.q.getsampleusingtime(0, str2double(simTime));

fileName = '';
if ~strcmp(fileName, '')
    savePath = strcat(folder, fileName);
else
    savePath = '';
end

tic
sr.animate(data, 'fps', 17, 'rate', 1, 'fileName', savePath); 
toc

