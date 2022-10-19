%% nmpc_matlab.m  NMPC First tests
% Design and test of NMPC controller for SR
%
% Following Pendulum SwingUp examples
% To open: openExample('mpc/SwingupControlOfPendulumUsingNMPCExample')
%

% ### OPTIONS ###
GEN_MEX = 1;
SIM = 1;

simTime = '10.5'; 
tStart = 0.5;

% --- NMPC ---
Ts = 0.5;
Tp = 10; % # of prediction steps
Tc = 7; % # of ctrl steps

% Weights
r_ee_W = 10; % Position position weight
psi_ee_W = 0; % Position orientation weight

fb_W = 100;
nb_W = 100;
taum_W = 0.1; 

% Traj
trajTime = 10;
circleRadius = 0.25;

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

sr.q = conf2;

mdl = 'nmpc_sim_ee'; 

q0 = sr.q;
q_dot_0 = sr.q_dot;


[~, xee0] = tr2rt(sr.Ttree.endeffector);
xee0 = [xee0; zeros(3, 1)];
xee0_dot = zeros(6, 1);

% xee_ref = [xee0(1)+0.2; 0; 0];
xee_ref = [1.1180+0.5; 0; 0]; 

fprintf('--- Config ---\n')
fprintf('SR initial state set to:\n')
fprintf('\t-q0:')
disp(q0.')
fprintf('\t-Xee0:')
disp(xee0.')
fprintf('\t-Xee_ref:')
disp(xee_ref.')

%% NMPC Controller
fprintf('--- Creating NMPC Controller ---\n')
% Parameters
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
if GEN_MEX
    nlmpc_ee.Model.StateFcn = "SR2_ee_state_func";
else
    nlmpc_ee.Model.StateFcn = "SR2_ee_state_func_mex";
end
nlmpc_ee.Model.OutputFcn = "SR2_ee_output_func";

nlmpc_ee.Model.IsContinuousTime = true;

% Cost Function
nlmpc_ee.Weights.OutputVariables = ones(1, 3)*r_ee_W; %[ones(1, 3)*r_ee_W, ones(1, 3)*psi_ee_W]; % [ree_x ree_y ree_z psi_ee_x psi_ee_y psi_ee_z]

nlobj.Weights.ManipulatedVariables = [ones(1, 3)*fb_W, ones(1, 3)*nb_W, ones(1, 2)*taum_W];

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
    tic
    fprintf('\n--- MEX file generation ---\n')
    path = fullfile('My Toolbox/src/nmpc_controller/');
    ctrl_save_name = 'nlmpc_ee_SR2_mex';

    save_path = fullfile(path, ctrl_save_name);
    
    set_param([mdl, '/NMPC Controller'],'UseMEX', 'on')
    set_param([mdl, '/NMPC Controller'],'mexname', ctrl_save_name)

    % path = []
    [coreData,onlineData] = getCodeGenerationData(nlmpc_ee,x0,u0);
    mexFcn = buildMEX(nlmpc_ee, save_path, coreData, onlineData);
    tMex = toc;
    fprintf('Time to generate: %.2f (min)\n', tMex/60);
else
    set_param([mdl, '/NMPC Controller'],'UseMEX', 'off')
end


%% State Estimation
% TODO: ADD EKF for Xe

%% Traj
Nsamp = 205;  % Make sure (Nsamp - 5) is multiple of 4
traj = circleTraj(xee0(1:3), circleRadius, trajTime, Nsamp, 'plane', 'xy', 'tStart', tStart);
trajRes = retime(traj, 'regular', 'linear', 'TimeStep', seconds(Ts));

ref = struct();
ref.time = seconds(trajRes.Time);
ref.signals.values = trajRes.EE_desired;
%% Simulink
if SIM
    fprintf('\n--- SIM ---\n')
    set_param(mdl, 'StopTime', simTime)
    waitBar = waitbar(0,'Simulation in progress...');
    
    % Sim Time Timer
    fprintf('Launching Simulation...\n')
%     fprintf('Current simulation time: 0.00');
    t = timer;
    t.Period = 2;
    t.ExecutionMode = 'fixedRate';
%     t.TimerFcn = @(myTimerObj, thisEvent)fprintf('\b\b\b\b%.2f', get_param(mdl, 'SimulationTime'));

    t.TimerFcn = @(myTimerObj, thisEvent)waitbar(get_param(mdl, 'SimulationTime')/str2double(simTime), ...
        waitBar, {'Simulation in progress...', ...
        [num2str(get_param(mdl, 'SimulationTime')), '/', simTime]});

    start(t)
    
    % Start Sim
%     profile on
    simRes = sim(mdl);      
%     profile off   
    stop(t);
    delete(t);   
    delete(waitBar)
    fprintf('\nTotal Sim Time (min): %.2f\n', simRes.getSimulationMetadata.TimingInfo.TotalElapsedWallTime/60);

%     profile viewer
end

%% Animate
fprintf('\n--- Animation ---\n')
data = simRes.q;

trajRes = struct();

trajRes.ref = ts2timetable(simRes.Xee_ref);
trajRes.ref.Properties.VariableNames{1} = 'EE_desired';

trajRes.Xee = simRes.Xee;

% Prediction data
xSeq = simRes.logsout.getElement('xSeq').Values; % predicted states, (Tp+1 x nx x timeStep). xSeq.
Xee_idx = [9, 10, 11];
pred = struct();
pred.Xee = xSeq;
pred.Xee.Data = xSeq.Data(:, Xee_idx, :); %Select ee states

% Save
fileName = '';
if ~strcmp(fileName, '')
    savePath = strcat(folder, fileName);
else
    savePath = '';
end

% Animate
tic
sr.animate(data, 'fps', 17, 'rate', 1, 'fileName', savePath, 'traj', trajRes, 'pred', pred, 'viz', 'on'); 
toc

%% Plots
% --- Tracking ---
figure
hold on
title("EE Trajectory Tracking")
xlabel('X [m]')
ylabel('Y [m]')
grid on
axis equal
plot(trajRes.ref.EE_desired(:, 1), trajRes.ref.EE_desired(:, 2))
plot(reshape(trajRes.Xee.Data(1, :, :), [], 1), reshape(trajRes.Xee.Data(2, :, :), [], 1))
legend('Ref', 'NMPC')
hold off

% --- Joint ---
figure
hold on 
for i=1:n
    subplot(n, 1, i)
    hold on
    grid on
    title(sprintf('Jnt%i', i))
    xlabel('Time [sec]')
    ylabel('Joint Angle [deg]')
    xlim([0, str2double(simTime)])

    tVect = simRes.q.Time;
    plot(tVect, reshape(rad2deg(simRes.q.Data(7, :, :)), [], 1))
    
    jnt = sr.findJointByConfigId(i);
    jntMin = rad2deg(jnt.PositionLimits(1));
    jntMax = rad2deg(jnt.PositionLimits(2));
    plot(tVect, repmat(jntMin, length(tVect), 1), 'k--')
    plot(tVect, repmat(jntMax, length(tVect), 1), 'k--')
end
hold off

% --- Torques ---
tau = simRes.logsout.getElement('tau').Values;
tau_tt = {8, 1};
for i=1:8
    tau_tt{i} = tau;
    tau_tt{i}.Data = tau.Data(:, i);
end
figure
subplot(3, 1, 1)
title('Base force')
hold on
plot(tau_tt{1})
plot(tau_tt{2})
plot(tau_tt{3})
legend('Fx', 'Fy', 'Fz')
hold off

subplot(3, 1, 2)
title('Base torque')
hold on
plot(tau_tt{4})
plot(tau_tt{5})
plot(tau_tt{6})
legend('nx', 'ny', 'nz')
hold off


subplot(3, 1, 3)
title('Joint torques')
hold on
plot(tau_tt{7})
plot(tau_tt{8})
legend('tau_1', 'tau_2')
hold off




