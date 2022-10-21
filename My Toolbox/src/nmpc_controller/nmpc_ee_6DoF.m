%% nmpc_matlab.m  NMPC First tests
% Design and test of NMPC controller for SR
%

% ### OPTIONS ###
GEN_MEX = 0;
SIM = 1;

simTime = '0.5'; 
tStart = 0.2;

% --- NMPC ---
Ts = 0.1;
Tp = 5; % # of prediction steps
Tc = 5; % # of ctrl steps

% Weights
r_ee_W = 100; % Position position weight
psi_ee_W = 0; % Position orientation weight

fb_W = 1;
nb_W = 1;
taum_W = 1; 

baseMaxForce = 200; % 5
baseMaxTorque = 200; % 5
motorMaxTorque = 200; % 200

% Traj
trajTime = 10;
circleRadius = 0.25;
plane = 'xy';

%% Config
clc
close all

if ~exist('sr', 'var')
    fprintf("Loading SR\n")
    load 'SR6.mat'
end
sr.homeConfig();

close all
qb0 = [0; 0; 0; 0; 0; 0];
qm0 = [0; deg2rad(60); deg2rad(-100); deg2rad(-50); 0; 0];

sr.q = [qb0; qm0];
% sr.show;

% conf1 = [0; 0; 0; 0; 0; 0; 0; 0];
% conf2 = [0; 0; 0; 0; 0; 0; pi/4; -pi/2];
% 
% sr.q = conf2;

mdl = 'nmpc_sim_ee'; 

q0 = sr.q;
q_dot_0 = sr.q_dot;


[~, xee0] = tr2rt(sr.Ttree.endeffector);
xee0 = [xee0; zeros(3, 1)];
xee0_dot = zeros(6, 1);

fprintf('--- Config ---\n')
fprintf('SpaceRobot: %s\n', sr.Name)
fprintf('SR initial config:\n')
fprintf('\t-q0:')
disp(q0.')
fprintf('\t-Xee0:')
disp(xee0.')

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
    nlmpc_ee.Model.StateFcn = "SR6_ee_state_func";
else
    nlmpc_ee.Model.StateFcn = "SR6_ee_state_func_mex";
end
nlmpc_ee.Model.OutputFcn = "SR6_ee_output_func";

nlmpc_ee.Model.IsContinuousTime = true;

% Cost Function
nlmpc_ee.Weights.OutputVariables = ones(1, 3)*r_ee_W; %[ones(1, 3)*r_ee_W, ones(1, 3)*psi_ee_W]; % [ree_x ree_y ree_z psi_ee_x psi_ee_y psi_ee_z]
nlmpc_ee.Weights.ManipulatedVariables = [ones(1, 3)*fb_W, ones(1, 3)*nb_W, ones(1, n)*taum_W];

% nlmpc_ee.Weights.ManipulatedVariablesRate = [ones(1, 3)*fb_rate_W, ones(1, 3)*nb_rate_W, ones(1, 2)*qm_W];


% --- Solver parameters ---
nlmpc_ee.Optimization.UseSuboptimalSolution = true;
% nlmpc_ee.Optimization.SolverOptions.MaxIterations = 400;
nlmpc_ee.Optimization.SolverOptions.Display = 'final-detailed';
nlmpc_ee.Optimization.SolverOptions.Algorithm = 'interior-point';
nlmpc_ee.Optimization.SolverOptions.MaxFunctionEvaluations = 10000;

% --- Constraints ---
% % Joint positions
% for i=1:sr.NumActiveJoints
%     jnt = sr.findJointByConfigId(i);
% 
%     nlmpc_ee.States(i+6).Min = jnt.PositionLimits(1);
%     nlmpc_ee.States(i+6).Max = jnt.PositionLimits(2);
% end

% Torques
maxTorques = [ones(3, 1)*baseMaxForce; ones(3, 1)*baseMaxTorque; ones(n, 1)*motorMaxTorque];

% Torque limits
for i=1:N
    nlmpc_ee.MV(i).Min = -maxTorques(i);
    nlmpc_ee.MV(i).Max = maxTorques(i);
end

% % Validate
x0 = [sr.q; xee0; sr.q_dot; xee0_dot];
u0 = zeros(N, 1);
% validateFcns(nlmpc_ee, x0, u0, []);
% 
% yref = [x0(1:6)', 0, 0];

%% Generate MEX file
if GEN_MEX
    tic
    fprintf('\n--- MEX file generation ---\n')
    path = fullfile('My Toolbox/src/nmpc_controller/');
    ctrl_save_name = 'nlmpc_ee_SR6_mex';

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
traj = circleTraj(xee0(1:3), circleRadius, trajTime, Nsamp, 'plane', plane, 'tStart', tStart);
traj = retime(traj, 'regular', 'linear', 'TimeStep', seconds(Ts));

ref = struct();
ref.time = seconds(traj.Time);
ref.signals.values = traj.EE_desired;
%% Simulink
if SIM
    fprintf('\n--- SIM ---\n')
    set_param(mdl, 'StopTime', simTime)
    waitBar = waitbar(0,'Simulation in progress...');
    
    % Sim Time Timer
    fprintf('Launching Simulation...\n')
    t = timer;
    t.Period = 2;
    t.ExecutionMode = 'fixedRate';
    t.TimerFcn = @(myTimerObj, thisEvent)waitbar(get_param(mdl, 'SimulationTime')/str2double(simTime), ...
        waitBar, {'Simulation in progress...', ...
        [num2str(get_param(mdl, 'SimulationTime')), '/', simTime]});

    start(t)
    
    try
    % Start Sim
%     profile on
    simRes = sim(mdl);      
    logsout = simRes.logsout;
    catch ME
        fprintf("ERROR during simulation:\n")
        disp(ME.identifier)
        runIDs = Simulink.sdi.getAllRunIDs;
        logsout  = Simulink.sdi.exportRun(runIDs(end)); 
    end
    %     profile off   
    stop(t);
    delete(t);   
    delete(waitBar)
    fprintf('Simulation DONE\n')
    fprintf('Total Sim Time (min): %.2f\n', simRes.getSimulationMetadata.TimingInfo.TotalElapsedWallTime/60);    
%     profile viewer
    
    solverStatus = logsout.getElement('nlpStat').Values;  
    failedIdx = find(solverStatus.Data <= 0);
    if ~isempty(failedIdx)
        fprintf('Solver failed at time steps:\n')
        arrayfun(@(idx) fprintf('\t - %.2f sec (flag %i)\n', solverStatus.Time(idx),  solverStatus.Data(idx)), failedIdx);
    else
        fprintf('Solver succesfull at every time step\n')
    end

    fprintf("Press a key to continue...\n");
    pause;
end

%% Animate
fprintf('\n--- Animation ---\n')

% Extract signals from sim
q = logsout.getElement('q').Values;
xSeq = logsout.getElement('xSeq').Values; % predicted states, (Tp+1 x nx x timeStep). xSeq.
ySeq = logsout.getElement('ySeq').Values; % predicted states, (Tp+1 x ny x timeStep). ySeq.
Xee = logsout.getElement('Xee').Values;
Xee_ref = logsout.getElement('Xee_ref').Values;
tau = logsout.getElement('tau').Values;

% Setup signals for animation
trajRes = struct();
trajRes.ref = ts2timetable(Xee_ref);
trajRes.ref.Properties.VariableNames{1} = 'EE_desired';
trajRes.Xee = Xee;

pred = struct();
pred.Xee = ySeq;

% Save options
fileName = '';
if ~strcmp(fileName, '')
    savePath = strcat(folder, fileName);
else
    savePath = '';
end

% Animate
tic
sr.animate(q, 'fps', 17, 'rate', 0.5, 'fileName', savePath, 'traj', trajRes, 'pred', pred, 'viz', 'on'); 
toc

%% Plots
% --- Tracking ---
figure

title("EE Trajectory Tracking")

subplot(1, 2, 1)
xlabel('X [m]')
ylabel('Y [m]')
grid on
axis equal
hold on
plot(trajRes.ref.EE_desired(:, 1), trajRes.ref.EE_desired(:, 2))
plot(reshape(trajRes.Xee.Data(1, :, :), [], 1), reshape(trajRes.Xee.Data(2, :, :), [], 1))
legend('Ref', 'NMPC')
hold off

subplot(1, 2, 2)
xlabel('X [m]')
ylabel('Z [m]')
grid on
axis equal
hold on
plot(trajRes.ref.EE_desired(:, 1), trajRes.ref.EE_desired(:, 3))
plot(reshape(trajRes.Xee.Data(1, :, :), [], 1), reshape(trajRes.Xee.Data(3, :, :), [], 1))
legend('Ref', 'NMPC')
hold off

% % --- Joint ---
% figure
% hold on 
% for i=1:n
%     subplot(n, 1, i)
%     hold on
%     grid on
%     title(sprintf('Jnt%i', i))
%     xlabel('Time [sec]')
%     ylabel('Joint Angle [deg]')
%     xlim([0, str2double(simTime)])
% 
%     tVect = q.Time;
%     plot(tVect, reshape(rad2deg(q.Data(6+i, :, :)), [], 1))
%     
%     jnt = sr.findJointByConfigId(i);
%     jntMin = rad2deg(jnt.PositionLimits(1));
%     jntMax = rad2deg(jnt.PositionLimits(2));
%     plot(tVect, repmat(jntMin, length(tVect), 1), 'k--')
%     plot(tVect, repmat(jntMax, length(tVect), 1), 'k--')
% end
% hold off

% % --- Torques ---
% tau_tt = {N, 1};
% for i=1:N
%     tau_tt{i} = tau;
%     tau_tt{i}.Data = tau.Data(:, i);
% end
% figure
% subplot(3, 1, 1)
% title('Base force')
% hold on
% plot(tau_tt{1})
% plot(tau_tt{2})
% plot(tau_tt{3})
% legend('Fx', 'Fy', 'Fz')
% hold off
% 
% subplot(3, 1, 2)
% title('Base torque')
% hold on
% plot(tau_tt{4})
% plot(tau_tt{5})
% plot(tau_tt{6})
% legend('nx', 'ny', 'nz')
% hold off
% 
% 
% subplot(3, 1, 3)
% title('Joint torques')
% hold on
% for i=7:N
%     name = sprintf('Body%i', i-6);
%     plot(tau_tt{i}, 'DisplayName', name)
% end
% legend
% hold off




