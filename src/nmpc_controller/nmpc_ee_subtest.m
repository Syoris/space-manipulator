%% nmpc_matlab.m  NMPC First tests
% Design and test of NMPC controller for SR
%

% close all
% clc
% clear all
fileName = 'ctrl4.mat';
load(fileName, 'q', 'q_dot', 'traj', 'sr', 'Xee', 'xSeq', 'logsout');

startTime = 45;
simTime = 10;

% ### OPTIONS ###
SIM = 1;
PLOT = 0;

% --- NMPC Params ---
Ts = 0.1;
Tp = 5; % # of prediction steps
Tc = 5; % # of ctrl steps

% --- Weights ---
r_ee_W = 20; % Position position weight        1000
psi_ee_W = 10; % Position orientation weight   628

fb_W = 0.5;
nb_W = 0.1;
taum_W = 0.1;

fb_rate_W = 0.1;
nb_rate_W = 0.1;
taum_rate_W = 0.1;

% --- Max Forces --- 
baseMaxForce = 5; % 5
baseMaxTorque = 5; % 5
motorMaxTorque = 200; % 200

% --- Start time data ---
x_start = xSeq.getsampleusingtime(startTime).Data(1, :);
q_start = q.getsampleusingtime(startTime).Data;
q_dot_start = q_dot.getsampleusingtime(startTime).Data;

[xee_start, xee_dot_start] = ee_speed(sr_info, q_start, q_dot_start);

xee_start = Xee.getsampleusingtime(startTime).Data;
% xee_dot_start = x_start(31:36)';

q_dot_start = zeros(N, 1);
xee_dot_start = zeros(6, 1);

traj = retime(traj, 'regular', 'linear', 'TimeStep', seconds(Ts));
tr = timerange(seconds(startTime), seconds(simTime + startTime));
traj = traj(tr, :);
% traj.Time = traj.Time - seconds(startTime);

traj.Time = traj.Time - seconds(startTime - 0.5);
tt2 = timetable([xee_start'; xee_start'],'RowTimes', [seconds(0); seconds(0.4)], 'VariableNames',{'EE_desired'});
traj = [tt2; traj];

ref = struct();
ref.time = seconds(traj.Time);
ref.signals.values = traj.EE_desired;
%% Config
sr.q = q_start;
sr.q_dot = zeros(12, 1); %q_dot_start;

sr.Bodies{6}.Joint.PositionLimits = [-inf, inf];

mdl = 'nmpc_sim_ee';

q0 = sr.q;
q_dot_0 = sr.q_dot;

% [Ree, xee0] = tr2rt(sr.Ttree.endeffector);
% psi_ee = tr2rpy(Ree, 'zyx').';

xee0 = xee_start;%[xee0; psi_ee];
xee0_dot = zeros(6, 1); % xee_dot_start; 

fprintf('--- Config ---\n')
fprintf('SpaceRobot: %s\n', sr.Name)
fprintf('SR initial config:\n')
fprintf('\t-qb0:')
disp(sr.q0.')
fprintf('\t-qm0:')
disp(sr.qm.')
fprintf('\t-Xee0:')
disp(xee_start.')
fprintf('\t-Xee0 Ref:')
disp(traj(1, :).EE_desired)

% sr.show;
%% NMPC Controller
fprintf('--- NMPC Controller Initialization ---\n')

% --- Parameters ---
fprintf('Parameters:\n')
fprintf('\tTs: %.2f\n', Ts)
fprintf('\tTp: %i (%.2f sec)\n', Tp, Tp * Ts)
fprintf('\tTc: %i (%.2f sec)\n', Tc, Tc * Ts)

n = sr.NumActiveJoints;
N = n + 6;
nx = 12 + 2 * n + 12; % Change to 24 + 2*n w/ EE
ny = 6; % EE states
nu = n + 6;

nlmpc_ee = nlmpc(nx, ny, nu);

nlmpc_ee.Ts = Ts;
nlmpc_ee.PredictionHorizon = Tp;
nlmpc_ee.ControlHorizon = Tc;

% --- Prediction Model ---
nlmpc_ee.Model.NumberOfParameters = 0;

nlmpc_ee.Model.StateFcn = "SR6_ee_state_func_mex";
nlmpc_ee.Model.OutputFcn = "SR6_ee_output_func";
nlmpc_ee.Model.IsContinuousTime = true;

% --- Scales ---
URange = [ones(1, 3)*2*baseMaxForce, ones(1, 3)*2*baseMaxTorque, ones(1, 6)*2*motorMaxTorque];
YRange = [ones(1, 3)*10, ones(1, 3)*2*pi];

for i = 1:nu
    nlmpc_ee.MV(i).ScaleFactor = URange(i);
end

for i = 1:ny
    nlmpc_ee.OV(i).ScaleFactor = YRange(i);
end


% --- Cost Function ---
nlmpc_ee.Weights.OutputVariables = [ones(1, 3)*r_ee_W, ones(1, 3)*psi_ee_W]; % [ree_x ree_y ree_z psi_ee_x psi_ee_y psi_ee_z]
nlmpc_ee.Weights.ManipulatedVariables = [ones(1, 3) * fb_W, ones(1, 3) * nb_W, ones(1, n) * taum_W];

nlmpc_ee.Weights.ManipulatedVariablesRate = [ones(1, 3)*fb_rate_W, ones(1, 3)*nb_rate_W, ones(1, n)*taum_rate_W];
% --- Solver parameters ---
nlmpc_ee.Optimization.UseSuboptimalSolution = true;
nlmpc_ee.Optimization.SolverOptions.MaxIterations = 5000;
nlmpc_ee.Optimization.SolverOptions.Display = 'none'; %'final-detailed';
nlmpc_ee.Optimization.SolverOptions.Algorithm = 'interior-point';
nlmpc_ee.Optimization.SolverOptions.MaxFunctionEvaluations = 10000;

% --- Constraints ---
% Joint positions
for i = 1:sr.NumActiveJoints
    jnt = sr.findJointByConfigId(i);

    nlmpc_ee.States(i + 6).Min = jnt.PositionLimits(1);
    nlmpc_ee.States(i + 6).Max = jnt.PositionLimits(2);
end

% Torques
maxTorques = [ones(3, 1) * baseMaxForce; ones(3, 1) * baseMaxTorque; ones(n, 1) * motorMaxTorque];

% Torque limits
for i = 1:N
    nlmpc_ee.MV(i).Min = -maxTorques(i);
    nlmpc_ee.MV(i).Max = maxTorques(i);
end



% % Validate
x0 = [sr.q; xee0; sr.q_dot; xee0_dot];
u0 = zeros(N, 1);

%% Simulink
if SIM
    simOk = true;
    fprintf('\n--- SIM ---\n')
    set_param(mdl, 'StopTime', num2str(simTime))
    waitBar = waitbar(0, 'Simulation in progress...');

    % Sim Time Timer
    simStartTime = datetime('now', 'TimeZone', 'local', 'Format', 'HH:mm:ss');
    fprintf('Simulation started at: %s\n', simStartTime)
    t = timer;
    t.Period = 2;
    t.ExecutionMode = 'fixedRate';
    t.TimerFcn = @(myTimerObj, thisEvent)waitbar(get_param(mdl, 'SimulationTime') / simTime, ...
        waitBar, {'Simulation in progress...', ...
            [num2str(get_param(mdl, 'SimulationTime')), '/', num2str(simTime)]});

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
        logsout = Simulink.sdi.exportRun(runIDs(end));
        simOk = false;
    end   

    %     profile off
    stop(t);
    delete(t);
    delete(waitBar)
    fprintf('Simulation DONE\n')
    if simOk
        fprintf('Total Sim Time (min): %.2f\n', simRes.getSimulationMetadata.TimingInfo.TotalElapsedWallTime / 60);
    end
    %     profile viewer

    solverStatus = logsout.getElement('nlpStat').Values;
    failedIdx = find(solverStatus.Data <= 0);

    if ~isempty(failedIdx)
        fprintf('Solver failed at time steps:\n')
        arrayfun(@(idx) fprintf('\t - %.2f sec (flag %i)\n', solverStatus.Time(idx), solverStatus.Data(idx)), failedIdx);
    else
        fprintf('Solver succesfull at every time step\n')
    end

    fprintf("Press a key to continue...\n");
    pause;
end

%% Animate
fprintf('\n--- Animation ---\n')
close all
rate = 1;
animStart = 0;

% Extract signals from sim
q = logsout.getElement('q').Values;
xSeq = logsout.getElement('xSeq').Values; % predicted states, (Tp+1 x nx x timeStep). xSeq.
ySeq = logsout.getElement('ySeq').Values; % predicted states, (Tp+1 x ny x timeStep). ySeq.
Xee = logsout.getElement('Xee').Values;
Xee_ref = logsout.getElement('Xee_ref').Values;
tau = logsout.getElement('tau').Values;
Xee_err = logsout.getElement('Xee_err').Values;

% Setup signals for animation
trajRes = struct();
trajRes.ref = ts2timetable(Xee_ref);
trajRes.ref.Properties.VariableNames{1} = 'EE_desired';
trajRes.Xee = Xee;

pred = struct();
pred.Xee = ySeq;

% Save options
fileName = '';
folder = fullfile('results/videos/');

if ~strcmp(fileName, '')
    savePath = fullfile(folder, fileName);
else
    savePath = '';
end

% Animate
tic
sr.animate(q, 'fps', 15, 'rate', rate, 'fileName', savePath, 'traj', trajRes, 'pred', pred, 'viz', 'on', 'tStart', animStart);
toc

%% Plots
if PLOT
%     % --- EE Position Tracking ---
%     figure
%     title("EE Position Trajectory Tracking")
%     subplot(1, 2, 1)
%     xlabel('X [m]')
%     ylabel('Y [m]')
%     grid on
%     axis equal
%     hold on
%     plot(trajRes.ref.EE_desired(:, 1), trajRes.ref.EE_desired(:, 2))
%     plot(reshape(trajRes.Xee.Data(1, :, :), [], 1), reshape(trajRes.Xee.Data(2, :, :), [], 1))
%     legend('Ref', 'NMPC')
%     hold off
%     
%     subplot(1, 2, 2)
%     xlabel('Y [m]')
%     ylabel('Z [m]')
%     grid on
%     axis equal
%     hold on
%     plot(trajRes.ref.EE_desired(:, 2), trajRes.ref.EE_desired(:, 3))
%     plot(reshape(trajRes.Xee.Data(2, :, :), [], 1), reshape(trajRes.Xee.Data(3, :, :), [], 1))
%     legend('Ref', 'NMPC')
%     hold off
%     
%     % --- EE Orientation Tracking ---
%     titles = {'\psi_{ee, x}', '\psi_{ee, y}', '\psi_{ee, z}'};
%     figure
%     sgtitle("EE Orientation Trajectory Tracking")
%     for i=1:3
%         subplot(3, 1, i)
%         title(titles{i})
%         xlabel('Time [sec]')
%         ylabel('[rad]')
%         grid on
%         axis equal
%         hold on
%         plot(trajRes.ref.Time, trajRes.ref.EE_desired(:, i+3), 'DisplayName', 'Ref')
%         plot(trajRes.Xee.Time, reshape(trajRes.Xee.Data(i+3, :, :), [], 1), 'DisplayName', 'NMPC')
%         legend
%         hold off
%     end
    
    % --- EE Tracking Errors---
    varNames = {'X','Y','Z', '\psi_x', '\psi_y', '\psi_z'};
    tt_err = timetable(seconds(Xee_err.Time),...
        reshape(100*Xee_err.Data(1, :, :), [], 1), ...
        reshape(100*Xee_err.Data(2, :, :), [], 1),...
        reshape(100*Xee_err.Data(3, :, :), [], 1),...
        reshape(180/pi*Xee_err.Data(4, :, :), [], 1),...
        reshape(180/pi*Xee_err.Data(5, :, :), [], 1),...
        reshape(180/pi*Xee_err.Data(6, :, :), [], 1),...
        'VariableNames',varNames ...
        );
    
    figure
    vars = {["X","Y","Z"],["\psi_x", "\psi_y", "\psi_z"]};
    yLabels = ["Position Error [cm]", "Orientation Error [deg]"];
    stackedplot(tt_err, vars, "Title","EE Tracking Error", "DisplayLabels",yLabels)
    
    % --- Joint ---
    figure
    hold on
    
    for i = 1:n
        subplot(n, 1, i)
        hold on
        grid on
        title(sprintf('Jnt%i', i))
        xlabel('Time [sec]')
        ylabel('Joint Angle [deg]')
        xlim([0, simTime])
    
        tVect = q.Time;
        plot(tVect, reshape(rad2deg(q.Data(6 + i, :, :)), [], 1))
    
        jnt = sr.findJointByConfigId(i);
        jntMin = rad2deg(jnt.PositionLimits(1));
        jntMax = rad2deg(jnt.PositionLimits(2));
        plot(tVect, repmat(jntMin, length(tVect), 1), 'k--')
        plot(tVect, repmat(jntMax, length(tVect), 1), 'k--')
    end
    
    hold off
    
    % --- Torques ---
    tau_tt = {N, 1};
    
    for i = 1:N
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
    
    for i = 7:N
        name = sprintf('Body%i', i - 6);
        plot(tau_tt{i}, 'DisplayName', name)
    end
    
    legend
    hold off
end
%% Error Stats
fprintf('\n--- ERROR Stats ---\n')

% Convert to cm and deg
err = reshape(Xee_err.Data, 6, [], 1);
err(1:3, :) = err(1:3, :).*100;
err(4:6, :) = err(4:6, :).*(180/pi);

% RMS
rmsError = sqrt(mean((err).^2, 2)).';
fprintf('RMS Errors:\n')
fprintf('\t X: %.2f [cm]\n', rmsError(1))
fprintf('\t Y: %.2f [cm]\n', rmsError(2))
fprintf('\t Z: %.2f [cm]\n', rmsError(3))
fprintf('\t psi_x: %.2f [deg]\n', rmsError(4))
fprintf('\t psi_y: %.2f [deg]\n', rmsError(5))
fprintf('\t psi_z: %.2f [deg]\n', rmsError(6))

averagePositionErr = mean(rmsError(1:3));
averageOriErr = mean(rmsError(4:6));
fprintf('\n\tAverage Position RMS Error: %.2f [cm]\n', averagePositionErr);
fprintf('\tAverage Orientation RMS Error: %.2f [deg]\n', averageOriErr);

% Max Error
errMax = max(err,[],2);
errMaxPos = max(errMax(1:3));
errMaxOri = max(errMax(4:6));

fprintf('\nMAX Errors:\n')
fprintf('\t X: %.2f [cm]\n', errMax(1))
fprintf('\t Y: %.2f [cm]\n', errMax(2))
fprintf('\t Z: %.2f [cm]\n', errMax(3))
fprintf('\t psi_x: %.2f [deg]\n', errMax(4))
fprintf('\t psi_y: %.2f [deg]\n', errMax(5))
fprintf('\t psi_z: %.2f [deg]\n', errMax(6))

fprintf('\n\t Max Position Error: %.2f [cm]\n', errMaxPos)
fprintf('\t Max Orientation Error: %.2f [cm]\n', errMaxOri)



