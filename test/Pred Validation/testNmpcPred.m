%% To test prediction of NMPC
% Check output if MV seq is applied in OL to SR

% % 6
logsoutComp = logsout;
N = 12;
% sr = sr6;
sr_info = SR6_info();

% 2
% logsout = logsout2;
% N = 8;
% sr = sr2;

time = 0.4;
fprintf("Comparing sim and prediction at time: %.2f\n", time);

% Get parameters
mvSeq = logsoutComp.getElement('mvSeq').Values; %  Sequence of all
mvKSeq = mvSeq.getsampleusingtime(time - 0.001, time + 0.001).Data; % (Tpx12)
% tt = ts2timetable(mvSeq);
mvTT = timetable(mvKSeq, 'TimeStep', seconds(Ts), 'VariableNames', {'mv'});
mvTT = retime(mvTT, 'regular', 'previous', 'TimeStep', seconds(0.001));

% Access specific time: mvTT(seconds(0.1), :)

xSeq = logsoutComp.getElement('xSeq').Values; %  Sequence of all
xKSeq = xSeq.getsampleusingtime(time - 0.001, time + 0.001).Data; % (Tpx12)
xInt = xSeq.getsampleusingtime(time, time + Ts * (Tp + 1)).Data; % Get robot states from sim over horizon
xInt = xInt(1, :, :);
xInt = reshape(xInt, 4, [], 1);

xTT = timetable(xKSeq, 'TimeStep', seconds(Ts), 'VariableNames', {'xk'});
XSim = timetable(xInt, 'TimeStep', seconds(Ts), 'VariableNames', {'X'});

% Initial Cond
q0 = xTT(seconds(0), :).xk(1:N);
x_dot_0 = xTT(seconds(0), :).xk(N + 7:2 * N + 6);

q_dot_0 = x_dot_0;
q_dot_0(4:6) = euler2omega_local(q0(4:6)', q_dot_0(4:6)')';

% x_dot_0 = [q_dot_0(1:3), omega2euler_local(q0(4:6)', q_dot_0(4:6)')', q_dot_0(7:end)];

[xee0, xee_dot_0] = ee_speed(sr_info, q0.', q_dot_0.');
xee0 = xee0.';
xee_dot_0 = xee_dot_0.';
xee0(4) = xee0(4); %+ 2*pi;

% xee0 = xTT(seconds(0), :).xk(N+1:N+6);
% xee_dot_0 = xTT(seconds(0), :).xk(2*N+7:2*N+12);

% [R, r] = tr2rt(sr.getTransform('endeffector', 'TargetFrame', 'inertial', 'config', q0', 'Symbolic', false));
% psi = tr2rpy(R, 'zyx');

%% --- Sim ---
% figsToPlot = {'Base', 'Joints', 'EE'};
fprintf('Launching sim...\n');
close all
simTime = Tp * Ts;
mdlPred = 'testNmpcPred_sim';
set_param(mdlPred, 'StopTime', num2str(simTime))
simRes = sim(mdlPred);

predLogsout = simRes.logsout;

%% --- Plots ---
close all
% runIDs = Simulink.sdi.getAllRunIDs;
% predLogsout = Simulink.sdi.exportRun(runIDs(end));
fprintf('Plotting...\n');
tau = predLogsout.getElement('tau').Values;
x = predLogsout.getElement('X').Values;
xPred = predLogsout.getElement('Xpred').Values;
x2 = predLogsout.getElement('X2').Values;

% % ### Torques ###
% names = {'fb_x', 'fb_y', 'fb_z', 'nb_x', 'nb_y', 'nb_z', '\tau_1', '\tau_2', '\tau_3', '\tau_4', '\tau_5', '\tau_6'};
% figure
% subplot(3, 1, 1)
% title('Base Force')
% hold on
% grid on
% for i = 1:3
%     plot(tau.Time, reshape(tau.Data(i, :, :), 1, []), 'DisplayName', names{i})
% end
% legend;
% hold off
% 
% subplot(3, 1, 2)
% title('Base Torque')
% hold on
% grid on
% for i = 4:6
%     plot(tau.Time, reshape(tau.Data(i, :, :), 1, []), 'DisplayName', names{i})
% end
% legend;
% hold off
% 
% subplot(3, 1, 3)
% title('Manip Torques')
% hold on
% grid on
% for i = 7:N
%     plot(tau.Time, reshape(tau.Data(i, :, :), 1, []), 'DisplayName', names{i})
% end
% legend;
% hold off
% sgtitle('Torques')

% ### Base Position ###
% idx = 1:6;
% names = {'rx', 'ry', 'rz', '\psi_{bx}', '\psi_{by}', '\psi_{bz}'};
% figure
% for i = 1:6
%     subplot(6, 1, i)
%     title(names{i})
%     hold on
%     grid on
%     plot(x.Time, reshape(x.Data(idx(i), :, :), 1, []), 'DisplayName', 'Sim')
%     plot(xPred.Time, reshape(xPred.Data(idx(i), :, :), 1, []), 'DisplayName', 'Pred')
%     %     plot(xSim.Time, reshape(xSim.Data(idx(i), :, :), 1, []), '--','DisplayName', 'Res')
%     legend;
%     hold off
% end
% sgtitle('Base position')

% ### Base Position ERROR ###
% idx = 1:6;
% names = {'rx', 'ry', 'rz', '\psi_{bx}', '\psi_{by}', '\psi_{bz}'};
% figure
% for i = 1:6
%     subplot(6, 1, i)
%     title(names{i})
%     hold on
%     grid on
%     plot(x.Time, abs(reshape(x.Data(idx(i), :, :), 1, []) - reshape(xPred.Data(idx(i), :, :), 1, [])), 'DisplayName', 'Sim')
%     legend;
%     hold off
% end
% sgtitle('Base position ERROR')

% % ### Joints ###
% idx = 7:N;
% figure
% 
% for i = 1:length(idx)
%     subplot(n, 1, i)
%     title(['Joint ', num2str(i)])
%     hold on
%     grid on
%     plot(x.Time, reshape(x.Data(idx(i), :, :), 1, []), 'DisplayName', 'Sim')
%     plot(xPred.Time, reshape(xPred.Data(idx(i), :, :), 1, []), 'DisplayName', 'Pred')
%     legend;
%     hold off
% end
% 
% sgtitle('Joints')

% ### EE ###
idx = N + 1:N + 6;
names = {'rx_ee', 'ry_ee', 'rz_ee', '\psi_{ee, x}', '\psi_{ee, y}', '\psi_{ee, z}'};
xlabels = {'[cm]', '[deg]'};
figure

for i = 1:3
    subplot(6, 1, i)
    title(names{i})
    hold on
    grid on
    plot(x.Time, reshape(x.Data(idx(i), :, :), 1, [])*100, 'DisplayName', 'Sim')
    plot(xPred.Time, reshape(xPred.Data(idx(i), :, :), 1, [])*100, 'DisplayName', 'Pred')
    plot(x2.Time, reshape(x2.Data(idx(i), :, :), 1, [])*100, '--', 'DisplayName', 'X2')
    ylabel('[cm]')
    legend;
    hold off
end
for i = 4:6
    subplot(6, 1, i)
    title(names{i})
    hold on
    grid on
    plot(x.Time, reshape(x.Data(idx(i), :, :), 1, [])*180/pi, 'DisplayName', 'Sim')
    plot(xPred.Time, reshape(xPred.Data(idx(i), :, :), 1, [])*180/pi, 'DisplayName', 'Pred')
    plot(x2.Time, reshape(x2.Data(idx(i), :, :), 1, [])*180/pi, '--', 'DisplayName', 'X2')
    ylabel('[deg]')
    legend;
    hold off
end
sgtitle('EE position')

% ### EE Error ###
idx = N + 1:N + 6;
names = {'rx_ee', 'ry_ee', 'rz_ee', '\psi_{ee, x}', '\psi_{ee, y}', '\psi_{ee, z}'};
figure
for i = 1:3
    subplot(6, 1, i)
    title(names{i})
    hold on
    grid on
    plot(x.Time, abs(reshape(x.Data(idx(i), :, :), 1, []) - reshape(xPred.Data(idx(i), :, :), 1, []))*100, 'DisplayName', 'Sim')
    plot(x.Time, abs(reshape(x2.Data(idx(i), :, :), 1, []) - reshape(xPred.Data(idx(i), :, :), 1, []))*100, '--', 'DisplayName', 'X2')
    ylabel('[cm]')
    legend;
    hold off
end
for i = 4:6
    subplot(6, 1, i)
    title(names{i})
    hold on
    grid on
    plot(x.Time, abs(reshape(x.Data(idx(i), :, :), 1, []) - reshape(xPred.Data(idx(i), :, :), 1, []))*180/pi, 'DisplayName', 'Sim')
    plot(x2.Time, abs(reshape(x2.Data(idx(i), :, :), 1, []) - reshape(xPred.Data(idx(i), :, :), 1, []))*180/pi, '--', 'DisplayName', 'X2')
    ylabel('[deg]')
    legend;
    hold off
end
sgtitle('EE Position Error')

% --- Speeds ---
% % ### Base Vels ###
% idx = N + 7:N + 12;
% names = {'vx', 'vy', 'vz', '\psi_{b, x}', '\psi_{b, y}', '\psi_{b, z}'};
% figure
% 
% for i = 1:6
%     subplot(6, 1, i)
%     title(names{i})
%     hold on
%     grid on
%     plot(x.Time, reshape(x.Data(idx(i), :, :), 1, []), 'DisplayName', 'Sim')
%     plot(xPred.Time, reshape(xPred.Data(idx(i), :, :), 1, []), 'DisplayName', 'Pred')
%     legend;
%     hold off
% end
% 
% sgtitle('Base Velocity')
% 
% % ### Joint Vels ###
% idx = N + 13:2 * N + 6;
% figure
% 
% for i = 1:length(idx)
%     subplot(n, 1, i)
%     title(['Joint ', num2str(i), ', speed'])
%     hold on
%     grid on
%     plot(x.Time, reshape(x.Data(idx(i), :, :), 1, []), 'DisplayName', 'Sim')
%     plot(xPred.Time, reshape(xPred.Data(idx(i), :, :), 1, []), '--', 'DisplayName', 'Pred')
%     legend;
%     hold off
% end
% sgtitle('Joints Velocities')

% ### Base Vels Error###
idx = N + 7:N + 12;
names = {'vx', 'vy', 'vz', '\psi_{b, x}', '\psi_{b, y}', '\psi_{b, z}'};
figure
for i = 1:3
    subplot(6, 1, i)
    title(names{i})
    hold on
    grid on
    plot(x.Time, abs(reshape(x.Data(idx(i), :, :), 1, []) - reshape(xPred.Data(idx(i), :, :), 1, []))*100, 'DisplayName', 'Sim')
    ylabel('[cm/s]')
    legend;
    hold off
end
for i = 4:6
    subplot(6, 1, i)
    title(names{i})
    hold on
    grid on
    plot(x.Time, abs(reshape(x.Data(idx(i), :, :), 1, []) - reshape(xPred.Data(idx(i), :, :), 1, []))*180/pi, 'DisplayName', 'Sim')
    plot(x.Time, abs(reshape(x2.Data(idx(i), :, :), 1, []) - reshape(xPred.Data(idx(i), :, :), 1, []))*180/pi, '--', 'DisplayName', 'X2')
    ylabel('[deg/s]')
    legend;
    hold off
end
sgtitle('Base Velocity Error')

% % ### Joint Vels Error ###
% idx = N + 13:2 * N + 6;
% figure
% for i = 1:length(idx)
%     subplot(n, 1, i)
%     title(['Joint ', num2str(i), ', speed'])
%     hold on
%     grid on
%     plot(x.Time, abs(reshape(x.Data(idx(i), :, :), 1, []) - reshape(xPred.Data(idx(i), :, :), 1, []))*180/pi, 'DisplayName', 'Sim')
%     ylabel('[deg]')
%     legend;
%     hold off
% end
% sgtitle('Joints Velocities Error')


% ### EE Vels ###
% idx = 2 * N + 7:2 * N + 12;
% names = {'vx_{ee}', 'vy_{ee}', 'vz_{ee}', '\psi_{ee, x}, dot', '\psi_{ee, y}, dot', '\psi_{ee, z}, dot'};
% figure
% for i = 1:6
%     subplot(6, 1, i)
%     title(names{i})
%     hold on
%     grid on
%     plot(x.Time, reshape(x.Data(idx(i), :, :), 1, []), 'DisplayName', 'Sim')
%     plot(xPred.Time, reshape(xPred.Data(idx(i), :, :), 1, []), 'DisplayName', 'Pred')
%     plot(x2.Time, reshape(x2.Data(idx(i), :, :), 1, []), '--', 'DisplayName', 'X2')
%     legend;
%     hold off
% end
% sgtitle('EE Velocities')

% ### EE Vels Error###
idx = 2 * N + 7:2 * N + 12;
names = {'vx_{ee}', 'vy_{ee}', 'vz_{ee}', '\psi_{ee, x}, dot', '\psi_{ee, y}, dot', '\psi_{ee, z}, dot'};
figure
for i = 1:3
    subplot(6, 1, i)
    title(names{i})
    hold on
    grid on
    plot(x.Time, abs(reshape(x.Data(idx(i), :, :), 1, []) - reshape(xPred.Data(idx(i), :, :), 1, []))*100, 'DisplayName', 'Sim')
%     legend;
    ylabel('[cm/s]')
    hold off
end
for i = 4:6
    subplot(6, 1, i)
    title(names{i})
    hold on
    grid on
    plot(x.Time, abs(reshape(x.Data(idx(i), :, :), 1, []) - reshape(xPred.Data(idx(i), :, :), 1, []))*180/pi, 'DisplayName', 'Sim')
    ylabel('[deg/s]')
%     legend;
    hold off
end
sgtitle('EE Velocities ERROR')

fprintf('Done\n');
