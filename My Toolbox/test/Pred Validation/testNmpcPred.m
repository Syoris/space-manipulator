%% To test prediction of NMPC
% Check output if MV seq is applied in OL to SR

time = 0.5;

% Get parameters
mvSeq = logsout.getElement('mvSeq').Values; %  Sequence of all
mvKSeq = mvSeq.getsampleusingtime(time).Data; % (Tpx12)
% tt = ts2timetable(mvSeq);
mvTT = timetable(mvKSeq, 'TimeStep',seconds(Ts),'VariableNames',{'mv'});
mvTT = retime(mvTT,'regular','previous','TimeStep',seconds(0.001));

% Access specific time: mvTT(seconds(0.1), :)

xSeq = logsout.getElement('xSeq').Values; %  Sequence of all
xKSeq = xSeq.getsampleusingtime(time).Data; % (Tpx12)
% tt = ts2timetable(mvSeq);
xTT = timetable(xKSeq, 'TimeStep',seconds(Ts),'VariableNames',{'xk'});

% Initial Cond
q0 = xTT(seconds(0), :).xk(1:N);
q_dot_0 = xTT(seconds(0), :).xk(N+7:2*N+6);

%% Plots
close all
simRes = sim('testNmpcPred_sim');  

predLogsout = simRes.logsout;

tau = predLogsout.getElement('tau').Values;
x = predLogsout.getElement('x').Values;
xPred = predLogsout.getElement('xPred').Values;

% Base Position
idx = 1:6;
names = {'rx', 'ry', 'rz', '\psi_{bx}', '\psi_{by}', '\psi_{bz}'};
figure
for i=1:6
    subplot(6, 1, i)
    title(names{i})
    hold on
    grid on
    plot(x.Time, reshape(x.Data(idx(i), :, :), 1, []), 'DisplayName', 'Sim')
    plot(xPred.Time, reshape(xPred.Data(idx(i), :, :), 1, []), 'DisplayName', 'Pred')
    legend;
    hold off
end
sgtitle('Base position')

% Joints
idx = 7:N;
figure
for i=1:length(idx)
    subplot(n, 1, i)
    title(['Joint ', num2str(i)])
    hold on
    grid on
    plot(x.Time, reshape(x.Data(idx(i), :, :), 1, []), 'DisplayName', 'Sim')
    plot(xPred.Time, reshape(xPred.Data(idx(i), :, :), 1, []), 'DisplayName', 'Pred')
    legend;
    hold off
end
sgtitle('Joints')

% EE
idx = N+1:N+6;
names = {'rx_ee', 'ry_ee', 'rz_ee', '\psi_{ee, x}', '\psi_{ee, y}', '\psi_{ee, z}'};
figure
for i=1:6
    subplot(6, 1, i)
    title(names{i})
    hold on
    grid on
    plot(x.Time, reshape(x.Data(idx(i), :, :), 1, []), 'DisplayName', 'Sim')
    plot(xPred.Time, reshape(xPred.Data(idx(i), :, :), 1, []), 'DisplayName', 'Pred')
    legend;
    hold off
end
sgtitle('EE position')

% --- Speeds ---
% Base Position
idx = N+7:N+12;
names = {'vx', 'vy', 'vz', '\omega_{b, x}', '\omega_{b, y}', '\omega_{b, z}'};
figure
for i=1:6
    subplot(6, 1, i)
    title(names{i})
    hold on
    grid on
    plot(x.Time, reshape(x.Data(idx(i), :, :), 1, []), 'DisplayName', 'Sim')
    plot(xPred.Time, reshape(xPred.Data(idx(i), :, :), 1, []), 'DisplayName', 'Pred')
    legend;
    hold off
end
sgtitle('Base Velocity')

% Joints
idx = N+13:2*N+6;
figure
for i=1:length(idx)
    subplot(n, 1, i)
    title(['Joint ', num2str(i), ', speed'])
    hold on
    grid on
    plot(x.Time, reshape(x.Data(idx(i), :, :), 1, []), 'DisplayName', 'Sim')
    plot(xPred.Time, reshape(xPred.Data(idx(i), :, :), 1, []), 'DisplayName', 'Pred')
    legend;
    hold off
end
sgtitle('Joints Velocities')

% EE
idx = 2*N+7:2*N+12;
names = {'vx_{ee}', 'vy_{ee}', 'vz_{ee}', '\omega_{ee, x}', '\omega_{ee, y}', '\omega_{ee, z}'};
figure
for i=1:6
    subplot(6, 1, i)
    title(names{i})
    hold on
    grid on
    plot(x.Time, reshape(x.Data(idx(i), :, :), 1, []), 'DisplayName', 'Sim')
    plot(xPred.Time, reshape(xPred.Data(idx(i), :, :), 1, []), 'DisplayName', 'Pred')
    legend;
    hold off
end
sgtitle('EE Velocities')