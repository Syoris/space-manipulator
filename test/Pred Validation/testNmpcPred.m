%% To test prediction of NMPC
% Check output if MV seq is applied in OL to SR

% % 6
logsoutComp = logsout;
N = 12;
% sr = sr6;

% 2
% logsout = logsout2;
% N = 8;
% sr = sr2;

time = 0.5;

% Get parameters
mvSeq = logsoutComp.getElement('mvSeq').Values; %  Sequence of all
mvKSeq = mvSeq.getsampleusingtime(time).Data; % (Tpx12)
% tt = ts2timetable(mvSeq);
mvTT = timetable(mvKSeq, 'TimeStep',seconds(Ts),'VariableNames',{'mv'});
mvTT = retime(mvTT,'regular','previous','TimeStep',seconds(0.001));

% Access specific time: mvTT(seconds(0.1), :)

xSeq = logsoutComp.getElement('xSeq').Values; %  Sequence of all
xKSeq = xSeq.getsampleusingtime(time).Data; % (Tpx12)
% tt = ts2timetable(mvSeq);
xTT = timetable(xKSeq, 'TimeStep',seconds(Ts),'VariableNames',{'xk'});

% Initial Cond
q0 = xTT(seconds(0), :).xk(1:N);
q_dot_0 = xTT(seconds(0), :).xk(N+7:2*N+6);
x_dot_0 = [q_dot_0(1:3), omega2euler(q0(4:6)', q_dot_0(4:6)')', q_dot_0(7:end)];

xee0 = xTT(seconds(0), :).xk(N+1:N+6);
xee_dot_0 = xTT(seconds(0), :).xk(2*N+7:2*N+12);


%% --- Sim ---
% figsToPlot = {'Base', 'Joints', 'EE'};

close all
simTime = Tp*Ts;
mdlPred = 'testNmpcPred_sim';
set_param(mdlPred, 'StopTime', num2str(simTime))
simRes = sim(mdlPred);  

predLogsout = simRes.logsout;

%% --- Plots ---
tau = predLogsout.getElement('tau').Values;
x = predLogsout.getElement('X').Values;
xPred = predLogsout.getElement('Xpred').Values;
x2 = predLogsout.getElement('X2').Values;

% xee3 = predLogsout.getElement('Xee3').Values;
% xee_dot3 = predLogsout.getElement('Xee_dot3').Values;

% % ### Base Position ###
% idx = 1:6;
% names = {'rx', 'ry', 'rz', '\psi_{bx}', '\psi_{by}', '\psi_{bz}'};
% figure
% for i=1:6
%     subplot(6, 1, i)
%     title(names{i})
%     hold on
%     grid on
%     plot(x.Time, reshape(x.Data(idx(i), :, :), 1, []), 'DisplayName', 'Sim')
%     plot(xPred.Time, reshape(xPred.Data(idx(i), :, :), 1, []), 'DisplayName', 'Pred')    
%     legend;
%     hold off
% end
% sgtitle('Base position')

% % ### Joints ###
% idx = 7:N;
% figure
% for i=1:length(idx)
%     subplot(n, 1, i)
%     title(['Joint ', num2str(i)])
%     hold on
%     grid on
%     plot(x.Time, reshape(x.Data(idx(i), :, :), 1, []), 'DisplayName', 'Sim')
%     plot(xPred.Time, reshape(xPred.Data(idx(i), :, :), 1, []), 'DisplayName', 'Pred')   
%     legend;
%     hold off
% end
% sgtitle('Joints')

% ### EE ###
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
    plot(x2.Time, reshape(x2.Data(idx(i), :, :), 1, []), '--', 'DisplayName', 'X2')
    legend;
    hold off
end
sgtitle('EE position')

% --- Speeds ---
% % ### Base Vels ###
% idx = N+7:N+12;
% names = {'vx', 'vy', 'vz', '\psi_{b, x}', '\psi_{b, y}', '\psi_{b, z}'};
% figure
% for i=1:6
%     subplot(6, 1, i)
%     title(names{i})
%     hold on
%     grid on
%     plot(x.Time, reshape(x.Data(idx(i), :, :), 1, []), 'DisplayName', 'Sim')
%     plot(xPred.Time, reshape(xPred.Data(idx(i), :, :), 1, []), 'DisplayName', 'Pred')
%     legend;
%     hold off
% end
% sgtitle('Base Velocity')

% % ### Joint Vels ###
% idx = N+13:2*N+6;
% figure
% for i=1:length(idx)
%     subplot(n, 1, i)
%     title(['Joint ', num2str(i), ', speed'])
%     hold on
%     grid on
%     plot(x.Time, reshape(x.Data(idx(i), :, :), 1, []), 'DisplayName', 'Sim')
%     plot(xPred.Time, reshape(xPred.Data(idx(i), :, :), 1, []), 'DisplayName', 'Pred')
%     legend;
%     hold off
% end
% sgtitle('Joints Velocities')

% ### EE Vels ###
idx = 2*N+7:2*N+12;
names = {'vx_{ee}', 'vy_{ee}', 'vz_{ee}', '\psi_{ee, x}', '\psi_{ee, y}', '\psi_{ee, z}'};
figure
for i=1:6
    subplot(6, 1, i)
    title(names{i})
    hold on
    grid on
    plot(x.Time, reshape(x.Data(idx(i), :, :), 1, []), 'DisplayName', 'Sim')
    plot(xPred.Time, reshape(xPred.Data(idx(i), :, :), 1, []), 'DisplayName', 'Pred')
    plot(x2.Time, reshape(x2.Data(idx(i), :, :), 1, []), '--','DisplayName', 'X2')
    legend;
    hold off
end
sgtitle('EE Velocities')