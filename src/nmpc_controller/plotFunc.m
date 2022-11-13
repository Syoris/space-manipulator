function plotFunc(sr, logsout, simTime, n, N)
% PLOTFUNC plot the results
    
    %% Setup
    % Extract signals from sim
    q = logsout.getElement('q').Values;
    q_dot = logsout.getElement('q_dot').Values;
    xSeq = logsout.getElement('xSeq').Values; % predicted states, (Tp+1 x nx x timeStep). xSeq.
    ySeq = logsout.getElement('ySeq').Values; % predicted states, (Tp+1 x ny x timeStep). ySeq.
    Xee = logsout.getElement('Xee').Values;
    Xee_ref = logsout.getElement('Xee_ref').Values;
    tau = logsout.getElement('tau').Values;
    Xee_err = logsout.getElement('Xee_err').Values;
    mFuel = logsout.getElement('m_fuel').Values;
    ySeq.Name = 'ySeq';
    
    % Setup signals for animation
    trajRes = struct();
    trajRes.ref = ts2timetable(Xee_ref);
    trajRes.ref.Properties.VariableNames{1} = 'EE_desired';
    trajRes.Xee = Xee;
    
    pred = struct();
    pred.Xee = ySeq;


    %% Plots
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
%         xlim([seconds(0) seconds(simTime)])
%         hold off
%     end
    
    % --- Base Orientation  ---
    titles = {'\psi_{b, x}', '\psi_{b, y}', '\psi_{b, z}'};
    figure
    sgtitle("Base Orientation")      
    for i = 1:3
        subplot(3, 1, i)
        hold on
        grid on
        title(titles{i})
        xlabel('Time [sec]')
        ylabel('Base Angle [deg]')
        xlim([0, simTime])
    
        tVect = q.Time;
        plot(tVect, reshape(rad2deg(q.Data(3 + i, :, :)), [], 1))
        if i~=1
            minAng = -30;
            maxAng = 30;
    
            plot(tVect, repmat(minAng, length(tVect), 1), 'k--')
            plot(tVect, repmat(maxAng, length(tVect), 1), 'k--')
            xlim([0 simTime])
        end
        hold off
    end

    % --- EE Tracking Errors---
    varNames = {'X','Y','Z', '\psi_x', '\psi_y', '\psi_z'};
    tt_err = timetable(seconds(Xee_err.Time),...
        reshape(Xee_err.Data(1, :, :), [], 1), ...
        reshape(Xee_err.Data(2, :, :), [], 1),...
        reshape(Xee_err.Data(3, :, :), [], 1),...
        reshape(Xee_err.Data(4, :, :), [], 1),...
        reshape(Xee_err.Data(5, :, :), [], 1),...
        reshape(Xee_err.Data(6, :, :), [], 1),...
        'VariableNames',varNames ...
        );
    
    figure
    vars = {["X","Y","Z"],["\psi_x", "\psi_y", "\psi_z"]};
    yLabels = ["Position Error [cm]", "Orientation Error [deg]"];
    stackedplot(tt_err, vars, "Title","EE Tracking Error", "DisplayLabels",yLabels)
    
%     % --- Joint ---
%     figure
%     hold on    
%     for i = 1:n
%         subplot(n, 1, i)
%         hold on
%         grid on
%         title(sprintf('Jnt%i', i))
%         xlabel('Time [sec]')
%         ylabel('Joint Angle [deg]')
%         xlim([0, simTime])
%     
%         tVect = q.Time;
%         plot(tVect, reshape(rad2deg(q.Data(6 + i, :, :)), [], 1))
%     
%         jnt = sr.findJointByConfigId(i);
%         jntMin = rad2deg(jnt.PositionLimits(1));
%         jntMax = rad2deg(jnt.PositionLimits(2));
%         plot(tVect, repmat(jntMin, length(tVect), 1), 'k--')
%         plot(tVect, repmat(jntMax, length(tVect), 1), 'k--')
%         xlim([0 simTime])
%     end
%     hold off
    
    % --- Torques ---
    tau_tt = {N, 1};    
    for i = 1:N
        tau_tt{i} = tau;
        tau_tt{i}.Data = tau.Data(:, i);
    end    
    figure
    subplot(4, 1, 1)
    title('Base force')
    hold on
    plot(tau_tt{1})
    plot(tau_tt{2})
    plot(tau_tt{3})
    legend('Fx', 'Fy', 'Fz')
    xlim([0 simTime])
    hold off
    
    subplot(4, 1, 2)
    title('Base torque')
    hold on
    plot(tau_tt{4})
    plot(tau_tt{5})
    plot(tau_tt{6})
    legend('nx', 'ny', 'nz')
    xlim([0 simTime])
    hold off
    
    subplot(4, 1, 3)
    title('Joint torques')
    hold on   
    for i = 7:9
        name = sprintf('Body%i', i - 6);
        plot(tau_tt{i}, 'DisplayName', name)
    end
    legend
    xlim([0 simTime])
    hold off
    
    subplot(4, 1, 4)
    title('Joint torques')
    hold on   
    for i = 10:N
        name = sprintf('Body%i', i - 6);
        plot(tau_tt{i}, 'DisplayName', name)
    end
    legend
    xlim([0 simTime])
    hold off

    % --- Fuel ---
    varNames = {'Fuel'};
    tt_fuel = timetable(seconds(mFuel.Time), mFuel.Data*1000, 'VariableNames',varNames);
    
    figure
    yLabels = "Fuel Mass [g]";
    stackedplot(tt_fuel, "Title","Fuel Consumption", "DisplayLabels",yLabels)

end