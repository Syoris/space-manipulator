function animateFunc(sr, logsout, rate, fileName)
% ANIMATEFUNC Animates results
    
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
    ySeq.Name = 'ySeq';
    
    % Setup signals for animation
    trajRes = struct();
    trajRes.ref = ts2timetable(Xee_ref);
    trajRes.ref.Properties.VariableNames{1} = 'EE_desired';
    trajRes.Xee = Xee;
    
    pred = struct();
    pred.Xee = ySeq;

    folder = fullfile('results/videos/');

    if ~strcmp(fileName, '')
        savePath = fullfile(folder, fileName);
    else
        savePath = '';
    end

    %% Animate
    tStart = 0;

    tic
    sr.animate(q, 'fps', 15, 'rate', rate, 'fileName', savePath, 'traj', trajRes, 'pred', pred, 'viz', 'on', 'tStart', tStart);
    toc


end