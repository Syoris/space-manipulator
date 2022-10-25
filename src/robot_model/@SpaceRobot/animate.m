function animate(obj, ts, varargin)
    %ANIMATE Animate the SpaceRobot.
    %   ANIMATE(ts) Animates SpaceRobot along inputted ts (`timeseries`)
    %
    %   ANIMATE(___, Name, Value)
    %
    %      'fps'            - Desired fps of the video. Will resample ts to match correct timestamps.
    %
    %      'rate'           - Rate of video playback i.e. rate=2 will play video at twice the speed
    %
    %                         Default: 1
    %
    %      'traj'           - Specify reference traj of the end effector. struct with fields:
    %                         traj.ref: Reference trajectory as a timetable with `EE_desired` as variable
    %                                   which is a (1x3) vect for each time [x_ref, y_ref, z_ref]
    %                         traj.Xee: SpaceRobot ee position as a timeseries
    %
    %      'fileName'       - To save animation as .avi video file to specified file name
    %
    %      'viz'            - Show space robot visualization
    %
    %
    %      'pred'           - Specify prediction trajectories. struct with fields:
    %                         pred.Xee: Prediction of the ee trajectory as a `timeseries`.
    %                                   with pred.Xee.Data (Tp+1, 3, NTimeSteps) so that:
    %                                       pred.Xee.Data(:, 1, i) is the prediction of x_ee at time step i
    %                                       pred.Xee.Data(:, 2, i) is the prediction of y_ee at time step i
    %                                       pred.Xee.Data(:, 3, i) is the prediction of z_ee at time step i
    %

    % parse inputs
    parser = inputParser;

    parser.addParameter('fps', 0, @(x)(validateattributes(x, {'numeric'}, ...
        {'real', 'nonnan', 'finite'})));

    parser.addParameter('traj', [], @(x)(validateattributes(x, {'struct'}, ...
        {'nonempty'})));

    parser.addParameter('rate', 1, @(x)(validateattributes(x, {'numeric'}, ...
        {'real', 'nonnan', 'positive'})));

    parser.addParameter('fileName', '');

    parser.addParameter('viz', 'on', ...
        @(x)any(validatestring(x, {'on', 'off'})));

    parser.addParameter('pred', [], @(x)(validateattributes(x, {'struct'}, ...
        {'nonempty'})));

    parser.parse(varargin{:});

    fps = parser.Results.fps;
    traj = parser.Results.traj;
    pred = parser.Results.pred;
    rate = parser.Results.rate;
    fileName = parser.Results.fileName;
    viz = parser.Results.viz;

    % Setup
    dim = [.2 .5 .3 .3]; % For annotation
    str = sprintf('t = %.2fs', 0);
    fpsSet = false;
    plotTraj = false;
    plotPred = false;

    if ~isempty(traj)
        plotTraj = true;
    end

    if ~isempty(pred)
        plotPred = true;
    end

    nFrame = length(ts.Time);
    Tend = ts.Time(end);
    tVect = ts.Time;

    if fps ~= 0
        fpsSet = true;
        tVect = ts.Time(1):1 / fps * rate:Tend;
        nFrame = length(tVect);

        ts = resample(ts, tVect);

        if plotTraj
            traj.XeeResamp = resample(traj.Xee, tVect);
        end

        if plotPred
%             pred.Xee = pred.Xee.resample(tVect);
            pred.Xee = retime(timeseries2timetable(pred.Xee), seconds(tVect), 'previous');
            
        end

        r = rateControl(fps);
    end

    % Setup video
    if ~isempty(fileName)
        myVideo = VideoWriter(fileName);
        myVideo.FrameRate = fps;
        myVideo.Quality = 100;
        open(myVideo);
    end

    % First frame
    obj.q = ts.Data(:, :, 1);
    h_annot = annotation('textbox', dim, 'String', str, 'FitBoxToText', 'on');
    obj.show('preserve', false, 'fast', true, 'visuals', viz);

    hold on

    if plotTraj
        plot3(traj.ref.EE_desired(:, 1), traj.ref.EE_desired(:, 2), traj.ref.EE_desired(:, 3), 'r')
        traj_line = animatedline('Color', 'b', 'LineWidth', 2);
    end

    if plotPred
        lPred = plot3(pred.Xee(1, :).ySeq(:, :, 1), pred.Xee(1, :).ySeq(:, :, 2), pred.Xee(1, :).ySeq(:, :, 3), 'g', 'LineWidth', 1.5);
        startPred = plot3(pred.Xee(1, :).ySeq(:, 1, 1), pred.Xee(1, :).ySeq(:, 1, 2), pred.Xee(1, :).ySeq(:, 1, 3), 'gX', 'LineWidth', 1.5);
%         startXee = plot3(traj.Xee.Data(1, :, 1), traj.Xee.Data(2, :, 1), traj.Xee.Data(3, :, 1), 'bX', 'LineWidth', 1.5);
    end

    hold off

    drawnow

    try

        % Animate
        for i = 1:nFrame
            curT = tVect(i);
            str = sprintf('t = %.2fs', curT);

            obj.q = ts.Data(:, :, i);
            obj.show('preserve', false, 'fast', true, 'visuals', viz);

            h_annot.set('String', str);

            if plotTraj
                addpoints(traj_line, traj.XeeResamp.Data(1, :, i), traj.XeeResamp.Data(2, :, i), traj.XeeResamp.Data(3, :, i));
            end

            if plotPred
                XeePredData = pred.Xee(i, :).ySeq;
%                 XeePredData = pred.Xee.getsampleusingtime(curT).Data;
%                 XeeData = traj.Xee.getsampleusingtime(curT).Data;
                    

                lPred.XData = XeePredData(:, :, 1);
                lPred.YData = XeePredData(:, :, 2);
                lPred.ZData = XeePredData(:, :, 3);

                startPred.XData = XeePredData(:, 1, 1);
                startPred.YData = XeePredData(:, 1, 2);
                startPred.ZData = XeePredData(:, 1, 3);
%                 if ~isempty(XeePredData)
%                     lPred.XData = XeePredData(:, 1);
%                     lPred.YData = XeePredData(:, 2);
%                     lPred.ZData = XeePredData(:, 3);
% 
%                     startPred.XData = XeePredData(1, 1);
%                     startPred.YData = XeePredData(1, 2);
%                     startPred.ZData = XeePredData(1, 3);
%                 end

%                 if ~isempty(XeeData)
%                     startXee.XData = XeeData(1, :);
%                     startXee.YData = XeeData(2, :);
%                     startXee.ZData = XeeData(3, :);
%                 end

            end

            drawnow;

            if ~isempty(fileName)
                frame = getframe(gcf);
                writeVideo(myVideo, frame);
            end

            if fpsSet
                waitfor(r);
            end

        end

        % Add one frame at the end
        obj.show('preserve', false, 'fast', true, 'visuals', viz);
        drawnow;

        if ~isempty(fileName)
            writeVideo(myVideo, frame);
        end

        waitfor(r);

        if ~isempty(fileName)
            fprintf("Saving video to %s", fileName)
            close(myVideo)
        end

    catch ME

        switch ME.identifier
            case 'MATLAB:class:InvalidHandle'
                obj.logger("Animation ended early", 'warning')
                return
            otherwise
                rethrow(ME)
        end

    end

end
