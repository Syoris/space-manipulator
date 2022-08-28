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

    % parse inputs
    parser = inputParser;

    parser.addParameter('fps', 0, @(x)(validateattributes(x, {'numeric'}, ...
                                     {'real', 'nonnan', 'finite'})));

    parser.addParameter('traj', [], @(x)(validateattributes(x, {'struct'}, ...
                                     {'nonempty'})));
    
    parser.addParameter('rate', 1, @(x)(validateattributes(x, {'numeric'}, ...
                                      {'real', 'nonnan', 'positive'})));

    parser.addParameter('fileName', '');

    parser.parse(varargin{:});
                
    fps = parser.Results.fps;
    traj = parser.Results.traj;
    rate = parser.Results.rate;
    fileName = parser.Results.fileName;

    % Setup
    dim = [.2 .5 .3 .3]; % For annotation
    str = sprintf('t = %.2fs', 0);
    fpsSet = false;
    plotTraj = false;

    if ~isempty(traj)
        plotTraj = true;
    end

    nFrame = length(ts.Time);
    Tend = ts.Time(end);
    tVect = ts.Time;

    if fps ~= 0
        fpsSet = true;
        tVect = ts.Time(1):1/fps*rate:Tend;
        nFrame = length(tVect);

        ts = resample(ts, tVect);
        if plotTraj
            traj.Xee = resample(traj.Xee, tVect);
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
    h_annot = annotation('textbox', dim,'String', str,'FitBoxToText','on');
    obj.show('preserve', false, 'fast', true, 'visuals', 'on');

    if plotTraj
        hold on
        plot3(traj.ref.EE_desired(:, 1), traj.ref.EE_desired(:, 2), traj.ref.EE_desired(:, 3), 'r')
        traj_line = animatedline('Color', 'b', 'LineWidth', 2);
        hold off
    end

    drawnow


    % Animate
    for i = 1:nFrame
        curT = tVect(i);
        str = sprintf('t = %.2fs', curT); 
        
        obj.q = ts.Data(:, :, i);
        obj.show('preserve', false, 'fast', true, 'visuals', 'on');        
        
        h_annot.set('String',str); 
        
        if plotTraj
            addpoints(traj_line, traj.Xee.Data(1, :, i), traj.Xee.Data(2, :, i), traj.Xee.Data(3, :, i));
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
    obj.show('preserve', false, 'fast', true, 'visuals', 'on'); 
    drawnow;
    if ~isempty(fileName)
        writeVideo(myVideo, frame);
    end
    waitfor(r);

    if ~isempty(fileName)
        close(myVideo)
    end


end