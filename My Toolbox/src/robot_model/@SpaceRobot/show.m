function ax = show(obj, varargin)
%SHOW Plot robot body frames
%   SHOW(ROBOT) plots in MATLAB figure the body frames of
%   ROBOT under current config
%
%   AX = SHOW(ROBOT, ___) returns the axes handle under which
%   the robot is plotted.
%
%   SHOW(___, Name, Value) provides additional options specified
%   by one or more Name, Value pair arguments. Name must appear
%   inside single quotes (''). You can specify several name-value
%   pair arguments in any order as Name1, Value1, ..., NameN, ValueN:
%
%      'Parent'         - Handle of the axes in which the body
%                         frames of the robot are to be rendered
%
%      'PreservePlot'   - When hold is on and show method is called
%                         repeatedly, this Boolean parameter
%                         determines whether the graphic
%                         objects resulted from previous calls
%                         of show method are preserved
%                         (true) or cleared (false).
%                         When 'PreservePlot' value is true,
%                         'FastUpdate' value must be false.
%
%                         Default: true
%
%      'Frames'         - A char vector to turn on and off the
%                         display of the body frames. The value
%                         can be either 'on' or 'off'.
%
%                         Default: 'on'
%
%      'Visuals'        - A char vector to turn on and off the
%                         display of the body visual meshes.
%                         The value can be either 'on' or 'off'.
%
%                         Default: 'on'
%
%   Example:
%       % Load predefined robot models
%       load exampleRobots
%
%       % Render Baxter robot frames in 3D plot at home
%       % configuration
%       show(baxter, baxter.randomConfiguration);
%
%       % Open plot browser for a list of body names
%       plotbrowser
%
%       % Plot Puma in the same plot as Baxter
%       hold on
%       show(puma1, puma1.homeConfiguration)
%
%
%       % Plot LBR robot into two subplots with different configuration
%       figure
%       subplot(1,2,1);
%       show(lbr, lbr.randomConfiguration);
%       hold on
%
%       subplot(1,2,2);
%       show(lbr, lbr.randomConfiguration);
%       hold on
%
%       % After executing the following four lines. You should
%       % still see one robot arm in both subplots.
%       subplot(1,2,1)
%       show(lbr, lbr.randomConfiguration, 'PreservePlot', false);
%
%       subplot(1,2,2)
%       show(lbr, lbr.randomConfiguration, 'PreservePlot', false);
%
%
%       % Playback a trajectory for a robot in two subplots
%       % Make a copy of lbr robot
%       lbrCpy = copy(lbr);
%
%       figure
%
%       % Start from home configuration
%       Q = lbr.homeConfiguration;
%       i = 1;
%       for i = 1:50
%           % Increment joint_a2 and joint_a4
%           Q(2).JointPosition = Q(2).JointPosition + 0.02;
%           Q(4).JointPosition = Q(4).JointPosition - 0.02;
%
%           % On the left subplot, preserve all previous
%           % drawings, on the right subplot, only keep the
%           % most recent drawing. Note the 'Parent' parameter
%           % selects in which axis the robot is drawn
%           show(lbr, Q, 'PreservePlot', false, 'Parent', subplot(1,2,1));
%           show(lbrCpy, Q, 'Parent', subplot(1,2,2));
%           hold on
%           drawnow
%       end
%
%       figure;
%       robot = loadrobot("rethinkBaxter");
%       show(robot, 'Collisions', 'on', 'Visuals', 'off');
%
%       %Animate robot configurations using FastUpdate
%       figure
%       robot = loadrobot("kinovaGen3","DataFormat","column");
%       robotConfigs = trapveltraj([randomConfiguration(robot) randomConfiguration(robot)],100);
%       for i = 1:100
%           robot.show(robotConfigs(:,i),'PreservePlot',false,'FastUpdate',true);
%           drawnow;
%       end
%
%   See also showdetails
%parseShowInputs Parse inputs to show method
    parser = inputParser;
    parser.StructExpand = false;
    parser.addParameter('Parent', [], ...
        @(x)robotics.internal.validation.validateAxesHandle(x));
    parser.addParameter('PreservePlot', true, ...
        @(x)validateattributes(x,{'logical', 'numeric'}, {'nonempty','scalar'}));
    parser.addParameter('FastUpdate', false, ...
        @(x)validateattributes(x,{'logical', 'numeric'}, {'nonempty','scalar'}));
    parser.addParameter('Visuals', 'on', ...
        @(x)any(validatestring(x, {'on', 'off'})));
    parser.addParameter('Collisions', 'off', ...
        @(x)any(validatestring(x, {'on', 'off'})));
    parser.addParameter('Frames', 'on', ...
        @(x)any(validatestring(x, {'on', 'off'})));
    parser.addParameter('Position', [0,0,0,0], ...
        @(x)(validateattributes(x, {'numeric'}, ...
        {'nonempty', 'real', 'nonnan', 'finite', 'vector', 'numel', 4})));

    parser.parse(varargin{:});
                
    fast = parser.Results.FastUpdate;
    preserve = logical(parser.Results.PreservePlot);
%     position = parser.Results.Position;
    parent = parser.Results.Parent;
    collisions = parser.Results.Collisions;
    visuals = parser.Results.Visuals;
    frames = parser.Results.Frames;

    if fast && preserve
        error("Fast and preserve cannot be true at the same time");
    elseif ~fast
        % simpleShow
        % [ax, bodyDisplayObjArray] = obj.simpleShow(config, parent, collisions, position, preserve, visuals, frames);
        
        displayVisuals = strcmpi(visuals,'on');
        displayCollisions = strcmpi(collisions,'on');
        displayFrames = strcmpi(frames,'on');    
        
        vizHelper = RobotVizHelper(obj);

        if isempty(parent)
            ax = newplot;
        else
            ax = newplot(parent);
        end
        
        if strcmp(ax.NextPlot, 'replace') || (isa(ax, 'matlab.ui.control.UIAxes') && ~ishold(ax))
            % When hold is off, for axes or uiaxes, respectively
            vizHelper.resetScene(ax);
            % Adjust axis limits according to robot position
            ax.XLim = ax.XLim + obj.Base.R(1);
            ax.YLim = ax.YLim + obj.Base.R(2);
            ax.ZLim = ax.ZLim + obj.Base.R(3);
        else % when hold is on
            if preserve == false
                % If preserve flag is false,
                % remove previous drawn objects, if they could be found
                % delete(findall(ax,'type','hgtransform','Tag', obj.ShowTag));
                % delete(findall(ax,'type','patch','Tag', obj.ShowTag));
                % delete(findall(ax,'type','line','Tag', obj.ShowTag));
            end
        end
        
        % converting the six-element vector of translations and
        % orientations ([x,y,z,yaw,pitch,roll]), respectively, to a
        % homogeneous transformation matrix

        tTree = obj.Ttree;

        % [ax bodyDisplayObjArray] = obj.simpleShow

        [ax, linkDisplayObjArray, fmanager] = vizHelper.drawRobot(ax, tTree, displayFrames, displayVisuals);


    else
        error("Fast not yet implemented")
        % ax = obj.fastShow(config, parent, collisions, position, visuals, frames);
    end
end