function [ax, bodyDisplayObjArray] = showSimple(obj, parent, collisions, preserve, visuals, frames)
    % simpleShow
    displayVisuals = strcmpi(visuals, 'on');
    displayCollisions = strcmpi(collisions, 'on');
    displayFrames = strcmpi(frames, 'on');

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
            delete(findall(ax, 'type', 'hgtransform', 'Tag', obj.ShowTag));
            delete(findall(ax, 'type', 'patch', 'Tag', obj.ShowTag));
            delete(findall(ax, 'type', 'line', 'Tag', obj.ShowTag));
        end

    end

    tTree = obj.Ttree;

    [ax, bodyDisplayObjArray, fmanager] = vizHelper.drawRobot(ax, tTree, displayFrames, displayVisuals);
end
