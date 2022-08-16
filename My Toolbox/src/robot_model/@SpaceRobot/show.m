function ax = show(obj, varargin)
%SHOW Plot robot body frames
%   SHOW(ROBOT) plots in MATLAB figure the body frames of
%   ROBOT under current config
%
%   AX = SHOW(ROBOT, ___) returns the axes handle under which
%   the robot is plotted.
%
%   SHOW(___, Name, Value) 
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
    % parse inputs
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
    parent = parser.Results.Parent;
    collisions = parser.Results.Collisions;
    visuals = parser.Results.Visuals;
    frames = parser.Results.Frames;

    if fast && preserve
        error("Fast and preserve cannot be true at the same time");
    elseif ~fast
        [ax, ~] = obj.showSimple(parent, collisions, preserve, visuals, frames);        

    else
        % error("Fast not yet implemented")
        ax = obj.showFast(parent, collisions, visuals, frames);
    end
end