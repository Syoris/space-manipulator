function T = getTransform(obj, linkName1, varargin)
%getTransform Get the transform between two link frames
%   T1 = getTransform(ROBOT, linkName1) computes a
%   transform T1 that converts points originally expressed in
%   linkName1 frame to be expressed in the robot's base frame.
%
%   T2 = getTransform(ROBOT, linkName1, 'TargetFrame', linkName2) computes
%   a transform T2 that converts points originally expressed in
%   linkName1 frame to be expressed in linkName2.
%   
%   T3 = getTransform(ROBOT, linkName1, 'TargetFrame', 'inertial') computes
%   a transform T3 that converts points originally expressed in
%   linkName1 frame to be expressed in inertial frame.
%
%   If no configuration q is specified, the output will be in symbolic form. 
%
%      'TargetFrame'    - To specify transform target frame. 
%
%                         Default: 'base'
%
%      'symbolic'        - Bool to output transform in symbolic or numeric form. 
%                         The value can be either true or false.
%
%                         Default: true
%
%      'Config'         - To specify robot configuration. Real vector of size (n+6, 1). 
%                         ** The joint limits are not checked **
%
%                         Default: []

    parser = inputParser;
    
    parser.addParameter('TargetFrame', 'base');

    parser.addParameter('symbolic', true, ...
        @(x)validateattributes(x,{'logical', 'numeric'}, {'nonempty','scalar'}));

    parser.addParameter('Config', [], ...
        @(x)(validateattributes(x, {'numeric'}, ...
        {'nonempty', 'real', 'nonnan', 'finite', 'vector', 'numel', obj.NumActiveJoints + 6})));


    parser.parse(varargin{:});
    
    targetFrame = parser.Results.TargetFrame;
    symbolic = parser.Results.symbolic;
    Config = parser.Results.Config;

    if ~isempty(Config) && symbolic
        warning('Config specified but symbolic result asked. The specified config will be ignored')    
    end
    
    
    if symbolic
        % Symbolic results
        tTree = obj.Ttree_symb;
    elseif ~isempty(Config)
        tTree = obj.getTtreeNum(Config);
    else
        tTree = obj.Ttree;
    end
    
    % 2-argument case: getTransform(ROBOT, linkName1)
    T1 = tTree.(linkName1);
    
    % Target Frame
    switch targetFrame            
        case 'inertial'
            T2 = eye(4);
        case 'base'
            T2 = tTree.(obj.BaseName);
        otherwise
            T2 = tTree.(targetFrame);
    end
    
    % Compute transform:
    %   T_1_2: frame 2 represented in frame 1
    %   T_b_1, T_b_2: Frames i repsrented in base frame
    %   T_1_2 = T_b_1^-1 * T_b_2
    R = T2(1:3,1:3).';
    p = -R*T2(1:3,4);
    T = [R,p;[0 0 0 1]]*T1; % the first term is inv(T2)

end