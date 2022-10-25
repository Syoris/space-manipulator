function tt = circleTraj(p0, r, tF, N, varargin)
%squareTraj : Generate a square traj
%   p0      - Initial position [x; y; z; psi_x; psi_y; psi_z]
%   r       - Circle radius
%   tF      - Final time
%   N       - Number of sample, where N-5 must be multiple of 4
%   'plane' - Trajectory plane. 'xy', 'xz', 'yz'
%   'tStart' - To delay start of trajectory
%   'rotAxis' - Axis to rotate ee

    parser = inputParser;
    parser.addParameter('plane', 'xy', ...
        @(x)any(validatestring(x, {'xy', 'xz', 'yz'})));
    parser.addParameter('tStart', 0);
        parser.addParameter('rotAxis', 'x', ...
        @(x)any(validatestring(x, {'x', 'y', 'z'})));

    parser.parse(varargin{:});
                
    plane = parser.Results.plane;
    tStart = parser.Results.tStart;
    rotAxis = parser.Results.rotAxis;

    % points
    points = repmat(p0, 1, N);

    % Find center
    center = p0(1:3);

    switch plane
        case 'xy'
            center(1) = center(1) + r;
            thOffset = -pi;

            i = 1;
            j = 2;
                
        case 'xz'
            center(1) = center(1) + r;
            thOffset = -pi;

            i = 1;
            j = 3;
    
        case 'yz'
            center(3) = center(3) + r;
            thOffset = -pi/2;

            i = 2;
            j = 3;
    end
    
    switch rotAxis
        case 'x'
            rIdx = 4;
                
        case 'y'
            rIdx = 5;
    
        case 'z'
            rIdx = 6;
    end

    % Compute Points    
    dTheta = 2*pi/(N-1);
    for k=1:N
        theta = (k-1)*dTheta;

        points(i, k) = center(i) + cos(theta + thOffset)*r;
        points(j, k) = center(j) + sin(theta + thOffset)*r;

        points(rIdx, k) = theta + p0(rIdx);
    end  

    tt = timetable(points','TimeStep',seconds(tF/(N-1)), 'VariableNames',{'EE_desired'});
    
    % Add start waiting time
    dT = tF/(N-1); % Time step per point

    if tStart>0
        tt.Time = tt.Time + seconds(tStart);
    
        tt2 = timetable(p0','RowTimes', seconds(0), 'VariableNames',{'EE_desired'});
        
        tt = [tt2; tt];
    end 
end