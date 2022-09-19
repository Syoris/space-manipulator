function tt = circleTraj(p0, r, tF, N, varargin)
%squareTraj : Generate a square traj
%   p0      - Initial position [x; y; z]
%   r       - Circle radius
%   tF      - Final time
%   N       - Number of sample, where N-5 must be multiple of 4
%   'plane' - Trajectory plane. 'xy', 'xz', 'yz'
%   'tStart' - To delay start of trajectory

    parser = inputParser;
    parser.addParameter('plane', 'xy', ...
        @(x)any(validatestring(x, {'xy', 'xz', 'yz'})));
    parser.addParameter('tStart', 0);

    parser.parse(varargin{:});
                
    plane = parser.Results.plane;
    tStart = parser.Results.tStart;
   
     
    % points
    points = repmat(p0, 1, N);
    points(:, 1) = p0;
    points(:, end) = p0;

    % Find center
    center = p0;

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


    % Compute Points    
    dTheta = 2*pi/(N-1);
    for k=1:N
        theta = (k-1)*dTheta + thOffset;

        points(i, k) = center(i) + cos(theta)*r;
        points(j, k) = center(j) + sin(theta)*r;
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