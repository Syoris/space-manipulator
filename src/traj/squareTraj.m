function tt = squareTraj(p0, l, tF, N, varargin)
%squareTraj : Generate a square traj
%   p0      - Initial position [x; y; z]
%   l       - Square Length
%   tF      - Final time
%   N       - Number of sample, where N-5 must be multiple of 4
%   'plane' - Trajectory plane. 'xy', 'xz', 'yz'

    parser = inputParser;
    parser.addParameter('plane', 'xy', ...
        @(x)any(validatestring(x, {'xy', 'xz', 'yz'})));

    parser.parse(varargin{:});
                
    plane = parser.Results.plane;
   
     
    % intPoints
    points = repmat(p0, 1, 5);
    points(:, 1) = p0;
    points(:, end) = p0;
    
    dx = [0     l       l;
          -l    -l      0];
    
    
    switch plane
        case 'xy'
            i = 1;
            j = 2;
    
        case 'xz'
            i = 1;
            j = 3;
    
        case 'yz'
            i = 2;
            j = 3;
        otherwise
            error('Invalid plane')
    end
    for k=1:3
        points(i, k+1) = p0(i) + dx(1, k);
        points(j, k+1) = p0(j) + dx(2, k);
    end
    
    pPerSide = (N - 5)/4;
    assert(mod((N - 5), 4) == 0, 'N-5 must be multiple of 4');
    
    % Interpolate
    intPoints = zeros(3, N);
    
    for i=1:4
        pInit = points(:, i);
        pF = points(:, i+1);
    
    
        int = (0:pPerSide+1) .* (pF-pInit)/(pPerSide+1) + pInit;
        idx = 1 + (pPerSide+1)*(i-1);
        intPoints(:, idx:idx+pPerSide+1) = int;
    end
    
    
    tt = timetable(intPoints','TimeStep',seconds(tF/(N-1)), 'VariableNames',{'EE_desired'});

end