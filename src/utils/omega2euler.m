function [eulerRate] = omega2euler(euler, w, varargin)
    %OMEGA2EULER Transfert body (??) angular rate to euler rates
    %   euler: rpy
    %   w: [wx; wy; wz]
    %
    %   'refFrame'      - Omega reference frame. 'body' or 'inertial'
    %                       Default: 'body'
    
    parser = inputParser;
    parser.addParameter('refFrame', 'body', ...
        @(x)any(validatestring(x, {'body', 'inertial'})));

    parser.parse(varargin{:});
                
    refFrame = parser.Results.refFrame;

    % In body frame
    switch refFrame
        case 'body'
            sx = sin(euler(1));
            cx = cos(euler(1));
            sy = sin(euler(2));
            cy = cos(euler(2));
        
            RotM_inv = [1,  sy*sx/cy,   sy*cx/cy;
                        0,  cx,         -sx;
                        0,  sx/cy,      cx/cy];
        case 'inertial'
            % In Inertial frame
            sy = sin(euler(2));
            cy = cos(euler(2));
            sz = sin(euler(3));
            cz = cos(euler(3));
        
            RotM_inv = [cz/cy,      sz/cy,      0;
                        -sz,        cz,         0;
                        sy*cz/cy,   sy*sz/cy,   1];
    end
    
    % Conversion
    eulerRate = RotM_inv * w;
end
