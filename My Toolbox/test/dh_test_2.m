% [a, alpha, d, theta]
clc
dhparams = [0.5     0       0       0;          % jnt0
            1   	0	    0   	pi/4;       % jnt1
            0.5	    0       0       -pi/2;      % jnt2
            0       0       0       0];         % ee

n = size(dhparams, 1);
T_mats = cell(n, 1);
for i=1:n
    dh = dhparams(i, :);
    a = dh(1);
    alpha = dh(2);
    d = dh(3);
    theta = dh(4);
      
    Ta = [eye(3), [a, 0, 0]'; [0, 0, 0, 1]];
    Talpha = [1, 0, 0, 0; 0, cos(alpha), -sin(alpha), 0;...
                0, sin(alpha), cos(alpha), 0; 0, 0, 0, 1];
    Td = [eye(3), [0, 0, d]'; [0, 0, 0, 1]];
    Ttheta = [cos(theta), -sin(theta), 0, 0; ...
           sin(theta), cos(theta), 0, 0; 0, 0, 1, 0;0, 0, 0, 1];
     
    TL = Ttheta*Td*Ta*Talpha;

    T_mats{i} = TL;
end

% Check Forward Kin
sc.homeConfig;
sc.q0 = zeros(6, 1);
[RJ,RL,rJ,rL,e,g]=Kinematics(eye(3),zeros(3, 1),[pi/4; -pi/2], robotSpart);
[RJdh,RLdh,rJdh,rLdh,edh,gdh]=Kinematics(eye(3),zeros(3, 1),[pi/4; -pi/2], robotDH);

T_tot = [eye(3), zeros(3, 1); [0 0 0 1]];
for i=1:3
    
    fprintf("--- i=%i ---\n", i)
%     fprintf('\tSC\n')
%     T_sc = sc.getTransform(sc.LinkNames{i}, sc.BaseName, 'symbRes', false)
    

    fprintf('\tDH\n')
    T_tot = T_tot*T_mats{i}

    fprintf('\tSPART\n')
    [RJ(:, :, i), rJ(:, i); [0 0 0 1]]


    fprintf('\tSPART DH\n')
    [RJdh(:, :, i), rJdh(:, i); [0 0 0 1]]


end



