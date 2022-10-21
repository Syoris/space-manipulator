% Generate SPART struct for 6DoF Robot
bodyLength = [0.1, 2.61, 2.62, 0.1, 0.1, 0.5, 0];
bodyMass = [0.36, 9.20, 9.20, 0.36, 0.36, 1.78, 0];
Jx = [0.82, 0.03, 0.03, 1.00, 0.82, 0.04, 0];
Jy = [1, 5.24, 5.24, 0.82, 0.82, 0.04, 0];
Jz = [0.82, 5.24, 5.24, 0.82, 1, 0.01, 0];

%          [a                   alpha       d           theta]      i
dhparams = [ ...
        bodyLength(1) pi / 2 0.06 0; %   1
        bodyLength(2) 0 0.10 0; %   2
        bodyLength(3) 0 0.10 0; %   3
        bodyLength(4) -pi / 2 0.10 0; %   4
        bodyLength(5) pi / 2 0.14 pi / 2; %   5
        0 0 bodyLength(6) 0; %   6
        0 0 0 0; ]; %   ee

% --- Definition ---
data.n = 6;

for i = 1:data.n
    data.man(1).type = 1;
    data.man(1).DH.d = dhparams(i, 3);
    data.man(1).DH.alpha = dhparams(i, 2);
    data.man(1).DH.a = dhparams(i, 1);
    data.man(1).DH.theta = dhparams(i, 4);
    data.man(1).b = [0; L1 / 2; 0];
end

data.base.T_L0_J1 = [eye(3), [0; 0; L0]; zeros(1, 3), 1]; % Anchor to base transform

%End-Effector
data.EE.theta = -pi / 2;
data.EE.d = 0;
