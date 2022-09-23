function [ dx ] = srode(t, x, u, p, w )      

%% Load params
%     load('SR2.mat', 'sr');
    bodyNames = {'SpacecraftBase', 'Body1', 'Body2', 'endeffector'};

%% 
    dx = zeros(16, 1);
    q = x(1:8);
    qb = q(1:6);
    qm = q(7:8);
    
    q_dot = x(9:16); 
    qb_dot = q_dot(1:6);
    qm_dot = q_dot(7:8);

    % Ttree
    tTreeArray = myfile(q);
    tTree = struct;
    for i = 1:length(bodyNames)
        tTree.(bodyNames{i}) = tTreeArray(:, 1 + (i - 1) * 4:i * 4);
    end
    
    
    % Speed
    dx(1:6) = qb_dot; % [vb; wb]
    dx(7:8) = qm_dot; % [qm_dot]
    
    % Accel
    q_ddot = zeros(8, 1);
%     q_ddot = sr.forwardDynamics(u, q, q_dot);
    dx(9:16) = q_ddot;


%%
%     dx(1) = x(9);
%     dx(2) = x(10);
%     dx(3) = x(11);
%     dx(4) = x(12);
%     dx(5) = x(13);
%     dx(6) = x(14);
%     dx(7) = x(15);
%     dx(8) = x(16);
%     
%     dx(9) = 0;
%     dx(10) = 0;
%     dx(11) = 0;
%     dx(12) = 0;
%     dx(13) = 0;
%     dx(14) = 0;
%     dx(15) = 0;
%     dx(16) = 0;


end
