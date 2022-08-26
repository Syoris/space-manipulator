%% Visualization Example
% To visualize SpaceRobot transforms and CoM positions.

%% Init
% To load vars and setup robot
clearvars
load 'SC_2DoF.mat'

% Config Test
close all
qm = [pi/4; -pi/2];
sc.qm = qm;
sc.q0 = [2; 3; 4; 0; pi/4; 0];

%% CoM
% Plot all CoM Positions
comPositions = sc.getCoMPosition();

figure
hold on
title('SpaceRobot CoM Positions')
sc.show('Visuals', 'off');
linkNames = fields(comPositions);
for i=1:length(linkNames)
    pose = comPositions.(linkNames{i});
    plot3(pose(1, 4), pose(2, 4), pose(3, 4),'r*', 'MarkerSize', 15);
end
xlim([1.5, 4])
axis equal
hold off

%% Transforms
% Plot the transform between different links parts
q2 = [1; 0.5; 0; 0; 0; 0; -pi/4; 0];

tTree = sc.forwardKinematics();

T_b_1 = sc.getTransform('Link1'); % Frame 1 wrt base, symbolic

T_1_2 = sc.getTransform('Link2', 'Link1', 'SymbRes', true); % Frame 2 wrt frame, symbolic

T_b_2 = sc.getTransform('Link2', 'spacecraftBase', 'SymbRes', false); % Frame 2 wrt base, numeric, current config

T_2_b = sc.getTransform('spacecraftBase', 'Link2', 'SymbRes', false); % Base wrt frame 2, numeric, current config

T_1_ee = sc.getTransform('endeffector', 'Link1', 'Config', q2, 'SymbRes', false); % ee wrt frame 1, numeric, at q2

T_i_2 = sc.getTransform('Link2', 'inertial', 'SymbRes', false); % Frame 2 wrt inertial frame, numeric, current conf, could also get with T_i_b * T_b_2

% Plot
% Transforms are plot using Spatial Math Toolbox by Peter Corke

figure
title('Transform Visualization')
hold on
sc.show('Visuals', 'off');

SE3(T_b_2).plot;
SE3(T_2_b).plot;
% SE3(T_1_ee).plot;
SE3(T_i_2).plot;

axis equal
hold off