%% Vizualisation Example
clearvars
run 'SC_2DoF_init.m'

% Config Test
close all
qm = [pi/4, -pi/2];
sc.JointsConfig = qm;
sc.BaseConfig = [2 3 4; 0 pi/4 0];

comPositions = sc.getCoMPosition();

figure
hold on
sc.show('Visuals', 'off');
linkNames = fields(comPositions);
for i=1:length(linkNames)
    pose = comPositions.(linkNames{i}).Transform;
    plot3(pose(1, 4), pose(2, 4), pose(3, 4),'r*', 'MarkerSize', 15);
end
hold off