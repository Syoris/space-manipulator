% Load Robots
load SC_2DoF.mat
load sc2.mat
load robot.mat
load robot2.mat

robot.DataFormat = 'column';
robot2.DataFormat = 'column';
%%
close all
sc.q = [0.5; zeros(7, 1)];
sc2.q = [0.5; zeros(7, 1)];

conf = [pi/4; 0];

figure
robot.show(conf);
title("Robot 1");

figure
robot2.show(conf);
title("Robot 2");

figure
sc.show('visuals', 'off');
title('SC');

figure
sc2.show('visuals', 'off');
title('SC2');

%%
sc.Links{1}.Joint.transformLink2Parent
sc2.Links{1}.Joint.transformLink2Parent

robot.getTransform(conf, 'body1')
robot.getTransform(conf, 'body2')
% robot.getTransform(conf, 'body2', 'body1')
