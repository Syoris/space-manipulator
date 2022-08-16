clc
load 'SC_2DoF.mat'

sc.homeConfig();

robot = loadrobot("kinovaGen3","DataFormat","column");
robotConfigs = trapveltraj([randomConfiguration(robot) randomConfiguration(robot)],100);
robot.show(robotConfigs(:,1),'PreservePlot',false,'FastUpdate',true);