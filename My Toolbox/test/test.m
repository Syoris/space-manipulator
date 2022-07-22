clearvars
filename='SC_2DoF.urdf';

% rigidBodyTree = importrobot(filename);
% 
% spaceRobot = SpaceRobot();
% 
% spaceRobot.NumBodies = rigidBodyTree.NumBodies;
% spaceRobot.Bodies = rigidBodyTree.Bodies;
% spaceRobot.Base = rigidBodyTree.Base;
% spaceRobot.BodyNames = rigidBodyTree.BodyNames;
% spaceRobot.BaseName = rigidBodyTree.BaseName;
% spaceRobot.DataFormat = rigidBodyTree.DataFormat;

sc = SpaceRobot('SC_2DoF.urdf');