%% 2 DoF Planar Manipulator
clc
close all
%% Parameters

lBase = 1;
l1 = 1;
l2 = 0.5;
% Materials

materials = struct();
materials.Blue = [0.5 0.7 1.0 1.0];
materials.Orange = [1.0 0.423529411765 0.0392156862745 1.0];
materials.Grey = [0.57 0.57 0.57 1.0];
materials.Red = [1 0 0 1];

%% Create Robot

sc = SpaceRobot;
sc.Name = 'spaceRobot';

% Base visual
sc.Base.addVisual('Box', [lBase, lBase, lBase], trvec2tform([0 0 0]), materials.Grey);

% Link 1
scLink1 = Link('Link1');

scJnt1 = Joint('jnt1', 'revolute');
scJnt1.HomePosition = pi/4;
scJnt1.Axis = [0 0 1];
tform = trvec2tform([lBase/2, 0, 0]); % User defined, tform: Homogeneous transformation matrix
scJnt1.setFixedTransform(tform);
scLink1.Joint = scJnt1;

visT = eul2tform([0 pi/2 0]);
visT(1:3, 4) = [l1/2 0 0]';
scLink1.addVisual('Cylinder', [0.05, l1], visT, materials.Blue);
scLink1.addVisual('Cylinder', [0.1, 0.1], trvec2tform([0 0 0]), materials.Orange);

% Link 2
scLink2 = Link('Link2');

scJnt2 = Joint('jnt2', 'revolute');
scJnt2.HomePosition = -pi/4;
scJnt2.Axis = [0 0 1];
tform = trvec2tform([l1, 0, 0]); % User defined, tform: Homogeneous transformation matrix
scJnt2.setFixedTransform(tform);
scLink2.Joint = scJnt2;

visT = eul2tform([0 pi/2 0]);
visT(1:3, 4) = [l2/2 0 0]';
scLink2.addVisual('Cylinder', [0.05, l2], visT, materials.Blue);
scLink2.addVisual('Cylinder', [0.1, 0.1], trvec2tform([0 0 0]), materials.Orange);

% EE
scEE = Link('endeffector');
tform = trvec2tform([l2, 0, 0]);
setFixedTransform(scEE.Joint, tform);
scEE.addVisual('Sphere', 0.1, trvec2tform([0 0 0]), materials.Red);

% Add Links
sc.addLink(scLink1,'spacecraftBase'); % Add body1 to base
sc.addLink(scLink2,'Link1'); % Add body2 to body1
sc.addLink(scEE,'Link2');

% Remove vars
clearvars -except sc