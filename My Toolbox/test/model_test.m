clc
clearvars
% Create Space Robot
sc = SpaceRobot;
sc.Name = 'spaceRobot';

[robotSpart,robot_keys] = urdf2robot('ModelTest.urdf');
%% Build Robot
% Robot Parameters
lBase = 1;
l1 = 1;
l2 = 0.5;

% Colors
materials = struct();
materials.Blue = [0.5 0.7 1.0 1.0];
materials.Orange = [1.0 0.423529411765 0.0392156862745 1.0];
materials.Grey = [0.57 0.57 0.57 1.0];
materials.Red = [1 0 0 1];

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

%% Config Test
close all
qm = [pi/4, -pi/2];

% SpaceRobot
homeConf = sc.homeConfiguration; % Get home config
newConf = homeConf;
newConf(1).JointPosition = qm(1);
newConf(2).JointPosition = qm(2);

sc.JointsConfig = newConf;  % Alternative: sc.setJointsConfig = qm;

baseConf = sc.BaseConfig;
baseConf.Position = [1.25, 1.5, 1.2];
baseConf.Rot = [0, 0, 0];
sc.BaseConfig = baseConf;  % sc.BaseConfig = [2 3 4; 0 pi/4 0];

tTree = sc.forwardKinematics; % Compute kin tree


% Animation test
nSamp = 100;
x0 = baseConf.Position(1);
y0 = baseConf.Position(2);
z0 = baseConf.Position(3);
theta0 = baseConf.Rot(2);

xFin = x0 + 10;
thetaFin = 2*pi;
xArray = linspace(x0, xFin, nSamp);
thetaArray = linspace(theta0, thetaFin, nSamp);

baseConfArray = cell(nSamp, 1);
for i = 1:nSamp
    baseConfArray{i} = [xArray(i), y0, z0; 0 thetaArray(i) 0];
end


framesPerSecond = 50;
r = rateControl(framesPerSecond);
for i = 1:nSamp
    sc.BaseConfig = baseConfArray{i};
    sc.show('PreservePlot',false);
    drawnow
    waitfor(r);
end

% % Spart
% R0 = eye(3);
% r0 = [0; 0; 0];
% [RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robotSpart);
% 
% figure
% hold on
% title("SPART")
% plotTransforms(rJ(:, 1)', rotm2quat(RJ(:, :, 1)), 'FrameSize', 0.1)
% plotTransforms(rJ(:, 2)', rotm2quat(RJ(:, :, 2)), 'FrameSize', 0.1)
% plotTransforms(rJ(:, 3)', rotm2quat(RJ(:, :, 3)), 'FrameSize', 0.1)
% hold off
