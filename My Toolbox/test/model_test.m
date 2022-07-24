clc
clearvars
% Create Space Robot
robot = rigidBodyTree;
sc = SpaceRobot;
sc.Name = 'spaceRobot';

[robotSpart,robot_keys] = urdf2robot('ModelTest.urdf');
%% Build Robot
% Link 1
scLink1 = Link('Link1');

scJnt1 = Joint('jnt1', 'revolute');
scJnt1.HomePosition = pi/4;
scJnt1.Axis = [0 0 1];
tform = trvec2tform([0.5, 0, 0]); % User defined, tform: Homogeneous transformation matrix
scJnt1.setFixedTransform(tform);
scLink1.Joint = scJnt1;

% toolbox
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
jnt1.HomePosition = pi/4;
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;

% Link 2
scLink2 = Link('Link2');

scJnt2 = Joint('jnt2', 'revolute');
scJnt2.HomePosition = -pi/4;
scJnt2.Axis = [0 0 1];
tform = trvec2tform([1, 0, 0]); % User defined, tform: Homogeneous transformation matrix
scJnt2.setFixedTransform(tform);
scLink2.Joint = scJnt2;

% toolbox
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.HomePosition = -pi/4;
setFixedTransform(jnt2, tform);
body2.Joint = jnt2;

% EE
scEE = Link('endeffector');
tform = trvec2tform([0.5, 0, 0]);
setFixedTransform(scEE.Joint, tform);

% toolbox
bodyEndEffector = rigidBody('endeffector');
setFixedTransform(bodyEndEffector.Joint,tform);
% Add Links
sc.addLink(scLink1,'spacecraftBase'); % Add body1 to base
sc.addLink(scLink2,'Link1'); % Add body2 to body1
sc.addLink(scEE,'Link2');

% toolbox
robot.addBody(body1,'base'); % Add body1 to base
robot.addBody(body2,'body1'); % Add body2 to body1
addBody(robot,bodyEndEffector,'body2');

%% Config Test
close all
qm = [pi/4, -pi/2];

% SpaceRobot
homeConf = sc.JointsConfig;
newConf = homeConf;
newConf(1).JointPosition = qm(1);
newConf(2).JointPosition = qm(2);

sc.setJointsConfig(newConf);  % Alternative: sc.setJointsConfig(qm)
tTree = sc.forwardKinematics;
figure
hold on
title("SpaceRobot")
plotTransforms(tform2trvec(tTree{1}), tform2quat(tTree{1}), 'FrameSize', 0.1)
plotTransforms(tform2trvec(tTree{2}), tform2quat(tTree{2}), 'FrameSize', 0.1)
plotTransforms(tform2trvec(tTree{3}), tform2quat(tTree{3}), 'FrameSize', 0.1)
hold off


% Toolbox
tform = getTransform(robot,newConf,'body2','base');
figure
title("Toolbox")
hold on
show(robot, newConf);
plotTransforms(tform2trvec(tform), tform2quat(tform))
hold off

% Spart
R0 = eye(3);
r0 = [0; 0; 0];
[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robotSpart);

figure
hold on
title("SPART")
plotTransforms(rJ(:, 1)', rotm2quat(RJ(:, :, 1)), 'FrameSize', 0.1)
plotTransforms(rJ(:, 2)', rotm2quat(RJ(:, :, 2)), 'FrameSize', 0.1)
plotTransforms(rJ(:, 3)', rotm2quat(RJ(:, :, 3)), 'FrameSize', 0.1)
hold off
