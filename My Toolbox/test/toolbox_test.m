clc
clearvars

% Create Space Robot
robot = rigidBodyTree;

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

% Link 1
tform = trvec2tform([lBase/2, 0, 0]); % User defined, tform: Homogeneous transformation matrix
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
jnt1.HomePosition = pi/4;
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;

% Link 2
tform = trvec2tform([l1, 0, 0]); % User defined, tform: Homogeneous transformation matrix
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.HomePosition = -pi/4;
setFixedTransform(jnt2, tform);
body2.Joint = jnt2;

% EE
tform = trvec2tform([l2, 0, 0]);
bodyEndEffector = rigidBody('endeffector');
setFixedTransform(bodyEndEffector.Joint,tform);


% Add Links
robot.addBody(body1,'base'); % Add body1 to base
robot.addBody(body2,'body1'); % Add body2 to body1
addBody(robot,bodyEndEffector,'body2');




%% Plot robot into two subplots with different configuration
figure
subplot(1,2,1);
show(robot, robot.randomConfiguration);
hold on

subplot(1,2,2);
show(robot, robot.randomConfiguration);
hold on

% After executing the following four lines. You should
% still see one robot arm in both subplots.
subplot(1,2,1)
show(robot, robot.randomConfiguration, 'PreservePlot', false);

subplot(1,2,2)
show(robot, robot.randomConfiguration, 'PreservePlot', false);


% Playback a trajectory for a robot in two subplots
% Make a copy of robot robot
lbrCpy = copy(robot);

figure

% Start from home configuration
Q = robot.homeConfiguration;
i = 1;
for i = 1:50
  % Increment joint_a2 and joint_a4
  Q(2).JointPosition = Q(2).JointPosition + 0.02;
  Q(1).JointPosition = Q(1).JointPosition - 0.02;

  % On the left subplot, preserve all previous
  % drawings, on the right subplot, only keep the
  % most recent drawing. Note the 'Parent' parameter
  % selects in which axis the robot is drawn
  show(robot, Q, 'PreservePlot', false, 'Parent', subplot(1,2,1));
  show(lbrCpy, Q, 'Parent', subplot(1,2,2));
  hold on
  drawnow
end
%%
figure;
robot = loadrobot("rethinkBaxter");
show(robot, 'Collisions', 'on', 'Visuals', 'off');

%Animate robot configurations using FastUpdate
figure
robot = loadrobot("kinovaGen3","DataFormat","column");
robotConfigs = trapveltraj([randomConfiguration(robot) randomConfiguration(robot)],100);
for i = 1:100
  robot.show(robotConfigs(:,i),'PreservePlot',false,'FastUpdate',true);
  drawnow;
end