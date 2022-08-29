%% Init robots
clc
load robot.mat

L0=0.5;
L1=1;
L2=0.5;

dhparams = [L0      0       0       0;
            L1   	0	    0   	0;      % [a, alpha, d, theta]
            L2	    0       0       0;];

T_b_manip = [eye(3), [L0; 0; 0]; zeros(1, 3), 1];

robot2 = rigidBodyTree;
n = 3;
bodies = cell(n,1);
joints = cell(n,1);
for i = 1:n  
    if i ==1
        joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
        bodies{i} = rigidBody(['body' num2str(i)]);
        setFixedTransform(joints{i}, T_b_manip);

    elseif i==3
        joints{i} = rigidBodyJoint('endeffector_jnt',"fixed");
        bodies{i} = rigidBody('endeffector');      
        setFixedTransform(joints{i},dhparams(i,:),"dh");

    else
        joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
        bodies{i} = rigidBody(['body' num2str(i)]);  
        setFixedTransform(joints{i},dhparams(i,:),"dh");
    end
    
    
    bodies{i}.Joint = joints{i};
    if i == 1 % Add first body to base
        addBody(robot2,bodies{i},"base")
    else % Add current body to previous body by name
        addBody(robot2,bodies{i},bodies{i-1}.Name)
    end
end

robot.DataFormat = 'column';
robot2.DataFormat = 'column';

% showdetails(robot);
% showdetails(robot2);


%% Show
close all
conf = [pi/4; 0];
% conf = [pi/4; -pi/2];


figure
robot.show(conf);
title("Robot 1");

figure
robot2.show(conf);
title("Robot 2");

%% Compare transform
clc
i = 3;

robot.Bodies{i}.Joint.JointToParentTransform
robot2.Bodies{i}.Joint.JointToParentTransform

disp('\n\n')

robot.Bodies{i}.Joint.ChildToJointTransform
robot2.Bodies{i}.Joint.ChildToJointTransform
