%% Init
% filename='SC_2DoF.urdf';
% [robotSpart,robot_keys] = urdf2robot(filename);
% 
clc
load 'SC_2DoF.mat'
% sc.homeConfig;

load robot.mat
robot.DataFormat = 'column';

% --- DH ---
L0=0.5;
L1=1;
L2=0.5;

dhparams = [L0      0       0       0;
            L1      0       0       0;      % [a, alpha, d, theta]
            L2   	0	    0   	0;      
            0	    0       0       0;];

qm = [pi/4; -pi/2];
%% SPART DH
%--- Manipulator Definition ----%
T_L0_J1=[eye(3),[0;0;0];zeros(1,3),1];

%Number of joints/links
data.n=3;

%First joint
data.man(1).type=1;
data.man(1).DH.d = 0;
data.man(1).DH.alpha = 0;
data.man(1).DH.a = L1;
data.man(1).DH.theta=0;
data.man(1).b = [L1/2;0;0];
data.man(1).mass=5;
data.man(1).I=diag([0.5,0.5,0.5]);

%Second joint
data.man(2).type=1;
data.man(2).DH.d = 0;
data.man(2).DH.alpha = 0;
data.man(2).DH.a = L2;
data.man(2).DH.theta=0;
data.man(2).b = [L2/2;0;0];
data.man(2).mass=2.5;
data.man(2).I=diag([1,1,1])/10;

%ee
data.man(3).type=0;
data.man(3).DH.d = 0;
data.man(3).DH.alpha = 0;
data.man(3).DH.a = 0;
data.man(3).DH.theta=0;
data.man(3).b = [0;0;0];
data.man(3).mass=2.5;
data.man(3).I=diag([1,1,1])/10;

%First joint location with respect to base
data.base.T_L0_J1 = T_L0_J1;

%Base-spacecraft mass and inertia
data.base.mass=10;
data.base.I=diag([9.3,9,3,9.3]);

%End-Effector
data.EE.theta=0;
data.EE.d=0;

%--- Create robot structure ---%
[robotDH,T_Ln_EE] = DH_Serial2robot(data);

%% Toolbox
% --- DH ---
robotDh = rigidBodyTree;
robotDh.DataFormat = 'column';
n = 4;
bodies = cell(n,1);
joints = cell(n,1);
jointType = {'fixed', 'revolute', 'revolute', 'fixed'};
linkNames = {'LinkBase', 'Link1', 'Link2', 'endeffector'};
jointNames = {'base', 'jnt1', 'jnt2', 'jnt_ee'};

for i = 1:n
    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)],jointType{i});
    setFixedTransform(joints{i},dhparams(i,:),"dh");
    bodies{i}.Joint = joints{i};
    if i == 1 % Add first body to base
        addBody(robotDh,bodies{i},"base")
    else % Add current body to previous body by name
        addBody(robotDh,bodies{i},bodies{i-1}.Name)
    end
end




%% Space Robot
% load 'SC_2DoF.mat'
sc2 = SpaceRobot;
sc2.Name = 'spaceRobotDH';

jointsHomePos = [pi/4; -pi/2];

L0=0.5;
L1=1;
L2=0.5;
dhparams = [L1      0       0       0;      % [a, alpha, d, theta]
            L2   	0	    0   	0;      
            0	    0       0       0;];

% Base Parameters
sc2.Base.ManipToBaseTransform = rt2tr(eye(3), [L0; 0; 0]);


% Link Parameters
jointsAxis = [[0 0 1];
              [0 0 1];
              [0 0 0]];

linksCoM = [[L1/2 0 0];
            [L2/2 0 0];
            [0 0 0];];

% Create Robot
nLinks = 3;
linksVect = cell(1, nLinks);
jointsVect = cell(1, nLinks);

linkNames = {'Link1', 'Link2', 'endeffector'};
jointNames = {'jnt1', 'jnt2', 'jnt_ee'};

links = cell(nLinks,1);
joints = cell(nLinks,1);

for i=1:nLinks
    links{i} = Link(linkNames{i});
    links{i}.CenterOfMass = linksCoM(i, :);
   
    if any(jointsAxis(i, :))
        newJoint = Joint(jointNames{i}, 'revolute');
        newJoint.Axis = jointsAxis(i, :);
        newJoint.HomePosition = jointsHomePos(i);
    else
        newJoint = Joint(jointNames{i}, 'fixed');
    end
    
    joints{i} = newJoint;
    joints{i}.setFixedTransform(dhparams(i,:),'dh');
    links{i}.Joint = joints{i};

    % Add link
    if i==1
        parent = sc2.BaseName;
    else
        parent = linkNames(i-1);
    end

    sc2.addLink(links{i}, parent);
end

% --- Initialize Matrices ---
sc2.initKin();

% sc2.initDyn('simpH', true, 'simpC', true);

% clearvars -except sc sc2

