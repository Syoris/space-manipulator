%% Space Robot
% SpaceRobot initialization from DH parameters

load 'SC_2DoF.mat'
sc2 = SpaceRobot;
sc2.Name = 'spaceRobotDH';

L0=0.5;
L1=1;
L2=0.5;
dhparams = [L1      0       0       0;      % [a, alpha, d, theta]
            L2   	0	    0   	0;      
            0	    0       0       0;];

% Base Parameters
mBase = 10;
comBase = [0 0 0];
intertiaBase = [9.3, 9.3, 9.3, 0, 0, 0];

sc2.Base.Mass = mBase;
sc2.Base.Inertia = intertiaBase;
sc2.Base.HomeConf = [0.5; 0.5; 0; 0; 0; 0]; % [Rx; Ry; Rz; r; p; y]
sc2.Base.ManipToBaseTransform = rt2tr(eye(3), [L0; 0; 0]);

% Link Parameters
jointsAxis = [[0 0 1];
              [0 0 1];
              [0 0 0]];

linksCoM = [[L1/2 0 0];
            [L2/2 0 0];
            [0 0 0];];

linksMass = [5; 2.5; 0];
linksInertia = [0.5, 0.5, 0.5, 0, 0, 0; 
                0.1, 0.1, 0.1, 0, 0, 0;
                0, 0, 0, 0, 0, 0];
jointsHomePos = [pi/4, -pi/2, 0];

% Create Robot
nLinks = 3;
linksVect = cell(1, nLinks);
jointsVect = cell(1, nLinks);

linkNames = {'Link1', 'Link2', 'endeffector'};
jointNames = {'jnt1', 'jnt2', 'jnt_ee'};

links = cell(nLinks,1);
joints = cell(nLinks,1);

for i=1:nLinks
    newLink = Link(linkNames{i});
     
    if any(jointsAxis(i, :))
        newJoint = Joint(jointNames{i}, 'revolute');
        newJoint.Axis = jointsAxis(i, :);
        newJoint.HomePosition = jointsHomePos(i);
    else
        newJoint = Joint(jointNames{i}, 'fixed');
    end

    newJoint.setFixedTransform(dhparams(i,:),'dh');
    newLink.Joint = newJoint;
    newLink.Mass = linksMass(i);
    newLink.Inertia = linksInertia(i, :);
    newLink.CenterOfMass = linksCoM(i, :);

    % Add link
    if i==1
        parent = sc2.BaseName;
    else
        parent = linkNames(i-1);
    end
    
    links{i} = newLink;
    joints{i} = newJoint;
    sc2.addLink(links{i}, parent);
end

%% --- Initialize Matrices ---
profile on
tic
sc2.initKin();
sc2.initDyn();
toc
profile viewer
profile off

sc2.homeConfig;


%% Save Robot
fprintf('Saving robot\n')
clearvars -except sc2
save 'Project/Models/SC2_2DoF.mat'

