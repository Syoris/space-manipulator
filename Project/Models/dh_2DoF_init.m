%% Space Robot
% SpaceRobot initialization from DH parameters

load 'SC_2DoF.mat'
sc2 = SpaceRobot;
sc2.Name = 'spaceRobotDH';

L0 = 0.5;
L1 = 1;
L2 = 0.5;
dhparams = [L1 0 0 0; % [a, alpha, d, theta]
        L2 0 0 0;
        0 0 0 0; ];

% Base Parameters
mBase = 10;
comBase = [0 0 0];
intertiaBase = [9.3, 9.3, 9.3, 0, 0, 0];

sc2.Base.Mass = mBase;
sc2.Base.Inertia = intertiaBase;
sc2.Base.HomeConf = [0.5; 0.5; 0; 0; 0; 0]; % [Rx; Ry; Rz; r; p; y]
sc2.Base.ManipToBaseTransform = rt2tr(eye(3), [L0; 0; 0]);

% Body Parameters
jointsAxis = [[0 0 1];
            [0 0 1];
            [0 0 0]];

bodiesCoM = [[L1 / 2 0 0];
        [L2 / 2 0 0];
        [0 0 0]; ];

bodiesMass = [5; 2.5; 0];
bodiesInertia = [0.5, 0.5, 0.5, 0, 0, 0;
            0.1, 0.1, 0.1, 0, 0, 0;
            0, 0, 0, 0, 0, 0];
jointsHomePos = [pi / 4, -pi / 2, 0];

% Create Robot
nBodies = 3;
bodiesVect = cell(1, nBodies);
jointsVect = cell(1, nBodies);

bodyNames = {'Body1', 'Body2', 'endeffector'};
jointNames = {'jnt1', 'jnt2', 'jnt_ee'};

bodies = cell(nBodies, 1);
joints = cell(nBodies, 1);

for i = 1:nBodies
    newBody = Body(bodyNames{i});

    if any(jointsAxis(i, :))
        newJoint = Joint(jointNames{i}, 'revolute');
        newJoint.Axis = jointsAxis(i, :);
        newJoint.HomePosition = jointsHomePos(i);
    else
        newJoint = Joint(jointNames{i}, 'fixed');
    end

    newJoint.setFixedTransform(dhparams(i, :), 'dh');
    newBody.Joint = newJoint;
    newBody.Mass = bodiesMass(i);
    newBody.Inertia = bodiesInertia(i, :);
    newBody.CenterOfMass = bodiesCoM(i, :);

    % Add body
    if i == 1
        parent = sc2.BaseName;
    else
        parent = bodyNames(i - 1);
    end

    bodies{i} = newBody;
    joints{i} = newJoint;
    sc2.addBody(bodies{i}, parent);
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
