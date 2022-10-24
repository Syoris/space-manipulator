%% 2 DoF Manipulator
% Create a 2DoF manipulator using the fixed transform
clc
close all
clearvars
fprintf("------ Initializing 2DoF Space Robot ------\n")
tic

% Materials
materials = struct();
materials.Blue = [0.5 0.7 1.0 1.0];
materials.Orange = [1.0 0.423529411765 0.0392156862745 1.0];
materials.Grey = [0.57 0.57 0.57 1.0];
materials.Red = [1 0 0 1];

%% Parameters
% Base Parameters
lBase = 1;
mBase = 10;
comBase = [0 0 0];
intertiaBase = [9.3, 9.3, 9.3, 0, 0, 0];

% Body Parameters
nBodies = 3;
bodiesVect = cell(1, nBodies);
jointsVect = cell(1, nBodies);

bodyNames = {'Body1', 'Body2', 'endeffector'};
jointNames = {'jnt1', 'jnt2', 'jnt_ee'};

jointsAxis = [[0 0 1];
            [0 0 1];
            [0 0 0]];

jointsHomePos = [pi / 4, -pi / 2, 0];

bodiesLength = [1; 0.5];
bodiesMass = [5; 2.5; 0];
bodiesInertia = [0.5, 0.5, 0.5, 0, 0, 0;
            0.1, 0.1, 0.1, 0, 0, 0;
            0, 0, 0, 0, 0, 0];

jointsT = {trvec2tform([lBase / 2, 0, 0]);
        trvec2tform([bodiesLength(1), 0, 0]);
        trvec2tform([bodiesLength(2), 0, 0])};

bodiesCoM = [[bodiesLength(1) / 2 0 0];
        [bodiesLength(2) / 2 0 0];
        [0 0 0]; ];

% Visuals
baseVisual = struct('Geometry', 'Box', ...
'Parameters', [lBase, lBase, lBase], ...
    'T', trvec2tform([0 0 0]), ...
    'Color', materials.Grey);

bodiesVisual = cell(nBodies, 1);
bodiesVisual{1}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.05, bodiesLength(1)], ...
    'T', trvec2tform([bodiesLength(1) / 2 0 0]) * eul2tform([0 pi / 2 0]), ...
    'Color', materials.Blue);
bodiesVisual{1}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.1, 0.1], ...
    'T', trvec2tform([0.0 0 0]), ...
    'Color', materials.Orange);

bodiesVisual{2}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.05, bodiesLength(2)], ...
    'T', trvec2tform([bodiesLength(2) / 2 0 0]) * eul2tform([0 pi / 2 0]), ...
    'Color', materials.Blue);
bodiesVisual{2}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.1, 0.1], ...
    'T', trvec2tform([0.0 0 0]), ...
    'Color', materials.Orange);

bodiesVisual{3}(end + 1) = struct('Geometry', 'Sphere', ...
    'Parameters', 0.1, ...
    'T', trvec2tform([0.0 0 0]), ...
    'Color', materials.Red);

%% Create Robot
sc = SpaceRobot;
sc.Name = 'spaceRobot';

% Base visual
sc.Base.addVisual(baseVisual.Geometry, baseVisual.Parameters, ...
baseVisual.T, baseVisual.Color);
sc.Base.Mass = mBase;
sc.Base.Inertia = intertiaBase;
sc.Base.HomeConf = [0.5; 0.5; 0; 0; 0; 0]; % [Rx; Ry; Rz; r; p; y]
sc.Base.ManipToBaseTransform = rt2tr(eye(3), [lBase / 2; 0; 0]);

for i = 1:nBodies
    newBody = Body(bodyNames{i});

    if any(jointsAxis(i, :))
        newJoint = Joint(jointNames{i}, 'revolute');
        newJoint.Axis = jointsAxis(i, :);
        newJoint.HomePosition = jointsHomePos(i);
    else
        newJoint = Joint(jointNames{i}, 'fixed');
    end

    newJoint.setFixedTransform(jointsT{i});
    newBody.Joint = newJoint;
    newBody.Mass = bodiesMass(i);
    newBody.Inertia = bodiesInertia(i, :);
    newBody.CenterOfMass = bodiesCoM(i, :);

    if ~isempty(bodiesVisual{i})

        for j = 1:length(bodiesVisual{i})
            vis = bodiesVisual{i}(j);
            newBody.addVisual(vis.Geometry, vis.Parameters, ...
                vis.T, vis.Color);
        end

    end

    bodiesVect{i} = newBody;
    jointsVect{i} = newJoint;
end

for i = 1:length(bodiesVect)

    if i == 1
        parent = sc.BaseName;
    else
        parent = bodyNames(i - 1);
    end

    sc.addBody(bodiesVect{i}, parent);
end

sc.homeConfig();
%% Initialize Matrices
sc.initKin();
sc.initDyn('simplify', true); % Can be very long

sc.showDetails();
% sc.show();

assert(sc.isNSkewSym());
assert(sc.isCOk(true));

fprintf('SpaceRobot Initialization Completed\n')
toc
%% Save Robot
fprintf('Saving robot\n')
clearvars -except sc
save 'models/SC_2DoF.mat'
