%% Space Robot
% SpaceRobot initialization from DH parameters

sr3 = SpaceRobot;
sr3.Name = 'SR3';
modelPath = fullfile('Project/Models/SR3');

L0 = 0.5;
L1 = 1;
L2 = 0.5;
L3 = 0.5;
dhparams = [ ...
        L1 0 0 0; % [a, alpha, d, theta]
        L2 0 0 0;
        L3 0 0 0;
        0 0 0 0; ];

% Base Parameters
mBase = 10;
comBase = [0 0 0];
intertiaBase = [9.3, 9.3, 9.3, 0, 0, 0];

sr3.Base.Mass = mBase;
sr3.Base.Inertia = intertiaBase;
sr3.Base.HomeConf = [0.5; 0.5; 0; 0; 0; 0]; % [Rx; Ry; Rz; r; p; y]
sr3.Base.ManipToBaseTransform = rt2tr(eye(3), [L0; 0; 0]);

% Body Parameters
jointsAxis = [[0 0 1];
            [0 0 1];
            [0 0 1];
            [0 0 0]];

bodiesCoM = [[L1 / 2 0 0];
        [L2 / 2 0 0];
        [L3 / 2 0 0];
        [0 0 0]; ];

bodiesMass = [5; 2.5; 2.5; 1];
bodiesInertia = [...
            0.5, 0.5, 0.5, 0, 0, 0;
            0.1, 0.1, 0.1, 0, 0, 0;
            0.1, 0.1, 0.1, 0, 0, 0;
            0.01, 0.01, 0.01, 0, 0, 0];
jointsHomePos = [pi / 4, -pi / 2, 0, 0];

jointsPositionLimits = [[-70*pi/180, 70*pi/180]; % elbow
                        [-150 *pi/180, 150*pi/180]; % wrist
                        [-inf, inf];
                        [-inf, inf]; % ee2
                        ];

% Create Robot
nBodies = 4;
bodiesVect = cell(1, nBodies);
jointsVect = cell(1, nBodies);

bodyNames = {'Body1', 'Body2', 'Body3', 'endeffector'};
jointNames = {'jnt1', 'jnt2', 'jnt3', 'jnt_ee'};

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
    newJoint.PositionLimits = jointsPositionLimits(i, :);
    newBody.Joint = newJoint;
    newBody.Mass = bodiesMass(i);
    newBody.Inertia = bodiesInertia(i, :);
    newBody.CenterOfMass = bodiesCoM(i, :);

    % Add body
    if i == 1
        parent = sr3.BaseName;
    else
        parent = bodyNames(i - 1);
    end

    bodies{i} = newBody;
    joints{i} = newJoint;
    sr3.addBody(bodies{i}, parent);
end

%% Visuals
% Materials
materials = struct();
materials.Blue = [0.5 0.7 1.0 1.0];
materials.Orange = [1.0 0.423529411765 0.0392156862745 1.0];
materials.Grey = [0.57 0.57 0.57 1.0];
materials.Red = [1 0 0 1];

baseVisual = struct('Geometry', 'Box', ...
'Parameters', [L0*2, L0*2, L0*2], ...
    'T', trvec2tform([0 0 0]), ...
    'Color', materials.Grey);

bodiesVisual = cell(nBodies, 1);
bodiesVisual{1}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.05, L1], ...
    'T', trvec2tform([L1/2 0 0]) * eul2tform([0 pi/2 0]), ...
    'Color', materials.Blue);
bodiesVisual{1}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.1, 0.1], ...
    'T', trvec2tform([0.0 0 0]), ...
    'Color', materials.Orange);

bodiesVisual{2}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.05, L2], ...
    'T', trvec2tform([L2/2 0 0]) * eul2tform([0 pi/2 0]), ...
    'Color', materials.Blue);
bodiesVisual{2}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.1, 0.1], ...
    'T', trvec2tform([0.0 0 0]), ...
    'Color', materials.Orange);

bodiesVisual{3}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.05, L3], ...
    'T', trvec2tform([L3/2 0 0]) * eul2tform([0 pi/2 0]), ...
    'Color', materials.Blue);
bodiesVisual{3}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.1, 0.1], ...
    'T', trvec2tform([0.0 0 0]), ...
    'Color', materials.Orange);

bodiesVisual{4}(end + 1) = struct('Geometry', 'Sphere', ...
    'Parameters', 0.1, ...
    'T', trvec2tform([0.0 0 0]), ...
    'Color', materials.Red);

sr3.Base.addVisual(baseVisual.Geometry, baseVisual.Parameters, ...
baseVisual.T, baseVisual.Color);

% Add visuals
for i = 1:nBodies    
    if ~isempty(bodiesVisual{i})

        for j = 1:length(bodiesVisual{i})
            vis = bodiesVisual{i}(j);
            sr3.Bodies{i}.addVisual(vis.Geometry, vis.Parameters, ...
                vis.T, vis.Color);
        end

    end
end

%% --- Initialize Matrices ---
% profile on
% tic
sr3.initKin();
% sr3.initDyn();
% toc
% profile viewer
% profile off

% Init mats
sr3.Base.initBase();

for i = 1:length(sr3.Bodies)
    sr3.Bodies{i}.initBody();
end

sr3_info = srInfoInit(sr3);
Struct2File(sr3_info, modelPath);

sr3.homeConfig;

%% --- Function ---
% Create rotation matrix function handle
fprintf("Generating rotation matrix function...\n")
% RFunc_gen(sr3, modelPath);


%% Generate mex for SR3
% x = zeros(16, 1);
% u = zeros(8, 1);
% 
% codegen -report SR3_state_func.m -args {x, u} -o Project\Models\SR3\SR3_state_func_mex.mexw64
% 
% % Generate mex for SR2 ee
% x = zeros(28, 1);
% u = zeros(8, 1);
% 
% codegen -report SR3_ee_state_func.m -args {x, u} -o Project\Models\SR3\SR3_ee_state_func_mex.mexw64
% 
% %%
% sr3.InfoFunc = @SR2_info;
% sr3.StateFunc = @SR2_state_func;
% sr3.StateFuncMex = @SR2_state_func_mex;

%% Save Robot
fprintf('Saving robot\n')
saveSR(sr3, 'FileName', 'SR3', 'Path', modelPath)
