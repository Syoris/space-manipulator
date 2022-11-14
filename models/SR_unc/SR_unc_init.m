%% SR_6DoF_init
% 6 DoF SpaceRobot initialization from DH parameters. Using the 6 DoF arm
% designed by MDA. Used with uncertainties.
% See [2016, Dubanchet] p.338
sr_unc = SpaceRobot;
sr_unc.Name = 'SR_unc';
sr_unc.Logging = 'debug';
nBodies = 7; % 6DoF + ee
modelPath = fullfile('models/SR_unc');
%% --- Manipulator Parameters ---
% ## Dynamic Parameters ##
% i   ρi      li      mi      Jx_i    Jy_i    Jz_i
% 1   2250    0.10    0.36    0.82    1.00    0.82
% 2   −       2.61    9.20    0.03    5.24    5.24
% 3   −       2.61    9.20    0.03    5.24    5.24
% 4   −       0.10    0.36    1.00    0.82    0.82
% 5   −       0.10    0.36    0.82    0.82    1.00
% 6   −       0.50    1.78    0.04    0.04    0.01

bodyLength = [0.1, 2.61, 2.61, 0.1, 0.1, 0.5, 0];
bodyMass = [0.36, 9.20, 9.20, 0.36, 0.36, 1.78, 5];
Jx = [0.82, 0.03, 0.03, 1.00, 0.82, 0.04, 0];
Jy = [1, 5.24, 5.24, 0.82, 0.82, 0.04, 0];
Jz = [0.82, 5.24, 5.24, 0.82, 1, 0.01, 0];

% ## DH Parameters ##
% i     ai      αi      di      θi
% 1     l1      π/2     0.06    θ1
% 2     l2      0       0.10    θ2
% 3     l3      0       0.10    θ3
% 4     l4      −π/2    0.10    θ4
% 5     l5      π/2     0.14    θ5+π/2
% 6     0       0       l6      θ6
% E     0       0       0       0

%          [a                   alpha       d           theta]      i
dhparams = [ ...
        bodyLength(1) pi / 2 0.06 0; %   1
        bodyLength(2) 0 0.10 0; %   2
        bodyLength(3) 0 0.10 0; %   3
        bodyLength(4) -pi / 2 0.10 0; %   4
        bodyLength(5) pi / 2 0.14 pi / 2; %   5
        0 0 bodyLength(6) 0; %   6
        0 0 0 0; ]; %   ee

% ## Body Parameters ##
jointsAxis = [[0 0 1];
            [0 0 1];
            [0 0 1];
            [0 0 1];
            [0 0 1];
            [0 0 1];
            [0 0 0]];

bodiesCoM = [[bodyLength(1) / 2 0 0.06];
        [bodyLength(2) / 2 0 0.1];
        [bodyLength(3) / 2 0 0.1];
        [bodyLength(4) / 2 0 0.1];
        [0 bodyLength(5) / 2 0.14];
        [0 0 bodyLength(6) / 2];
        [0 0 0]; ];

jointsHomePos = [0; deg2rad(60); deg2rad(-100); deg2rad(-50); 0; 0];

jointsPositionLimits = [[-inf, inf]; % shoulder
                    [0, pi]; % shoulder
                    [deg2rad(-170), deg2rad(170)]; % elbow
                    [-inf, inf];%[deg2rad(-170), deg2rad(90)]; % wrist
                    [-inf, inf]; % wrist
                    [-inf, inf]; % wrist
                    [-inf, inf]; % ee
                    ];

bodyNames = {'Body1', 'Body2', 'Body3', 'Body4', 'Body5', 'Body6', 'endeffector'};
jointNames = {'jnt1', 'jnt2', 'jnt3', 'jnt4', 'jnt5', 'jnt6', 'jnt_ee'};

% --- Base Parameters ---
sizeBase = [2.0, 2.0, 2.0]; % Base [height, width, depth] [m], [z, x, y]
mBase = 200; % [kg]
comBase = [0 0 0];
intertiaBase = [50, 50, 50, 0, 0, 0]; % [Ixx Iyy Izz Iyz Ixz Ixy]

sr_unc.Base.Mass = mBase;
sr_unc.Base.Inertia = intertiaBase;
sr_unc.Base.HomeConf = [0; 0; 0; 0; 0; 0]; % [Rx; Ry; Rz; r; p; y]
sr_unc.Base.ManipToBaseTransform = rt2tr(eye(3), [0; 0; sizeBase(1) / 2]);

%% --- Visuals ---
% Materials
materials = struct();
materials.Blue = [0.5 0.7 1.0 1.0];
materials.Orange = [1.0 0.423529411765 0.0392156862745 1.0];
materials.Grey = [0.57 0.57 0.57 1.0];
materials.Red = [1 0 0 1];

% Base
baseVisual = struct('Geometry', 'Box', ...
'Parameters', [sizeBase(2), sizeBase(3), sizeBase(1)], ...
    'T', trvec2tform([0 0 0]), ...
    'Color', materials.Grey);

% Bodies
bodiesVisual = cell(nBodies, 1);
bodiesVisual{1}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.05, 0.1], ...
    'T', trvec2tform([0.05 0 0.06]) * eul2tform([0 pi / 2 0]), ...
    'Color', materials.Blue);
bodiesVisual{1}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.05, 0.06 + 0.05], ...
    'T', trvec2tform([0.0 0 (0.06 + 0.05) / 2]), ...
    'Color', materials.Orange);

bodiesVisual{2}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.05, bodyLength(2)], ...
    'T', trvec2tform([bodyLength(2) / 2 0 0.1]) * eul2tform([0 pi / 2 0]), ...
    'Color', materials.Blue);
bodiesVisual{2}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.05, 0.2], ...
    'T', trvec2tform([0 0 0.05]), ...
    'Color', materials.Orange);

bodiesVisual{3}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.05, bodyLength(3)], ...
    'T', trvec2tform([bodyLength(3) / 2 0 0.1]) * eul2tform([0 pi / 2 0]), ...
    'Color', materials.Blue);
bodiesVisual{3}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.05, 0.2], ...
    'T', trvec2tform([0 0 0.05]), ...
    'Color', materials.Orange);

bodiesVisual{4}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.05, bodyLength(4)], ...
    'T', trvec2tform([bodyLength(4) / 2 0 0.1]) * eul2tform([0 pi / 2 0]), ...
    'Color', materials.Blue);
bodiesVisual{4}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.05, 0.2], ...
    'T', trvec2tform([0 0 0.05]), ...
    'Color', materials.Orange);

bodiesVisual{5}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.05, 0.1], ...
    'T', trvec2tform([0 0.05 0.14]) * eul2tform([0 0 pi / 2]), ...
    'Color', materials.Blue);
bodiesVisual{5}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.05, 0.24], ...
    'T', trvec2tform([0.0 0 0.07]), ...
    'Color', materials.Orange);

bodiesVisual{6}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.05, bodyLength(6)], ...
    'T', trvec2tform([0 0 bodyLength(6) / 2]) * eul2tform([0 0 0]), ...
    'Color', materials.Blue);
bodiesVisual{6}(end + 1) = struct('Geometry', 'Cylinder', ...
    'Parameters', [0.07, 0.1], ...
    'T', trvec2tform([0.0 0 0]) * eul2tform([0 0 0]), ...
    'Color', materials.Orange);

bodiesVisual{7}(end + 1) = struct('Geometry', 'Box', ...
    'Parameters', [0.3, 0.1, 0.1], ...
    'T', trvec2tform([0.0 0 0]), ...
    'Color', materials.Red);

bodiesVisual{7}(end + 1) = struct('Geometry', 'Box', ...
    'Parameters', [0.05, 0.1, 0.2], ...
    'T', trvec2tform([-0.1250 0 0.1]), ...
    'Color', materials.Red);

bodiesVisual{7}(end + 1) = struct('Geometry', 'Box', ...
    'Parameters', [0.05, 0.1, 0.2], ...
    'T', trvec2tform([0.1250 0 0.1]), ...
    'Color', materials.Red);

% --- Robot Initialization ---
sr_unc.Base.addVisual(baseVisual.Geometry, baseVisual.Parameters, ...
baseVisual.T, baseVisual.Color);

bodiesVect = cell(1, nBodies);
jointsVect = cell(1, nBodies);

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
    newBody.Mass = bodyMass(i);
    newBody.Inertia = [Jx(i), Jy(i), Jz(i), 0, 0, 0];
    newBody.CenterOfMass = bodiesCoM(i, :);

    % Visual
    if ~isempty(bodiesVisual{i})

        for j = 1:length(bodiesVisual{i})
            vis = bodiesVisual{i}(j);
            newBody.addVisual(vis.Geometry, vis.Parameters, ...
                vis.T, vis.Color);
        end

    end

    % Add body
    if i == 1
        parent = sr_unc.BaseName;
    else
        parent = bodyNames(i - 1);
    end

    bodies{i} = newBody;
    joints{i} = newJoint;
    sr_unc.addBody(bodies{i}, parent);
end

%% --- Initialize Matrices ---
sr_unc.Base.initBase();

for i = 1:length(sr_unc.Bodies)
    sr_unc.Bodies{i}.initBody();
end

sr_unc.initKin();

sr_unc.homeConfig;
sr_unc.show;

%% --- Generate functions ---
% Create rotation matrix function handle
fprintf('Generating RFunc\n')
RFunc_gen(sr_unc, modelPath);

fprintf('Creating struct file\n')
sr_unc_info = srInfoInit(sr_unc);
Struct2File(sr_unc_info, modelPath);

sr_unc.InfoFunc = @SR_unc_info_struct;

% Not needed for sim only
sr_unc.StateFunc = @SR6_state_func;
sr_unc.StateFuncMex = @SR6_state_func_mex;

%% --- Save Robot ---
fprintf('Saving robot\n')
saveSR(sr_unc, 'FileName', 'SR_unc', 'Path', modelPath)
