%% SC_6DoF_init
% 6 DoF SpaceRobot initialization from DH parameters. Using the 6 DoF arm
% designed by MDA. See [2016, Dubanchet] p.338

sc = SpaceRobot;
sc.Name = 'spaceRobot_6DoF';
sc.Logging = 'debug';
nLinks = 7; % 6DoF + ee

% --- Manipulator Parameters ---
% ## Dynamic Parameters ##
% i   ρi      li      mi      Jx_i    Jy_i    Jz_i
% 1   2250    0.10    0.36    0.82    1.00    0.82
% 2   −       2.61    9.20    0.03    5.24    5.24
% 3   −       2.61    9.20    0.03    5.24    5.24 
% 4   −       0.10    0.36    1.00    0.82    0.82 
% 5   −       0.10    0.36    0.82    0.82    1.00 
% 6   −       0.50    1.78    0.04    0.04    0.01

linkLength =    [0.1,   2.61,   2.62,   0.1,    0.1,    0.5,    0];
linkMass =      [0.36,  9.20,   9.20,   0.36,   0.36,   1.78,   0];
Jx =        [0.82,  0.03,   0.03,   1.00,   0.82,   0.04,   0];
Jy =        [1,     5.24,   5.24,   0.82,   0.82,   0.04,   0];
Jz =        [0.82,  5.24,   5.24,   0.82,   1,      0.01,   0];

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
dhparams = [linkLength(1)       pi/2        0.06        0;      %   1  
            linkLength(2)     	0	        0.10   	    0;      %   2      
            linkLength(3)  	    0           0.10        0;      %   3
            linkLength(4)  	    -pi/2       0.10        0;      %   4
            linkLength(5)  	    pi/2        0.14        pi/2;   %   5
            0  	                0           linkLength(6)   0;      %   6
            0  	                0           0           0;];    %   ee

% ## Link Parameters ##
jointsAxis = [[0 0 1];
              [0 0 1];
              [0 0 1];
              [0 0 1];
              [0 0 1];
              [0 0 1];
              [0 0 0]];

linksCoM = [[linkLength(1)/2 0 0.06];
            [linkLength(2)/2 0 0.1];
            [linkLength(3)/2 0 0.1];
            [linkLength(4)/2 0 0.1];
            [0 linkLength(5)/2 0.14];
            [0 0 linkLength(6)/2];
            [0 0 0];];

jointsHomePos = [pi/4; pi/4; -pi/2; pi/4; 0; 0];

jointsPositionLimits = [[-pi, pi];                  % shoulder
                        [0, pi];                    % shoulder
                        [-170*pi/180, 170*pi/180];  % elbow
                        [-170*pi/180, 90*pi/180];   % wrist
                        [-pi, pi];    % wrist
                        [-pi, pi];                  % wrist
                        [-inf, inf];                % ee
                        ];



linkNames = {'Link1', 'Link2', 'Link3', 'Link4', 'Link5', 'Link6', 'endeffector'};
jointNames = {'jnt1', 'jnt2', 'jnt3', 'jnt4', 'jnt5', 'jnt6',  'jnt_ee'};

% --- Base Parameters ---
sizeBase = [2.6, 1.7, 1.8]; % Base [height, width, depth] [m], [z, x, y]
mBase = 786; % [kg]
comBase = [0 0 0];
intertiaBase = [401.5, 655.0, 632.1, 0, 0, 0]; % [Ixx Iyy Izz Iyz Ixz Ixy]

sc.Base.Mass = mBase;
sc.Base.Inertia = intertiaBase;
sc.Base.HomeConf = [0; 0; 0; 0; 0; 0]; % [Rx; Ry; Rz; r; p; y]
sc.Base.ManipToBaseTransform = rt2tr(eye(3), [0; 0; sizeBase(1)/2]);

% --- Visuals ---
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

% Links
linksVisual = cell(nLinks, 1);
linksVisual{1}(end+1) = struct('Geometry', 'Cylinder', ...
                               'Parameters', [0.05, 0.1], ...
                               'T', trvec2tform([0.05 0 0.06])*eul2tform([0 pi/2 0]), ...
                               'Color', materials.Blue);
linksVisual{1}(end+1) = struct('Geometry', 'Cylinder', ...
                               'Parameters', [0.05, 0.06+0.05], ...
                               'T', trvec2tform([0.0 0 (0.06+0.05)/2]), ...
                               'Color', materials.Orange);

linksVisual{2}(end+1) = struct('Geometry', 'Cylinder', ...
                               'Parameters', [0.05, linkLength(2)], ...
                               'T', trvec2tform([linkLength(2)/2 0 0.1])*eul2tform([0 pi/2 0]), ...
                               'Color', materials.Blue);
linksVisual{2}(end+1) = struct('Geometry', 'Cylinder', ...
                               'Parameters', [0.05, 0.2], ...
                               'T', trvec2tform([0 0 0.05]), ...
                               'Color', materials.Orange);

linksVisual{3}(end+1) = struct('Geometry', 'Cylinder', ...
                               'Parameters', [0.05, linkLength(3)], ...
                               'T', trvec2tform([linkLength(3)/2 0 0.1])*eul2tform([0 pi/2 0]), ...
                               'Color', materials.Blue);
linksVisual{3}(end+1) = struct('Geometry', 'Cylinder', ...
                               'Parameters', [0.05, 0.2], ...
                               'T', trvec2tform([0 0 0.05]), ...
                               'Color', materials.Orange);

linksVisual{4}(end+1) = struct('Geometry', 'Cylinder', ...
                               'Parameters', [0.05, linkLength(4)], ...
                               'T', trvec2tform([linkLength(4)/2 0 0.1])*eul2tform([0 pi/2 0]), ...
                               'Color', materials.Blue);
linksVisual{4}(end+1) = struct('Geometry', 'Cylinder', ...
                               'Parameters', [0.05, 0.2], ...
                               'T', trvec2tform([0 0 0.05]), ...
                               'Color', materials.Orange);

linksVisual{5}(end+1) = struct('Geometry', 'Cylinder', ...
                               'Parameters', [0.05, 0.1], ...
                               'T', trvec2tform([0 0.05 0.14])*eul2tform([0 0 pi/2]), ...
                               'Color', materials.Blue);
linksVisual{5}(end+1) = struct('Geometry', 'Cylinder', ...
                               'Parameters', [0.05, 0.24], ...
                               'T', trvec2tform([0.0 0 0.07]), ...
                               'Color', materials.Orange);

linksVisual{6}(end+1) = struct('Geometry', 'Cylinder', ...
                               'Parameters', [0.05, linkLength(6)], ...
                               'T', trvec2tform([0 0 linkLength(6)/2])*eul2tform([0 0 0]), ...
                               'Color', materials.Blue);
linksVisual{6}(end+1) = struct('Geometry', 'Cylinder', ...
                               'Parameters', [0.07, 0.1], ...
                               'T', trvec2tform([0.0 0 0])*eul2tform([0 0 0]), ...
                               'Color', materials.Orange);

linksVisual{7}(end+1) = struct('Geometry', 'Sphere', ...
                               'Parameters', 0.1, ...
                               'T', trvec2tform([0.0 0 0]), ...
                               'Color', materials.Red);


% --- Robot Initialization ---
sc.Base.addVisual(baseVisual.Geometry, baseVisual.Parameters, ...
                  baseVisual.T, baseVisual.Color);

linksVect = cell(1, nLinks);
jointsVect = cell(1, nLinks);

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
    newJoint.PositionLimits = jointsPositionLimits(i, :);
    newLink.Joint = newJoint;
    newLink.Mass = linkMass(i);
    newLink.Inertia = [Jx(i), Jy(i), Jz(i), 0, 0, 0];
    newLink.CenterOfMass = linksCoM(i, :);
    
    % Visual
    if ~isempty(linksVisual{i})
        for j=1:length(linksVisual{i})
            vis = linksVisual{i}(j);
            newLink.addVisual(vis.Geometry, vis.Parameters, ...
                              vis.T, vis.Color);
        end
    end

    % Add link
    if i==1
        parent = sc.BaseName;
    else
        parent = linkNames(i-1);
    end
          
    links{i} = newLink;
    joints{i} = newJoint;
    sc.addLink(links{i}, parent);
end

%% --- Initialize Matrices ---
tic
sc.initKin();
toc
tic
sc.initDyn('simplify', false);
toc

sc.homeConfig;


%% Save Robot
fprintf('Saving robot\n')
clearvars -except sc
save 'Project/Models/SC_6DoF.mat'