%% 2 DoF Planar Manipulator
clc
close all
clearvars

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

% Links Parameters
nLinks = 3;
linksVect = cell(1, nLinks);
jointsVect = cell(1, nLinks);

linkNames = {'Link1', 'Link2', 'endeffector'};
jointNames = {'jnt1', 'jnt2', 'jnt_ee'};

jointsAxis = [[0 0 1];
              [0 0 1];
              [0 0 0]];


jointsHomePos = [pi/4, -pi/2, 0];

linksLength = [1; 0.5];
linksMass = [5; 2.5; 1];
linksInertia = [1, 1, 1, 0, 0, 0; 
                0.5, 0.5, 0.5, 0, 0, 0;
                0.1, 0.1, 0.1, 0, 0, 0];

jointsT = {trvec2tform([lBase/2, 0, 0]);
           trvec2tform([linksLength(1), 0, 0]);
           trvec2tform([linksLength(2), 0, 0])};

linksCoM = [[linksLength(1)/2 0 0];
            [linksLength(2)/2 0 0];
            [0 0 0];];

% Visuals
baseVisual = struct('Geometry', 'Box', ...
                    'Parameters', [lBase, lBase, lBase], ...
                    'T', trvec2tform([0 0 0]), ...
                    'Color', materials.Grey);

linksVisual = cell(nLinks, 1);
linksVisual{1}(end+1) = struct('Geometry', 'Cylinder', ...
                               'Parameters', [0.05, linksLength(1)], ...
                               'T', trvec2tform([linksLength(1)/2 0 0])*eul2tform([0 pi/2 0]), ...
                               'Color', materials.Blue);
linksVisual{1}(end+1) = struct('Geometry', 'Cylinder', ...
                               'Parameters', [0.1, 0.1], ...
                               'T', trvec2tform([0.0 0 0]), ...
                               'Color', materials.Orange);

linksVisual{2}(end+1) = struct('Geometry', 'Cylinder', ...
                               'Parameters', [0.05, linksLength(2)], ...
                               'T', trvec2tform([linksLength(2)/2 0 0])*eul2tform([0 pi/2 0]), ...
                               'Color', materials.Blue);
linksVisual{2}(end+1) = struct('Geometry', 'Cylinder', ...
                               'Parameters', [0.1, 0.1], ...
                               'T', trvec2tform([0.0 0 0]), ...
                               'Color', materials.Orange);

linksVisual{3}(end+1) = struct('Geometry', 'Sphere', ...
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

for i=1:nLinks
    newLink = Link(linkNames{i});
    if any(jointsAxis(i, :))
        newJoint = Joint(jointNames{i}, 'revolute');
        newJoint.Axis = jointsAxis(i, :);
        newJoint.HomePosition = jointsHomePos(i);
    else
        newJoint = Joint(jointNames{i}, 'fixed');
    end
    
    newJoint.setFixedTransform(jointsT{i});
    newLink.Joint = newJoint;
    newLink.Mass = linksMass(i);
    newLink.Inertia = linksInertia(i, :);
    newLink.CenterOfMass = linksCoM(i, :);
    
    if ~isempty(linksVisual{i})
        for j=1:length(linksVisual{i})
            vis = linksVisual{i}(j);
            newLink.addVisual(vis.Geometry, vis.Parameters, ...
                              vis.T, vis.Color);
        end
    end

    linksVect{i} = newLink;
    jointsVect{i} = newJoint;
end

for i = 1:length(linksVect)
    if i==1
        parent = sc.BaseName;
    else
        parent = linkNames(i-1);
    end

    sc.addLink(linksVect{i}, parent); % Add body1 to base
end
 
% Remove vars
clearvars -except sc