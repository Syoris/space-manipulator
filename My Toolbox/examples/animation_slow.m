%% Vizualisation Example
clearvars
load 'SC_2DoF.mat'

% Config Test
close all
qm = [pi/4, -pi/2];

% SpaceRobot
homeConf = sc.homeConfiguration; % Get home config
newConf = homeConf;
newConf(1).JointPosition = qm(1);
newConf(2).JointPosition = qm(2);

sc.JointsConfig = newConf;  % Alternative: sc.setJointsConfig = qm;

baseConf = sc.BaseConfig;
baseConf.Position = [1.25, 1.5, 1.2];
baseConf.Rot = [0, 0, 0];
sc.BaseConfig = baseConf;  % sc.BaseConfig = [2 3 4; 0 pi/4 0];

tTree = sc.forwardKinematics; % Compute kin tree


% Animation test
nSamp = 100;
x0 = baseConf.Position(1);
y0 = baseConf.Position(2);
z0 = baseConf.Position(3);
theta0 = baseConf.Rot(2);

xFin = x0 + 10;
thetaFin = 2*pi;
xArray = linspace(x0, xFin, nSamp);
thetaArray = linspace(theta0, thetaFin, nSamp);

baseConfArray = cell(nSamp, 1);
for i = 1:nSamp
    baseConfArray{i} = [xArray(i), y0, z0; 0 thetaArray(i) 0];
end


framesPerSecond = 50;
r = rateControl(framesPerSecond);
for i = 1:nSamp
    sc.BaseConfig = baseConfArray{i};
    sc.show('PreservePlot',false, 'Visuals', 'off');
    drawnow
    waitfor(r);
end