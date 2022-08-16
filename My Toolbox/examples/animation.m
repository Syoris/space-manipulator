%% Vizualisation Example
clearvars
load 'SC_2DoF.mat'

% Config Test
close all
qm0 = [pi/4; -pi/2];
X0 = [0; 0; 0];
phi0 = [0; 0; 0];

% Animation
dx = 10;
dy = 0;
dz = 0;

% SpaceRobot
sc.qm = qm0;
sc.q0 = [X0; phi0];

% Animation test
nSamp = 100;

thetaFin = 2*pi;

xArray = linspace(X0(1), X0(1)+dx, nSamp);
yArray = linspace(X0(2), X0(2)+dy, nSamp);
zArray = linspace(X0(3), X0(3)+dz, nSamp);
thetaArray = linspace(phi0(1), thetaFin, nSamp);

baseConfArray = cell(nSamp, 1);
for i = 1:nSamp
    baseConfArray{i} = [xArray(i); yArray(i); zArray(i); thetaArray(i); 0; 0];
end

sc.q0 = baseConfArray{1};
sc.show('preserve', false, 'fast', true, 'visuals', 'on');

framesPerSecond = 25;
r = rateControl(framesPerSecond);

tic
for i = 1:nSamp
    sc.q0 = baseConfArray{i};
    sc.show('preserve', false, 'fast', true, 'visuals', 'on');
    drawnow
    waitfor(r);
end
toc
r.DesiredPeriod
r.statistics.AveragePeriod