close all
sc.q0 = [-3; 0.5; 0; 0; 0; 0];

th1 = 0;
th2 = 0;
th3 = 0;
th4 = 0;
th5 = 0;
th6 = 0;

sc.qm = [th1; th2; th3; th4; th5; th6];

sc.show()
% axis equal
% xlim([3, 5])

%% CoM
close all
comPositions = sc.getCoMPosition();

figure

title('SpaceRobot CoM Positions')
sc.show('Visuals', 'off');
linkNames = fields(comPositions);
hold on
for i=1:length(linkNames)
    pose = comPositions.(linkNames{i});
    plot3(pose(1, 4), pose(2, 4), pose(3, 4),'r*', 'MarkerSize', 15);
end
xlim([1.5, 4])
axis equal
hold off

%% 
close all
T0 = eye(4);
T1 = [rpy2r(0, pi/2, 0), [2; 0; 0]; zeros(1, 3), 1];

hold on
SE3(T0).plot;
SE3(T1).plot;
axis equal
hold off