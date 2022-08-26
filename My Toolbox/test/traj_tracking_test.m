%% Trajectory Tracking
clc
close all
load 'SC_2DoF.mat'
sc.homeConfig();

% Parameters
% sc.qm = [0; -pi/4];
dA = 1;
Gb = ones(6, 1)*10;
Gm = ones(2, 1)*25;
simTime = '10.0';
Nsamp = 205;  % Make sure (Nsamp - 5) is multiple of 4
squareLength = 0.5;

% Run controller design if gains not present
if ~exist('Kp_b', 'var') || ~exist('Kd_b', 'var') || ~exist('Kp_m', 'var') || ~exist('Kd_m', 'var')
    fprintf("Finding controller gains...\n")
    run('ctrl_synth')
end


% Compute Traj
[~, p0] = tr2rt(sc.getTransform('endeffector', 'inertial', 'SymbRes', false));
traj = squareTraj(p0, squareLength, str2double(simTime), Nsamp, 'plane', 'xy');

% hold on
% sc.show('preserve', false, 'fast', true, 'visuals', 'on');
% plot3(traj.EE_desired(:, 1), traj.EE_desired(:, 2), traj.EE_desired(:, 3), 'r')
% plot3(traj.EE_desired(1, 1), traj.EE_desired(1, 2), traj.EE_desired(1, 3), 'bx')
% axis equal
% hold off


% Launch Sim
fprintf("Simulating...\n")
set_param('traj_tracking', 'StopTime', simTime)
simRes = sim('traj_tracking');
fprintf("Done\n")


%% Animation
close all
filename = 'gifTest2.gif';
saveAnim = false;

% Parameters
dim = [.2 .5 .3 .3];
framesPerSecond = length(simRes.tout)/simRes.tout(end);
str = sprintf('t = %.2fs', 0);
sc.q = simRes.q.Data(:, :, 1);

% First frame
h_annot = annotation('textbox', dim,'String', str,'FitBoxToText','on');
sc.show('preserve', false, 'fast', true, 'visuals', 'on');
hold on
plot3(traj.EE_desired(:, 1), traj.EE_desired(:, 2), traj.EE_desired(:, 3), 'r')
traj_line = plot3(simRes.Xee_ref.Data(1, :, 1:1), simRes.Xee_ref.Data(2, :, 1:1), simRes.Xee_ref.Data(3, :, 1:1), 'b');
traj_point = plot3(simRes.Xee_ref.Data(1, :, 1), simRes.Xee_ref.Data(2, :, 1), simRes.Xee_ref.Data(3, :, 1), 'b*');
hold off
drawnow


f = getframe;
[im,map] = rgb2ind(f.cdata,256,'nodither');
im(1,1,1,20) = 0;

for i = 1:length(simRes.q.Time)
    sc.q = simRes.q.Data(:, :, i);
    sc.show('preserve', false, 'fast', true, 'visuals', 'on');
    curT = simRes.q.Time(i);
    str = sprintf('t = %.2fs', curT);    
    h_annot.set('String',str);
    drawnow
    
%     hold on
%     traj_point = plot3(simRes.Xee_ref.Data(1, :, i), simRes.Xee_ref.Data(2, :, i), simRes.Xee_ref.Data(3, :, i), 'b.');
%     hold off

    if saveAnim
        f = getframe;
        im(:,:,1,i) = rgb2ind(f.cdata,map,'nodither');
    end
end

if saveAnim
    imwrite(im,map,filename,'DelayTime',1/framesPerSecond,'LoopCount',inf)
end