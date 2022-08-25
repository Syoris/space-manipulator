%% Initialization
clc
load 'SC_2DoF.mat'
sc.homeConfig();

tTree = sc.forwardKinematics();
[~, p0] = tr2rt(tTree.endeffector);

traj = squareTraj(p0, 0.5, 10, 101, 'plane', 'xy');

%% Animation
close all
filename = 'gifTest2.gif';
saveAnim = false;

% Parameters
dim = [.2 .5 .3 .3];
simRes = out;
framesPerSecond = length(simRes.tout)/simRes.tout(end);
str = sprintf('t = %.2fs', 0);
sc.q = simRes.q.Data(:, :, 1);

% First frame
h_annot = annotation('textbox', dim,'String', str,'FitBoxToText','on');
sc.show('preserve', false, 'fast', true, 'visuals', 'on');
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
    
    if saveAnim
        f = getframe;
        im(:,:,1,i) = rgb2ind(f.cdata,map,'nodither');
    end
end

if saveAnim
    imwrite(im,map,filename,'DelayTime',1/framesPerSecond,'LoopCount',inf)
end

sc.homeConfig;