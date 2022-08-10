% Plot Transform between different links
clearvars
run 'SC_2DoF_init.m'

sc.BaseConfig = [1 1 1; 0 0 0];

sc.JointsConfig = [pi/4, -pi/2];  % Alternative: sc.setJointsConfig = qm;
tTree = sc.forwardKinematics();

T_1_2 = sc.getTransform('Link2', 'Link1'); % To represent frame 2 wrt frame 1
T_b_2 = sc.getTransform('Link2'); % To represent frame 2 wrt base
T_2_b = sc.getTransform('spacecraftBase', 'Link2'); % To represent base wrt frame 2
T_i_2 = tTree.('spacecraftBase')*T_b_2; % frame wrt inertial frame
T_1_ee = sc.getTransform('endeffector', 'Link1'); % To represent ee wrt frame 1

figure
hold
sc.show('Visuals', 'off')
% plotTransforms(tform2trvec(T_1_2), tform2quat(T_1_2), "FrameSize", 0.1);
plotTransforms(tform2trvec(T_b_2), tform2quat(T_b_2), "FrameSize", 0.1);
plotTransforms(tform2trvec(T_i_2), tform2quat(T_i_2), "FrameSize", 0.5);
% plotTransforms(tform2trvec(T_2_b), tform2quat(T_2_b), "FrameSize", 0.1);
% plotTransforms(tform2trvec(T_1_ee), tform2quat(T_1_ee), "FrameSize", 0.1);
hold off