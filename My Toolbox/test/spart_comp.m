%% Spart/Own comp
close all

comPositions = sc.getCoMPosition();

figure
hold on
sc.show('Visuals', 'off')
for i=1:3
%     plot3(rJ(1, i), rJ(2, i), rJ(3, i), 'r*', 'MarkerSize', 15)
    plot3(rL(1, i), rL(2, i), rL(3, i), 'k*', 'MarkerSize', 15)
end

linkNames = fields(comPositions);
for i=1:length(linkNames)
    pose = comPositions.(linkNames{i}).Transform;
    plot3(pose(1, 4), pose(2, 4), pose(3, 4),'bp', 'MarkerSize', 15);
end

% plotTransforms(tform2trvec(T_b_2), tform2quat(T_b_2), "FrameSize", 0.1);
hold off