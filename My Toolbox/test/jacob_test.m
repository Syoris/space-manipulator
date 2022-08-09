%% Computing of jacobians
% To compare jacobians with the ones computed by SPART. Run SPART_example.m
% to compare. Make sure initial configs are the same
run 'SPART_example.m'
load 'SC_2DoF.mat'

% Initial condition
% qm=[pi/6; -pi/4];
% R0=eye(3);  %Rotation from Base-spacecraft to inertial
% r0=[0; 0; 0]; %Position of the base-spacecraft

sc.JointsConfig = qm';
sc.BaseConfig = [r0'; 0, 0, 0];

% %% Jacobians
% comPoses = sc.getCoMPosition();
% Jacobians = sc.comJacobians();
% for i=1:sc.NumLinks
%     linkName = sc.LinkNames{i};
%     J_i = Jacobians.(linkName);
%     
%     fprintf('\n##### Link %i #####\n', i);
%     fprintf('Mine:\n')
%     disp(J_i);
%     fprintf('SPART:\n')
%     disp(J_S{i});
%    
% %     fprintf('--- J_%i1', i)
% %     J_i(1:3, 4:6)
% %     J_S{i}(1:3, 4:6)
% % 
% %     fprintf('--- J_%i2', i)
% %     J_i(1:3, 7:7+sc.NumActiveJoints-1)
% %     J_S{i}(1:3, 7:7+sc.NumActiveJoints-1)
% % 
% %     % J_i3
% %     fprintf('--- J_%i3', i)
% %     J_i(4:6, 7:7+sc.NumActiveJoints-1)
% %     J_S{i}(4:6, 7:7+sc.NumActiveJoints-1)
% end

%% Mass Matrix

H = sc.massMatrix;

% Spart comp
fprintf('\n##### Mass Matrix #####\n')
fprintf('--- Computed ---');
H

fprintf('--- SPART ---');
H_spart
return

%% CoM Positions in Base
sc.BaseConfig = [0.5, 1, 0; 0, 0, 0];
comPoses = sc.getCoMPosition();

rk = zeros(3, 1, sc.NumActiveJoints);

T_I_B = sc.Base.BaseToParentTransform; % Transform of base to I
R_temp = T_I_B(1:3,1:3)';
T_B_I = [R_temp, -R_temp*T_I_B(1:3,4) ;[0 0 0 1]]; % Inverse

for i=1:sc.NumActiveJoints
    T_I_i = comPoses.(sc.LinkNames{i}); % Transform of i to I
    
    T_B_i = T_B_I * T_I_i;

    rk(:, :, i) = T_B_i(1:3, 4);
end



