%% Computing of jacobians
% To compare jacobians with the ones computed by SPART. Run SPART_example.m
% to compare. Make sure initial configs are the same

load 'SC_2DoF.mat'

% Initial condition
qm=[pi/6; -pi/4];
R0=eye(3);  %Rotation from Base-spacecraft to inertial
r0=[0; 0; 0]; %Position of the base-spacecraft

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
H = zeros(6+sc.NumActiveJoints);

% Base
Jacobians = sc.comJacobiansBase();
Jv_B = Jacobians.(sc.BaseName)(1:3, :); % Base speed jacobian
Jw_B = Jacobians.(sc.BaseName)(4:6, :); % Base rotation jacobian
inertias = sc.getInertiaM();

bV = sc.Base.Mass*(Jv_B'*Jv_B);
bW = Jw_B'*inertias.(sc.BaseName)*Jw_B;

H = H + bV + bW;

for i=1:sc.NumLinks
    link = sc.Links{i};
    joint = link.Joint;
    
    % Compute for active joints
    if joint.Q_id ~= -1   
        disp(link.Name)
        Jv_i = Jacobians.(link.Name)(1:3, :);
        Jw_i = Jacobians.(link.Name)(4:6, :);
        I_i = inertias.(link.Name);
        
        iv_1 = link.Mass*(Jv_B'*Jv_B);
        iv_2 = 2*link.Mass*(Jv_B'*Jv_i);
        iv_2_2 = link.Mass*( Jv_B'*Jv_i + Jv_i'*Jv_B);
        iv_3 = link.Mass*(Jv_i'*Jv_i);
        iw = Jw_i' * I_i * Jw_i;
        
        H = H + iv_1 + iv_2_2 + iv_3 + iw;
    end
end
H1 = H;

% Spart comp
fprintf('\n##### Mass Matrix #####\n')
fprintf('--- Computed ---');
H1

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



