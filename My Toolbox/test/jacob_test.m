%% Computing of jacobians
% To compare jacobians with the ones computed by SPART. Run SPART_example.m
% to compare. Make sure initial configs are the same

load 'SC_2DoF.mat'

% Initial condition
sc.JointsConfig = qm';
sc.BaseConfig = [r0'; 0, 0, 0];

%% Jacobians
Jacobians = sc.comJacobians();
for i=1:sc.NumLinks
    linkName = sc.LinkNames{i};
    J_i = Jacobians.(linkName);
    
    fprintf('\n##### Link %i #####\n', i);
    fprintf('Mine:\n')
    disp(J_i);
    fprintf('SPART:\n')
    disp(J_S{i});
   
%     fprintf('--- J_%i1', i)
%     J_i(1:3, 4:6)
%     J_S{i}(1:3, 4:6)
% 
%     fprintf('--- J_%i2', i)
%     J_i(1:3, 7:7+sc.NumActiveJoints-1)
%     J_S{i}(1:3, 7:7+sc.NumActiveJoints-1)
% 
%     % J_i3
%     fprintf('--- J_%i3', i)
%     J_i(4:6, 7:7+sc.NumActiveJoints-1)
%     J_S{i}(4:6, 7:7+sc.NumActiveJoints-1)
end

%% Mass Matrix
H = zeros(6+sc.NumActiveJoints);

% Base
Jacobians = sc.comJacobians();
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

% Spart comp
fprintf('\n##### Mass Matrix #####\n')
fprintf('--- Computed ---');
H

fprintf('--- SPART ---');
H_spart

