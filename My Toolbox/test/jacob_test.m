%% Computing of jacobians
load 'SC_2DoF.mat'

% Initial condition
sc.JointsConfig = [pi/4, -pi/2];
sc.BaseConfig = [0.5, 0.5, 0; 0, 0, 0];

%% Comparison
Jacobians = sc.comJacobians();
J_S = {{J01, Jm1}; {J02, Jm2}; {J03, Jm3}};
for i=1:sc.NumLinks
    linkName = sc.LinkNames{i};
    J_i = Jacobians.(linkName);
    
    fprintf('\n##### Link %i #####\n', i);
   
    fprintf('--- J_%i1', i)
    J_i(1:3, 4:6)
    J_S{i}{1}(4:6, 1:3)

    fprintf('--- J_%i2', i)
    J_i(1:3, 7:7+sc.NumActiveJoints-1)
    J_S{i}{2}(4:6, :)

    % J_i3
    fprintf('--- J_%i3', i)
    J_i(4:6, 7:7+sc.NumActiveJoints-1)
    J_S{i}{2}(1:3, :)
end