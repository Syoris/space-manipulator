clearvars
load 'SC_2DoF.mat'
sc.homeConfig;
sc.q0_dot = [0.1; 0.2; 0.3; 0.01; 0.02; 0.03];
sc.qm_dot = [pi / 2; -pi / 4] / 10;

valStruct = struct();
valStruct.q = sc.q;
valStruct.q_dot = sc.q_dot;
valStruct.n = sc.NumActiveJoints;

% --- Kinematics Vars ---
valStruct.tTree = sc.Ttree;

valStruct.GetTrans = struct();

for i = 1:sc.NumBodies
    valStruct.GetTrans.(sc.BodyNames{i}) = sc.getTransform(sc.BodyNames{i}, 'TargetFrame', sc.BaseName, 'symbolic', false);
end

valStruct.JacobsCoM = sc.JacobsCoM;
valStruct.CoMPositions = sc.getCoMPosition();

% --- Dyn ---
valStruct.H = sc.H;
valStruct.C = sc.C;
valStruct.Q = sc.Q;

f0 = [0; 0; 0.5]; % Force on baseC
n0 = [0; 0; 0]; % Torque on base
tau_qm = [0; 0]; % Joints torque
F = [f0; n0; tau_qm];
valStruct.F = F;
valStruct.q_ddot = sc.forwardDynamics(F);

% --- Save test struct ---
clearvars -except valStruct
save test / SC_2DoF_Val.mat'
clearvars
