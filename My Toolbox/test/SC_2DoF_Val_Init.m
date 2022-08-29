clearvars
load 'SC_2DoF.mat'
sc.homeConfig;
sc.q0_dot = [0.1; 0.2; 0.3; 0.01; 0.02; 0.03];
sc.qm_dot = [pi/2; -pi/4]/10; 

test = struct();
test.q = sc.q;
test.q_dot = sc.q_dot;
test.n = sc.NumActiveJoints;

% --- Kinematics Vars --- 
test.tTree = sc.Ttree;

test.JacobsCoM = sc.JacobsCoM;
test.CoMPositions = sc.getCoMPosition();

% --- Dyn ---
test.H = sc.H;
test.C = sc.C;
test.Q = sc.Q;

f0 = [0; 0; 0.5]; % Force on baseC
n0 = [0; 0; 0]; % Torque on base
tau_qm=[0; 0]; % Joints torque
F = [f0;n0;tau_qm];
test.F = F;
test.q_ddot = sc.forwardDynamics(F);

% --- Save test struct ---
clearvars -except test
save 'My Toolbox/test/SC_2DoF_Test.mat'
clearvars
