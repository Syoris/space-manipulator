%% SC_Validation.m
% To validate SpaceRobot properties and methods. To make sure changes
% didn't break anything. Compares with known "good" values.
% Will test:
%   - Kinematic tree (Ttree)
%   - CoM Jacobians (JacobsCoM)
%   - CoM Positions
%   - Dyn Matrices (H, C, Q)
%   - Forward dynamics

% --- Tunable Parameters ---
scToVal = sc2;                  % SpaceRobot to validate
validationName = 'SC_2DoF';     % Name of validation file 


% --- Validation ---
% Load validation
clc
fileName = [validationName '_Test.mat'];
load(fileName)

% Set config 
scToVal.q = test.q;
scToVal.q_dot = test.q_dot;

% --- Kin Tree ---
fprintf('\n##### Kinematic Tree #####\n');
tTree = scToVal.Ttree;

f = fields(test.tTree);
for i=1:length(f)
    Tval = test.tTree.(f{i});
    Tsc = tTree.(f{i});

    fprintf('\n\t --- %s ---\n', f{i});

    fprintf('SC:\n')
    fprintf('\n')
    disp(Tsc);

    fprintf('Val:\n')
    fprintf('\n')
    disp(Tval);
end


%% --- Jacobians ---
Jacobians = scToVal.JacobsCoM;
for i=1:scToVal.NumLinks
    linkName = scToVal.LinkNames{i};
    J_i = Jacobians.(linkName);
    
    fprintf('\n##### Link %i #####\n', i);
    fprintf('Mine:\n')
    fprintf('\n')
    disp(J_i);
   
    assert(isequal(round(J_i, 5), round(J_i_S, 5)), 'Jacobians not matching for link %s', linkName);   
end



% --- CoM Positions ---
comPoses = scToVal.getCoMPosition();



% --- Dyn Matrices ---
fprintf('\n##### H - Mass Matrix #####\n')
fprintf('--- Computed ---\n');
tic
disp(scToVal.H);
toc

fprintf('--- SPART ---\n');
tic
H_spart_val = double(subs(H_spart, q, q_val));
disp(H_spart_val)
toc

% C - Non-Linear Effect
fprintf('\n##### C Matrix #####\n')
fprintf('--- Computed ---\n');
tic
disp(scToVal.C);
toc

fprintf('--- SPART ---\n');
tic
disp(double(subs(C_spart, [q; q_dot], [q_val; q_dot_val])))
toc

% C Matrix Check
assert(scToVal.isNSkewSym());
assert(scToVal.isCOk(true));

% Q Matrix - Force Jacobian



% --- Forward Dyn ---
fprintf("\n### Foward Dynamics ###\n")
fprintf('-- Computed --\n')
tic
F = [f0;n0;tau_qm];
q_ddot = scToVal.forwardDynamics(test.F);
disp(q_ddot)
toc

fprintf("Same result: %i\n", all(round(q_ddot, 5) == round([u0dot_FD(4:6); u0dot_FD(1:3); umdot_FD], 5)))


