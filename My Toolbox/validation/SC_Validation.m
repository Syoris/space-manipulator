%% SC_Validation.m
% To validate SpaceRobot properties and methods. To make sure changes
% didn't break anything. Compares with known "good" values.
% Will valStruct:
%   - Kinematic tree (Ttree)
%   - CoM Jacobians (JacobsCoM)
%   - CoM Positions
%   - Dyn Matrices (H, C, Q)
%   - Forward dynamics

% --- Tunable Parameters ---
scToVal = sc2;                  % SpaceRobot to validate
validationName = 'SC_2DoF';     % Name of validation file 
toPrint = true;

% --- Validation ---
% Load validation
clc
fileName = [validationName '_Val.mat'];
load(fileName)

% Set config 
scToVal.q = valStruct.q;
scToVal.q_dot = valStruct.q_dot;

%% ### Validation ###
% --- Kin Tree ---
fprintf('##### Kinematic Tree #####\n');
tTree = scToVal.Ttree;

f = fields(valStruct.tTree);
for i=1:length(f)
    Tval = valStruct.tTree.(f{i});
    Tsc = tTree.(f{i});
    
    if toPrint
        fprintf('\n\t --- %s ---\n', f{i});
    
        fprintf('SC:\n')
        fprintf('\n')
        disp(Tsc);
    
        fprintf('Val:\n')
        fprintf('\n')
        disp(Tval);
    end

    assert(isequal(round(Tval, 5), round(Tsc, 5)), 'Forward Kin not matching for body %s', f{i});
end
fprintf("Kinematic Tree OK\n")

% --- GetTransform ---
% valStruct GetTransform between all bodies and base
fprintf('##### GetTransform #####\n');

f = fields(valStruct.GetTrans);
for i=1:length(f)
    Tval = valStruct.GetTrans.(f{i});
    Tsc = scToVal.getTransform(f{i}, 'TargetFrame', scToVal.BaseName, 'symbolic', false);
    
    if toPrint
        fprintf('\n\t --- %s ---\n', f{i});
    
        fprintf('SC:\n')
        fprintf('\n')
        disp(Tsc);
    
        fprintf('Val:\n')
        fprintf('\n')
        disp(Tval);
    end

    assert(isequal(round(Tval, 5), round(Tsc, 5)), 'Forward Kin not matching for body %s', f{i});
end
fprintf("GetTransform OK\n")

% --- Jacobians ---
fprintf('##### Jacobians #####\n');
Jacobians = scToVal.JacobsCoM;
for i=1:scToVal.NumBodies
    linkName = scToVal.BodyNames{i};
    Jval = Jacobians.(linkName);
    Jsc = sc2.JacobsCoM.(linkName);

    if toPrint
        fprintf('\n\t --- %s ---\n', f{i});
    
        fprintf('SC:\n')
        fprintf('\n')
        disp(Jsc);
    
        fprintf('Val:\n')
        fprintf('\n')
        disp(Jval);
    end
   
    assert(isequal(round(Jval, 5), round(Jsc, 5)), 'Jacobians not matching for body %s', linkName);   
end
fprintf("Jacobians OK\n")

% --- CoM Positions ---
fprintf('##### CoM Positions #####\n');
comPosesVal = scToVal.getCoMPosition();
comPosesSC = sc2.getCoMPosition();

for i=1:scToVal.NumBodies
    linkName = scToVal.BodyNames{i};
    [~, tVal] = tr2rt(comPosesVal.(linkName));
    [~, tSc] = tr2rt(comPosesSC.(linkName));

    if toPrint
        fprintf('\n\t --- %s ---\n', f{i});
    
        fprintf('SC:\n')
        fprintf('\n')
        disp(tSc);
    
        fprintf('Val:\n')
        fprintf('\n')
        disp(tVal);
    end
   
    assert(isequal(round(tVal, 5), round(tSc, 5)), 'CoM Positions not matching for body %s', linkName);   
end
fprintf("CoM Positions OK\n")


%% --- Dyn Matrices ---

% H - Mass Matrix
fprintf('\n##### H - Mass Matrix #####\n')
H_val = valStruct.H;
H_sc = scToVal.H;
if toPrint
    fprintf('--- Validation ---\n');
    disp(H_val);

    fprintf('\n--- Computed ---\n');  
    disp(H_sc);
end
assert(isequal(round(H_val, 5), round(H_sc, 5)), 'H matrices not matching'); 
fprintf("H Matrix OK\n")


% C - Non-Linear Effect
fprintf('\n##### C - Non-Linear Effect #####\n')
C_val = valStruct.C;
C_sc = scToVal.C;
if toPrint
    fprintf('--- Validation ---\n');
    disp(C_val);

    fprintf('\n--- Computed ---\n');  
    disp(C_sc);
end
assert(isequal(round(C_val, 5), round(C_sc, 5)), 'C matrices not matching'); 
fprintf("C Matrix OK\n")

% C Matrix Check
% assert(scToVal.isNSkewSym());
% assert(scToVal.isCOk(true));

% Q Matrix - Force Jacobian



% --- Forward Dyn ---
fprintf("\n##### Foward Dynamics #####\n")
q_ddot_val = valStruct.q_ddot;
q_ddot_sc = scToVal.forwardDynamics(valStruct.F);

if toPrint
    fprintf('--- Validation ---\n');
    disp(q_ddot_val);

    fprintf('\n--- Computed ---\n');  
    disp(q_ddot_sc);
end
assert(isequal(round(q_ddot_val, 5), round(q_ddot_sc, 5)), 'q_ddot not matching'); 
fprintf("Forward Dyn OK\n")


