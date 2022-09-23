%% sr_state_function.m
% To compute SR state function indenpendantly from class

%% Create Rot matrix function
nk = sr.NumBodies;
fileName = ['RFunc_', sr.Name];

Rb_symb = sr.Base.RotM_symb;
R_array_symb = sym(zeros(3, 3*nk));

for i=1:nk
    body = sr.Bodies{i};
    body_idx = body.Id;
    
    R_i = body.ParentRotM;

    R_array_symb(:, 3*body_idx-2: body_idx*3) = R_i;
end

matlabFunction(Rb_symb, R_array_symb, 'File', fileName, 'Vars', {sr.q_symb}, 'Outputs',{'Rb','Rm'});

%% SR class vars extraction
% Extract SR parameters from the class and put them in a struct
clc
close all
load('SR2.mat', 'sr');

sr_info = struct();

% Create info struct
nk = sr.NumBodies;
n = sr.NumActiveJoints;
N = n + 6;

sr_info.Name = sr.Name;

sr_info.nk = nk;
sr_info.n = n;
sr_info.N = N;

sr_info.BodyNames = sr.BodyNames;


% Save Ttree to matlab file


% Create transform struct: Symbolic transform of each body wrt previous one
Ab = sr.Base.A;
Pb = sr.Base.P;
Mb = sr.Base.M;

sr_info.jnt_idx = zeros(nk, 1);
sr_info.A = {Ab, zeros(6, 6, nk)};
sr_info.M = {Mb, zeros(6, 6, nk)};
sr_info.P = {Pb, zeros(6, 1, nk)};

for i=1:nk
    body = sr.Bodies{i};
    body_idx = body.Id;
    
    sr_info.jnt_idx(body_idx) = body.Joint.Q_id;
    sr_info.A{2}(:, :, body_idx) = body.A;
    sr_info.M{2}(:, :, body_idx) = body.M;
    sr_info.P{2}(:, :, body_idx) = body.P;
end

%% Compute Forward Dyn
clc

% Random Config
q0 = diag([1; 1; 1; 1; 1; 1]) * rand(6, 1);
qm = diag([1; 1]) * rand(2, 1);

q0_dot = diag([1; 1; 1; 1; 1; 1]) * rand(6, 1);
qm_dot = diag([1; 1]) * rand(2, 1);

q = [q0; qm];
q_dot = [q0_dot; qm_dot];
q_ddot = zeros(N, 1);
q0_ddot = q_ddot(1:6);
qm_ddot = q_ddot(7:end);

% 1 - Kinematics
[Rb, Rm] = RFunc_SR2(q);
Rm = reshape(Rm, 3, 3, []); % Split Rm to nk 3x3 arrays

% 2 - Kinetics
[t, t_dot, Omega, A, A_dot] = Kin(sr_info, q, q_dot, q_ddot, {Rb, Rm});

% 3 - ID
[tau, wen] = ID(sr_info, t, t_dot, Omega, A, A_dot);

% 4 - Mass Mat
D = MassM(sr_info, q, A);
D2 = sr.MassMat(q);

run spart_twist_test.m 

names = {'bb', 'ba', 'ab', 'aa'};
idx = {{1:6, 1:6}, {1:6, 7:8}, {7:8, 1:6}, {7:8, 7:8}};
for i=1:4
    fprintf('--- %s ---\n', names{i})
    fprintf('NEW\n')
    disp(D(idx{i}{1}, idx{i}{2}))
    fprintf('OLD\n')
    disp(D2(idx{i}{1}, idx{i}{2}))
    fprintf('SPART\n')
    disp(H_spart(idx{i}{1}, idx{i}{2}))
end    

isequal(D, D2)

% 5 - FD





