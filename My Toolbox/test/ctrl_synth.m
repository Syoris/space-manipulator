%% Init
clc
load 'SC_2DoF.mat'
sc.homeConfig;
n = sc.NumActiveJoints;
N = n + 6;

q_name = cellstr(arrayfun(@string, sc.q_symb));
q_dot_name = cellstr(arrayfun(@string, sc.q_dot_symb));
x = [q_name; q_dot_name];

err_b = split(sprintf('err_b(%i);', 1:6), ';');
err_b = err_b(1:6);

err_b_dot = split(sprintf('err_b_dot(%i);', 1:6), ';');
err_b_dot = err_b_dot(1:6);

err_m = split(sprintf('err_m(%i);', 1:n), ';');
err_m = err_m(1:end-1);

err_m_dot = split(sprintf('err_m_dot(%i);', 1:n), ';');
err_m_dot = err_m_dot(1:end-1);

qb = split(sprintf('qb(%i);', 1:6), ';');
qb = qb(1:end-1);
qb_dot = split(sprintf('qb_dot(%i);', 1:6), ';');
qb_dot = qb_dot(1:end-1);

qm = split(sprintf('qm(%i);', 1:n), ';');
qm = qm(1:end-1);
qm_dot = split(sprintf('qm_dot(%i);', 1:n), ';');
qm_dot = qm_dot(1:end-1);

tau_b = split(sprintf('tau_b(%i);', 1:6), ';');
tau_b = tau_b(1:end-1);
tau_m = split(sprintf('tau_m(%i);', 1:n), ';');
tau_m = tau_m(1:end-1);


% Coordinated Base controller Synthesis
A = [zeros(N, N), eye(N);
     -sc.H^-1, zeros(N, N)];

B = [zeros(N, N); sc.H^-1];

C = eye(2*N);
D = zeros(2*N, N);

sys = ss(A, B, C, D);
sys.InputName = [tau_b; tau_m];
sys.StateName = x;
y = [qb; qm; qb_dot; qm_dot];
sys.OutputName = y;

%% Base Controller
B_err = AnalysisPoint('B_err');
Tau_b = AnalysisPoint('Tau_b');

Kp_b = tunableGain('gainblock',6,6);
Kp_b.Gain.Free = eye(6);
Kp_b.InputName = 'err_b';
Kp_b.OutputName = 'up_b';

Kd_b = tunableGain('gainblock',6,6);
Kd_b.Gain.Free = eye(6);
Kd_b.InputName = 'err_b_dot';
Kd_b.OutputName = 'ud_b';

Db = tunableGain('Base_Decoupler',eye(12));
Db.Gain.Free = zeros(12);
Db.InputName = 'B_err';
Db.OutputName = [err_b; err_b_dot];

sum_bC = sumblk('tau_b = up_b + ud_b',6);

Cb = connect(Kp_b ,Kd_b, sum_bC, [err_b; err_b_dot], 'tau_b');

%% Manip Controller
M_err = AnalysisPoint('M_err');
Tau_M = AnalysisPoint('Tau_M');

Kp_m = tunableGain('gainblock',n,n);
Kp_m.Gain.Free = eye(n);
Kp_m.InputName = 'err_m';
Kp_m.OutputName = 'up_m';

Kd_m = tunableGain('gainblock',n,n);
Kd_m.Gain.Free = eye(n);
Kd_m.InputName = 'err_m_dot';
Kd_m.OutputName = 'ud_m';

sum_mC = sumblk('tau_m = up_m + ud_m',n);

Cm = connect(Kp_m ,Kd_m, sum_mC, [err_m; err_m_dot], 'tau_m');

%% Full Syst

sum_b_err = sumblk('err_b = qb_ref - qb', 6);
sum_b_err.OutputName = err_b;
sum_m_err = sumblk('err_m = qm_ref - qm', n);

sum_b_err_dot = sumblk('err_b_dot = qb_dot_ref - qb_dot', 6);
sum_m_err_dot = sumblk('err_m_dot = qm_dot_ref - qm_dot', n);

CL = connect(sum_b_err, sum_m_err, sum_b_err_dot, sum_m_err_dot, sys, ...
            {'qb_ref', 'qb_dot_ref', 'qm_ref', 'qm_dot_ref'}, ...
            y);






%% Init
clc
load 'SC_2DoF.mat'
sc.homeConfig;
n = sc.NumActiveJoints;
N = n + 6;

x0 = [sc.q; sc.q_dot];

f0 = [0; 0.1; 0];     % Forces on base [fx, fy, fz], in base frame
n0 = [0; 0; 0];     % Torques on base [nx, ny, nz], in base frame
tau_qm = [0; 0];      % Joint torques


% Coordinated Base controller Synthesis
A = [zeros(N, N), eye(N);
     zeros(N, N), zeros(N, N)];

B = [zeros(N, N); sc.H^-1];

C = eye(2*N);
D = zeros(2*N, N);

%% Tuning
mdl = 'Coordinated_ctrl_tuning';
st0 = slTuner(mdl,{'Cp_b','Cd_b', 'Cp_m', 'Cd_m'});
addPoint(st0,{'qb_ref','qb'});

% T = getIOTransfer(st,'qb_ref','qb');
% stepplot(T)


req1 = TuningGoal.Tracking('qb_ref','qb',1);

[st,fSoft,~,info] = systune(st0, req1);

T = getIOTransfer(st,'qb_ref','qb');
stepplot(T)

% req2 = TuningGoal.Gain('delta fin','delta fin',tf(25,[1 0]));
% req3 = TuningGoal.Margins('delta fin',7,45);
% max_gain = frd([2 200 200],[0.02 2 200]);
% req4 = TuningGoal.Gain('delta fin','az',max_gain);



