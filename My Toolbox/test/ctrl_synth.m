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

q = split(sprintf('q(%i);', 1:N), ';');
q = q(1:end-1);
q_dot = split(sprintf('q_dot(%i);', 1:N), ';');
q_dot = q_dot(1:end-1);

% Coordinated Base controller Synthesis
A = [zeros(N, N), eye(N);
     -sc.H^-1, zeros(N, N)];

B = [zeros(N, N); sc.H^-1];

C = eye(2*N);
D = zeros(2*N, N);

sys = ss(A, B, C, D);
sys.InputName = 'tau';
sys.StateName = x;
sys.OutputName = [q; q_dot];

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

Dm = tunableGain('M_Decoupler',eye(2*n));
Dm.Gain.Free = zeros(2*n);
Dm.InputName = 'M_err';
Dm.OutputName = [err_m; err_m_dot];

sum_mC = sumblk('tau_m = up_m + ud_m',n);

Cm = connect(Dm ,Kp_m ,Kd_m, sum_mC, 'M_err', 'tau_m');

%% Full Syst

sum_mE = sumblk('tau_m = up_m + ud_m',n);

