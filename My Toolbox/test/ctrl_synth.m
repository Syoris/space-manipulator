%% Init
clc
warning off
load 'SC_2DoF.mat'
sc.homeConfig;
n = sc.NumActiveJoints;
N = n + 6;

x0 = [sc.q; sc.q_dot];

% Coordinated Base controller Synthesis
A = [zeros(N, N), eye(N);
     zeros(N, N), zeros(N, N)];

B = [zeros(N, N); sc.H^-1];

C = eye(2*N);
D = zeros(2*N, N);

%% Tuning
mdl = 'Coordinated_ctrl_tuning';
st0 = slTuner(mdl,{'Kp_b','Kd_b', 'Kp_m', 'Kd_m'});
maxKpb = 1000;
maxKdb = 1000;
maxKpm = 1000;
maxKdm = 1000;

G1 = tunableGain('Kp_b', eye(6));
G1.Gain.Free = eye(6);
G1.Gain.Maximum = diag(maxKpb);

G2 = tunableGain('Kd_b', eye(6));
G2.Gain.Free = eye(6);
G2.Gain.Maximum = diag(maxKdb);

G3 = tunableGain('Kp_m', eye(n));
G3.Gain.Free = eye(n);
G3.Gain.Maximum = diag(maxKpm);

G4 = tunableGain('Kd_m', eye(n));
G4.Gain.Free = eye(n);
G4.Gain.Maximum = diag(maxKdm);

setBlockParam(st0,'Kp_b',G1,'Kd_b',G2,'Kp_m',G3, 'Kd_m', G4);
% getBlockParam(st0, 'Kp_b').Gain.Value

addPoint(st0,{'qb_ref','qb'});
addPoint(st0,{'qb_dot_ref','qb_dot'});
addPoint(st0,{'qm_ref','qm'});
addPoint(st0,{'qm_dot_ref','qm_dot'});

% T = getIOTransfer(st,'qb_ref','qb');
% stepplot(T)


req1 = TuningGoal.Tracking('qb_ref','qb',10);
req2 = TuningGoal.Tracking('qb_dot_ref','qb_dot',1);
req3 = TuningGoal.Tracking('qm_ref','qm',1);
req4 = TuningGoal.Tracking('qm_dot_ref','qm_dot',1);

% [st,fSoft,~,info] = systune(st0, [req1, req2, req3, req4]);
[st,fSoft,~,info] = systune(st0, [req1, req3]);

% T1= getIOTransfer(st,'qb_ref','qb');
% stepplot(T1)
% 
% T2 = getIOTransfer(st,'qm_ref','qm');
% stepplot(T2)

showTunable(st)

Kp_b = diag(getBlockParam(st, 'Kp_b').Gain.Value);
Kd_b = diag(getBlockParam(st, 'Kd_b').Gain.Value);
Kp_m = diag(getBlockParam(st, 'Kp_m').Gain.Value);
Kd_m = diag(getBlockParam(st, 'Kd_m').Gain.Value);

% req2 = TuningGoal.Gain('delta fin','delta fin',tf(25,[1 0]));
% req3 = TuningGoal.Margins('delta fin',7,45);
% max_gain = frd([2 200 200],[0.02 2 200]);
% req4 = TuningGoal.Gain('delta fin','az',max_gain);


% writeBlockValue(st)
% showTunable(st)

warning on