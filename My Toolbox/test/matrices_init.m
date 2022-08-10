%% Initialization of H and C
clc
run 'SPART_example.m'
load 'SC_2DoF_old.mat'

% Spacecraft State
qm_val=[pi/6; -pi/4];
r0_val = [0.5; 0.2; 1];
delta0_val = [0; 0; 0];

r0_dot_val = [0; 0; 0];
w0_val = [0; 0; 0];
qm_dot_val = [4;-1]*pi/180; %Joint velocities

% Initial condition
syms 't' 'Rx' 'Ry' 'Rz' 'r' 'p' 'y' 'qm1(t)' 'qm2(t)'
syms 'Rx_d' 'Ry_d' 'Rz_d' 'wx' 'wy' 'wz' 'qm1_d' 'qm2_d'

R0_val = rpy2r(delta0_val');
r0= [Rx;Ry;Rz];
delta0 = [r;p;y];
R0 = rpy2r(delta0');
qm= [qm1(t); qm2(t)];

q = [r0; delta0; qm];

q_dot = [Rx_d; Ry_d; Rz_d; wx; wy; wz; qm1_d; qm2_d];
qm_dot = [qm1_d; qm2_d];

q_val = [r0_val; delta0_val; qm_val];
q_dot_val = [r0_dot_val; w0_val; qm_dot_val];

% SPART
filename='SC_2DoF.urdf';
[robotSpart,robot_keys] = urdf2robot(filename);

% Spacecraft state
sc.JointsConfig = qm_val';
sc.JointsSpeed = qm_dot_val';
sc.BaseConfig = [r0_val'; delta0_val'];
sc.BaseSpeed = [r0_dot_val'; w0_val'];


%% H
sc.initMats();
sc.initMassMat();




%% C