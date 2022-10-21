%% SR3
sr = sr3;
n = sr3.NumActiveJoints;
nk = sr3.NumBodies;
N = n+6;

q0 = zeros(N, 1);
q_dot_0 = zeros(N, 1);

taub = diag([1; 1; 1; 1; 1; 1]) * rand(6, 1);
taum = diag([1; 1; 1]) * rand(n, 1);
tau = [taub; taum];

%% SR6
sr = sr6;
n = sr6.NumActiveJoints;
nk = sr6.NumBodies;
N = n+6;

q0 = zeros(N, 1);
q_dot_0 = zeros(N, 1);

taub = diag([0; 0; 0; 0; 0; 0]) * rand(6, 1);
taum = diag([0; 0; 0; 0; 0; 1]) * rand(n, 1);
tau = [taub; taum];


