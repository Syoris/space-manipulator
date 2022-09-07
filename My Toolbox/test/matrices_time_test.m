%% Init
% clc
load 'SC_2DoF.mat'
sc.homeConfig;
sc.q_dot = [1; 2; 3; 4; 5; 6; pi/2; pi/4];

%% Just H Comp
fprintf("\n--- H ---\n")
f1 = matlabFunction(sc.H_symb);
f2 = matlabFunction(sc.H_symb, 'Vars', {sc.q_symb});

N = 100;
disp("Unoptimized")
tic
for i=1:N
    sc.getH(sc.q, sc.q_dot);
end
t0 = toc;
fprintf("\t Average: %f\n", t0/N)

disp("Min Vars")
tic
for i=1:N
    f1(sc.q(5), sc.q(7), sc.q(8), sc.q(4), sc.q(6));
end
t1 = toc;
fprintf("\t Average: %f\n", t1/N)

disp("Full vect Vars")
tic
for i=1:N
    f2(sc.q);
end
t2 = toc;
fprintf("\t Average: %f\n", t2/N)

%% All Mats
fprintf("\n--- All Mats ---\n")
f1_1 = matlabFunction(sc.H_symb, 'Vars', {sc.q_symb, sc.q_dot_symb});
f1_2 = matlabFunction(sc.C_symb, 'Vars', {sc.q_symb, sc.q_dot_symb});
f1_3 = matlabFunction(sc.Q_symb, 'Vars', {sc.q_symb, sc.q_dot_symb});

f2 = matlabFunction(sc.H_symb, sc.C_symb, sc.Q_symb, 'Vars', {sc.q_symb, sc.q_dot_symb});

N = 100;
disp("Unoptimized")
tic
for i=1:N
    sc.H();
    sc.C();
    sc.Q();
end
t0 = toc;
fprintf("\t Average: %f\n", t0/N)

disp("Three Funcs")
tic
for i=1:N
    H = f1_1(sc.q, sc.q_dot);
    C = f1_2(sc.q, sc.q_dot);
    Q = f1_3(sc.q, sc.q_dot);
end
t1 = toc;
fprintf("\t Average: %f\n", t1/N)

disp("Full vect Vars")
tic
for i=1:N
    [H, C, Q] = f2(sc.q, sc.q_dot);
end
t2 = toc;
fprintf("\t Average: %f\n", t2/N)

disp("In Class")
tic
for i=1:N
    [H, C, Q] = sc.getMats(sc.q, sc.q_dot);
end
t2 = toc;
fprintf("\t Average: %f\n", t2/N)

%% Kinematics Test
% sc: SpaceRobot with Ttree in symbolic form simplified
% sc2: SpaceRobot with Ttree in symbolic form NOT simplified

q0 = [0.1; 0.2; 0.3; pi/2; pi/2; pi/2];
q0_dot = ones(6, 1)*0.1;

qm = ones(6, 1)*pi/4;
qm_dot = ones(6, 1)*pi/40;

sc.q = [q0; qm];
sc.q_dot = [q0_dot; qm_dot];

sc2.q = [q0; qm];
sc2.q_dot = [q0_dot; qm_dot];

N = 100;
disp("Simplified")
tic
for i=1:N
    sc.Ttree;
end
t0 = toc;
fprintf("\t Average: %f\n", t0/N)

disp("Not Simplified")
tic
for i=1:N
    sc2.Ttree;
end
t0 = toc;
fprintf("\t Average: %f\n", t0/N)

%% Inertial Jacobians
q0 = [0.1; 0.2; 0.3; pi/2; pi/2; pi/2];
q0_dot = ones(6, 1)*0.1;

qm = ones(6, 1)*pi/4;
qm_dot = ones(6, 1)*pi/40;

sc.q = [q0; qm];
sc.q_dot = [q0_dot; qm_dot];

sc2.q = [q0; qm];
sc2.q_dot = [q0_dot; qm_dot];

N = 100;
disp("Simplified")
tic
for i=1:N
    sc.JacobsCoM;
end
t0 = toc;
fprintf("\t Average: %f\n", t0/N)

disp("Not Simplified")
tic
for i=1:N
    sc2.JacobsCoM;
end
t0 = toc;
fprintf("\t Average: %f\n", t0/N)