% Position
sr2.q0 = diag([0; 0; 0; 0; 0; 0]) * rand(6, 1);
sr2.qm = diag([1; 0]) * rand(2, 1);

% Speed
sr2.q0_dot = diag([0; 0; 0; 1; 0; 0]) * rand(6, 1);
sr2.qm_dot = diag([0; 0]) * rand(2, 1);

% Accel
q0_ddot = diag([0; 0; 0; 0; 0; 0]) * rand(6, 1);
qm_ddot = diag([0; 0]) * rand(2, 1);

% Set values
q = sr2.q;
q_dot = sr2.q_dot;

qb = q(1:6);
qm = q(7:end);

qb_dot = q_dot(1:6);
qm_dot = q_dot(7:end);

fprintf('NEW CONFIG\n')

clc

C_symb = sr2.C;
C_D = sr2.CMat;


run spart_script.m

h_S = C_spart * q_dot;
h_Symb = C_symb * q_dot;
h_D = C_D*q_dot;

Rb = sr2.Ttree.spacecraftBase(1:3, 1:3);

fprintf("--- C Matrix ---\n")
fprintf('SPART:\n')
disp(C_spart)
fprintf('Symb\n')
disp(sr2.C)
fprintf('DeNOC\n')
disp(C)

fprintf("--- h ---\n")
fprintf('[\tSPART\t\t\t\tSymb\t\t\t\tDeNOC]\n')
disp([h_S, zeros(8, 1), h_Symb, zeros(8, 1), h_D, [Rb.'*h_S(1:3); h_S(4:end)]])

if isequal(round(h_S, 5), round(h_Symb, 5)) && isequal(round(h_S, 5), round(h_D, 5))
    fprintf('All equal\n')
elseif ~isequal(round(h_Symb, 5), round(h_D, 5))
    fprintf('Symb and DeNOC diff\n')
else
    fprintf('DeNOC and Symb same, Spart diff\n')
end
if isequal(round(h_S, 5), round(h_D, 5))
    fprintf('Spart and DeNOC same\n')
end