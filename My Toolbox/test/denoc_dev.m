clearvars
load 'SC_2DoF.mat'
sc.homeConfig;
sc.q0_dot = [0.1; 0.2; 0.3; 0.01; 0.02; 0.03];
sc.qm_dot = [pi/2; -pi/4]/10; 

%% ### Body matrices ###
body = sc.Bodies{2};

% --- ParentRotM and Parent Length ---
% To set once
% rotM: R_i_i-1: From frame i to i-1
% L: vector from frame O_i-1 to E_i-1 -> length of previous link
T = body.Joint.transformBody2ParentSymb;
T_val = body.Joint.transformBody2Parent;
[rotM, L] = tr2rt(T);    
body.ParentRotM = rotM;
body.Parent.Length = double(L);

% Computed in the get
rotM_val = double(subs(body.ParentRotM, body.Joint.SymbVar, body.Joint.Position));
R_full = [rotM_val, zeros(3, 3); zeros(3, 3), rotM_val];


% --- A ---
% Compute once, ind from config
body.A = [eye(3), -skew(body.Parent.Length); zeros(3, 3), eye(3)];

% --- P ---
body.P = [zeros(3,1); body.Joint.Axis.'];

% --- M ---
body.M = [body.Mass*eye(3), zeros(3, 3); zeros(3, 3), body.InertiaM];
