%% nmpc_matlab.m  NMPC First tests
% Design and test of NMPC controller for SR
%
% Following Pendulum SwingUp examples
% To open: openExample('mpc/SwingupControlOfPendulumUsingNMPCExample')
%

clc
close all
load 'SR2.mat'
sr.homeConfig();
sr.q = [0; 0; 0; 0; 0; 0; 0; 0];

% --- Create NMPC ---
% Parameters
Ts = 0.1;
Tp = 10; % # of prediction steps
Tc = 5; % # of ctrl steps

% 
n = sr.NumActiveJoints;
nx = 12 + 2*n; % Change to 24 + 2*n w/ EE
ny = 8;
nu = n+6;

nlobj = nlmpc(nx,ny,nu);


nlobj.Ts = Ts;
nlobj.PredictionHorizon = Tp;
nlobj.ControlHorizon = Tc;

% Prediction Model
nlobj.Model.NumberOfParameters = 0;
nlobj.Model.StateFcn = "sr_state_func";
nlobj.Model.OutputFcn = "sr_output_func";

nlobj.Model.IsContinuousTime = true;

% Cost Function
r_W = 1; % Position position weight
psi_W = 1; % Position orientation weight
qm_W = 1; % Joint position weight

fb_W = 1;
nb_W = 0.1;
taum_W = 0.1;

fb_rate_W = 1;
nb_rate_W = 1;
taum_rate_W = 1; 

nlobj.Weights.OutputVariables = [ones(1, 3)*r_W, ones(1, 3)*psi_W, ones(1, 2)*qm_W]; % [rx ry rz psi_x psi_y psi_z qm1 qm2]

% nlobj.Weights.ManipulatedVariables = [ones(1, 3)*fb_W, ones(1, 3)*nb_W, ones(1, 2)*taum_W];

% nlobj.Weights.ManipulatedVariablesRate = [ones(1, 3)*fb_rate_W, ones(1, 3)*nb_rate_W, ones(1, 2)*qm_W];


% --- Constraints ---
% % Joint 1
% nlobj.OV(7).Min = -pi/2;
% nlobj.OV(7).Max = pi/2;
% 
% % Joint 2
% nlobj.OV(8).Min = -pi/2;
% nlobj.OV(8).Max = pi/2;
% 
% % Torque limits
% for i=1:8
%     nlobj.MV(i).Min = -10;
%     nlobj.MV(i).Max = 10;
% end

% Validate
x0 = [sr.q; sr.q_dot];
u0 = zeros(8, 1);
validateFcns(nlobj, x0, u0, []);

yref = [x0(1:6)', 0, 0];

%% State Estimation
% TODO: ADD EKF for Xe

%% Closed-Loop Simulation in MATLAB(R)
% Specify the initial conditions for simulations by setting the initial
% plant state and output values. Also, specify the initial state of the
% extended Kalman filter.

% --- Initial Conds ---
% The initial conditions are set to home config. Speeds are null
sr.homeConfig();
sr.q = zeros(8, 1);
x = [sr.q; sr.q_dot];
y = [sr.q];

% Input torques to zeros
mv = zeros(8, 1);

% --- Reference ---
% From home config go to zeros(8, 1). At 10sec, move base to [0.5; 0.5; 0]
% keeping manipulator at same position

yref1 = [0.5, 0, 0, 0, 0, 0, 0, 0];
% yref2 = [0, 0, 0, zeros(1, 5)];

% --- Sim Setup ---
% nloptions = nlmpcmoveopt;
% nloptions.Parameters = {sr};

% --- Sim ---
profile on
tic
Duration = 1.0;
hbar = waitbar(0,'Simulation Progress');
xHistory = x;
nloptions = nlmpcmoveopt;
minTime = Inf;

for ct = 1:(Duration/Ts)
    tstart = tic;
    % Set references
%     if ct*Ts<10
%         yref = yref1;
%     else
%         yref = yref2;
%     end
    yref = yref1;

    % Correct previous prediction using current measurement.
%     xk = correct(EKF, y);
    xk = x;

    % Compute optimal control moves.
    [mv,nloptions,info] = nlmpcmove(nlobj,xk,mv,yref,[],nloptions);
    
    % Predict prediction model states for the next iteration.
%     predict(EKF, [mv; Ts]);
    
    % Implement first optimal control move and update plant states.
    x = sr_state_func(x,mv);
    
    % Generate sensor data with some white noise.
%     y = x([1 3]) + randn(2,1)*0.01; 
    y = sr_output_func(x, mv); 
    
    % Save plant states for display.
    xHistory = [xHistory x]; %#ok<*AGROW>
    time = ct*Ts;
    waitbar(time/Duration,hbar);
    

    tElasped = toc(tstart);
    minTime = min(tElasped, minTime);
    fprintf('Time: %.2f \t (computed in: %.2f)\n', time, tElasped)
end
close(hbar)
averageTime = toc/(Duration/Ts);
fprintf("Average time: %.2f\n", averageTime)
fprintf("Min time: %.2f\n", minTime)

profile viewer
profile off

%% Plot results
figure
subplot(2,2,1)
plot(0:Ts:Duration,xHistory(1,:))
xlabel('time')
ylabel('[m]')
title('SC X')

subplot(2,2,2)
plot(0:Ts:Duration,xHistory(2,:))
xlabel('time')
ylabel('[m]')
title('SC Y')

subplot(2,2,3)
plot(0:Ts:Duration,xHistory(7,:))
xlabel('time')
ylabel('theta')
title('Joint 1')

subplot(2,2,4)
plot(0:Ts:Duration,xHistory(8,:))
xlabel('time')
ylabel('theta')
title('Joint 2')

% Animate
data = timeseries(xHistory(1:8, :), 0:Ts:Duration);
figure
tic
sr.animate(data, 'fps', 17, 'rate', 1); 
toc

%% Simulink

% To use optional parameters in the prediction model, the model has a
% Simulink Bus block connected to the |params| input port of the Nonlinear
% MPC Controller block. To configure this bus block to use the |Ts|
% parameter, create a Bus object in the MATLAB workspace and configure
% the Bus Creator block to use this object. To do so, use the
% <docid:mpc_ref#mw_8f9e5d62-cfe1-499a-9420-e5099dedfe76> function. In this
% example, name the Bus object |'myBusObject'|.
mdl = 'nmpc_sim';
% createParameterBus(nlobj,[mdl '/Nonlinear MPC Controller'],'myBusObject',{test.Bodies});

% % Animation
% % clc
% % close all
% folder = 'Project/Videos/';
% fileName = '';
% 
% if ~strcmp(fileName, '')
%     savePath = strcat(folder, fileName);
% else
%     savePath = '';
% end
% 
% simRes = out;
% 
% % trajRes = struct();
% % trajRes.ref = traj;
% % trajRes.Xee = simRes.Xee;
% 
% tic
% sr.animate(simRes.q, 'fps', 17, 'rate', 1, 'fileName', savePath); 
% toc
