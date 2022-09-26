clear;
clc
close all
load('SR2.mat', 'sr');
sr.homeConfig;
sr.q = zeros(8, 1);
ref = [0.5; 0; 0; 0; 0; 0; 0; 0];

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 

    acadoSet('problemname', 'nmpc_test'); 
    
    DifferentialState q(8) q_dot(8);
    Control u(8);
    
    % Set default objects
    f = acado.DifferentialEquation();
    f.linkMatlabODE('srode');
    
    ocp = acado.OCP(0.0, 1, 5);


    h={q(1), q(2), q(3), q(4), q(5), q(6), q(7), q(8)};  % the LSQ-Function

    Q = eye(8);                             % The weighting matrix
    
    ocp.minimizeLSQ( Q, h, ref);             % Minimize this Least Squares Term
    
    ocp.subjectTo( f );
    ocp.subjectTo( 'AT_START', q ==  sr.q );
    ocp.subjectTo( 'AT_START', q_dot ==  sr.q_dot);

%     ocp.subjectTo( -10 <= u <= 10);
    
    algo = acado.OptimizationAlgorithm(ocp);   
    
    % !!
    % algo.set( 'HESSIAN_APPROXIMATION', 'EXACT_HESSIAN' );    
    % DO NOT USE EXACT HESSIAN WHEN LINKING TO MATLAB ODE
    % !!

    algo.set( 'KKT_TOLERANCE', 1e-1 );
    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

%%
profile on
fprintf("Running ocp\n")
tic;
out = nmpc_test_RUN();                % Run the test
toc;
profile viewer
profile off

draw;